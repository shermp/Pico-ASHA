/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jerzy Kasenberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <algorithm>
#include <array>
#include <stdio.h>
#include <string.h>

#include <pico/time.h>

#if PICO_RP2040
#include <hardware/regs/usb.h>
#include <hardware/structs/usb.h>
#include <hardware/watchdog.h>
#endif

//#include <speex/speex_resampler.h>

#include <tusb.h>

#include "usb_descriptors.h"
#include "usb_common.hpp"

#include "asha_audio.hpp"
#include "asha_comms.hpp"

namespace asha
{

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTOTYPES
//--------------------------------------------------------------------+

static uint32_t current_sample_rate  = CFG_TUD_AUDIO_FUNC_1_RESOLUTION_RX * 1000;

constexpr uint8_t audio_channel_count = CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1;

static bool supported_sample_rate(uint32_t rate)
{
  return rate == 16'000u || rate == 48'000u;
}

static uint16_t samples_per_frame(uint32_t rate)
{
  return rate == 16'000u ? pcm_packet_samples :
         rate == 48'000u ? pcm_max_samples : 0u;
}

static uint16_t bytes_per_frame(uint32_t rate)
{
  return samples_per_frame(rate) * sizeof(PCMStereoSample);
}

static bool valid_audio_channel(uint8_t channel)
{
  return channel < audio_channel_count;
}

// Audio controls
// Current states
static int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1] = {};       // +1 for master channel 0
static int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1] = {};    // +1 for master channel 0

// Buffer for speaker data, can support up to 48KHz
alignas(uint32_t) static std::array<PCMStereoSample, pcm_max_samples> spk_buf{};

static const std::array<PCMStereoSample, pcm_max_samples> silence_buff{};

// A counter for the number of consecutive silence audio packets
// Note: this number will be approximately milliseconds
static uint32_t silence_counter = 0ul;

// The silence_counter threshold by wish to signal streaming stopped
static constexpr uint32_t silence_timeout = 10'000ul;

// A single empty FIFO check is normal when the USB and local audio clocks
// straddle each other.  Treat only a sustained lack of complete USB frames as
// the host having stopped audio, rather than stopping an ASHA stream on a
// momentary underrun.
static constexpr int64_t audio_packet_timeout_us = 5'000;
static absolute_time_t last_pcm_packet_time = 0;

static void set_sample_rate(uint32_t rate)
{
  if (current_sample_rate == rate) { return; }
  // Control handling and audio_task run on core 0. Discard queued old-rate
  // samples before the next read and allow the new stream time to refill.
  tud_audio_clear_ep_out_ff();
  current_sample_rate = rate;
  silence_counter = 0;
  last_pcm_packet_time = get_absolute_time();
}

USBSettings usb_settings = {};

USBSettings::operator bool() const
{
  return (uac_version == 1 || uac_version == 2) 
          && (min_vol <= usb_vol_max && min_vol >= usb_vol_min)
          && (max_vol <= usb_vol_max && max_vol >= usb_vol_min)
          && (max_vol >= min_vol);
}

// Audio task pacing

constexpr uint64_t audio_start_delay_us = 100u;

static absolute_time_t next_audio_task_time = 0;
static void audio_task();

void tud_cdc_line_state_cb([[maybe_unused]] uint8_t itf,
                           [[maybe_unused]] bool dtr,
                           [[maybe_unused]] bool rts)
{}

#if PICO_RP2040
static uint32_t last_usb_sof_us = 0;

extern "C" void tud_sof_cb(uint32_t /* frame_count */)
{
  last_usb_sof_us = time_us_32();
}

static void usb_stall_watchdog()
{
  uint32_t const now = time_us_32();
  uint32_t const status = usb_hw->sie_status;
  if (!tud_mounted() || tud_suspended()
      || (status & USB_SIE_STATUS_SUSPENDED_BITS)
      || !(status & USB_SIE_STATUS_BIT_STUFF_ERROR_BITS)) {
    last_usb_sof_us = now;
  } else if (now - last_usb_sof_us >= 1'000'000u) {
    // Captured stall: bit-stuff error with no SOFs while configured/awake.
    // Do not poll SOF_RD: reading it would acknowledge TinyUSB's SOF IRQ.
    watchdog_reboot(0, 0, 0);
  }
}
#endif

/*------------- MAIN -------------*/
void usb_main(void)
{
  TU_LOG1("Headset running\n");
  next_audio_task_time = make_timeout_time_us(audio_start_delay_us);

  while (1)
  {
    tud_task(); // TinyUSB device task

#if PICO_RP2040
    usb_stall_watchdog();
#endif

    // Keep FIFO reads in the same foreground context as TinyUSB control
    // handling. In particular, this prevents a sample-rate interface change
    // from clearing the FIFO while a timer interrupt is reading it.
    audio_task();
    comm::usb_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
extern "C" void tud_mount_cb(void)
{
#if PICO_RP2040
  last_usb_sof_us = time_us_32();
  // TinyUSB's E15 workaround times bulk IN transfers from the last SOF IRQ.
  // With FIFO-count audio feedback, nothing keeps that IRQ enabled while CDC
  // is idle. Keep it running so the next CDC transfer uses a current frame
  // timestamp and cannot bypass the workaround with a stale one.
  tud_sof_cb_enable(true);
#endif
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void)remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// UAC1 Helper Functions
//--------------------------------------------------------------------+

static bool audio10_set_req_ep(tusb_control_request_t const *p_request, uint8_t *pBuff) {
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);

  switch (ctrlSel) {
    case AUDIO10_EP_CTRL_SAMPLING_FREQ:
      if (p_request->bRequest == AUDIO10_CS_REQ_SET_CUR) {
        // Request uses 3 bytes
        TU_VERIFY(p_request->wLength == 3);

        uint32_t const rate = (uint32_t)pBuff[0]
                            | ((uint32_t)pBuff[1] << 8)
                            | ((uint32_t)pBuff[2] << 16);
        TU_VERIFY(supported_sample_rate(rate));
        set_sample_rate(rate);

        
        TU_LOG2("EP set current freq: %" PRIu32 "\r\n", current_sample_rate);

        return true;
      }
      break;

    // Unknown/Unsupported control
    default:
      TU_BREAKPOINT();
      return false;
  }

  return false;
}

static bool audio10_get_req_ep(uint8_t rhport, tusb_control_request_t const *p_request) {
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);

  switch (ctrlSel) {
    case AUDIO10_EP_CTRL_SAMPLING_FREQ:
      if (p_request->bRequest == AUDIO10_CS_REQ_GET_CUR) {
        TU_LOG2("EP get current freq\r\n");

        uint8_t freq[3];
        freq[0] = (uint8_t) (current_sample_rate & 0xFF);
        freq[1] = (uint8_t) ((current_sample_rate >> 8) & 0xFF);
        freq[2] = (uint8_t) ((current_sample_rate >> 16) & 0xFF);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, freq, sizeof(freq));
      }
      break;

    // Unknown/Unsupported control
    default:
      TU_BREAKPOINT();
    return false;
  }

  return false;
}

static bool audio10_set_req_entity(tusb_control_request_t const *p_request, uint8_t *pBuff) {
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

  // If request is for our feature unit
  if (entityID == UAC1_ENTITY_FEATURE_UNIT) {
    TU_VERIFY(valid_audio_channel(channelNum));
    switch (ctrlSel) {
      case AUDIO10_FU_CTRL_MUTE:
        switch (p_request->bRequest) {
          case AUDIO10_CS_REQ_SET_CUR:
            // Only 1st form is supported
            TU_VERIFY(p_request->wLength == 1);

            mute[channelNum] = pBuff[0];

            TU_LOG2("    Set Mute: %d of channel: %u\r\n", mute[channelNum], channelNum);
            return true;

          default:
            return false; // not supported
        }

      case AUDIO10_FU_CTRL_VOLUME:
        switch (p_request->bRequest) {
          case AUDIO10_CS_REQ_SET_CUR:
            // Only 1st form is supported
            TU_VERIFY(p_request->wLength == 2);

            volume[channelNum] = (int16_t)tu_unaligned_read16(pBuff);
            TU_LOG2("    Set Volume: %d of channel: %u\r\n", volume[channelNum], channelNum);
            return true;

          default:
            return false; // not supported
        }

        // Unknown/Unsupported control
      default:
        TU_BREAKPOINT();
        return false;
    }
  }

  return false;
}

static bool audio10_get_req_entity(uint8_t rhport, tusb_control_request_t const *p_request) {
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

  // If request is for our feature unit
  if (entityID == UAC1_ENTITY_FEATURE_UNIT) {
    TU_VERIFY(valid_audio_channel(channelNum));
    switch (ctrlSel) {
      case AUDIO10_FU_CTRL_MUTE:
        // Audio control mute cur parameter block consists of only one byte - we thus can send it right away
        // There does not exist a range parameter block for mute
        TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &mute[channelNum], 1);

      case AUDIO10_FU_CTRL_VOLUME:
        switch (p_request->bRequest) {
          case AUDIO10_CS_REQ_GET_CUR:
            TU_LOG2("    Get Volume of channel: %u\r\n", channelNum);
            {
              int16_t vol = (int16_t) volume[channelNum];
              return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &vol, sizeof(vol));
            }

          case AUDIO10_CS_REQ_GET_MIN:
            TU_LOG2("    Get Volume min of channel: %u\r\n", channelNum);
            {
              int16_t min = usb_settings.min_vol;
              return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &min, sizeof(min));
            }

          case AUDIO10_CS_REQ_GET_MAX:
            TU_LOG2("    Get Volume max of channel: %u\r\n", channelNum);
            {
              int16_t max = usb_settings.max_vol;
              return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &max, sizeof(max));
  }

          case AUDIO10_CS_REQ_GET_RES:
            TU_LOG2("    Get Volume res of channel: %u\r\n", channelNum);
            {
              int16_t res = usb_vol_res;
              return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &res, sizeof(res));
            }
            // Unknown/Unsupported control
          default:
            TU_BREAKPOINT();
            return false;
        }
        break;

        // Unknown/Unsupported control
      default:
        TU_BREAKPOINT();
    return false;
  }
  }

  return false;
}

//--------------------------------------------------------------------+
// UAC2 Helper Functions
//--------------------------------------------------------------------+

// List of supported sample rates for UAC2
const uint32_t sample_rates[] = {16000, 48000};

#define N_SAMPLE_RATES TU_ARRAY_SIZE(sample_rates)

static bool audio20_clock_get_request(uint8_t rhport, tusb_control_request_t const *request) {
  uint8_t const entity_id = TU_U16_HIGH(request->wIndex);
  uint8_t const ctrl_sel = TU_U16_HIGH(request->wValue);
  TU_ASSERT(entity_id == UAC2_ENTITY_CLOCK);

  if (ctrl_sel == AUDIO20_CS_CTRL_SAM_FREQ) {
    if (request->bRequest == AUDIO20_CS_REQ_CUR) {
      TU_LOG1("Clock get current freq %" PRIu32 "\r\n", current_sample_rate);

      audio20_control_cur_4_t curf = {(int32_t) tu_htole32(current_sample_rate)};
      return tud_audio_buffer_and_schedule_control_xfer(rhport, request, &curf, sizeof(curf));
    } else if (request->bRequest == AUDIO20_CS_REQ_RANGE) {
      audio20_control_range_4_n_t(N_SAMPLE_RATES) rangef =
          {
              .wNumSubRanges = tu_htole16(N_SAMPLE_RATES)};
      TU_LOG1("Clock get %d freq ranges\r\n", N_SAMPLE_RATES);
      for (uint8_t i = 0; i < N_SAMPLE_RATES; i++) {
        rangef.subrange[i].bMin = (int32_t) sample_rates[i];
        rangef.subrange[i].bMax = (int32_t) sample_rates[i];
        rangef.subrange[i].bRes = 0;
        TU_LOG1("Range %d (%d, %d, %d)\r\n", i, (int) rangef.subrange[i].bMin, (int) rangef.subrange[i].bMax, (int) rangef.subrange[i].bRes);
      }

      return tud_audio_buffer_and_schedule_control_xfer(rhport, request, &rangef, sizeof(rangef));
    }
  } else if (ctrl_sel == AUDIO20_CS_CTRL_CLK_VALID &&
             request->bRequest == AUDIO20_CS_REQ_CUR) {
    audio20_control_cur_1_t cur_valid = {.bCur = 1};
    TU_LOG1("Clock get is valid %u\r\n", cur_valid.bCur);
    return tud_audio_buffer_and_schedule_control_xfer(rhport, request, &cur_valid, sizeof(cur_valid));
  }
  TU_LOG1("Clock get request not supported, entity = %u, selector = %u, request = %u\r\n",
          entity_id, ctrl_sel, request->bRequest);
  return false;
}

static bool audio20_clock_set_request(tusb_control_request_t const *request, uint8_t const *buf) {
  uint8_t const entity_id = TU_U16_HIGH(request->wIndex);
  uint8_t const ctrl_sel = TU_U16_HIGH(request->wValue);
  TU_ASSERT(entity_id == UAC2_ENTITY_CLOCK);
  TU_VERIFY(request->bRequest == AUDIO20_CS_REQ_CUR);

  if (ctrl_sel == AUDIO20_CS_CTRL_SAM_FREQ) {
    TU_VERIFY(request->wLength == sizeof(audio20_control_cur_4_t));

    uint32_t const rate = tu_le32toh((uint32_t) ((audio20_control_cur_4_t const *) buf)->bCur);
    TU_VERIFY(supported_sample_rate(rate));
    set_sample_rate(rate);

    TU_LOG1("Clock set current freq: %" PRIu32 "\r\n", current_sample_rate);

    return true;
  } else {
    TU_LOG1("Clock set request not supported, entity = %u, selector = %u, request = %u\r\n",
            entity_id, ctrl_sel, request->bRequest);
    return false;
  }
}

static bool audio20_feature_unit_get_request(uint8_t rhport, tusb_control_request_t const *request) {
  uint8_t const entity_id = TU_U16_HIGH(request->wIndex);
  uint8_t const ctrl_sel = TU_U16_HIGH(request->wValue);
  uint8_t const channel_num = TU_U16_LOW(request->wValue);
  TU_ASSERT(entity_id == UAC2_ENTITY_FEATURE_UNIT);
  TU_VERIFY(valid_audio_channel(channel_num));

  if (ctrl_sel == AUDIO20_FU_CTRL_MUTE && request->bRequest == AUDIO20_CS_REQ_CUR) {
    audio20_control_cur_1_t mute1 = {.bCur = mute[channel_num]};
    TU_LOG1("Get channel %u mute %d\r\n", channel_num, mute1.bCur);
    return tud_audio_buffer_and_schedule_control_xfer(rhport, request, &mute1, sizeof(mute1));
  } else if (ctrl_sel == AUDIO20_FU_CTRL_VOLUME) {
    if (request->bRequest == AUDIO20_CS_REQ_RANGE) {
      audio20_control_range_2_n_t(1) range_vol = {
          .wNumSubRanges = tu_htole16(1),
          .subrange = {{.bMin = tu_htole16(usb_settings.min_vol),
                          .bMax = tu_htole16(usb_settings.max_vol), 
                          .bRes = tu_htole16(usb_vol_res)}}};
      TU_LOG1("Get channel %u volume range (%d, %d, %u) \r\n", channel_num,
              range_vol.subrange[0].bMin, range_vol.subrange[0].bMax, range_vol.subrange[0].bRes);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, request, &range_vol, sizeof(range_vol));
    } else if (request->bRequest == AUDIO20_CS_REQ_CUR) {
      audio20_control_cur_2_t cur_vol = {.bCur = tu_htole16(volume[channel_num])};
      TU_LOG1("Get channel %u volume %d\r\n", channel_num, cur_vol.bCur);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, request, &cur_vol, sizeof(cur_vol));
    }
  }
  TU_LOG1("Feature unit get request not supported, entity = %u, selector = %u, request = %u\r\n",
          entity_id, ctrl_sel, request->bRequest);

  return false;
}

static bool audio20_feature_unit_set_request(tusb_control_request_t const *request, uint8_t const *buf) {
  uint8_t const entity_id = TU_U16_HIGH(request->wIndex);
  uint8_t const ctrl_sel = TU_U16_HIGH(request->wValue);
  uint8_t const channel_num = TU_U16_LOW(request->wValue);
  TU_ASSERT(entity_id == UAC2_ENTITY_FEATURE_UNIT);
  TU_VERIFY(request->bRequest == AUDIO20_CS_REQ_CUR);
  TU_VERIFY(valid_audio_channel(channel_num));

  if (ctrl_sel == AUDIO20_FU_CTRL_MUTE) {
    TU_VERIFY(request->wLength == sizeof(audio20_control_cur_1_t));

    mute[channel_num] = ((audio20_control_cur_1_t const *) buf)->bCur;

    TU_LOG1("Set channel %d Mute: %d\r\n", channel_num, mute[channel_num]);

    return true;
  } else if (ctrl_sel == AUDIO20_FU_CTRL_VOLUME) {
    TU_VERIFY(request->wLength == sizeof(audio20_control_cur_2_t));

    volume[channel_num] = ((audio20_control_cur_2_t const *) buf)->bCur;

    TU_LOG1("Set channel %d volume: %d\r\n", channel_num, volume[channel_num]);

    return true;
  } else {
    TU_LOG1("Feature unit set request not supported, entity = %u, selector = %u, request = %u\r\n",
            entity_id, ctrl_sel, request->bRequest);
    return false;
  }
}

static bool audio20_get_req_entity(uint8_t rhport, tusb_control_request_t const *request) {
  uint8_t const entity_id = TU_U16_HIGH(request->wIndex);

  if (entity_id == UAC2_ENTITY_CLOCK)
    return audio20_clock_get_request(rhport, request);
  if (entity_id == UAC2_ENTITY_FEATURE_UNIT)
    return audio20_feature_unit_get_request(rhport, request);
  else {
    TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\r\n",
            entity_id, TU_U16_HIGH(request->wValue), request->bRequest);
  }
  return false;
}

static bool audio20_set_req_entity(tusb_control_request_t const *request, uint8_t *buf) {
  uint8_t const entity_id = TU_U16_HIGH(request->wIndex);

  if (entity_id == UAC2_ENTITY_FEATURE_UNIT)
    return audio20_feature_unit_set_request(request, buf);
  if (entity_id == UAC2_ENTITY_CLOCK)
    return audio20_clock_set_request(request, buf);
  TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n",
          entity_id, TU_U16_HIGH(request->wValue), request->bRequest);

  return false;
}

//--------------------------------------------------------------------+
// Main Callback Functions
//--------------------------------------------------------------------+

extern "C" bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;
  (void) p_request;
  // uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  // uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  // TU_LOG2("Set interface %d alt %d\r\n", itf, alt);

  return true;
}

// The ASHA encoder consumes audio using the Pico clock, so report FIFO-level
// feedback to let the USB host match that clock instead of drifting into an
// underrun or overrun.
extern "C" void tud_audio_feedback_params_cb(uint8_t func_id,
                                              uint8_t alt_itf,
                                              audio_feedback_params_t *feedback_param) {
  (void)func_id;
  (void)alt_itf;

  feedback_param->method = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
  feedback_param->sample_freq = current_sample_rate;
  feedback_param->fifo_count.fifo_threshold = 4u * bytes_per_frame(current_sample_rate);
}

// Invoked when audio class specific set request received for an EP
extern "C" bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
  (void) rhport;
  (void) pBuff;

  if (tud_audio_version() == 1) {
    return audio10_set_req_ep(p_request, pBuff);
  } else if (tud_audio_version() == 2) {
    // We do not support any requests here
  }

  return false;// Yet not implemented
}

// Invoked when audio class specific get request received for an EP
extern "C" bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;

  if (tud_audio_version() == 1) {
    return audio10_get_req_ep(rhport, p_request);
  } else if (tud_audio_version() == 2) {
    // We do not support any requests here
  }

  return false;// Yet not implemented
}

// Invoked when audio class specific set request received for an entity
extern "C" bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf) {
  (void) rhport;

  if (tud_audio_version() == 1) {
    return audio10_set_req_entity(p_request, buf);
  } else if (tud_audio_version() == 2) {
    return audio20_set_req_entity(p_request, buf);
  }

  return false;
}

// Invoked when audio class specific get request received for an entity
extern "C" bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;

  if (tud_audio_version() == 1) {
    return audio10_get_req_entity(rhport, p_request);
  } else if (tud_audio_version() == 2) {
    return audio20_get_req_entity(rhport, p_request);
  }

  return false;
}

extern "C" bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  (void) rhport;
  (void) p_request;

  // uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  // uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  return true;
}

extern "C" bool tud_audio_rx_done_isr(uint8_t rhport, uint16_t n_bytes_received,
                                       uint8_t func_id, uint8_t ep_out,
                                       uint8_t cur_alt_setting)
{
  (void)rhport;
  (void)n_bytes_received;
  (void)func_id;
  (void)ep_out;
  (void)cur_alt_setting;
  return true;
}


//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

static void audio_task()
{
  if (!time_reached(next_audio_task_time)) {
    return;
  }

  // Advance from the prior deadline rather than from now so task timing does
  // not accumulate drift when an iteration starts late.
  next_audio_task_time = delayed_by_us(next_audio_task_time, 1'000);

  // Always get the current USB volume
  asha_audio_set_curr_usb_vol(mute[0] ? usb_vol_mute : volume[0],
                              mute[1] ? usb_vol_mute : volume[1],
                              mute[2] ? usb_vol_mute : volume[2]);

  uint16_t const samples = samples_per_frame(current_sample_rate);
  uint16_t const frame_bytes = bytes_per_frame(current_sample_rate);
  absolute_time_t const now = get_absolute_time();
  if (samples && tud_audio_available() >= frame_bytes) {
    uint16_t const bytes_read = tud_audio_read(spk_buf.data(), frame_bytes);
    if (bytes_read == frame_bytes) {
      last_pcm_packet_time = now;
      if (std::all_of(spk_buf.begin(), spk_buf.begin() + samples, [](PCMStereoSample s) {return s.left == 0 && s.right == 0; })) {
        if (silence_counter < silence_timeout) {
          ++silence_counter;
        }
      } else {
        silence_counter = 0;
      }
      asha_audio_set_pcm_streaming_enabled((silence_counter >= silence_timeout) ? false : true);
      asha_audio_encode_1ms_pcm(std::span{spk_buf}.first(samples));
      return;
    }
  }

  if (absolute_time_diff_us(last_pcm_packet_time, now) >= audio_packet_timeout_us) {
    // A new run of zero PCM after the host restarts should get its own full
    // silence interval, rather than inheriting the previous stream's count.
    silence_counter = 0;
    asha_audio_set_pcm_streaming_enabled(false);
  }
  // Continue encoding silence while the USB FIFO catches up or the host stops.
  asha_audio_encode_1ms_pcm(std::span{silence_buff}.first(samples));
}

} // namespace asha
