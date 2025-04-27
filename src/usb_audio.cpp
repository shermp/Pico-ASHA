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
#include <stdio.h>
#include <string.h>

#include <pico/time.h>
#include <tusb.h>

#include "usb_descriptors.h"

#include "asha_audio.hpp"

namespace asha
{

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTOTYPES
//--------------------------------------------------------------------+

// List of supported sample rates
#if defined(__RX__)
  static const uint32_t sample_rates[] = {16000};
#else
  static const uint32_t sample_rates[] = {16000};
#endif

static uint32_t current_sample_rate  = 16000;

#define N_SAMPLE_RATES  TU_ARRAY_SIZE(sample_rates)

namespace USBVol 
{
/* ASHA volume range is -48dB to 0dB, and is set as an int8_t
 * with a range of -128 to 0. This gives steps of 0.375 dB.

 * USB volume control is an int16_t, from -128dB to +128dB, in
 * steps of 1/256 dB. The ASHA resolution is 96/256dB per step 
 * which gives a range of -12192 to 0 and a resolution of 96.
 */

  constexpr int16_t max =  -3072; // -12dB
  constexpr int16_t min = -12192; // -47.625dB
  constexpr int16_t res =     96; // 96/256 is the step between ASHA volume levels
  constexpr int16_t mute = 0x8000;
}

// Audio controls
// Current states
static int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1] = {};       // +1 for master channel 0
static int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1] = {};    // +1 for master channel 0

// Buffer for microphone data
[[maybe_unused]] static int16_t mic_buf[CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 2] = {};
// Buffer for speaker data
static int16_t spk_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 2] = {};
// Speaker data size received in the last frame
static int spk_data_size;
// Resolution per format
const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX,
                                                                        CFG_TUD_AUDIO_FUNC_1_FORMAT_2_RESOLUTION_RX};
// Current resolution, update on format change
uint8_t current_resolution;

// Last time a packet was recieved, used to detect when a host stops sending audio
static absolute_time_t last_packet_time = 0;

// A counter for the number of consecutive silence audio packets
// Note: this number will be approximately milliseconds
static uint32_t silence_counter = 0ul;

// The silence_counter threshold by wish to signal streaming stopped
static constexpr uint32_t silence_timeout = 10'000ul;

void audio_task(void);
void serial_task(void);

void tud_cdc_line_state_cb([[maybe_unused]] uint8_t itf, 
                           [[maybe_unused]] bool dtr, 
                           [[maybe_unused]] bool rts)
{}

// void serial_task()
// {
//     int ch;
//     while ((ch = stdio_getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
//       char c = (char)ch;
//       if (c == '\r') {
//         complete_std_line = curr_stdin_buff;
//         curr_stdin_buff.clear();
//         if (usb_ser_ctx) {
//             async_context_set_work_pending(usb_ser_ctx, &stdin_pending_worker);
//         }
//         return;
//       } else if (c == '\n') {
//         return;
//       }
//       curr_stdin_buff.append(1, c);
//     }
// }

/*------------- MAIN -------------*/
void usb_main(void)
{
  TU_LOG1("Headset running\n");
  while (1)
  {
    tud_task(); // TinyUSB device task
    audio_task();
    //serial_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
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

// Helper for clock get requests
static bool tud_audio_clock_get_request(uint8_t rhport, audio_control_request_t const *request)
{
  TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);

  if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
  {
    if (request->bRequest == AUDIO_CS_REQ_CUR)
    {
      TU_LOG1("Clock get current freq %lu\n", current_sample_rate);

      audio_control_cur_4_t curf = { (int32_t) tu_htole32(current_sample_rate) };
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
    }
    else if (request->bRequest == AUDIO_CS_REQ_RANGE)
    {
      audio_control_range_4_n_t(N_SAMPLE_RATES) rangef = {};
      rangef.wNumSubRanges = tu_htole16(N_SAMPLE_RATES);
      TU_LOG1("Clock get %d freq ranges\n", N_SAMPLE_RATES);
      for(uint8_t i = 0; i < N_SAMPLE_RATES; i++)
      {
        rangef.subrange[i].bMin = (int32_t) sample_rates[i];
        rangef.subrange[i].bMax = (int32_t) sample_rates[i];
        rangef.subrange[i].bRes = 0;
        TU_LOG1("Range %d (%d, %d, %d)\n", i, (int)rangef.subrange[i].bMin, (int)rangef.subrange[i].bMax, (int)rangef.subrange[i].bRes);
      }
      
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
    }
  }
  else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID &&
           request->bRequest == AUDIO_CS_REQ_CUR)
  {
    audio_control_cur_1_t cur_valid = { .bCur = 1 };
    TU_LOG1("Clock get is valid %u\n", cur_valid.bCur);
    return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
  }
  TU_LOG1("Clock get request not supported, entity = %u, selector = %u, request = %u\n",
          request->bEntityID, request->bControlSelector, request->bRequest);
  return false;
}

// Helper for clock set requests
static bool tud_audio_clock_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
  (void)rhport;

  TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
  TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

  if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
  {
    TU_VERIFY(request->wLength == sizeof(audio_control_cur_4_t));

    current_sample_rate = (uint32_t) ((audio_control_cur_4_t const *)buf)->bCur;

    TU_LOG1("Clock set current freq: %ld\n", current_sample_rate);

    return true;
  }
  else
  {
    TU_LOG1("Clock set request not supported, entity = %u, selector = %u, request = %u\n",
            request->bEntityID, request->bControlSelector, request->bRequest);
    return false;
  }
}

// Helper for feature unit get requests
static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio_control_request_t const *request)
{
  TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);

  if (request->bControlSelector == AUDIO_FU_CTRL_MUTE && request->bRequest == AUDIO_CS_REQ_CUR)
  {
    audio_control_cur_1_t mute1 = { .bCur = mute[request->bChannelNumber] };
    TU_LOG1("Get channel %u mute %d\n", request->bChannelNumber, mute1.bCur);
    return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &mute1, sizeof(mute1));
  }
  else if (UAC2_ENTITY_SPK_FEATURE_UNIT && request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
  {
    if (request->bRequest == AUDIO_CS_REQ_RANGE)
    {
      audio_control_range_2_n_t(1) range_vol = {
        .wNumSubRanges = tu_htole16(1),
        .subrange = { { .bMin = tu_htole16(USBVol::min), 
                        .bMax = tu_htole16(USBVol::max),
                        .bRes = tu_htole16(USBVol::res) } }
      };
      TU_LOG1("Get channel %u volume range (%d, %d, %u) dB\n", request->bChannelNumber,
              range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
    }
    else if (request->bRequest == AUDIO_CS_REQ_CUR)
    {
      audio_control_cur_2_t cur_vol = { .bCur = tu_htole16(volume[request->bChannelNumber]) };
      TU_LOG1("Get channel %u volume %d dB\n", request->bChannelNumber, cur_vol.bCur / 256);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
    }
  }
  TU_LOG1("Feature unit get request not supported, entity = %u, selector = %u, request = %u\n",
          request->bEntityID, request->bControlSelector, request->bRequest);

  return false;
}

// Helper for feature unit set requests
static bool tud_audio_feature_unit_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
  (void)rhport;

  TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
  TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

  if (request->bControlSelector == AUDIO_FU_CTRL_MUTE)
  {
    TU_VERIFY(request->wLength == sizeof(audio_control_cur_1_t));

    mute[request->bChannelNumber] = ((audio_control_cur_1_t const *)buf)->bCur;

    TU_LOG1("Set channel %d Mute: %d\n", request->bChannelNumber, mute[request->bChannelNumber]);

    return true;
  }
  else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
  {
    TU_VERIFY(request->wLength == sizeof(audio_control_cur_2_t));

    volume[request->bChannelNumber] = ((audio_control_cur_2_t const *)buf)->bCur;

    TU_LOG1("Set channel %d volume: %d\n", request->bChannelNumber, volume[request->bChannelNumber]);

    return true;
  }
  else
  {
    TU_LOG1("Feature unit set request not supported, entity = %u, selector = %u, request = %u\n",
            request->bEntityID, request->bControlSelector, request->bRequest);
    return false;
  }
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific get request received for an entity
extern "C" bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
  audio_control_request_t const *request = (audio_control_request_t const *)p_request;

  if (request->bEntityID == UAC2_ENTITY_CLOCK)
    return tud_audio_clock_get_request(rhport, request);
  if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
    return tud_audio_feature_unit_get_request(rhport, request);
  else
  {
    TU_LOG1("Get request not handled, entity = %d, selector = %d, request = %d\n",
            request->bEntityID, request->bControlSelector, request->bRequest);
  }
  return false;
}

// Invoked when audio class specific set request received for an entity
extern "C" bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
  audio_control_request_t const *request = (audio_control_request_t const *)p_request;

  if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
    return tud_audio_feature_unit_set_request(rhport, request, buf);
  if (request->bEntityID == UAC2_ENTITY_CLOCK)
    return tud_audio_clock_set_request(rhport, request, buf);
  TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\n",
          request->bEntityID, request->bControlSelector, request->bRequest);

  return false;
}

extern "C" bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void)rhport;

  [[maybe_unused]] uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  [[maybe_unused]] uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  return true;
}

extern "C" bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void)rhport;
  [[maybe_unused]] uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  TU_LOG2("Set interface %d alt %d\n", itf, alt);
  
  // Clear buffer when streaming format is changed
  spk_data_size = 0;
  if(alt != 0)
  {
    current_resolution = resolutions_per_format[alt-1];
  }

  return true;
}

extern "C" bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting)
{
  (void)rhport;
  (void)func_id;
  (void)ep_out;
  (void)cur_alt_setting;

  //printf("tud_audio_rx_done_pre_read_cb\n");
  spk_data_size = tud_audio_read(spk_buf, n_bytes_received);
  return true;
}

extern "C" bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
  (void)rhport;
  (void)itf;
  (void)ep_in;
  (void)cur_alt_setting;

  // This callback could be used to fill microphone data separately
  return true;
}

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

void audio_task(void)
{

  // Dividing the USB volume by USBVol::res gives a volume that
  // matches the ASHA volume
  AudioBuffer::Volume v = {
    .l = mute[1] ? volume_mute : (int8_t)(volume[1] / USBVol::res), 
    .r = mute[2] ? volume_mute : (int8_t)(volume[2] / USBVol::res)
  };
    
  audio_buff.set_volume(v);

  if (spk_data_size == pcm_stereo_packet_size * 2) {
    last_packet_time = get_absolute_time();
    if (std::all_of(std::begin(spk_buf), std::end(spk_buf), [](int16_t i) {return i == 0; })) {
      ++silence_counter;
    } else {
      silence_counter = 0;
    }
    audio_buff.pcm_streaming = (silence_counter >= silence_timeout) ? false : true;
    audio_buff.encode_1ms_audio(spk_buf);
    spk_data_size = 0;
  } else {
      absolute_time_t now = get_absolute_time();
      if (absolute_time_diff_us(last_packet_time, now) > 5000) {
        audio_buff.pcm_streaming = false;
        last_packet_time = now;
      }
  }
}

} // namespace asha