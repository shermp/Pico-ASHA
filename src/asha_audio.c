#include <stdatomic.h>
#include <string.h>

#ifdef PICO_ASHA_ENC_STATS
#include <pico/time.h>
#endif

#include <dsp/filtering_functions.h>
#include <dsp/support_functions.h>

#include <g722/g722_enc_dec.h>

#include "asha_audio.h"
#include "asha_audio_coefficients.h"


#define ASHA_BLOCK_SIZE 48

static q15_t fir_c[ASHA_NUM_TAPS] ={};
static q15_t p_state_l[ASHA_NUM_TAPS + ASHA_BLOCK_SIZE - 1];
static q15_t p_state_r[ASHA_NUM_TAPS + ASHA_BLOCK_SIZE - 1];

static arm_fir_decimate_instance_q15 fir_s_l = {};
static arm_fir_decimate_instance_q15 fir_s_r = {};

struct AshaAudioEncBuffer {
    uint8_t l[ASHA_SDU_SIZE_BYTES_ALIGNED];
    uint8_t r[ASHA_SDU_SIZE_BYTES_ALIGNED];
#ifdef PICO_ASHA_ENC_STATS
    int16_t encode_times[20];
#endif
};

static atomic_bool pcm_streaming;
static atomic_bool encode_audio;
static atomic_bool encode_mono;

static atomic_uint_fast32_t write_index;

static atomic_int_least16_t vol_m;
static atomic_int_least16_t vol_l;
static atomic_int_least16_t vol_r;

g722_encode_state_t enc_state_l;
g722_encode_state_t enc_state_r;

static struct AshaAudioEncBuffer enc_ring_buff[ASHA_G722_RING_SIZE];
static unsigned int g_offset;
static unsigned int enc_time_index;
static uint8_t seq_num;

static int16_t pcm_buff_l[ASHA_PCM_MAX_SAMPLES];
static int16_t pcm_buff_r[ASHA_PCM_MAX_SAMPLES];

static int16_t pcm_buff_16khz_l[ASHA_PCM_PACKET_SIZE];
static int16_t pcm_buff_16khz_r[ASHA_PCM_PACKET_SIZE];

static inline uint32_t ring_buff_index(const uint32_t index)
{
    return index & ASHA_G722_RING_SIZE_MASK;
}

static void reset_encoders()
{
    g722_encode_init(&enc_state_l, 64000, G722_PACKED);
    g722_encode_init(&enc_state_r, 64000, G722_PACKED);
}

static void reset_decimators()
{
    arm_fir_decimate_init_q15(&fir_s_l, ASHA_NUM_TAPS, 48000/16000, fir_c, p_state_l, ASHA_BLOCK_SIZE);
    arm_fir_decimate_init_q15(&fir_s_r, ASHA_NUM_TAPS, 48000/16000, fir_c, p_state_r, ASHA_BLOCK_SIZE);
}

void asha_audio_init()
{
    memset(enc_ring_buff, 0, sizeof(enc_ring_buff));
    pcm_streaming = false;
    encode_audio = false;
    encode_mono = false;
    write_index = 0u;
    vol_l = ASHA_USB_VOL_MIN;
    vol_r = ASHA_USB_VOL_MIN;
    g_offset = 1;
    seq_num = 0;
    enc_time_index = 0;
    reset_encoders();
    arm_float_to_q15(coefficients, fir_c, ASHA_NUM_TAPS);
    reset_decimators();

}

uint32_t asha_audio_get_write_index()
{
    uint32_t wi = write_index;
    return wi;
}

void asha_audio_encode_1ms_pcm(struct PCMStereoSample *samples, uint16_t count)
{
#ifdef PICO_ASHA_ENC_STATS
    absolute_time_t start_time = get_absolute_time();
#endif
    bool enc_audio = encode_audio;
    uint32_t w_index = write_index;
    if (!enc_audio) return;
    int buff_index = 0;
    struct AshaAudioEncBuffer* buff = &enc_ring_buff[ring_buff_index(w_index)];

    // Separate interleaved stereo samples to separate channels
    bool mono = encode_mono;
    if (mono) {
        int16_t val;
        for (unsigned int i = 0; i < count; ++i) {
            val = (int16_t)(((int32_t)samples[i].left + (int32_t)samples[i].right) / 2);
            pcm_buff_l[buff_index] = val;
            pcm_buff_r[buff_index] = val;
            ++buff_index;
        }       
    } else {
        for (unsigned int i = 0; i < count; ++i) {
            pcm_buff_l[buff_index] = samples[i].left;
            pcm_buff_r[buff_index] = samples[i].right;
            ++buff_index;
        }
    }
    int16_t* pcm_l = NULL;
    int16_t* pcm_r = NULL;
    if (count == ASHA_PCM_MAX_SAMPLES) {
        arm_fir_decimate_fast_q15(&fir_s_l, pcm_buff_l, pcm_buff_16khz_l, ASHA_BLOCK_SIZE);
        if (!mono) {
            arm_fir_decimate_fast_q15(&fir_s_r, pcm_buff_r, pcm_buff_16khz_r, ASHA_BLOCK_SIZE);
        }
        pcm_l = pcm_buff_16khz_l;
        pcm_r = pcm_buff_16khz_r;
    } else {
        pcm_l = pcm_buff_l;
        pcm_r = pcm_buff_r;
    }

    g722_encode(&enc_state_l, buff->l + g_offset, pcm_l, ASHA_PCM_PACKET_SIZE);
    if (mono) {
        memcpy(buff->r + g_offset, buff->l + g_offset, ASHA_G722_1MS_SIZE_BYTES);
    } else {
        g722_encode(&enc_state_r, buff->r + g_offset, pcm_r, ASHA_PCM_PACKET_SIZE);
    }
    
    g_offset += ASHA_G722_1MS_SIZE_BYTES;
#ifdef PICO_ASHA_ENC_STATS
    int64_t time_diff = absolute_time_diff_us(start_time, get_absolute_time());
    buff->encode_times[enc_time_index] = (int16_t)time_diff;
    ++enc_time_index;
#endif
    if (g_offset >= ASHA_SDU_SIZE_BYTES) {
        buff->l[0] = seq_num;
        buff->r[0] = seq_num;
        ++seq_num;
        g_offset = 1;
        write_index += 1;
        enc_time_index = 0;
    }
}

uint8_t* asha_audio_get_encoded_at_index(enum AshaAudioSide side, uint32_t index)
{
    struct AshaAudioEncBuffer* buff = &enc_ring_buff[ring_buff_index(index)];
    return side == AudioLeft ? buff->l : buff->r;
}

#ifdef PICO_ASHA_ENC_STATS
int16_t* asha_audio_get_encoding_time_at_index(uint32_t index)
{
    return (&enc_ring_buff[ring_buff_index(index)])->encode_times;
}
#endif

void asha_audio_set_curr_usb_vol(int16_t main_vol, int16_t left_vol, int16_t right_vol)
{
    vol_m = main_vol;
    vol_l = left_vol;
    vol_r = right_vol;
}

int16_t asha_audio_get_curr_usb_vol(enum AshaAudioSide side)
{
    int16_t vol;
    if (vol_l != vol_r) {
        // Volume levels are different for each side, user likely wants this
        vol = side == AudioLeft ? vol_l : vol_r;
    } else {
        // Choose the lowest volume. Some OS drivers seem to prefer setting the "main"
        // channel (observed on MacOS with UAC1). Some set all the same (Windows UAC1) 
        // and some set the left/right pair (Windows UAC2). Channels that are not changed 
        // are often set to the loudest level - we don't want that!
        vol = (vol_m < vol_l) ? vol_m : vol_l;
    }
    return vol;
}

void asha_audio_set_encoding_enabled(bool enabled)
{
    encode_audio = enabled;
}

bool asha_audio_get_encoding_enabled()
{
    bool enabled = encode_audio;
    return enabled;
}

void asha_audio_set_encode_mono(bool mono)
{
    encode_mono = mono;
}

bool asha_audio_get_encode_mono()
{
    bool mono = encode_mono;
    return mono;
}

void asha_audio_set_pcm_streaming_enabled(bool enabled)
{
    pcm_streaming = enabled;
}

bool asha_audio_get_pcm_streaming_enabled()
{
    bool pcm = pcm_streaming;
    return pcm;
}