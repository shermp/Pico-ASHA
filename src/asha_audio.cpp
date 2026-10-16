#include <atomic>
#include <string.h>

#ifdef PICO_ASHA_ENC_STATS
#include <pico/time.h>
#endif

#include <g722/g722_enc_dec.h>

#include "asha_audio.h"
#include "asha_fir.h"

static int16_t p_state_l[ASHA_FIR_STATE_SAMPLES];
static int16_t p_state_r[ASHA_FIR_STATE_SAMPLES];
static uint16_t pcm_sample_count;

struct AshaAudioEncBuffer {
    uint8_t l[ASHA_SDU_SIZE_BYTES_ALIGNED];
    uint8_t r[ASHA_SDU_SIZE_BYTES_ALIGNED];
#ifdef PICO_ASHA_ENC_STATS
    int16_t encode_times[20];
#endif
};

static std::atomic_bool pcm_streaming{false};
static std::atomic_bool encode_audio{false};
static std::atomic_bool encode_mono{false};
static std::atomic_bool requested_encode_mono{false};
// Core 1 requests/reads resets; core 0 publishes first_frame via applied_id.
static struct {
    std::atomic_uint_least32_t requested_id{0};
    std::atomic_uint_least32_t applied_id{0};
    uint32_t first_frame{0};
} encoder_resets[AudioResetRight + 1];

static std::atomic_uint_fast32_t write_index{0};

static std::atomic_int_least16_t vol_m{0};
static std::atomic_int_least16_t vol_l{0};
static std::atomic_int_least16_t vol_r{0};

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

static inline int16_t get_mono_sample(struct PCMStereoSample sample)
{
    return (int16_t)( ( (int32_t)sample.left + (int32_t)sample.right) / 2 );
}

static void reset_encoders()
{
    g722_encode_init(&enc_state_l, 64000, G722_PACKED);
    g722_encode_init(&enc_state_r, 64000, G722_PACKED);
}

static void reset_decimators()
{
    asha_fir_reset(p_state_l);
    asha_fir_reset(p_state_r);
}

static void reset_stream_encoder()
{
    reset_encoders();
    reset_decimators();
    g_offset = 1;
    seq_num = 0;
    enc_time_index = 0;
    encode_mono.store(requested_encode_mono.load());
}

void asha_audio_init()
{
    memset(enc_ring_buff, 0, sizeof(enc_ring_buff));
    pcm_streaming = false;
    encode_audio = false;
    encode_mono = false;
    requested_encode_mono = false;
    for (unsigned i = AudioResetSession; i <= AudioResetRight; ++i) {
        encoder_resets[i].requested_id = 0;
        encoder_resets[i].first_frame = 0;
        encoder_resets[i].applied_id = 0;
    }
    write_index = 0u;
    pcm_sample_count = 0;
    vol_l = ASHA_USB_VOL_MIN;
    vol_r = ASHA_USB_VOL_MIN;
    reset_stream_encoder();

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
    uint32_t const session_request_id = encoder_resets[AudioResetSession].requested_id;
    if (session_request_id != encoder_resets[AudioResetSession].applied_id) {
        // The hearing aid resets its decoder after an ACP stop/start. Reset on
        // this producer core so no partial SDU or stale predictor state leaks
        // into the next stream.
        reset_stream_encoder();
        encoder_resets[AudioResetSession].first_frame = write_index;
        encoder_resets[AudioResetSession].applied_id = session_request_id;
    }

    if (count != pcm_sample_count) {
        // A USB rate change invalidates FIR history, not the aid's G.722
        // predictor or the partially encoded 20 ms frame.
        reset_decimators();
        pcm_sample_count = count;
    }

    bool enc_audio = encode_audio;
    uint32_t w_index = write_index;
    if (!enc_audio) return;
    int buff_index = 0;
    struct AshaAudioEncBuffer* buff = &enc_ring_buff[ring_buff_index(w_index)];

    // Mode changes and per-side resets must not split an SDU.
    if (g_offset == 1) {
        encode_mono.store(requested_encode_mono.load());
        for (unsigned i = AudioResetLeft; i <= AudioResetRight; ++i) {
            uint32_t const request_id = encoder_resets[i].requested_id;
            if (request_id != encoder_resets[i].applied_id) {
                g722_encode_init(i == AudioResetLeft ? &enc_state_l : &enc_state_r, 64000, G722_PACKED);
                encoder_resets[i].first_frame = w_index;
                encoder_resets[i].applied_id = request_id;
            }
        }
    }

    // Separate interleaved stereo samples to separate channels
    bool mono = encode_mono;
    if (mono) {
        for (unsigned int i = 0; i < count; ++i) {
            int16_t sample = get_mono_sample(samples[i]);
            pcm_buff_l[buff_index] = sample;
            pcm_buff_r[buff_index] = sample;
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
        asha_fir_decimate_48_to_16_q15(pcm_buff_l, pcm_buff_16khz_l, p_state_l);
        asha_fir_decimate_48_to_16_q15(pcm_buff_r, pcm_buff_16khz_r, p_state_r);
        pcm_l = pcm_buff_16khz_l;
        pcm_r = pcm_buff_16khz_r;
    } else {
        pcm_l = pcm_buff_l;
        pcm_r = pcm_buff_r;
    }

    g722_encode(&enc_state_l, buff->l + g_offset, pcm_l, ASHA_PCM_PACKET_SIZE);
    g722_encode(&enc_state_r, buff->r + g_offset, pcm_r, ASHA_PCM_PACKET_SIZE);
    
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

uint32_t asha_audio_request_reset(enum AshaAudioReset reset)
{
    if ((unsigned)reset > AudioResetRight) return 0;
    return ++encoder_resets[reset].requested_id;
}

bool asha_audio_get_reset_frame(enum AshaAudioReset reset, uint32_t request_id, uint32_t* index)
{
    if ((unsigned)reset > AudioResetRight || request_id == 0
        || encoder_resets[reset].requested_id != request_id
        || encoder_resets[reset].applied_id != request_id) return false;
    *index = encoder_resets[reset].first_frame;
    // The write index publishes complete SDUs; an applied reset alone is not enough.
    return write_index > *index;
}

bool asha_audio_get_encoding_enabled()
{
    bool enabled = encode_audio;
    return enabled;
}

void asha_audio_set_encode_mono(bool mono)
{
    requested_encode_mono = mono;
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
