#include <stdatomic.h>
#include <string.h>
#include <g722/g722_enc_dec.h>

#include "asha_audio.h"

struct AshaAudioEncBuffer {
    uint8_t l[ASHA_SDU_SIZE_BYTES_ALIGNED];
    uint8_t r[ASHA_SDU_SIZE_BYTES_ALIGNED];
};

static atomic_bool pcm_streaming;
static atomic_bool encode_audio;
static atomic_bool encode_mono;

static atomic_uint_fast32_t write_index;
static atomic_int_least16_t vol_l;
static atomic_int_least16_t vol_r;

g722_encode_state_t enc_state_l;
g722_encode_state_t enc_state_r;

static struct AshaAudioEncBuffer enc_ring_buff[ASHA_G722_RING_SIZE];
static unsigned int g_offset;
static uint8_t seq_num;

static int16_t pcm_buff_l[ASHA_PCM_PACKET_SIZE];
static int16_t pcm_buff_r[ASHA_PCM_PACKET_SIZE];

static inline uint32_t ring_buff_index(const uint32_t index)
{
    return index & ASHA_G722_RING_SIZE_MASK;
}

static void reset_encoders()
{
    g722_encode_init(&enc_state_l, 64000, G722_PACKED);
    g722_encode_init(&enc_state_r, 64000, G722_PACKED);
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
    reset_encoders();
}

uint32_t asha_audio_get_write_index()
{
    uint32_t wi = write_index;
    return wi;
}

void asha_audio_encode_1ms_pcm(int16_t *stereo_pcm)
{
    bool enc_audio = encode_audio;
    uint32_t w_index = write_index;
    if (!enc_audio) return;
    int buff_index = 0;
    struct AshaAudioEncBuffer* buff = &enc_ring_buff[ring_buff_index(w_index)];
    bool mono = encode_mono;
    if (mono) {
        int16_t val;
        for (unsigned int i = 0; i < ASHA_PCM_STEREO_PACKET_SIZE; i += 2) {
            val = (int16_t)(((int32_t)stereo_pcm[i] + (int32_t)stereo_pcm[i + 1]) / 2);
            pcm_buff_l[buff_index] = val;
            pcm_buff_r[buff_index] = val;
            ++buff_index;
        }
        //g722_encode(&enc_state_l, g_ring_buff[ring_buff_index(w_index)].l.data() + g_offset, pcm_buff.l.data(), pcm_buff.l.size());
        g722_encode(&enc_state_l, buff->l + g_offset, pcm_buff_l, ASHA_PCM_PACKET_SIZE);
        memcpy(buff->r + g_offset, buff->l + g_offset, ASHA_G722_1MS_SIZE_BYTES);
    } else {
        for (unsigned int i = 0; i < ASHA_PCM_STEREO_PACKET_SIZE; i += 2) {
            pcm_buff_l[buff_index] = stereo_pcm[i];
            pcm_buff_r[buff_index] = stereo_pcm[i + 1];
            ++buff_index;
        }
        g722_encode(&enc_state_l, buff->l + g_offset, pcm_buff_l, ASHA_PCM_PACKET_SIZE);
        g722_encode(&enc_state_r, buff->r + g_offset, pcm_buff_r, ASHA_PCM_PACKET_SIZE);
    }
    g_offset += ASHA_G722_1MS_SIZE_BYTES;
    if (g_offset >= ASHA_SDU_SIZE_BYTES) {
        buff->l[0] = seq_num;
        buff->r[0] = seq_num;
        ++seq_num;
        g_offset = 1;
        write_index += 1;
    }
}

uint8_t* asha_audio_get_encoded_at_index(enum AshaAudioSide side, uint32_t index)
{
    struct AshaAudioEncBuffer* buff = &enc_ring_buff[ring_buff_index(index)];
    return side == AudioLeft ? buff->l : buff->r;
}

void asha_audio_set_curr_usb_vol(enum AshaAudioSide side, int16_t usb_volume)
{
    if (side == AudioLeft) {
        vol_l = usb_volume;
    } else {
        vol_r = usb_volume;
    }
}

int16_t asha_audio_get_curr_usb_vol(enum AshaAudioSide side)
{
    int16_t vol = side == AudioLeft ? vol_l : vol_r;
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