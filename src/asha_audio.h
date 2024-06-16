#ifndef ASHA_AUDIO_H
#define ASHA_AUDIO_H

#include <stdint.h>
#include <stdbool.h>

#include <g722/g722_enc_dec.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ASHA_G722_RING_BUFF_SIZE 16U
#define ASHA_G722_RING_BUFF_SIZE_MASK (ASHA_G722_RING_BUFF_SIZE - 1U)

#define ASHA_PCM_PACKET_SIZE 16 // 16 samples per ms
#define ASHA_PCM_STEREO_PACKET_SIZE (ASHA_PCM_PACKET_SIZE * 2)
#define ASHA_PCM_PACKET_SIZE_BYTES (ASHA_PCM_PACKET_SIZE * 2)
#define ASHA_PCM_20MS_SIZE_BYTES (ASHA_PCM_PACKET_SIZE_BYTES * 20)
#define ASHA_G722_1MS_SIZE_BYTES (ASHA_PCM_PACKET_SIZE_BYTES / 4)
#define ASHA_G722_20MS_SIZE_BYTES (ASHA_G722_1MS_SIZE_BYTES * 20)
#define ASHA_SDU_SIZE_BYTES (ASHA_G722_20MS_SIZE_BYTES + 1)

#define ASHA_RING_BUFF_INDEX(x) ((x) & ASHA_G722_RING_BUFF_SIZE_MASK)

struct asha_audio {
    volatile int8_t volume;
    volatile bool pcm_streaming;
    volatile bool encode_audio;

    int write_offset;
    volatile uint32_t packet_index;
    uint8_t seq_num;

    uint8_t g_l_buff[ASHA_G722_RING_BUFF_SIZE][ASHA_SDU_SIZE_BYTES];
    uint8_t g_r_buff[ASHA_G722_RING_BUFF_SIZE][ASHA_SDU_SIZE_BYTES];
    uint8_t g_m_buff[ASHA_G722_RING_BUFF_SIZE][ASHA_SDU_SIZE_BYTES];

    g722_encode_state_t g_l_state;
    g722_encode_state_t g_r_state;
    g722_encode_state_t g_m_state;

    int16_t tmp_l_pcm[ASHA_PCM_PACKET_SIZE];
    int16_t tmp_r_pcm[ASHA_PCM_PACKET_SIZE];
    int16_t tmp_m_pcm[ASHA_PCM_PACKET_SIZE];
};

void asha_audio_init(struct asha_audio *audio);

void asha_audio_g_l_reset(struct asha_audio *audio);
void asha_audio_g_r_reset(struct asha_audio *audio);
void asha_audio_g_m_reset(struct asha_audio *audio);
void asha_audio_g_reset(struct asha_audio *audio);

void asha_audio_g_enc_1ms(struct asha_audio *audio, int16_t stereo_pcm[ASHA_PCM_STEREO_PACKET_SIZE]);

//uint32_t asha_audio_latest_index(struct asha_audio *audio);
//uint8_t* asha_audio_g_get_20ms(struct asha_audio *audio, uint32_t index);

extern struct asha_audio asha_shared;

#ifdef __cplusplus
}
#endif

#endif // ASHA_AUDIO_H