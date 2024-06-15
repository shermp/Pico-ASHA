#ifndef PICO_ASHA_H
#define PICO_ASHA_H

#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ASHA_PCM_AUDIO_BUFF_SIZE 64U
#define ASHA_PCM_AUDIO_BUFF_SIZE_MASK (ASHA_PCM_AUDIO_BUFF_SIZE - 1U)
#define ASHA_PCM_AUDIO_FRAME_SIZE 16
#define ASHA_PCM_AUDIO_STEREO_FRAME_SIZE 32

struct asha_pcm_audio {
    int16_t left [ASHA_PCM_AUDIO_BUFF_SIZE][ASHA_PCM_AUDIO_FRAME_SIZE];
    int16_t right[ASHA_PCM_AUDIO_BUFF_SIZE][ASHA_PCM_AUDIO_FRAME_SIZE];
    int16_t mono [ASHA_PCM_AUDIO_BUFF_SIZE][ASHA_PCM_AUDIO_FRAME_SIZE];
    // This will overflow... after approx 49 days
    volatile uint32_t write_index;
    volatile int8_t  volume;
    volatile bool    streaming;
};

void asha_pcm_audio_init(struct asha_pcm_audio *state);

void asha_pcm_audio_write_audio(struct asha_pcm_audio *state, int16_t *stereo_audio);

void asha_pcm_copy_audio(int16_t src_buff[ASHA_PCM_AUDIO_BUFF_SIZE][ASHA_PCM_AUDIO_FRAME_SIZE], 
                         uint32_t index, 
                         int16_t *out, 
                         int count);

extern struct asha_pcm_audio pcm_buff;

#ifdef __cplusplus
}
#endif

#endif // PICO_ASHA_H