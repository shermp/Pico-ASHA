#include <string.h>
#include "pico_asha.h"

void asha_pcm_audio_init(struct asha_pcm_audio *state)
{
    memset(state->left, 0, sizeof(state->left));
    memset(state->right, 0, sizeof(state->right));
    state->write_index = 0;
    state->volume = -128;
    state->streaming = false;
}

void asha_pcm_audio_write_audio(struct asha_pcm_audio *state, int16_t *stereo_audio)
{
    for (int i = 0; i < ASHA_PCM_AUDIO_STEREO_FRAME_SIZE; i += 2) {
        int frame_index = i / 2;
        uint32_t wi = state->write_index & ASHA_PCM_AUDIO_BUFF_SIZE_MASK;
        state->left[wi][frame_index] = stereo_audio[i];
        state->right[wi][frame_index] = stereo_audio[i + 1];
        int32_t left = stereo_audio[i];
        int32_t right = stereo_audio[i + 1];
        state->mono[wi][frame_index] = (int16_t) ((left >> 1) + (right >> 1));
    }
    state->write_index++;
}

void asha_pcm_copy_audio(int16_t src_buff[ASHA_PCM_AUDIO_BUFF_SIZE][ASHA_PCM_AUDIO_FRAME_SIZE], 
                         uint32_t index, 
                         int16_t *out, 
                         int count)
{
    uint32_t idx = index;
    for (int i = 0; i < count; ++i) {
        memcpy(out + (i * ASHA_PCM_AUDIO_FRAME_SIZE), src_buff[idx & ASHA_PCM_AUDIO_BUFF_SIZE_MASK], sizeof(*src_buff));
        idx++;
    }
}
