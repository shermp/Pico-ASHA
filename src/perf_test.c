#ifdef ASHA_PERF_TEST

#include <stdio.h>
#include <pico/time.h>
#include <g722/g722_enc_dec.h>

#include "pico_asha.h"

void perf_main(void)
{
    int16_t tmp_pcm[1][ASHA_PCM_AUDIO_FRAME_SIZE] = {0};

    int64_t times[10] = {0};

    g722_encode_state_t left_enc_state = {};
    g722_encode_state_t right_enc_state = {};
    g722_encode_state_t mono_enc_state = {};

    uint8_t left_g722[160] = {0};
    uint8_t right_g722[160] = {0};
    uint8_t mono_g722[160] = {0};

    uint32_t wanted_index = 20;
    uint32_t curr_index = 0;

    while (1) {
        if (pcm_buff.write_index >= wanted_index) {
            g722_encode_init(&left_enc_state, 64000, G722_PACKED);
            g722_encode_init(&right_enc_state, 64000, G722_PACKED);
            g722_encode_init(&mono_enc_state, 64000, G722_PACKED);
            int k = 0;
            for (; curr_index < wanted_index; curr_index += 1) {
                absolute_time_t begin = get_absolute_time();
                asha_pcm_copy_audio(pcm_buff.left, curr_index, tmp_pcm, 1);
                g722_encode(&left_enc_state, left_g722, tmp_pcm, sizeof(tmp_pcm) / sizeof(int16_t));

                asha_pcm_copy_audio(pcm_buff.right, curr_index, tmp_pcm, 1);
                g722_encode(&right_enc_state, right_g722, tmp_pcm, sizeof(tmp_pcm) / sizeof(int16_t));

                // asha_pcm_copy_audio(pcm_buff.mono, curr_index, tmp_pcm, 1);
                // g722_encode(&mono_enc_state, mono_g722, tmp_pcm, sizeof(tmp_pcm) / sizeof(int16_t));
                
                absolute_time_t end = get_absolute_time();
                times[k] = absolute_time_diff_us(begin, end);
                ++k;
            }
            wanted_index += 200;
            printf("Volume is: %d. Encode times (us): %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld\n", 
                    pcm_buff.volume,
                    times[0], times[1], times[2], times[3], times[4],
                    times[5], times[6], times[7], times[8], times[9]);
        }
    }
}

#endif // ASHA_PERF_TEST