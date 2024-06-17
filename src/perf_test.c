#ifdef ASHA_PERF_TEST

#include <stdio.h>
#include <pico/time.h>
#include <g722/g722_enc_dec.h>

#include "asha_audio.h"

void perf_main(void)
{
    asha_shared.encode_audio = true;

    int64_t times[10] = {0};

    uint32_t curr_index = 0;

    absolute_time_t encode_ts[256];
    uint8_t encode_ts_index = 0;

    while (1) {
        if (curr_index < asha_shared.packet_index) {
                                           
            encode_ts[encode_ts_index] = get_absolute_time();
            ++encode_ts_index;
            ++curr_index;
            
            if (encode_ts_index == 0) {
                printf("Encode times (us):\n");
                for (int i = 1; i < 256; ++i) {
                    printf("%lld ", absolute_time_diff_us(encode_ts[i-1], encode_ts[i]));
                }
                printf("\n");
            }
        }
    }
}

#endif // ASHA_PERF_TEST