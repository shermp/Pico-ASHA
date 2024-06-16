#include <pico/stdio.h>
#include <pico/multicore.h>

#include "asha_audio.h"

void asha_main();

#ifdef ASHA_PERF_TEST
void perf_main(void);
#endif

struct asha_audio asha_shared;

int main()
{
    asha_audio_init(&asha_shared);
    stdio_init_all();
    sleep_ms(2000);
#ifdef ASHA_PERF_TEST
    multicore_launch_core1(perf_main);
#else
    multicore_launch_core1(asha_audio_encode_loop);
#endif
    sleep_ms(50);
    asha_main();
}