#include <pico/multicore.h>

#include "pico_asha.h"

void asha_main();
void usb_main(void);

#ifdef ASHA_PERF_TEST
void perf_main(void);
#endif

struct asha_pcm_audio pcm_buff = {};

int main()
{
    asha_pcm_audio_init(&pcm_buff);
    stdio_init_all();
    sleep_ms(2000);
#ifdef ASHA_PERF_TEST
    multicore_launch_core1(perf_main);
#else
    multicore_launch_core1(asha_main);
#endif
    sleep_ms(1000);
    usb_main();
    while(1) {
        sleep_ms(1000);
    }
    //multicore_launch_core1(usb_main);
}