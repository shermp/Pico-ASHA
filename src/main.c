#include <pico/stdio.h>
#include <pico/multicore.h>

#include "asha_audio.h"

void asha_main();
void usb_main(void);

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
    multicore_launch_core1(asha_main);
#endif
    sleep_ms(1000);
    usb_main();
    while(1) {
        sleep_ms(1000);
    }
    //multicore_launch_core1(usb_main);
}