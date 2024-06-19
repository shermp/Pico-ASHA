#include <stdio.h>
#include <pico/stdio.h>
#include <pico/multicore.h>
#include <pico/flash.h>

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
    // Apparently there be dragons when using flash and
    // multicore. And guess where BTStack saves pairing
    // info? Calling this function on the current core
    // (core 0) should make it work.
    if (!flash_safe_execute_core_init()) {
        printf("flash_safe_execute_core_init failed."
               "Pairing data will not be saved.\n");
    }
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