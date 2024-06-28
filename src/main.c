#include <stdio.h>
#include <pico/stdio.h>
#include <pico/multicore.h>
#include <pico/flash.h>
// TinyUSB
#include "bsp/board.h"
#include "tusb.h"

#include "pico_asha.h"

#include "asha_audio.h"

void asha_main();
void usb_main(void);

#ifdef ASHA_PERF_TEST
void perf_main(void);
#endif

struct asha_audio asha_shared;
#ifdef ASHA_USB_SERIAL
char pico_uid[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1 + 4];
#else
char pico_uid[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
#endif

int main()
{
    // Get serial
    pico_get_unique_board_id_string(pico_uid, sizeof pico_uid);
#ifdef ASHA_USB_SERIAL
    const char uid_suffix[4] = "-CDC";
    memcpy(pico_uid + (sizeof(pico_uid) - sizeof(uid_suffix) - 1), uid_suffix, sizeof(uid_suffix));
#endif
    // Init TinyUSB before stdio init
    board_init();
    // init device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);

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