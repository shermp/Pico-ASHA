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

queue_t asha_g_queue;
queue_t asha_command_queue;
queue_t asha_usb_status_queue;

int main()
{
    stdio_init_all();
    sleep_ms(2000);

    // Setup the audio, command and status queues between
    // the Bluetooth core and USB core
    queue_init(&asha_g_queue, sizeof(asha_g722_sdu_t), ASHA_G722_QUEUE_SIZE);
    queue_init(&asha_command_queue, sizeof(enum ASHAEncodeCmd), ASHA_COMMAND_QUEUE_SIZE);
    queue_init(&asha_usb_status_queue, sizeof(enum ASHAUsbStatus), ASHA_USB_STATUS_QUEUE_SIZE);

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