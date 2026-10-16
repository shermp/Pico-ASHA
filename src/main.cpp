#include <stdio.h>
#include <pico/stdio.h>
#include <pico/multicore.h>
#include <pico/flash.h>
#include <hardware/resets.h>
// TinyUSB
#include <bsp/board.h>
#include <tusb.h>

#include "asha_unique_id.hpp"
#include "asha_audio.hpp"
#include "asha_comms.hpp"

#include "runtime_settings.hpp"
#include "hearing_aid.hpp"
#include "usb_common.hpp"

namespace asha
{
char pico_uid[pico_uid_size];

RuntimeSettings runtime_settings = {};

HearingAid ha_1 = HearingAid();
HearingAid ha_2 = HearingAid();

// extern "C" required by multicore_launch_core1
extern "C" void bt_main();

void usb_main();

extern "C" int main()
{
    // OpenOCD's soft reset can leave core 1 outside its boot-ROM launch
    // state. Reset it before touching the audio state shared by both cores.
    multicore_reset_core1();

    // A soft reset can also leave the previous USB connection active.
    // Drop the pull-up now so the host sees a disconnect throughout the
    // startup delay, before TinyUSB initializes and reconnects the device.
    reset_block(RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);
    sleep_ms(250);

    // Apparently there be dragons when using flash and
    // multicore. And guess where BTStack saves pairing
    // info? Calling this function on the current core
    // (core 0) should make it work.
    flash_safe_execute_core_init();
    // Init global shared variables
    asha_audio_init();
    comm::init();

    // Get serial
    pico_get_unique_board_id_string(pico_uid, sizeof pico_uid);

    const char uid_suffix[4] = {'-', 'C', 'D', 'C'};
    memcpy(pico_uid + (sizeof(pico_uid) - sizeof(uid_suffix) - 1), uid_suffix, sizeof(uid_suffix));

    sleep_ms(250);
    multicore_launch_core1(bt_main);

    while (!runtime_settings) {
        sleep_ms(50);
    }

    usb_settings = runtime_settings.get_usb_settings();

    // Init TinyUSB before stdio init
    board_init();
    // init device stack on configured roothub port
    const tusb_rhport_init_t usb_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,
    };
    tusb_init(BOARD_TUD_RHPORT, &usb_init);

    stdio_init_all();
    sleep_ms(250);
    usb_main();
    while(1) {
        sleep_ms(1000);
    }
}

} // namespace asha
