#include <stdio.h>
#include <pico/stdio.h>
#include <pico/multicore.h>
#include <pico/flash.h>
#include <pico/bootrom.h>
#include <pico/stdio_uart.h>
#include <hardware/resets.h>
#include <hardware/uart.h>
// TinyUSB
#include <bsp/board.h>
#include <tusb.h>

#include "asha_unique_id.hpp"
#include "asha_audio.hpp"
#include "asha_comms.hpp"

#include "runtime_settings.hpp"
#include "hearing_aid.hpp"
#include "usb_common.hpp"
#include "startup.hpp"

namespace asha
{
char pico_uid[pico_uid_size];

constinit RuntimeSettings runtime_settings;
constinit StartupResult bt_startup;

HearingAid ha_1 = HearingAid();
HearingAid ha_2 = HearingAid();

// extern "C" required by multicore_launch_core1
extern "C" void bt_main();

void usb_main();

[[noreturn]] static void startup_failed(StartupError error)
{
    // Stop a late core-1 completion before entering recovery. Never start USB
    // with default or unpublished settings after a failure or timeout.
    multicore_reset_core1();
    stdio_uart_init();
    // Use UART directly: USB is not running, and a stopped core could have
    // held a stdio lock. BOOTSEL provides recovery without a reboot loop.
    uart_puts(uart_default, "Pico-ASHA startup failed: ");
    switch (error) {
        case StartupError::CYW43:
            uart_puts(uart_default, "CYW43 initialization");
            break;
        case StartupError::RuntimeSettings:
            uart_puts(uart_default, "runtime settings initialization");
            break;
        default:
            uart_puts(uart_default, "Bluetooth startup timed out");
            break;
    }
    uart_puts(uart_default, "; entering BOOTSEL recovery\n");
    uart_tx_wait_blocking(uart_default);
    reset_usb_boot(0, 0);
}

static USBSettings wait_for_bt_startup()
{
    auto const deadline = make_timeout_time_ms(10000);
    USBSettings settings;
    StartupError error = StartupError::None;
    for (;;) {
        auto const status = bt_startup.read(settings, error);
        if (status == StartupStatus::Ready) return settings;
        if (status == StartupStatus::Failed) startup_failed(error);
        if (time_reached(deadline)) startup_failed(StartupError::Timeout);
        sleep_ms(1);
    }
}

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

    usb_settings = wait_for_bt_startup();

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
