#include <stdio.h>
#include "pico/stdio.h"
#include "pico/multicore.h"
#include "pico/flash.h"
// TinyUSB
#include "bsp/board.h"
#include "tusb.h"

#include "asha_unique_id.hpp"
#include "asha_audio.hpp"

#ifdef ASHA_PERF_METRICS
#include "perf_metrics.hpp"
#endif

namespace asha
{
#ifdef ASHA_PERF_METRICS
#include "perf_metrics.hpp"
PerfMetrics perf_metrics;
#endif

AudioBuffer audio_buff;

char pico_uid[pico_uid_size];

// extern "C" required by multicore_launch_core1
extern "C" void bt_main();

void usb_main();

extern "C" int main()
{
    // Init global shared variables
    patom::PseudoAtomicInit();
    audio_buff.init();
    // Get serial
    pico_get_unique_board_id_string(pico_uid, sizeof pico_uid);
#ifdef ASHA_USB_SERIAL
    const char uid_suffix[4] = {'-', 'C', 'D', 'C'};
    memcpy(pico_uid + (sizeof(pico_uid) - sizeof(uid_suffix) - 1), uid_suffix, sizeof(uid_suffix));
#endif
    // Init TinyUSB before stdio init
    board_init();
    // init device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);

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
    multicore_launch_core1(bt_main);

    sleep_ms(1000);
    usb_main();
    while(1) {
        sleep_ms(1000);
    }
}

} // namespace asha