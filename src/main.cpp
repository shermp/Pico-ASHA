#include <stdio.h>
#include <pico/stdio.h>
#include <pico/multicore.h>
#include <pico/flash.h>
// TinyUSB
#include <bsp/board.h>
#include <tusb.h>

#include "asha_unique_id.hpp"
#include "asha_audio.hpp"
#include "asha_usb_serial.hpp"
#include "runtime_settings.hpp"
#include "asha_logging.h"
#include "hearing_aid.hpp"

namespace asha
{

AudioBuffer audio_buff;
async_context_t *bt_async_ctx = nullptr;
async_when_pending_worker_t bt_audio_pending_worker = {};

etl::string<stdin_str_size> curr_stdin_buff = {};
etl::string<stdin_str_size> complete_std_line = {};

async_context_t *usb_ser_ctx = nullptr;
async_when_pending_worker_t stdin_pending_worker = {};

char pico_uid[pico_uid_size];

RuntimeSettings runtime_settings = {};

etl::circular_buffer<etl::string<log_line_len>, log_lines> log_buffer = {};
async_context_t *logging_ctx = nullptr;
async_when_pending_worker_t logging_pending_worker = {};

HearingAid ha_1 = {};
HearingAid ha_2 = {};

// extern "C" required by multicore_launch_core1
extern "C" void bt_main();

void usb_main();

extern "C" int main()
{
    // Apparently there be dragons when using flash and
    // multicore. And guess where BTStack saves pairing
    // info? Calling this function on the current core
    // (core 0) should make it work.
    flash_safe_execute_core_init();
    // Init global shared variables
    patom::PseudoAtomicInit();
    audio_buff.init();

    // Get serial
    pico_get_unique_board_id_string(pico_uid, sizeof pico_uid);

    const char uid_suffix[4] = {'-', 'C', 'D', 'C'};
    memcpy(pico_uid + (sizeof(pico_uid) - sizeof(uid_suffix) - 1), uid_suffix, sizeof(uid_suffix));

    // Init TinyUSB before stdio init
    board_init();
    // init device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);

    stdio_init_all();
    sleep_ms(250);
    
    multicore_launch_core1(bt_main);

    sleep_ms(250);
    usb_main();
    while(1) {
        sleep_ms(1000);
    }
}

} // namespace asha