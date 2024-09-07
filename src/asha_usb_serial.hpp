#pragma once

#include "etl/string.h"
#include "pico/async_context.h"

namespace asha
{
    namespace SerCmd {
        constexpr const char AshaFWVers[] = "asha_fw_ver";
        constexpr const char Status[] = "status";
        constexpr const char ConnDevices[] = "conn_devices";
        constexpr const char Unpair[] = "unpair";
        constexpr const char ClearDevDb[] = "clear_dev_db";
        constexpr const char WaitUSBSerCx[] = "wait_usb_ser_cx";
        constexpr const char UartSerial[] = "uart_serial";
        constexpr const char LogLevel[] = "log_level";
        constexpr const char HCIDump[] = "hci_dump";
    };

    constexpr size_t stdin_str_size = 512u;

    extern etl::string<stdin_str_size> curr_stdin_buff;
    extern etl::string<stdin_str_size> complete_std_line;
    //extern etl::string<stdin_str_size> response_json;

    extern async_context_t *usb_ser_ctx;
    extern async_when_pending_worker_t stdin_pending_worker;

} //namespace asha