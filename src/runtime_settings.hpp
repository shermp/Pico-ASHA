#pragma once

#include <cstdint>

#include "btstack_tlv.h"

namespace asha
{

constexpr uint32_t str_to_tag(const char tag[5])
{
    return ((((uint32_t) tag[0]) << 24 ) | (((uint32_t) tag[1]) << 16) | (((uint32_t) tag[2]) << 8) | tag[3]);
}

enum LogLevel : uint32_t {
    Error = 0,
    Info = 1,
    Scan = 2,
    Audio = 3
};

constexpr const char* log_level_to_str(enum LogLevel log_level)
{
    using asha::LogLevel;
    switch (log_level)
    {
    case Error:
        return "ERROR";
    case Info:
        return "INFO";
    case Scan:
        return "SCAN";
    case Audio:
        return "AUDIO";
    default:
        return "NONE";
    }
}

struct RuntimeSettings
{
    enum Tag : uint32_t {
        SerialUARTEnabled = str_to_tag("PAUA"),
        LogLevel = str_to_tag("PALL"),
        HCIDump = str_to_tag("PAHC"),
    };
    bool serial_uart_enabled = false;
    bool hci_dump_enabled = false;
    enum LogLevel log_level = LogLevel::Info;

    void init();

    void get_settings();

    bool set_uart_enabled(bool is_enabled);
    bool set_hci_dump_enabled(bool is_enabled);
    bool set_log_level(enum LogLevel level);

private:
    // used to store remote device in TLV
    const btstack_tlv_t * tlv_impl = nullptr;
    void *                tlv_ctx = nullptr;

    template <typename T>
    bool get_tlv_tag(enum Tag tag, T& var) 
    {
        if (!tlv_impl) return false;
        int len = tlv_impl->get_tag(tlv_ctx, tag, (uint8_t*)&var, sizeof(var));
        return (len == sizeof(var));
    }

    template <typename T>
    bool store_tlv_tag(enum Tag tag, const T& var)
    {
        if (!tlv_impl) return false;
        return tlv_impl->store_tag(tlv_ctx, tag, (const uint8_t*)&var, sizeof(var)) == 0;
    }
};

extern RuntimeSettings runtime_settings;

} // namespace asha