#include "runtime_settings.hpp"

#include "util.hpp"

namespace asha
{

enum LogLevel str_to_log_level(const char* log_level)
{
    if (str_eq(log_level, "ERROR")) {
        return LogLevel::Error;
    } else if (str_eq(log_level, "INFO")) {
        return LogLevel::Info;
    } else if (str_eq(log_level, "SCAN")) {
        return LogLevel::Scan;
    } else if (str_eq(log_level, "AUDIO")) {
        return LogLevel::Audio;
    } else {
        return LogLevel::None;
    }
}

void RuntimeSettings::init()
{
    btstack_tlv_get_instance(&tlv_impl, &tlv_ctx);
}

/* Get settings from flash. Settings not found in flash
   will be set to their default setting. Default settings
   are not written to flash to both conserve flash and
   to make updating default settings easier */
void RuntimeSettings::get_settings()
{
    if (!get_tlv_tag(Tag::WaitForUSBSerial, wait_for_usb_serial_cx)) {
        wait_for_usb_serial_cx = true;
    }
    if (!get_tlv_tag(Tag::SerialUARTEnabled, serial_uart_enabled)) {
        serial_uart_enabled = false;
    }
    if (!get_tlv_tag(Tag::HCIDump, hci_dump_enabled)) {
        hci_dump_enabled = false;
    }
    if (!get_tlv_tag(Tag::LogLevel, log_level)) {
        log_level = LogLevel::Info;
    }
}

bool RuntimeSettings::set_wait_for_usb_serial_cx(bool wait)
{
    if (wait_for_usb_serial_cx != wait) {
        wait_for_usb_serial_cx = wait;
        return store_tlv_tag(Tag::WaitForUSBSerial, wait_for_usb_serial_cx);
    }
    return false;
}

bool RuntimeSettings::set_uart_enabled(bool is_enabled)
{
    if (serial_uart_enabled != is_enabled) {
        serial_uart_enabled = is_enabled;
        return store_tlv_tag(Tag::SerialUARTEnabled, serial_uart_enabled);
    }
    return false;
}

bool RuntimeSettings::set_hci_dump_enabled(bool is_enabled)
{
    if (hci_dump_enabled != is_enabled) {
        hci_dump_enabled = is_enabled;
        return store_tlv_tag(Tag::HCIDump, hci_dump_enabled);
    }
    return false;
}

bool RuntimeSettings::set_log_level(enum LogLevel level)
{
    if (log_level != level) {
        log_level = level;
        return store_tlv_tag(Tag::LogLevel, log_level);
    }
    return false;
}

} // namespace asha