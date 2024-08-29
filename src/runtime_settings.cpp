#include "runtime_settings.hpp"

namespace asha
{

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

bool RuntimeSettings::set_uart_enabled(bool is_enabled)
{
    if (serial_uart_enabled != is_enabled) {
        serial_uart_enabled = is_enabled;
        return store_tlv_tag(Tag::SerialUARTEnabled, serial_uart_enabled);
    }
    return true;
}

bool RuntimeSettings::set_hci_dump_enabled(bool is_enabled)
{
    if (hci_dump_enabled != is_enabled) {
        hci_dump_enabled = is_enabled;
        return store_tlv_tag(Tag::HCIDump, hci_dump_enabled);
    }
    return true;
}

bool RuntimeSettings::set_log_level(enum LogLevel level)
{
    if (log_level != level) {
        log_level = level;
        return store_tlv_tag(Tag::LogLevel, log_level);
    }
    return true;
}

} // namespace asha