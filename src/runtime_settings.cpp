#include "runtime_settings.hpp"

#include <hardware/structs/watchdog.h>

#include "util.hpp"

namespace asha
{

// Scratch-register encoding for deferred TLV writes across a watchdog reboot.
// scratch[0] = command word; scratch[1] = packed data (both commands).
// HCI dump:     scratch[1] bit 0 = enabled flag.
// USB settings: scratch[1] bits  0-13 = (min_vol - ASHA_USB_VOL_MIN),
//               scratch[1] bits 14-27 = (max_vol - ASHA_USB_VOL_MIN),
//               scratch[1] bit  28    = (uac_version - 1).
// Scratch registers survive a watchdog reset but clear on power-on reset.
static constexpr uint32_t SCRATCH_CMD_HCI_DUMP     = 0x41534801u; // 'A','S','H',1
static constexpr uint32_t SCRATCH_CMD_USB_SETTINGS = 0x41534802u; // 'A','S','H',2

void RuntimeSettings::init()
{
    mutex_enter_blocking(&mtx);
    btstack_tlv_get_instance(&tlv_impl, &tlv_ctx);
    apply_pending_scratch();
    get_settings();
    mutex_exit(&mtx);
}

bool RuntimeSettings::get_hci_dump_enabled()
{
    mutex_enter_blocking(&mtx);
    bool enabled = hci_dump_enabled;
    mutex_exit(&mtx);
    return enabled;
}

bool RuntimeSettings::get_full_set_paired()
{
    mutex_enter_blocking(&mtx);
    bool paired = full_set_paired;
    mutex_exit(&mtx);
    return paired;
}

USBSettings RuntimeSettings::get_usb_settings()
{
    mutex_enter_blocking(&mtx);
    USBSettings settings = usb_settings;
    mutex_exit(&mtx);
    return settings;
}

RuntimeSettings::operator bool()
{
    mutex_enter_blocking(&mtx);
    bool ok = (tlv_impl && tlv_ctx && got_settings);
    mutex_exit(&mtx);
    return ok;
}

/* Get settings from flash. Settings not found in flash
   will be set to their default setting. Default settings
   are not written to flash to both conserve flash and
   to make updating default settings easier */
void RuntimeSettings::get_settings()
{
    if (!get_tlv_tag(Tag::HCIDump, hci_dump_enabled)) {
        hci_dump_enabled = false;
    }
    if (!get_tlv_tag(Tag::FullSetPaired, full_set_paired)) {
        full_set_paired = false;
    }
    if (!get_tlv_tag(Tag::USBSetting, usb_settings)) {
        if (!usb_settings) {
            usb_settings = USBSettings();
        }
    }
    got_settings = true;
}

bool RuntimeSettings::set_hci_dump_enabled(bool is_enabled)
{
    bool res = false;
    mutex_enter_blocking(&mtx);
    if (hci_dump_enabled != is_enabled) {
        hci_dump_enabled = is_enabled;
        res = store_tlv_tag(Tag::HCIDump, hci_dump_enabled);
    }
    mutex_exit(&mtx);
    return res;
}

bool RuntimeSettings::set_full_set_paired(bool have_full_set)
{
    bool res = false;
    mutex_enter_blocking(&mtx);
    if (full_set_paired != have_full_set) {
        full_set_paired = have_full_set;
        res = store_tlv_tag(Tag::FullSetPaired, have_full_set);
    }
    mutex_exit(&mtx);
    return res;
}

bool RuntimeSettings::set_usb_settings(USBSettings const &settings)
{
    bool res = false;
    mutex_enter_blocking(&mtx);
    if (settings && settings != usb_settings) {
        usb_settings = settings;
        res = store_tlv_tag(Tag::USBSetting, usb_settings);
    }
    mutex_exit(&mtx);
    return res;
}

void RuntimeSettings::defer_hci_dump(bool enabled)
{
    watchdog_hw->scratch[0] = SCRATCH_CMD_HCI_DUMP;
    watchdog_hw->scratch[1] = enabled ? 1u : 0u;
}

void RuntimeSettings::defer_usb_settings(USBSettings const& s)
{
    watchdog_hw->scratch[0] = SCRATCH_CMD_USB_SETTINGS;
    watchdog_hw->scratch[1] =
          (uint32_t)(s.min_vol - ASHA_USB_VOL_MIN)          // bits 0-13
        | ((uint32_t)(s.max_vol - ASHA_USB_VOL_MIN) << 14)  // bits 14-27
        | ((uint32_t)(s.uac_version - 1u) << 28);           // bit 28
}

// Called from init() after btstack_tlv_get_instance(), before get_settings().
// Writes any deferred setting to TLV so get_settings() picks it up normally.
void RuntimeSettings::apply_pending_scratch()
{
    switch (watchdog_hw->scratch[0]) {
    case SCRATCH_CMD_HCI_DUMP: {
        bool enabled = watchdog_hw->scratch[1] != 0u;
        store_tlv_tag(Tag::HCIDump, enabled);
        break;
    }
    case SCRATCH_CMD_USB_SETTINGS: {
        uint32_t p = watchdog_hw->scratch[1];
        USBSettings s;
        s.min_vol     = (int16_t)(p & 0x3FFFu) + ASHA_USB_VOL_MIN;
        s.max_vol     = (int16_t)((p >> 14) & 0x3FFFu) + ASHA_USB_VOL_MIN;
        s.uac_version = (uint16_t)((p >> 28) & 1u) + 1u;
        store_tlv_tag(Tag::USBSetting, s);
        break;
    }
    default:
        break;
    }
    watchdog_hw->scratch[0] = 0u;
    watchdog_hw->scratch[1] = 0u;
}

} // namespace asha