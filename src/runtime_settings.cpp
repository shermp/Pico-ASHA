#include "runtime_settings.hpp"

#include "util.hpp"

namespace asha
{

void RuntimeSettings::init()
{
    mutex_enter_blocking(&mtx);
    btstack_tlv_get_instance(&tlv_impl, &tlv_ctx);
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

} // namespace asha