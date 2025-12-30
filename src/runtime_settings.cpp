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

uint16_t RuntimeSettings::get_uac_version()
{
    mutex_enter_blocking(&mtx);
    uint16_t vers = uac_version;
    mutex_exit(&mtx);
    return vers;
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
    if (!get_tlv_tag(Tag::UACVersion, uac_version)) {
        if (uac_version < 1 || uac_version > 2) {
            uac_version = 2U;
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

bool RuntimeSettings::set_uac_version(uint16_t version)
{
    bool res = false;
    mutex_enter_blocking(&mtx);
    if (uac_version != version && (version == 1 || version == 2)) {
        uac_version = version;
        res = store_tlv_tag(Tag::UACVersion, uac_version);
    }
    mutex_exit(&mtx);
    return res;
}

} // namespace asha