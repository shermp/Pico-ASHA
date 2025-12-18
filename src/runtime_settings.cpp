#include "runtime_settings.hpp"

#include "util.hpp"

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
    if (!get_tlv_tag(Tag::HCIDump, hci_dump_enabled)) {
        hci_dump_enabled = false;
    }
    if (!get_tlv_tag(Tag::FullSetPaired, full_set_paired)) {
        full_set_paired = false;
    }
}

bool RuntimeSettings::set_hci_dump_enabled(bool is_enabled)
{
    if (hci_dump_enabled != is_enabled) {
        hci_dump_enabled = is_enabled;
        return store_tlv_tag(Tag::HCIDump, hci_dump_enabled);
    }
    return false;
}

bool RuntimeSettings::set_full_set_paired(bool have_full_set)
{
    if (full_set_paired != have_full_set) {
        full_set_paired = have_full_set;
        return store_tlv_tag(Tag::FullSetPaired, have_full_set);
    }
    return false;
}

} // namespace asha