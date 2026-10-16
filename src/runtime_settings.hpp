#pragma once

#include <cstdint>

#include <btstack_tlv.h>

#include "asha_audio.hpp"
#include "usb_common.hpp"

namespace asha
{

constexpr uint32_t str_to_tag(const char tag[5])
{
    return ((((uint32_t) tag[0]) << 24 ) | (((uint32_t) tag[1]) << 16) | (((uint32_t) tag[2]) << 8) | tag[3]);
}

// Access only on core 1: initialize before powering on Bluetooth, then use
// the serialized Bluetooth callbacks. Core 0 receives a startup snapshot.
struct RuntimeSettings
{
    [[nodiscard]] bool init();

    bool get_hci_dump_enabled() const;
    bool get_full_set_paired() const;
    USBSettings get_usb_settings() const;

    bool set_full_set_paired(bool have_full_set);

    // Store a pending setting in watchdog scratch registers so it can be
    // written to TLV after the watchdog reboot, avoiding a race between the
    // watchdog countdown and BTstack's own TLV writes.  Call immediately
    // before watchdog_enable().
    static void defer_hci_dump(bool enabled);
    static void defer_usb_settings(USBSettings const& settings);

private:
    enum Tag : uint32_t {
        HCIDump = str_to_tag("PAHC"),
        FullSetPaired = str_to_tag("PAFS"),
        USBSetting = str_to_tag("PAUS"),
    };

    // used to store remote device in TLV
    const btstack_tlv_t * tlv_impl = nullptr;
    void *                tlv_ctx = nullptr;

    bool hci_dump_enabled = false;
    bool full_set_paired = false;
    USBSettings usb_settings = {};

    void get_settings();
    void apply_pending_scratch();

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