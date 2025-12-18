#pragma once

#include <cstdint>

#include <btstack_tlv.h>

namespace asha
{

constexpr uint32_t str_to_tag(const char tag[5])
{
    return ((((uint32_t) tag[0]) << 24 ) | (((uint32_t) tag[1]) << 16) | (((uint32_t) tag[2]) << 8) | tag[3]);
}

struct RuntimeSettings
{
    enum Tag : uint32_t {
        HCIDump = str_to_tag("PAHC"),
        FullSetPaired = str_to_tag("PAFS"),
    };
    bool hci_dump_enabled = false;
    bool full_set_paired = false;


    void init();

    void get_settings();

    bool set_hci_dump_enabled(bool is_enabled);
    bool set_full_set_paired(bool have_full_set);

    explicit operator bool() const {return tlv_impl && tlv_ctx; }

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