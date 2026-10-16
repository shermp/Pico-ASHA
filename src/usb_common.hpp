#pragma once

#include <cstdint>

#include "asha_audio.hpp"

namespace asha
{
    struct USBSettings {
        uint16_t uac_version = 2U;
        int16_t min_vol = usb_vol_min;
        int16_t max_vol = usb_vol_max;

        explicit operator bool() const;
        bool operator==(const USBSettings&) const = default;
    };

    extern USBSettings usb_settings;
}