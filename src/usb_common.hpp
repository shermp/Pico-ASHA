#pragma once

#include <cstdint>

#include "asha_audio.h"

namespace asha
{
    struct USBSettings {
        uint16_t uac_version = 2U;
        int16_t min_vol = ASHA_USB_VOL_MIN;
        int16_t max_vol = ASHA_USB_VOL_MAX;

        explicit operator bool() const;
        bool operator==(const USBSettings&) const = default;
    };

    extern USBSettings usb_settings;
}