#pragma once

#include "pico/unique_id.h"

namespace asha
{
#ifdef ASHA_USB_SERIAL
    constexpr size_t pico_uid_size = (2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1 + 4);
#else
    constexpr size_t pico_uid_size = (2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1);
#endif

extern char pico_uid[pico_uid_size];
} // namespace asha
