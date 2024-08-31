#pragma once

#include "pico/unique_id.h"

namespace asha
{

constexpr size_t pico_uid_size = (2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1 + 4);
extern char pico_uid[pico_uid_size];

} // namespace asha
