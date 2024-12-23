#pragma once

#include <cstdint>

#include <btstack.h>

#include "asha_led.hpp"
#include "runtime_settings.hpp"
#include "asha_logging.h"


#define PACKET_HANDLER_PARAMS [[maybe_unused]] uint8_t packet_type, [[maybe_unused]] uint16_t channel, uint8_t *packet, [[maybe_unused]] uint16_t size

namespace asha
{

extern LEDManager led_mgr;
extern LEDManager::Pattern none_connected;
extern LEDManager::Pattern one_connected;

} // namespace asha