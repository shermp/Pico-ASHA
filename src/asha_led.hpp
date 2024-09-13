#pragma once

#include <cstdint>

#include "pico/async_context.h"

namespace asha
{

class LEDManager
{
public:
    struct Pattern {
        // Number of bits in the sequence. Can be up to 32
        size_t len = 0;

        // Time between each bit in the sequence
        uint32_t interval_ms = 0;

        // Delay after last bit before restarting sequence
        uint32_t delay_ms = 0;

        // Sequence bit pattern. If bit is set, LED will be ON, otherwise
        // LED will be OFF
        uint32_t pattern = 0;

        // Current position in sequence
        size_t pos = 0;
    };

    enum class State { On, Off };

    LEDManager();

    void set_ctx(async_context_t* ctx) {led_ctx = ctx;}
    void set_led(State led_state);
    void set_led_pattern(Pattern const& pattern);
private:
    void disable_curr_led_pattern();
    static void handle_led_work(async_context_t* ctx, async_at_time_worker_t *worker);
    async_context_t *led_ctx = nullptr;
    async_at_time_worker_t led_worker = {.do_work = &LEDManager::handle_led_work};
    Pattern curr_pattern = {};
};

} // namespace asha