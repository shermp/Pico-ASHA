#include <bitset>

#include "asha_led.hpp"
#include "pico/cyw43_arch.h"

namespace asha
{

LEDManager::LEDManager()
{
    led_worker.user_data = this;
    led_worker.do_work = &LEDManager::handle_led_work;
}

void LEDManager::set_led(LEDManager::State led_state)
{
    disable_curr_led_pattern();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, (led_state == State::On) ? true : false);
}

void LEDManager::set_led_pattern(LEDManager::Pattern const& pattern)
{
    if (pattern.len > 32 || !led_ctx) return;
    disable_curr_led_pattern();
    curr_pattern = pattern;
    async_context_add_at_time_worker_in_ms(led_ctx, &led_worker, 0);
}

void LEDManager::handle_led_work(async_context_t* ctx, async_at_time_worker_t *worker)
{
    LEDManager *this_ = (LEDManager*)worker->user_data;
    Pattern& p = this_->curr_pattern;
    std::bitset<32> pattern_bitset(p.pattern);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, pattern_bitset[p.pos]);
    ++p.pos;
    if (p.pos >= p.len) {
        p.pos = 0;
        async_context_add_at_time_worker_in_ms(ctx, worker, p.interval_ms + p.delay_ms);
    } else {
        async_context_add_at_time_worker_in_ms(ctx, worker, p.interval_ms);
    }
}

void LEDManager::disable_curr_led_pattern()
{
    if (led_ctx) {
        async_context_remove_at_time_worker(led_ctx, &led_worker);
    }
}

} // namespace asha