#include "core/relay_timing.h"

uint32_t relay_timing_on_ms(float power_percent, uint32_t window_ms)
{
    if (window_ms == 0) {
        return 0;
    }
    if (power_percent <= 0.0f) {
        return 0;
    }
    if (power_percent >= 100.0f) {
        return window_ms;
    }

    uint32_t on_ms = (uint32_t)((power_percent / 100.0f) * (float)window_ms + 0.5f);
    if (on_ms > window_ms) {
        on_ms = window_ms;
    }
    return on_ms;
}

int relay_timing_should_energize(uint32_t elapsed_ms, uint32_t on_ms)
{
    return elapsed_ms < on_ms ? 1 : 0;
}
