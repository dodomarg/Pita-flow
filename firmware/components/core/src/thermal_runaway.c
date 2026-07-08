#include "core/thermal_runaway.h"

#include <stddef.h>

void thermal_runaway_reset(thermal_runaway_monitor_t *monitor)
{
    if (monitor == NULL) {
        return;
    }
    monitor->armed = 0;
    monitor->baseline_c = 0.0f;
    monitor->elapsed_ms = 0;
}

int thermal_runaway_update(thermal_runaway_monitor_t *monitor,
                           const thermal_runaway_config_t *config,
                           int heating,
                           float temperature_c,
                           uint32_t dt_ms)
{
    if (monitor == NULL || config == NULL) {
        return 0;
    }

    /* Not heating (heater off or already at setpoint), or a disabled window:
     * disarm so steady-state holding is never mistaken for a stall. */
    if (!heating || config->window_ms == 0) {
        monitor->armed = 0;
        monitor->elapsed_ms = 0;
        return 0;
    }

    /* First tick of a heating window: capture the baseline and start timing. */
    if (!monitor->armed) {
        monitor->armed = 1;
        monitor->baseline_c = temperature_c;
        monitor->elapsed_ms = 0;
        return 0;
    }

    /* Sufficient progress: rebaseline and restart the window. */
    if (temperature_c >= monitor->baseline_c + config->min_rise_c) {
        monitor->baseline_c = temperature_c;
        monitor->elapsed_ms = 0;
        return 0;
    }

    monitor->elapsed_ms += dt_ms;
    if (monitor->elapsed_ms >= config->window_ms) {
        return 1; /* Heating but temperature is not climbing -> runaway. */
    }

    return 0;
}
