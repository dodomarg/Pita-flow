#ifndef CORE_THERMAL_RUNAWAY_H
#define CORE_THERMAL_RUNAWAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Open-loop / thermal-runaway guard configuration.
 *
 * While a heater is energized and the process is still expected to be climbing
 * toward its setpoint, the measured temperature must rise by at least
 * min_rise_c within window_ms. If it fails to (heater disconnected, SSR failed
 * open, thermocouple detached and reading a fixed value, etc.) the guard trips
 * and the caller must disable the heaters and enter a fault state.
 */
typedef struct {
    float min_rise_c;   /* Minimum temperature rise that counts as progress. */
    uint32_t window_ms; /* Time allowed to achieve min_rise_c while heating. */
} thermal_runaway_config_t;

typedef struct {
    int armed;            /* 1 once a heating window is being watched. */
    float baseline_c;     /* Temperature captured when the window (re)started. */
    uint32_t elapsed_ms;  /* Time accumulated in the current window. */
} thermal_runaway_monitor_t;

/** Clears the monitor state (disarms the window). */
void thermal_runaway_reset(thermal_runaway_monitor_t *monitor);

/**
 * Advances the monitor by dt_ms.
 *
 * `heating` must be 1 only when the heater is energized AND the temperature is
 * still expected to be climbing (i.e. meaningfully below setpoint); pass 0 when
 * the heater is off or the process is already at/near its setpoint, so steady
 * "hold" operation is not mistaken for a stall.
 *
 * Returns 1 when a thermal-runaway condition is detected (heating, but the
 * temperature failed to rise by config->min_rise_c within config->window_ms),
 * and 0 otherwise. Progress (a sufficient rise) rearms the window with a fresh
 * baseline so a slow-but-steady ramp does not trip the guard.
 */
int thermal_runaway_update(thermal_runaway_monitor_t *monitor,
                           const thermal_runaway_config_t *config,
                           int heating,
                           float temperature_c,
                           uint32_t dt_ms);

#ifdef __cplusplus
}
#endif

#endif /* CORE_THERMAL_RUNAWAY_H */
