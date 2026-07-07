#include "core/safety.h"

#include <stddef.h>

uint32_t safety_evaluate(const safety_inputs_t *inputs, const safety_limits_t *limits)
{
    uint32_t faults = SAFETY_FAULT_NONE;

    if (inputs == NULL || limits == NULL) {
        /* Missing data is itself unsafe: treat as a watchdog-style fault. */
        return SAFETY_FAULT_CONTROL_WATCHDOG_TIMEOUT;
    }

    if (inputs->thermocouple_fault_bit) {
        faults |= SAFETY_FAULT_THERMOCOUPLE_FAULT_BIT;
    }
    if (inputs->thermocouple_open_or_short) {
        faults |= SAFETY_FAULT_THERMOCOUPLE_OPEN_SHORT;
    }
    if (inputs->plate_adc_implausible) {
        faults |= SAFETY_FAULT_PLATE_ADC_IMPLAUSIBLE;
    }
    if (inputs->chamber_temperature_c > limits->chamber_absolute_max_c) {
        faults |= SAFETY_FAULT_CHAMBER_OVER_TEMP;
    }
    if (inputs->plate_temperature_c > limits->plate_absolute_max_c) {
        faults |= SAFETY_FAULT_PLATE_OVER_TEMP;
    }
    if (inputs->ms_since_last_control_tick > limits->control_watchdog_timeout_ms) {
        faults |= SAFETY_FAULT_CONTROL_WATCHDOG_TIMEOUT;
    }
    if (inputs->system_is_idle_or_fault && inputs->relay_commanded_on) {
        faults |= SAFETY_FAULT_RELAY_COMMANDED_WHILE_IDLE;
    }
    if (!inputs->active_profile_valid) {
        faults |= SAFETY_FAULT_INVALID_PROFILE;
    }

    return faults;
}
