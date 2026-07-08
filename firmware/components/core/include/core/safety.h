#ifndef CORE_SAFETY_H
#define CORE_SAFETY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SAFETY_FAULT_NONE = 0,
    SAFETY_FAULT_THERMOCOUPLE_FAULT_BIT = 1 << 0,
    SAFETY_FAULT_THERMOCOUPLE_OPEN_SHORT = 1 << 1,
    SAFETY_FAULT_PLATE_ADC_IMPLAUSIBLE = 1 << 2,
    SAFETY_FAULT_CHAMBER_OVER_TEMP = 1 << 3,
    SAFETY_FAULT_PLATE_OVER_TEMP = 1 << 4,
    SAFETY_FAULT_CONTROL_WATCHDOG_TIMEOUT = 1 << 5,
    SAFETY_FAULT_RELAY_COMMANDED_WHILE_IDLE = 1 << 6,
    SAFETY_FAULT_INVALID_PROFILE = 1 << 7,
    SAFETY_FAULT_THERMAL_RUNAWAY = 1 << 8,
} safety_fault_flags_t;

typedef struct {
    float chamber_absolute_max_c;
    float plate_absolute_max_c;
    uint32_t control_watchdog_timeout_ms;
} safety_limits_t;

typedef struct {
    /* Sensor status inputs. */
    int thermocouple_fault_bit;
    int thermocouple_open_or_short;
    int plate_adc_implausible;

    /* Measured process values. */
    float chamber_temperature_c;
    float plate_temperature_c;

    /* Task health. */
    uint32_t ms_since_last_control_tick;

    /* Commanded state consistency checks. */
    int system_is_idle_or_fault;
    int relay_commanded_on;

    /* Profile validity. */
    int active_profile_valid;

    /* Open-loop / thermal-runaway guard result (heater energized but the
     * temperature is not climbing as expected). Computed statefully by the
     * caller via the thermal_runaway module. */
    int thermal_runaway;
} safety_inputs_t;

/**
 * Evaluates all configured safety checks against the given inputs and
 * limits, returning a bitmask of safety_fault_flags_t values (SAFETY_FAULT_NONE
 * if no fault is present). This function is pure and has no side effects;
 * the caller is responsible for acting on a non-zero result by disabling
 * both heater outputs immediately.
 */
uint32_t safety_evaluate(const safety_inputs_t *inputs, const safety_limits_t *limits);

#ifdef __cplusplus
}
#endif

#endif /* CORE_SAFETY_H */
