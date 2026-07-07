#ifndef CORE_HEATER_CONTROL_H
#define CORE_HEATER_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HEATER_CONTROL_MODE_BANG_BANG = 0,
    HEATER_CONTROL_MODE_PID = 1,
} heater_control_mode_t;

typedef struct {
    /* Bang-bang hysteresis band, in degrees C, applied around target. */
    float hysteresis_c;

    /* PID constants, used only when mode == HEATER_CONTROL_MODE_PID. */
    float kp;
    float ki;
    float kd;

    /* Output power limits, percent. */
    float min_power_percent;
    float max_power_percent;
} heater_control_config_t;

typedef struct {
    heater_control_mode_t mode;
    heater_control_config_t config;

    /* PID internal state. */
    float integral;
    float previous_error;
    int has_previous_error;

    /* Bang-bang internal state: 1 if currently driving full power. */
    int bang_bang_on;
} heater_control_t;

/* Initializes controller state. config is copied by value. */
void heater_control_init(heater_control_t *ctrl, heater_control_mode_t mode,
                          const heater_control_config_t *config);

/**
 * Computes the next demanded power percentage (clamped to
 * [min_power_percent, max_power_percent]) given the current measured
 * temperature, target temperature, and elapsed time since the previous
 * update in seconds (dt_s must be > 0 for PID mode; ignored in bang-bang
 * mode).
 */
float heater_control_update(heater_control_t *ctrl, float measured_c, float target_c, float dt_s);

/** Resets integral/derivative state, e.g. when re-entering a control phase. */
void heater_control_reset(heater_control_t *ctrl);

#ifdef __cplusplus
}
#endif

#endif /* CORE_HEATER_CONTROL_H */
