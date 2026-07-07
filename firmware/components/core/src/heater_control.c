#include "core/heater_control.h"

#include <stddef.h>

static float clamp_power(const heater_control_config_t *config, float power)
{
    if (power < config->min_power_percent) {
        return config->min_power_percent;
    }
    if (power > config->max_power_percent) {
        return config->max_power_percent;
    }
    return power;
}

void heater_control_init(heater_control_t *ctrl, heater_control_mode_t mode,
                          const heater_control_config_t *config)
{
    if (ctrl == NULL || config == NULL) {
        return;
    }
    ctrl->mode = mode;
    ctrl->config = *config;
    ctrl->integral = 0.0f;
    ctrl->previous_error = 0.0f;
    ctrl->has_previous_error = 0;
    ctrl->bang_bang_on = 0;
}

void heater_control_reset(heater_control_t *ctrl)
{
    if (ctrl == NULL) {
        return;
    }
    ctrl->integral = 0.0f;
    ctrl->previous_error = 0.0f;
    ctrl->has_previous_error = 0;
    ctrl->bang_bang_on = 0;
}

static float heater_control_update_bang_bang(heater_control_t *ctrl, float measured_c, float target_c)
{
    float error = target_c - measured_c;
    float half_band = ctrl->config.hysteresis_c / 2.0f;

    if (error > half_band) {
        ctrl->bang_bang_on = 1;
    } else if (error < -half_band) {
        ctrl->bang_bang_on = 0;
    }
    /* Within the hysteresis band: keep previous state to avoid chatter. */

    float power = ctrl->bang_bang_on ? ctrl->config.max_power_percent : ctrl->config.min_power_percent;
    return clamp_power(&ctrl->config, power);
}

static float heater_control_update_pid(heater_control_t *ctrl, float measured_c, float target_c, float dt_s)
{
    if (dt_s <= 0.0f) {
        dt_s = 0.0f;
    }

    float error = target_c - measured_c;

    ctrl->integral += error * dt_s;

    float derivative = 0.0f;
    if (ctrl->has_previous_error && dt_s > 0.0f) {
        derivative = (error - ctrl->previous_error) / dt_s;
    }
    ctrl->previous_error = error;
    ctrl->has_previous_error = 1;

    float output = (ctrl->config.kp * error) + (ctrl->config.ki * ctrl->integral) +
                   (ctrl->config.kd * derivative);

    float clamped = clamp_power(&ctrl->config, output);

    /* Basic anti-windup: if we clamped, undo the integral contribution that
     * pushed us past the limit so it does not keep growing unbounded. */
    if (clamped != output && ctrl->config.ki != 0.0f) {
        ctrl->integral -= error * dt_s;
    }

    return clamped;
}

float heater_control_update(heater_control_t *ctrl, float measured_c, float target_c, float dt_s)
{
    if (ctrl == NULL) {
        return 0.0f;
    }

    if (ctrl->mode == HEATER_CONTROL_MODE_PID) {
        return heater_control_update_pid(ctrl, measured_c, target_c, dt_s);
    }
    return heater_control_update_bang_bang(ctrl, measured_c, target_c);
}
