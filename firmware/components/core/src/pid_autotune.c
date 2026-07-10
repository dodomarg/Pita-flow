#include "core/pid_autotune.h"

#include <stddef.h>

#define AUTOTUNE_PI 3.14159265358979323846f

/* Minimum time in a half-cycle before a target crossing may toggle the relay,
 * to reject measurement noise near the setpoint. */
#define AUTOTUNE_MIN_HALF_MS 1000u

void pid_autotune_init(pid_autotune_t *at, const pid_autotune_config_t *config)
{
    if (at == NULL || config == NULL) {
        return;
    }
    at->config = *config;
    if (at->config.target_cycles < 3) {
        at->config.target_cycles = 3;
    }
    if (at->config.output_max <= 0.0f) {
        at->config.output_max = 100.0f;
    }

    at->heating = 1;
    at->bias = at->config.output_max * 0.5f;
    at->d = at->config.output_max * 0.5f;
    at->min_t = at->config.target_c;
    at->max_t = at->config.target_c;
    at->t1_ms = 0;
    at->t2_ms = 0;
    at->t_high_ms = 0;
    at->t_low_ms = 0;
    at->elapsed_ms = 0;
    at->cycle = 0;
    at->output = at->config.output_max; /* start driving up toward target */
    at->kp = 0.0f;
    at->ki = 0.0f;
    at->kd = 0.0f;
    at->have_result = 0;
    at->status = PID_AUTOTUNE_RUNNING;
}

float pid_autotune_update(pid_autotune_t *at, float temperature_c, uint32_t dt_ms)
{
    if (at == NULL) {
        return 0.0f;
    }
    if (at->status != PID_AUTOTUNE_RUNNING) {
        return 0.0f;
    }

    at->elapsed_ms += dt_ms;

    /* Safety aborts. */
    if (at->config.max_safe_c > 0.0f && temperature_c > at->config.max_safe_c) {
        at->status = PID_AUTOTUNE_FAILED;
        at->output = 0.0f;
        return 0.0f;
    }
    if (at->config.timeout_ms > 0 && at->elapsed_ms > at->config.timeout_ms) {
        at->status = PID_AUTOTUNE_FAILED;
        at->output = 0.0f;
        return 0.0f;
    }

    const float max_pow = at->config.output_max;
    const float margin = max_pow * 0.1f;

    /* Heating half: temperature has risen above target -> switch to cooling. */
    if (at->heating && temperature_c > at->config.target_c) {
        if (at->elapsed_ms - at->t2_ms > AUTOTUNE_MIN_HALF_MS) {
            at->heating = 0;
            at->output = at->bias - at->d;
            at->t1_ms = at->elapsed_ms;
            at->t_high_ms = at->t1_ms - at->t2_ms;
            at->max_t = at->config.target_c;
        }
    }

    /* Cooling half: temperature has fallen below target -> switch to heating,
     * rebalance the relay, and (after a few cycles) compute the gains. */
    if (!at->heating && temperature_c < at->config.target_c) {
        if (at->elapsed_ms - at->t1_ms > AUTOTUNE_MIN_HALF_MS) {
            at->heating = 1;
            at->t2_ms = at->elapsed_ms;
            at->t_low_ms = at->t2_ms - at->t1_ms;

            if (at->cycle > 0) {
                uint32_t span = at->t_low_ms + at->t_high_ms;
                if (span > 0) {
                    at->bias += (at->d * (float)((long)at->t_high_ms - (long)at->t_low_ms)) / (float)span;
                }
                if (at->bias < margin) at->bias = margin;
                if (at->bias > max_pow - margin) at->bias = max_pow - margin;
                at->d = (at->bias > max_pow * 0.5f) ? (max_pow - at->bias) : at->bias;

                if (at->cycle > 2) {
                    float amplitude = at->max_t - at->min_t;
                    if (amplitude < 0.01f) amplitude = 0.01f;
                    float ku = (4.0f * at->d) / (AUTOTUNE_PI * amplitude * 0.5f);
                    float tu = (float)(at->t_low_ms + at->t_high_ms) / 1000.0f;
                    if (tu < 0.001f) tu = 0.001f;
                    at->kp = 0.6f * ku;
                    at->ki = 2.0f * at->kp / tu;
                    at->kd = at->kp * tu / 8.0f;
                    at->have_result = 1;
                }
            }

            at->output = at->bias + at->d;
            at->cycle++;
            at->min_t = at->config.target_c;

            if (at->cycle > at->config.target_cycles) {
                at->status = at->have_result ? PID_AUTOTUNE_DONE : PID_AUTOTUNE_FAILED;
                at->output = 0.0f;
                return at->output;
            }
        }
    }

    /* Track the oscillation extremes within the current half-cycle. */
    if (temperature_c > at->max_t) at->max_t = temperature_c;
    if (temperature_c < at->min_t) at->min_t = temperature_c;

    if (at->output < 0.0f) at->output = 0.0f;
    if (at->output > max_pow) at->output = max_pow;
    return at->output;
}

pid_autotune_status_t pid_autotune_get_status(const pid_autotune_t *at)
{
    return (at != NULL) ? at->status : PID_AUTOTUNE_FAILED;
}

int pid_autotune_get_result(const pid_autotune_t *at, float *kp, float *ki, float *kd)
{
    if (at == NULL || !at->have_result) {
        return 0;
    }
    if (kp != NULL) *kp = at->kp;
    if (ki != NULL) *ki = at->ki;
    if (kd != NULL) *kd = at->kd;
    return 1;
}
