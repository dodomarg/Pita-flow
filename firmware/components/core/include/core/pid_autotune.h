#ifndef CORE_PID_AUTOTUNE_H
#define CORE_PID_AUTOTUNE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Relay-feedback (Astrom-Hagglund / Marlin-style) PID autotune.
 *
 * The tuner drives the heater output as a relay around a target temperature,
 * measures the amplitude and period of the resulting limit cycle, and applies
 * Ziegler-Nichols rules to produce gains for the positional PID used by
 * heater_control (output = kp*e + ki*integral(e) + kd*d(e)/dt), so the results
 * can be used directly.
 *
 * Usage: init once, then call pid_autotune_update() every control tick with the
 * measured temperature; drive the heater with the returned power percent until
 * the status leaves PID_AUTOTUNE_RUNNING, then read the gains.
 */
typedef enum {
    PID_AUTOTUNE_RUNNING = 0,
    PID_AUTOTUNE_DONE = 1,
    PID_AUTOTUNE_FAILED = 2,
} pid_autotune_status_t;

typedef struct {
    float target_c;      /* Temperature to oscillate around. */
    float output_max;    /* Relay "high" output level in percent (e.g. 100). */
    int target_cycles;   /* Oscillation cycles to observe before finishing (>=3). */
    uint32_t timeout_ms; /* Abort as FAILED if not done within this time (0 = off). */
    float max_safe_c;    /* Abort as FAILED if temperature exceeds this (0 = off). */
} pid_autotune_config_t;

typedef struct {
    pid_autotune_config_t config;
    int heating;
    float bias;
    float d;
    float min_t;
    float max_t;
    uint32_t t1_ms;
    uint32_t t2_ms;
    uint32_t t_high_ms;
    uint32_t t_low_ms;
    uint32_t elapsed_ms;
    int cycle;
    float output;
    float kp;
    float ki;
    float kd;
    int have_result;
    pid_autotune_status_t status;
} pid_autotune_t;

/** Initializes the tuner and begins the first (heating) half-cycle. */
void pid_autotune_init(pid_autotune_t *at, const pid_autotune_config_t *config);

/**
 * Advances the tuner by dt_ms and returns the commanded output power percent
 * (0..output_max). Returns 0 once the tuner is no longer RUNNING.
 */
float pid_autotune_update(pid_autotune_t *at, float temperature_c, uint32_t dt_ms);

pid_autotune_status_t pid_autotune_get_status(const pid_autotune_t *at);

/**
 * Writes the tuned gains if a result is available. Returns 1 if gains were
 * written, 0 otherwise.
 */
int pid_autotune_get_result(const pid_autotune_t *at, float *kp, float *ki, float *kd);

#ifdef __cplusplus
}
#endif

#endif /* CORE_PID_AUTOTUNE_H */
