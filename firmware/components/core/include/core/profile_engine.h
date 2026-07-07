#ifndef CORE_PROFILE_ENGINE_H
#define CORE_PROFILE_ENGINE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROFILE_ENGINE_MAX_PHASES 16
#define PROFILE_ENGINE_MAX_NAME_LEN 32

typedef enum {
    BOTTOM_HEATER_MODE_OFF = 0,
    BOTTOM_HEATER_MODE_LIMITED = 1,
    BOTTOM_HEATER_MODE_ENABLED = 2,
} bottom_heater_mode_t;

typedef struct {
    char name[PROFILE_ENGINE_MAX_NAME_LEN];
    uint32_t duration_s;
    float target_chamber_c;
    float ramp_rate_c_per_s;
    bottom_heater_mode_t bottom_heater_mode;
    float bottom_plate_limit_c;
    int top_heater_enabled;
} profile_phase_t;

typedef struct {
    char name[PROFILE_ENGINE_MAX_NAME_LEN];
    profile_phase_t phases[PROFILE_ENGINE_MAX_PHASES];
    uint32_t phase_count;
} reflow_profile_t;

typedef enum {
    PROFILE_ENGINE_STATE_IDLE = 0,
    PROFILE_ENGINE_STATE_RUNNING = 1,
    PROFILE_ENGINE_STATE_COMPLETE = 2,
    PROFILE_ENGINE_STATE_FAULT = 3,
} profile_engine_state_t;

typedef struct {
    const reflow_profile_t *active_profile;
    profile_engine_state_t state;
    uint32_t current_phase_index;
    float elapsed_in_phase_s;
} profile_engine_t;

/** Returns 1 if the profile is well-formed (non-empty, all phases valid), 0 otherwise. */
int profile_engine_validate(const reflow_profile_t *profile);

/**
 * Loads a validated profile and resets engine state to idle.
 * Returns 0 on success, -1 if the profile fails validation.
 */
int profile_engine_load(profile_engine_t *engine, const reflow_profile_t *profile);

/** Starts running the active profile from phase 0. Returns 0 on success, -1 if no valid profile is loaded. */
int profile_engine_start(profile_engine_t *engine);

/** Forces the engine into the fault state; the caller is responsible for disabling heaters. */
void profile_engine_force_fault(profile_engine_t *engine);

/** Stops the run and returns the engine to idle. */
void profile_engine_stop(profile_engine_t *engine);

/**
 * Advances the engine by tick_s seconds of elapsed time. Should be called
 * periodically by the profile task. Has no effect unless state is RUNNING.
 */
void profile_engine_tick(profile_engine_t *engine, float tick_s);

/** Returns the phase currently in effect, or NULL if not running. */
const profile_phase_t *profile_engine_current_phase(const profile_engine_t *engine);

/**
 * Computes the current target chamber temperature accounting for the
 * configured ramp rate within the current phase, given how long the phase
 * has been active. Returns 0.0f if not running.
 */
float profile_engine_current_target_c(const profile_engine_t *engine, float previous_phase_target_c);

#ifdef __cplusplus
}
#endif

#endif /* CORE_PROFILE_ENGINE_H */
