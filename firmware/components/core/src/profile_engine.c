#include "core/profile_engine.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

int profile_engine_validate(const reflow_profile_t *profile)
{
    if (profile == NULL) {
        return 0;
    }
    if (profile->phase_count == 0 || profile->phase_count > PROFILE_ENGINE_MAX_PHASES) {
        return 0;
    }
    for (uint32_t i = 0; i < profile->phase_count; i++) {
        const profile_phase_t *phase = &profile->phases[i];
        if (phase->duration_s == 0) {
            return 0;
        }
        if (phase->target_chamber_c < 0.0f) {
            return 0;
        }
        if (phase->ramp_rate_c_per_s < 0.0f) {
            return 0;
        }
        if (phase->name[0] == '\0') {
            return 0;
        }
    }
    return 1;
}

int profile_engine_load(profile_engine_t *engine, const reflow_profile_t *profile)
{
    if (engine == NULL) {
        return -1;
    }
    if (!profile_engine_validate(profile)) {
        return -1;
    }
    engine->active_profile = profile;
    engine->state = PROFILE_ENGINE_STATE_IDLE;
    engine->current_phase_index = 0;
    engine->elapsed_in_phase_s = 0.0f;
    return 0;
}

int profile_engine_start(profile_engine_t *engine)
{
    if (engine == NULL || engine->active_profile == NULL) {
        return -1;
    }
    if (!profile_engine_validate(engine->active_profile)) {
        return -1;
    }
    engine->state = PROFILE_ENGINE_STATE_RUNNING;
    engine->current_phase_index = 0;
    engine->elapsed_in_phase_s = 0.0f;
    return 0;
}

void profile_engine_force_fault(profile_engine_t *engine)
{
    if (engine == NULL) {
        return;
    }
    engine->state = PROFILE_ENGINE_STATE_FAULT;
}

void profile_engine_stop(profile_engine_t *engine)
{
    if (engine == NULL) {
        return;
    }
    engine->state = PROFILE_ENGINE_STATE_IDLE;
    engine->current_phase_index = 0;
    engine->elapsed_in_phase_s = 0.0f;
}

void profile_engine_tick(profile_engine_t *engine, float tick_s)
{
    if (engine == NULL || engine->state != PROFILE_ENGINE_STATE_RUNNING) {
        return;
    }
    if (engine->active_profile == NULL || tick_s <= 0.0f) {
        return;
    }

    engine->elapsed_in_phase_s += tick_s;

    const profile_phase_t *phase = &engine->active_profile->phases[engine->current_phase_index];

    if (engine->elapsed_in_phase_s >= (float)phase->duration_s) {
        engine->elapsed_in_phase_s = 0.0f;
        engine->current_phase_index++;
        if (engine->current_phase_index >= engine->active_profile->phase_count) {
            engine->state = PROFILE_ENGINE_STATE_COMPLETE;
            engine->current_phase_index = engine->active_profile->phase_count - 1;
        }
    }
}

const profile_phase_t *profile_engine_current_phase(const profile_engine_t *engine)
{
    if (engine == NULL || engine->active_profile == NULL) {
        return NULL;
    }
    if (engine->state != PROFILE_ENGINE_STATE_RUNNING && engine->state != PROFILE_ENGINE_STATE_COMPLETE) {
        return NULL;
    }
    if (engine->current_phase_index >= engine->active_profile->phase_count) {
        return NULL;
    }
    return &engine->active_profile->phases[engine->current_phase_index];
}

float profile_engine_current_target_c(const profile_engine_t *engine, float previous_phase_target_c)
{
    const profile_phase_t *phase = profile_engine_current_phase(engine);
    if (phase == NULL) {
        return 0.0f;
    }

    if (phase->ramp_rate_c_per_s <= 0.0f) {
        return phase->target_chamber_c;
    }

    float max_delta = phase->ramp_rate_c_per_s * (float)engine->elapsed_in_phase_s;
    if (phase->target_chamber_c >= previous_phase_target_c) {
        float target = previous_phase_target_c + max_delta;
        return target > phase->target_chamber_c ? phase->target_chamber_c : target;
    }
    float target = previous_phase_target_c - max_delta;
    return target < phase->target_chamber_c ? phase->target_chamber_c : target;
}
