#include "core/built_in_profile.h"

#include <string.h>

static reflow_profile_t s_profile;
static int s_initialized = 0;

static void set_phase(profile_phase_t *phase, const char *name, uint32_t duration_s,
                       float target_chamber_c, float ramp_rate_c_per_s,
                       bottom_heater_mode_t bottom_mode, float bottom_plate_limit_c,
                       int top_enabled)
{
    memset(phase, 0, sizeof(*phase));
    strncpy(phase->name, name, sizeof(phase->name) - 1);
    phase->duration_s = duration_s;
    phase->target_chamber_c = target_chamber_c;
    phase->ramp_rate_c_per_s = ramp_rate_c_per_s;
    phase->bottom_heater_mode = bottom_mode;
    phase->bottom_plate_limit_c = bottom_plate_limit_c;
    phase->top_heater_enabled = top_enabled;
}

const reflow_profile_t *built_in_profile_get(void)
{
    if (s_initialized) {
        return &s_profile;
    }

    memset(&s_profile, 0, sizeof(s_profile));
    strncpy(s_profile.name, "built-in-default", sizeof(s_profile.name) - 1);
    s_profile.phase_count = 4;

    set_phase(&s_profile.phases[0], "preheat", 60, 150.0f, 1.5f,
              BOTTOM_HEATER_MODE_LIMITED, 150.0f, 1);
    set_phase(&s_profile.phases[1], "soak", 90, 170.0f, 0.5f,
              BOTTOM_HEATER_MODE_LIMITED, 150.0f, 1);
    set_phase(&s_profile.phases[2], "reflow", 60, 220.0f, 1.0f,
              BOTTOM_HEATER_MODE_OFF, 0.0f, 1);
    set_phase(&s_profile.phases[3], "cooldown", 60, 50.0f, 2.0f,
              BOTTOM_HEATER_MODE_OFF, 0.0f, 0);

    s_initialized = 1;
    return &s_profile;
}
