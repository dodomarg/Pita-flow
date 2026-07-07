#ifndef MAIN_BUILT_IN_PROFILE_H
#define MAIN_BUILT_IN_PROFILE_H

#include "core/profile_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Returns the single built-in reflow profile used until profile persistence
 * and custom profile editing (later milestones) are implemented. The
 * values below are reasonable placeholders for a leaded-solder-style
 * profile and are expected to be tuned/replaced once real thermal
 * characterization data is available (see docs/firmware-plan.md
 * "Auto-Calibration Roadmap").
 */
const reflow_profile_t *built_in_profile_get(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_BUILT_IN_PROFILE_H */
