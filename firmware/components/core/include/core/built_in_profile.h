#ifndef CORE_BUILT_IN_PROFILE_H
#define CORE_BUILT_IN_PROFILE_H

#include "core/profile_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Returns the single built-in reflow profile used as a bring-up fallback
 * on any board target (ESP-IDF reference app or Wio Terminal) before a
 * profile is loaded from persistent/microSD storage. The values below are
 * reasonable placeholders for a leaded-solder-style profile and are
 * expected to be tuned/replaced once real thermal characterization data is
 * available (see docs/firmware-plan.md "Auto-Calibration Roadmap").
 */
const reflow_profile_t *built_in_profile_get(void);

#ifdef __cplusplus
}
#endif

#endif /* CORE_BUILT_IN_PROFILE_H */
