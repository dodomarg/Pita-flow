#ifndef CORE_RELAY_TIMING_H
#define CORE_RELAY_TIMING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Given a demanded power percentage (0..100) and a window duration expressed
 * in milliseconds, computes the number of milliseconds the relay should be
 * energized ("on time") within that window using time-proportional control.
 *
 * power_percent is clamped to [0, 100]. window_ms must be > 0.
 * Returns the on-time in milliseconds, in the range [0, window_ms].
 */
uint32_t relay_timing_on_ms(float power_percent, uint32_t window_ms);

/**
 * Decides whether the relay should be energized at a given elapsed time
 * (elapsed_ms) within the current window, given the on-time computed by
 * relay_timing_on_ms(). elapsed_ms should be in the range [0, window_ms).
 *
 * Returns 1 (energize) if elapsed_ms < on_ms, 0 (de-energize) otherwise.
 */
int relay_timing_should_energize(uint32_t elapsed_ms, uint32_t on_ms);

#ifdef __cplusplus
}
#endif

#endif /* CORE_RELAY_TIMING_H */
