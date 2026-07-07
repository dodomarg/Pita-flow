#ifndef WEB_API_H
#define WEB_API_H

#include <stdint.h>

#include "core/profile_engine.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Lightweight status snapshot shared between the control task and the
 * web/API task. The web task must treat this as read-mostly and copy it
 * quickly (e.g. under a short-held mutex) rather than holding a lock while
 * building HTTP responses, so that polling clients cannot stall the
 * control loop.
 */
typedef struct {
    float chamber_temperature_c;
    float plate_temperature_c;
    float top_heater_power_percent;
    float bottom_heater_power_percent;
    profile_engine_state_t profile_state;
    uint32_t current_phase_index;
    uint32_t safety_fault_flags;
} web_api_status_t;

/** Starts the HTTP server and registers the /api/* endpoints described in docs/firmware-plan.md. */
esp_err_t web_api_start(void);

/**
 * Called by the control/telemetry task to publish the latest status
 * snapshot for the web task to serve. Copies status by value; safe to call
 * frequently from the control task.
 */
void web_api_publish_status(const web_api_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* WEB_API_H */
