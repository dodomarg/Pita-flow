#ifndef CORE_UI_LOCAL_H
#define CORE_UI_LOCAL_H

#include <stdint.h>

#include "core/profile_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UI_LOCAL_BUTTON_PREVIOUS = 0,
    UI_LOCAL_BUTTON_SELECT = 1,
    UI_LOCAL_BUTTON_NEXT = 2,
} ui_local_button_t;

typedef enum {
    UI_LOCAL_NOTIFICATION_NONE = 0,
    UI_LOCAL_NOTIFICATION_PROFILE_CHANGED = 1,
    UI_LOCAL_NOTIFICATION_RUN_STARTED = 2,
    UI_LOCAL_NOTIFICATION_RUN_STOPPED = 3,
    UI_LOCAL_NOTIFICATION_PHASE_CHANGED = 4,
    UI_LOCAL_NOTIFICATION_COMPLETE = 5,
    UI_LOCAL_NOTIFICATION_FAULT = 6,
    UI_LOCAL_NOTIFICATION_PROFILE_LOAD_FAILED = 7,
} ui_local_notification_t;

typedef struct {
    uint32_t profile_count;
    uint32_t selected_profile_index;
    uint32_t active_profile_index;
    profile_engine_state_t observed_state;
    uint32_t observed_phase_index;
} ui_local_controller_t;

typedef struct {
    const char *selected_profile_name;
    const char *active_phase_name;
    profile_engine_state_t engine_state;
    uint32_t selected_profile_index;
    uint32_t active_profile_index;
    uint32_t profile_count;
    uint32_t current_phase_index;
    int can_select_previous;
    int can_select_next;
    int can_toggle_run;
} ui_local_status_t;

/** Initializes local Wio Terminal-oriented UI state. */
void ui_local_controller_init(ui_local_controller_t *controller, uint32_t profile_count, uint32_t selected_profile_index);

/** Updates the available profile count and clamps the selected index if needed. */
void ui_local_controller_set_profile_count(ui_local_controller_t *controller, uint32_t profile_count);

/**
 * Applies a button press to the selected profile / run state. The caller owns
 * the actual display and speaker drivers; this function only returns the
 * notification that should be rendered or sounded.
 */
ui_local_notification_t ui_local_controller_handle_button(ui_local_controller_t *controller,
                                                          ui_local_button_t button,
                                                          profile_engine_t *engine,
                                                          const reflow_profile_t *profiles);

/**
 * Observes engine/fault state and emits edge-triggered notifications suitable
 * for screen refreshes or speaker tones.
 */
ui_local_notification_t ui_local_controller_observe_engine(ui_local_controller_t *controller,
                                                           const profile_engine_t *engine,
                                                           uint32_t safety_fault_flags);

/** Produces a view-model snapshot for display rendering. */
void ui_local_controller_snapshot(const ui_local_controller_t *controller,
                                  const profile_engine_t *engine,
                                  const reflow_profile_t *profiles,
                                  ui_local_status_t *out_status);

#ifdef __cplusplus
}
#endif

#endif /* CORE_UI_LOCAL_H */
