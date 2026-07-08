#include "core/ui_local.h"

#include <string.h>

static uint32_t clamp_selected_index(uint32_t profile_count, uint32_t selected_profile_index)
{
    if (profile_count == 0) {
        return 0;
    }
    if (selected_profile_index >= profile_count) {
        return profile_count - 1;
    }
    return selected_profile_index;
}

void ui_local_controller_init(ui_local_controller_t *controller, uint32_t profile_count, uint32_t selected_profile_index)
{
    if (controller == NULL) {
        return;
    }

    memset(controller, 0, sizeof(*controller));
    controller->profile_count = profile_count;
    controller->selected_profile_index = clamp_selected_index(profile_count, selected_profile_index);
    controller->active_profile_index = controller->selected_profile_index;
    controller->observed_state = PROFILE_ENGINE_STATE_IDLE;
}

void ui_local_controller_set_profile_count(ui_local_controller_t *controller, uint32_t profile_count)
{
    if (controller == NULL) {
        return;
    }

    controller->profile_count = profile_count;
    controller->selected_profile_index = clamp_selected_index(profile_count, controller->selected_profile_index);
    controller->active_profile_index = clamp_selected_index(profile_count, controller->active_profile_index);
}

ui_local_notification_t ui_local_controller_handle_button(ui_local_controller_t *controller,
                                                          ui_local_button_t button,
                                                          profile_engine_t *engine,
                                                          const reflow_profile_t *profiles)
{
    if (controller == NULL || engine == NULL) {
        return UI_LOCAL_NOTIFICATION_NONE;
    }

    if (engine->state == PROFILE_ENGINE_STATE_RUNNING) {
        if (button == UI_LOCAL_BUTTON_SELECT) {
            profile_engine_stop(engine);
            controller->observed_state = engine->state;
            controller->observed_phase_index = engine->current_phase_index;
            return UI_LOCAL_NOTIFICATION_RUN_STOPPED;
        }
        return UI_LOCAL_NOTIFICATION_NONE;
    }

    if (controller->profile_count == 0 || profiles == NULL) {
        return UI_LOCAL_NOTIFICATION_NONE;
    }

    if (button == UI_LOCAL_BUTTON_PREVIOUS) {
        controller->selected_profile_index =
            (controller->selected_profile_index == 0) ? (controller->profile_count - 1) : (controller->selected_profile_index - 1);
        return UI_LOCAL_NOTIFICATION_PROFILE_CHANGED;
    }

    if (button == UI_LOCAL_BUTTON_NEXT) {
        controller->selected_profile_index =
            (controller->selected_profile_index + 1U) % controller->profile_count;
        return UI_LOCAL_NOTIFICATION_PROFILE_CHANGED;
    }

    if (button == UI_LOCAL_BUTTON_SELECT) {
        const reflow_profile_t *selected_profile = &profiles[controller->selected_profile_index];
        if (profile_engine_load(engine, selected_profile) != 0) {
            return UI_LOCAL_NOTIFICATION_PROFILE_LOAD_FAILED;
        }
        if (profile_engine_start(engine) != 0) {
            return UI_LOCAL_NOTIFICATION_PROFILE_LOAD_FAILED;
        }
        controller->active_profile_index = controller->selected_profile_index;
        controller->observed_state = engine->state;
        controller->observed_phase_index = engine->current_phase_index;
        return UI_LOCAL_NOTIFICATION_RUN_STARTED;
    }

    return UI_LOCAL_NOTIFICATION_NONE;
}

ui_local_notification_t ui_local_controller_observe_engine(ui_local_controller_t *controller,
                                                           const profile_engine_t *engine,
                                                           uint32_t safety_fault_flags)
{
    if (controller == NULL || engine == NULL) {
        return UI_LOCAL_NOTIFICATION_NONE;
    }

    ui_local_notification_t notification = UI_LOCAL_NOTIFICATION_NONE;

    if (safety_fault_flags != 0 || engine->state == PROFILE_ENGINE_STATE_FAULT) {
        if (controller->observed_state != PROFILE_ENGINE_STATE_FAULT) {
            notification = UI_LOCAL_NOTIFICATION_FAULT;
        }
    } else if (engine->state == PROFILE_ENGINE_STATE_COMPLETE &&
               controller->observed_state != PROFILE_ENGINE_STATE_COMPLETE) {
        notification = UI_LOCAL_NOTIFICATION_COMPLETE;
    } else if (engine->state == PROFILE_ENGINE_STATE_RUNNING &&
               controller->observed_state == PROFILE_ENGINE_STATE_RUNNING &&
               engine->current_phase_index != controller->observed_phase_index) {
        notification = UI_LOCAL_NOTIFICATION_PHASE_CHANGED;
    }

    controller->observed_state = engine->state;
    controller->observed_phase_index = engine->current_phase_index;
    return notification;
}

void ui_local_controller_snapshot(const ui_local_controller_t *controller,
                                  const profile_engine_t *engine,
                                  const reflow_profile_t *profiles,
                                  ui_local_status_t *out_status)
{
    if (controller == NULL || engine == NULL || out_status == NULL) {
        return;
    }

    memset(out_status, 0, sizeof(*out_status));
    out_status->engine_state = engine->state;
    out_status->selected_profile_index = controller->selected_profile_index;
    out_status->active_profile_index = controller->active_profile_index;
    out_status->profile_count = controller->profile_count;
    out_status->current_phase_index = engine->current_phase_index;
    out_status->can_toggle_run = (controller->profile_count > 0);
    out_status->can_select_previous = (controller->profile_count > 1) && (engine->state != PROFILE_ENGINE_STATE_RUNNING);
    out_status->can_select_next = out_status->can_select_previous;

    if (profiles != NULL && controller->profile_count > 0 && controller->selected_profile_index < controller->profile_count) {
        out_status->selected_profile_name = profiles[controller->selected_profile_index].name;
    }

    const profile_phase_t *phase = profile_engine_current_phase(engine);
    if (phase != NULL) {
        out_status->active_phase_name = phase->name;
    }
}
