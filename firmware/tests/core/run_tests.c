/* Minimal, dependency-free test harness for the hardware-independent core
 * firmware logic. Builds and runs with plain gcc (no ESP-IDF required):
 * see run_tests.sh in this directory for the exact build command.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "core/heater_control.h"
#include "core/ntc_convert.h"
#include "core/profile_catalog.h"
#include "core/profile_engine.h"
#include "core/relay_timing.h"
#include "core/safety.h"
#include "core/ui_local.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                                       \
    do {                                                                                   \
        g_checks++;                                                                       \
        if (!(cond)) {                                                                    \
            g_failures++;                                                                 \
            fprintf(stderr, "FAIL: %s:%d: %s\n", __FILE__, __LINE__, #cond);               \
        }                                                                                  \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                              \
    do {                                                                                   \
        g_checks++;                                                                        \
        double _a = (double)(a), _b = (double)(b), _eps = (double)(eps);                   \
        if (fabs(_a - _b) > _eps) {                                                         \
            g_failures++;                                                                  \
            fprintf(stderr, "FAIL: %s:%d: %s (%f) !~= %s (%f)\n", __FILE__, __LINE__,       \
                    #a, _a, #b, _b);                                                        \
        }                                                                                   \
    } while (0)

static void test_ntc_convert(void)
{
    ntc_params_t params = {
        .pulldown_resistor_ohm = 100000.0f,
        .nominal_resistance_ohm = 100000.0f,
        .nominal_temperature_c = 25.0f,
        .beta = 3950.0f,
    };

    /* At the balance point (raw_adc == adc_max/2), R_ntc == R_pulldown, so the
     * temperature should equal the nominal temperature. */
    float temp_c = 0.0f;
    int rc = ntc_convert_adc_to_celsius(2048, 4095, &params, &temp_c);
    CHECK(rc == 0);
    CHECK_NEAR(temp_c, 25.0f, 0.5f);

    /* NTC on the high side / pulldown on the low side: higher raw_adc =>
     * lower R_ntc => hotter temperature. */
    float hot_temp_c = 0.0f;
    float cold_temp_c = 0.0f;
    CHECK(ntc_convert_adc_to_celsius(3000, 4095, &params, &hot_temp_c) == 0);
    CHECK(ntc_convert_adc_to_celsius(1000, 4095, &params, &cold_temp_c) == 0);
    CHECK(hot_temp_c > cold_temp_c);

    /* Open circuit (raw_adc == 0, pulled fully to gnd) and shorted
     * (raw_adc == adc_max, pulled fully to supply) are faults. */
    CHECK(ntc_convert_adc_to_celsius(4095, 4095, &params, &temp_c) != 0);
    CHECK(ntc_convert_adc_to_celsius(0, 4095, &params, &temp_c) != 0);

    CHECK(ntc_convert_adc_to_celsius(2048, 4095, NULL, &temp_c) != 0);
    CHECK(ntc_convert_adc_to_celsius(2048, 0, &params, &temp_c) != 0);

    CHECK(ntc_reading_is_implausible(0, 4095, 10) == 1);
    CHECK(ntc_reading_is_implausible(4095, 4095, 10) == 1);
    CHECK(ntc_reading_is_implausible(5, 4095, 10) == 1);
    CHECK(ntc_reading_is_implausible(2048, 4095, 10) == 0);
}

static void test_relay_timing(void)
{
    CHECK(relay_timing_on_ms(0.0f, 1000) == 0);
    CHECK(relay_timing_on_ms(100.0f, 1000) == 1000);
    CHECK(relay_timing_on_ms(35.0f, 1000) == 350);
    CHECK(relay_timing_on_ms(-10.0f, 1000) == 0);
    CHECK(relay_timing_on_ms(150.0f, 1000) == 1000);
    CHECK(relay_timing_on_ms(50.0f, 0) == 0);

    CHECK(relay_timing_should_energize(0, 350) == 1);
    CHECK(relay_timing_should_energize(349, 350) == 1);
    CHECK(relay_timing_should_energize(350, 350) == 0);
    CHECK(relay_timing_should_energize(999, 0) == 0);
}

static void test_heater_control_bang_bang(void)
{
    heater_control_t ctrl;
    heater_control_config_t config = {
        .hysteresis_c = 2.0f,
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .min_power_percent = 0.0f,
        .max_power_percent = 100.0f,
    };
    heater_control_init(&ctrl, HEATER_CONTROL_MODE_BANG_BANG, &config);

    /* Well below target: full power. */
    CHECK_NEAR(heater_control_update(&ctrl, 100.0f, 150.0f, 1.0f), 100.0f, 0.01f);
    /* Well above target: no power. */
    CHECK_NEAR(heater_control_update(&ctrl, 160.0f, 150.0f, 1.0f), 0.0f, 0.01f);
    /* Within hysteresis band after being off: should remain off (avoid chatter). */
    CHECK_NEAR(heater_control_update(&ctrl, 149.5f, 150.0f, 1.0f), 0.0f, 0.01f);
}

static void test_heater_control_pid_clamps(void)
{
    heater_control_t ctrl;
    heater_control_config_t config = {
        .hysteresis_c = 0.0f,
        .kp = 10.0f,
        .ki = 1.0f,
        .kd = 0.0f,
        .min_power_percent = 0.0f,
        .max_power_percent = 100.0f,
    };
    heater_control_init(&ctrl, HEATER_CONTROL_MODE_PID, &config);

    /* Large error should saturate at max_power_percent, not exceed it. */
    float power = heater_control_update(&ctrl, 0.0f, 200.0f, 1.0f);
    CHECK(power <= 100.0f);
    CHECK(power >= 0.0f);
    CHECK_NEAR(power, 100.0f, 0.01f);
}

static void test_profile_engine(void)
{
    reflow_profile_t profile;
    memset(&profile, 0, sizeof(profile));
    strncpy(profile.name, "test", sizeof(profile.name) - 1);
    profile.phase_count = 2;

    strncpy(profile.phases[0].name, "preheat", sizeof(profile.phases[0].name) - 1);
    profile.phases[0].duration_s = 10;
    profile.phases[0].target_chamber_c = 150.0f;
    profile.phases[0].ramp_rate_c_per_s = 0.0f;
    profile.phases[0].bottom_heater_mode = BOTTOM_HEATER_MODE_ENABLED;
    profile.phases[0].bottom_plate_limit_c = 160.0f;
    profile.phases[0].top_heater_enabled = 1;

    strncpy(profile.phases[1].name, "reflow", sizeof(profile.phases[1].name) - 1);
    profile.phases[1].duration_s = 5;
    profile.phases[1].target_chamber_c = 220.0f;
    profile.phases[1].ramp_rate_c_per_s = 1.0f;
    profile.phases[1].bottom_heater_mode = BOTTOM_HEATER_MODE_OFF;
    profile.phases[1].bottom_plate_limit_c = 0.0f;
    profile.phases[1].top_heater_enabled = 1;

    CHECK(profile_engine_validate(&profile) == 1);

    profile_engine_t engine;
    CHECK(profile_engine_load(&engine, &profile) == 0);
    CHECK(engine.state == PROFILE_ENGINE_STATE_IDLE);

    CHECK(profile_engine_start(&engine) == 0);
    CHECK(engine.state == PROFILE_ENGINE_STATE_RUNNING);
    CHECK(profile_engine_current_phase(&engine) == &profile.phases[0]);

    /* Advance almost to the end of phase 0. */
    for (int i = 0; i < 9; i++) {
        profile_engine_tick(&engine, 1.0f);
    }
    CHECK(engine.current_phase_index == 0);

    /* Cross into phase 1. */
    profile_engine_tick(&engine, 1.0f);
    CHECK(engine.current_phase_index == 1);
    CHECK(profile_engine_current_phase(&engine) == &profile.phases[1]);

    /* Ramp rate: after 3s in phase 1, target should have ramped partway from
     * the previous phase's target (150) towards 220 at 1 C/s => 153. */
    float target = profile_engine_current_target_c(&engine, 150.0f);
    CHECK_NEAR(target, 150.0f, 0.01f);
    profile_engine_tick(&engine, 3.0f);
    target = profile_engine_current_target_c(&engine, 150.0f);
    CHECK_NEAR(target, 153.0f, 0.01f);

    /* Finish phase 1 (5s total) and confirm completion. */
    profile_engine_tick(&engine, 2.0f);
    CHECK(engine.state == PROFILE_ENGINE_STATE_COMPLETE);

    /* Fault handling. */
    profile_engine_force_fault(&engine);
    CHECK(engine.state == PROFILE_ENGINE_STATE_FAULT);
    profile_engine_tick(&engine, 1.0f); /* No effect while faulted. */
    CHECK(engine.state == PROFILE_ENGINE_STATE_FAULT);

    profile_engine_stop(&engine);
    CHECK(engine.state == PROFILE_ENGINE_STATE_IDLE);

    /* Invalid profile rejected. */
    reflow_profile_t empty;
    memset(&empty, 0, sizeof(empty));
    profile_engine_t engine2;
    CHECK(profile_engine_load(&engine2, &empty) == -1);
}

static void test_safety(void)
{
    safety_limits_t limits = {
        .chamber_absolute_max_c = 250.0f,
        .plate_absolute_max_c = 200.0f,
        .control_watchdog_timeout_ms = 500,
    };

    safety_inputs_t ok_inputs;
    memset(&ok_inputs, 0, sizeof(ok_inputs));
    ok_inputs.chamber_temperature_c = 100.0f;
    ok_inputs.plate_temperature_c = 80.0f;
    ok_inputs.ms_since_last_control_tick = 100;
    ok_inputs.active_profile_valid = 1;

    CHECK(safety_evaluate(&ok_inputs, &limits) == SAFETY_FAULT_NONE);

    safety_inputs_t over_temp = ok_inputs;
    over_temp.chamber_temperature_c = 300.0f;
    CHECK((safety_evaluate(&over_temp, &limits) & SAFETY_FAULT_CHAMBER_OVER_TEMP) != 0);

    safety_inputs_t plate_over = ok_inputs;
    plate_over.plate_temperature_c = 250.0f;
    CHECK((safety_evaluate(&plate_over, &limits) & SAFETY_FAULT_PLATE_OVER_TEMP) != 0);

    safety_inputs_t watchdog = ok_inputs;
    watchdog.ms_since_last_control_tick = 1000;
    CHECK((safety_evaluate(&watchdog, &limits) & SAFETY_FAULT_CONTROL_WATCHDOG_TIMEOUT) != 0);

    safety_inputs_t tc_fault = ok_inputs;
    tc_fault.thermocouple_fault_bit = 1;
    CHECK((safety_evaluate(&tc_fault, &limits) & SAFETY_FAULT_THERMOCOUPLE_FAULT_BIT) != 0);

    safety_inputs_t relay_while_idle = ok_inputs;
    relay_while_idle.system_is_idle_or_fault = 1;
    relay_while_idle.relay_commanded_on = 1;
    CHECK((safety_evaluate(&relay_while_idle, &limits) & SAFETY_FAULT_RELAY_COMMANDED_WHILE_IDLE) != 0);

    safety_inputs_t invalid_profile = ok_inputs;
    invalid_profile.active_profile_valid = 0;
    CHECK((safety_evaluate(&invalid_profile, &limits) & SAFETY_FAULT_INVALID_PROFILE) != 0);

    CHECK(safety_evaluate(NULL, &limits) != SAFETY_FAULT_NONE);
    CHECK(safety_evaluate(&ok_inputs, NULL) != SAFETY_FAULT_NONE);
}

static void test_profile_catalog_yaml(void)
{
    const char *yaml =
        "profiles:\n"
        "  - name: leaded\n"
        "    phases:\n"
        "      - name: preheat\n"
        "        duration_s: 60\n"
        "        target_chamber_c: 150\n"
        "        ramp_rate_c_per_s: 1.5\n"
        "        bottom_heater_mode: limited\n"
        "        bottom_plate_limit_c: 150\n"
        "        top_heater_enabled: true\n"
        "      - name: reflow\n"
        "        duration_s: 45\n"
        "        target_chamber_c: 220\n"
        "        ramp_rate_c_per_s: 1.0\n"
        "        bottom_heater_mode: off\n"
        "        bottom_plate_limit_c: 0\n"
        "        top_heater_enabled: true\n"
        "  - name: low-temp\n"
        "    phases:\n"
        "      - name: warmup\n"
        "        duration_s: 50\n"
        "        target_chamber_c: 120\n"
        "        ramp_rate_c_per_s: 1.2\n"
        "        bottom_heater_mode: enabled\n"
        "        bottom_plate_limit_c: 130\n"
        "        top_heater_enabled: true\n"
        "      - name: cooldown\n"
        "        duration_s: 40\n"
        "        target_chamber_c: 40\n"
        "        ramp_rate_c_per_s: 2.0\n"
        "        bottom_heater_mode: off\n"
        "        bottom_plate_limit_c: 0\n"
        "        top_heater_enabled: false\n";

    reflow_profile_catalog_t catalog;
    CHECK(profile_catalog_parse_yaml(yaml, &catalog) == 0);
    CHECK(catalog.profile_count == 2);

    const reflow_profile_t *profile0 = profile_catalog_get(&catalog, 0);
    const reflow_profile_t *profile1 = profile_catalog_get(&catalog, 1);
    CHECK(profile0 != NULL);
    CHECK(profile1 != NULL);
    CHECK(strcmp(profile0->name, "leaded") == 0);
    CHECK(profile0->phase_count == 2);
    CHECK(strcmp(profile0->phases[0].name, "preheat") == 0);
    CHECK(profile0->phases[0].bottom_heater_mode == BOTTOM_HEATER_MODE_LIMITED);
    CHECK(profile0->phases[1].bottom_heater_mode == BOTTOM_HEATER_MODE_OFF);
    CHECK(strcmp(profile1->name, "low-temp") == 0);
    CHECK(profile1->phases[1].top_heater_enabled == 0);
    CHECK(profile_catalog_get(&catalog, 99) == NULL);

    const char *invalid_yaml =
        "profiles:\n"
        "  - name: broken\n"
        "    phases:\n"
        "      - name: preheat\n"
        "        duration_s: 60\n"
        "        target_chamber_c: 150\n"
        "        ramp_rate_c_per_s: 1.5\n"
        "        bottom_heater_mode: invalid-mode\n"
        "        bottom_plate_limit_c: 150\n"
        "        top_heater_enabled: true\n";
    CHECK(profile_catalog_parse_yaml(invalid_yaml, &catalog) != 0);
}

static void test_ui_local_controller(void)
{
    reflow_profile_t profiles[2];
    memset(profiles, 0, sizeof(profiles));

    strncpy(profiles[0].name, "profile-a", sizeof(profiles[0].name) - 1);
    profiles[0].phase_count = 2;
    strncpy(profiles[0].phases[0].name, "preheat", sizeof(profiles[0].phases[0].name) - 1);
    profiles[0].phases[0].duration_s = 2;
    profiles[0].phases[0].target_chamber_c = 150.0f;
    profiles[0].phases[0].ramp_rate_c_per_s = 0.0f;
    profiles[0].phases[0].bottom_heater_mode = BOTTOM_HEATER_MODE_LIMITED;
    profiles[0].phases[0].bottom_plate_limit_c = 150.0f;
    profiles[0].phases[0].top_heater_enabled = 1;
    strncpy(profiles[0].phases[1].name, "reflow", sizeof(profiles[0].phases[1].name) - 1);
    profiles[0].phases[1].duration_s = 2;
    profiles[0].phases[1].target_chamber_c = 220.0f;
    profiles[0].phases[1].ramp_rate_c_per_s = 1.0f;
    profiles[0].phases[1].bottom_heater_mode = BOTTOM_HEATER_MODE_OFF;
    profiles[0].phases[1].bottom_plate_limit_c = 0.0f;
    profiles[0].phases[1].top_heater_enabled = 1;

    strncpy(profiles[1].name, "profile-b", sizeof(profiles[1].name) - 1);
    profiles[1].phase_count = 1;
    strncpy(profiles[1].phases[0].name, "short", sizeof(profiles[1].phases[0].name) - 1);
    profiles[1].phases[0].duration_s = 3;
    profiles[1].phases[0].target_chamber_c = 180.0f;
    profiles[1].phases[0].ramp_rate_c_per_s = 0.0f;
    profiles[1].phases[0].bottom_heater_mode = BOTTOM_HEATER_MODE_ENABLED;
    profiles[1].phases[0].bottom_plate_limit_c = 140.0f;
    profiles[1].phases[0].top_heater_enabled = 1;

    profile_engine_t engine;
    memset(&engine, 0, sizeof(engine));

    ui_local_controller_t controller;
    ui_local_controller_init(&controller, 2, 0);
    CHECK(controller.selected_profile_index == 0);

    ui_local_status_t status;
    ui_local_controller_snapshot(&controller, &engine, profiles, &status);
    CHECK(strcmp(status.selected_profile_name, "profile-a") == 0);
    CHECK(status.can_select_next == 1);

    CHECK(ui_local_controller_handle_button(&controller, UI_LOCAL_BUTTON_NEXT, &engine, profiles) ==
          UI_LOCAL_NOTIFICATION_PROFILE_CHANGED);
    CHECK(controller.selected_profile_index == 1);

    CHECK(ui_local_controller_handle_button(&controller, UI_LOCAL_BUTTON_SELECT, &engine, profiles) ==
          UI_LOCAL_NOTIFICATION_RUN_STARTED);
    CHECK(engine.state == PROFILE_ENGINE_STATE_RUNNING);
    CHECK(controller.active_profile_index == 1);

    CHECK(ui_local_controller_handle_button(&controller, UI_LOCAL_BUTTON_NEXT, &engine, profiles) ==
          UI_LOCAL_NOTIFICATION_NONE);
    CHECK(controller.selected_profile_index == 1);

    CHECK(ui_local_controller_observe_engine(&controller, &engine, 0) == UI_LOCAL_NOTIFICATION_NONE);
    profile_engine_tick(&engine, 1.0f);
    CHECK(ui_local_controller_observe_engine(&controller, &engine, 0) == UI_LOCAL_NOTIFICATION_NONE);

    CHECK(ui_local_controller_handle_button(&controller, UI_LOCAL_BUTTON_SELECT, &engine, profiles) ==
          UI_LOCAL_NOTIFICATION_RUN_STOPPED);
    CHECK(engine.state == PROFILE_ENGINE_STATE_IDLE);

    CHECK(ui_local_controller_handle_button(&controller, UI_LOCAL_BUTTON_PREVIOUS, &engine, profiles) ==
          UI_LOCAL_NOTIFICATION_PROFILE_CHANGED);
    CHECK(controller.selected_profile_index == 0);

    CHECK(ui_local_controller_handle_button(&controller, UI_LOCAL_BUTTON_SELECT, &engine, profiles) ==
          UI_LOCAL_NOTIFICATION_RUN_STARTED);
    profile_engine_tick(&engine, 2.0f);
    CHECK(ui_local_controller_observe_engine(&controller, &engine, 0) == UI_LOCAL_NOTIFICATION_PHASE_CHANGED);
    profile_engine_tick(&engine, 2.0f);
    CHECK(ui_local_controller_observe_engine(&controller, &engine, 0) == UI_LOCAL_NOTIFICATION_COMPLETE);

    profile_engine_force_fault(&engine);
    CHECK(ui_local_controller_observe_engine(&controller, &engine, SAFETY_FAULT_INVALID_PROFILE) ==
          UI_LOCAL_NOTIFICATION_FAULT);
}

int main(void)
{
    test_ntc_convert();
    test_relay_timing();
    test_heater_control_bang_bang();
    test_heater_control_pid_clamps();
    test_profile_engine();
    test_safety();
    test_profile_catalog_yaml();
    test_ui_local_controller();

    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
