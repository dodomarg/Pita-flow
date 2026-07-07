#include <inttypes.h>
#include <string.h>

#include "built_in_profile.h"
#include "config_store.h"
#include "core/heater_control.h"
#include "core/profile_engine.h"
#include "core/relay_timing.h"
#include "core/safety.h"
#include "drivers/plate_ntc.h"
#include "drivers/relay_output.h"
#include "drivers/thermocouple_max31855.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "web_api.h"
#include "wifi_init.h"

/* ESP32-C3 Super Mini pin assignment, see docs/firmware-plan.md
 * "ESP32-C3 Super Mini Pin Assignment". */
#define PIN_MAX31855_CLK 6
#define PIN_MAX31855_MISO 2
#define PIN_MAX31855_CS 10
#define PIN_PLATE_NTC_ADC_CHANNEL 0 /* GPIO0 / ADC1_CH0 */
#define PIN_TOP_HEATER 4
#define PIN_BOTTOM_HEATER 5

#define CONTROL_TASK_PERIOD_MS 100
#define TELEMETRY_TASK_PERIOD_MS 500

static const char *TAG = "main";

static system_config_t s_config;
static profile_engine_t s_profile_engine;
static heater_control_t s_top_heater_ctrl;
static heater_control_t s_bottom_heater_ctrl;

static SemaphoreHandle_t s_status_mutex;
static web_api_status_t s_latest_status;

static void publish_status_snapshot(const web_api_status_t *status)
{
    if (xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s_latest_status = *status;
        xSemaphoreGive(s_status_mutex);
    }
}

static void init_hardware(void)
{
    thermocouple_max31855_config_t tc_config = {
        .spi_host = SPI2_HOST,
        .pin_clk = PIN_MAX31855_CLK,
        .pin_miso = PIN_MAX31855_MISO,
        .pin_cs = PIN_MAX31855_CS,
        .clock_speed_hz = 4000000,
    };
    ESP_ERROR_CHECK(thermocouple_max31855_init(&tc_config));

    plate_ntc_config_t ntc_config = {
        .adc_channel = PIN_PLATE_NTC_ADC_CHANNEL,
        .ntc_params =
            {
                .pulldown_resistor_ohm = 4700.0f, /* See docs/firmware-plan.md "Confirmed Hardware Interfaces". */
                .nominal_resistance_ohm = 100000.0f,
                .nominal_temperature_c = 25.0f,
                .beta = 3950.0f,
            },
        .oversample_count = 8,
    };
    ESP_ERROR_CHECK(plate_ntc_init(&ntc_config));

    relay_output_config_t relay_config = {
        .gpio_top_heater = PIN_TOP_HEATER,
        .gpio_bottom_heater = PIN_BOTTOM_HEATER,
    };
    /* relay_output_init() forces both relays off immediately (fail-safe default-off). */
    ESP_ERROR_CHECK(relay_output_init(&relay_config));
}

/* High-priority safety/control task: reads sensors, advances the profile
 * state machine, evaluates safety, computes heater demand, and drives the
 * relay time-proportional windows. Combines the "Safety/control task" and
 * "Profile task" roles from docs/firmware-plan.md, per the plan's note that
 * these may be combined initially while preserving the same separation of
 * concerns in code (profile_engine vs. safety vs. heater_control modules).
 */
static void control_task(void *arg)
{
    (void)arg;

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS);

    int64_t last_tick_us = esp_timer_get_time();
    uint32_t window_elapsed_ms = 0;

    uint32_t last_phase_index = 0;
    float previous_phase_target_c = 0.0f;
    int have_last_phase = 0;

    const safety_limits_t safety_limits = {
        .chamber_absolute_max_c = s_config.chamber_absolute_max_c,
        .plate_absolute_max_c = s_config.plate_absolute_max_c,
        .control_watchdog_timeout_ms = s_config.control_watchdog_timeout_ms,
    };

    for (;;) {
        vTaskDelayUntil(&last_wake_time, period_ticks);

        int64_t now_us = esp_timer_get_time();
        uint32_t ms_since_last = (uint32_t)((now_us - last_tick_us) / 1000);
        last_tick_us = now_us;

        thermocouple_max31855_reading_t tc_reading;
        memset(&tc_reading, 0, sizeof(tc_reading));
        esp_err_t tc_err = thermocouple_max31855_read(&tc_reading);

        float plate_temp_c = 0.0f;
        int plate_implausible = 0;
        esp_err_t ntc_err = plate_ntc_read(&plate_temp_c, &plate_implausible);

        profile_engine_tick(&s_profile_engine, (float)CONTROL_TASK_PERIOD_MS / 1000.0f);

        const profile_phase_t *phase = profile_engine_current_phase(&s_profile_engine);

        if (!have_last_phase || s_profile_engine.current_phase_index != last_phase_index) {
            previous_phase_target_c = (phase != NULL) ? tc_reading.chamber_temperature_c : 0.0f;
            last_phase_index = s_profile_engine.current_phase_index;
            have_last_phase = 1;
        }

        safety_inputs_t inputs;
        memset(&inputs, 0, sizeof(inputs));
        inputs.thermocouple_fault_bit = (tc_err != ESP_OK) || tc_reading.fault;
        inputs.thermocouple_open_or_short =
            tc_reading.fault_open_circuit || tc_reading.fault_short_to_gnd || tc_reading.fault_short_to_vcc;
        inputs.plate_adc_implausible = (ntc_err != ESP_OK) || plate_implausible;
        inputs.chamber_temperature_c = tc_reading.chamber_temperature_c;
        inputs.plate_temperature_c = plate_temp_c;
        inputs.ms_since_last_control_tick = ms_since_last;
        inputs.system_is_idle_or_fault = (s_profile_engine.state == PROFILE_ENGINE_STATE_IDLE ||
                                           s_profile_engine.state == PROFILE_ENGINE_STATE_FAULT);
        inputs.relay_commanded_on = 0; /* Updated below once demand is known. */
        inputs.active_profile_valid = (s_profile_engine.active_profile != NULL) &&
                                       profile_engine_validate(s_profile_engine.active_profile);

        uint32_t faults = safety_evaluate(&inputs, &safety_limits);

        float top_power_percent = 0.0f;
        float bottom_power_percent = 0.0f;

        if (faults != SAFETY_FAULT_NONE || phase == NULL) {
            relay_output_force_all_off();
            heater_control_reset(&s_top_heater_ctrl);
            heater_control_reset(&s_bottom_heater_ctrl);
            if (faults != SAFETY_FAULT_NONE) {
                profile_engine_force_fault(&s_profile_engine);
                ESP_LOGE(TAG, "Safety fault detected: 0x%02" PRIx32, faults);
            }
        } else {
            float target_c = profile_engine_current_target_c(&s_profile_engine, previous_phase_target_c);
            float dt_s = (float)CONTROL_TASK_PERIOD_MS / 1000.0f;

            if (phase->top_heater_enabled) {
                top_power_percent = heater_control_update(&s_top_heater_ctrl, tc_reading.chamber_temperature_c,
                                                            target_c, dt_s);
            } else {
                heater_control_reset(&s_top_heater_ctrl);
            }

            if (phase->bottom_heater_mode == BOTTOM_HEATER_MODE_OFF) {
                heater_control_reset(&s_bottom_heater_ctrl);
            } else {
                float bottom_target_c = phase->bottom_plate_limit_c;
                if (phase->bottom_heater_mode == BOTTOM_HEATER_MODE_LIMITED) {
                    bottom_target_c = (bottom_target_c > s_config.bottom_plate_ceiling_c)
                                           ? s_config.bottom_plate_ceiling_c
                                           : bottom_target_c;
                }
                bottom_power_percent =
                    heater_control_update(&s_bottom_heater_ctrl, plate_temp_c, bottom_target_c, dt_s);
            }

            uint32_t top_on_ms = relay_timing_on_ms(top_power_percent, s_config.relay_window_ms);
            uint32_t bottom_on_ms = relay_timing_on_ms(bottom_power_percent, s_config.relay_window_ms);

            int top_on = relay_timing_should_energize(window_elapsed_ms, top_on_ms);
            int bottom_on = relay_timing_should_energize(window_elapsed_ms, bottom_on_ms);

            relay_output_set(RELAY_OUTPUT_TOP_HEATER, top_on);
            relay_output_set(RELAY_OUTPUT_BOTTOM_HEATER, bottom_on);
        }

        window_elapsed_ms += CONTROL_TASK_PERIOD_MS;
        if (window_elapsed_ms >= s_config.relay_window_ms) {
            window_elapsed_ms = 0;
        }

        web_api_status_t status = {
            .chamber_temperature_c = tc_reading.chamber_temperature_c,
            .plate_temperature_c = plate_temp_c,
            .top_heater_power_percent = top_power_percent,
            .bottom_heater_power_percent = bottom_power_percent,
            .profile_state = s_profile_engine.state,
            .current_phase_index = s_profile_engine.current_phase_index,
            .safety_fault_flags = faults,
        };
        publish_status_snapshot(&status);
    }
}

/* Lower-priority telemetry task: forwards the latest status snapshot to the
 * web/API layer without touching sensors or relays directly. */
static void telemetry_task(void *arg)
{
    (void)arg;
    const TickType_t period_ticks = pdMS_TO_TICKS(TELEMETRY_TASK_PERIOD_MS);

    for (;;) {
        web_api_status_t status;
        if (xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            status = s_latest_status;
            xSemaphoreGive(s_status_mutex);
            web_api_publish_status(&status);
        }
        vTaskDelay(period_ticks);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(config_store_init());
    ESP_ERROR_CHECK(config_store_load(&s_config));

    init_hardware();

    heater_control_init(&s_top_heater_ctrl, HEATER_CONTROL_MODE_BANG_BANG, &s_config.top_heater_config);
    heater_control_init(&s_bottom_heater_ctrl, HEATER_CONTROL_MODE_BANG_BANG, &s_config.bottom_heater_config);

    if (profile_engine_load(&s_profile_engine, built_in_profile_get()) != 0) {
        ESP_LOGE(TAG, "Built-in profile failed validation; refusing to start");
        return;
    }
    profile_engine_start(&s_profile_engine);

    s_status_mutex = xSemaphoreCreateMutex();
    memset(&s_latest_status, 0, sizeof(s_latest_status));

    /* Wi-Fi/web failures must never block or affect heater control: connect
     * best-effort and continue regardless of the outcome. */
    if (wifi_init_station_and_wait() == ESP_OK) {
        if (web_api_start() != ESP_OK) {
            ESP_LOGW(TAG, "web_api_start failed; continuing without web API");
        }
    } else {
        ESP_LOGW(TAG, "Continuing without Wi-Fi/web API; control loop is unaffected");
    }

    xTaskCreate(control_task, "control_task", 4096, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(telemetry_task, "telemetry_task", 3072, NULL, tskIDLE_PRIORITY + 3, NULL);
}
