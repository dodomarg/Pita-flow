/**
 * Wio Terminal front-panel firmware for Pita-flow.
 *
 * This target reuses the hardware-independent `core` component
 * (firmware/components/core) unmodified: profile engine, safety
 * evaluation, NTC math, relay time-proportional timing, heater control,
 * YAML profile catalog parsing, and local UI (button/notification) state.
 *
 * Everything below this line is Wio Terminal-specific glue: reading the
 * external MAX31855 thermocouple and plate NTC divider, driving the two
 * heater relay outputs, and driving the built-in screen/buttons/speaker/
 * microSD slot. See docs/firmware-plan.md "Platform" and "Confirmed
 * Hardware Interfaces".
 *
 * NOTE: the external MAX31855/NTC/relay pin assignments below are
 * bring-up placeholders on the Wio Terminal's 40-pin (Raspberry Pi
 * compatible) header, consistent with docs/firmware-plan.md's note that
 * "the exact heater/sensor pin mapping ... should be finalized during
 * board bring-up" -- adjust PIN_* below to match the actual add-on board.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Seeed_Arduino_FS.h>
#include <TFT_eSPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C" {
#include "core/built_in_profile.h"
#include "core/heater_control.h"
#include "core/ntc_convert.h"
#include "core/profile_catalog.h"
#include "core/profile_engine.h"
#include "core/relay_timing.h"
#include "core/safety.h"
#include "core/thermal_runaway.h"
#include "core/ui_local.h"
}

/* ---- External hardware pin assignments (Wio Terminal 40-pin header) ---- */
#define PIN_MAX31855_CLK BCM11
#define PIN_MAX31855_MISO BCM9
#define PIN_MAX31855_CS BCM8
#define PIN_TOP_HEATER_RELAY BCM23
#define PIN_BOTTOM_HEATER_RELAY BCM24
#define PIN_PLATE_NTC_ADC RPI_A0

#define ADC_RESOLUTION_BITS 12
#define ADC_MAX ((1u << ADC_RESOLUTION_BITS) - 1u)
#define NTC_OVERSAMPLE_COUNT 8

#define CONTROL_TICK_MS 100
#define RELAY_WINDOW_MS 1000
#define UI_POLL_MS 30
#define RENDER_MS 100

/* The thermal-runaway guard only arms while the temperature is at least this
 * far below its setpoint, so steady-state "hold" (heater cycling with little
 * rise) is never mistaken for a stalled/broken heater. */
#define RUNAWAY_ARM_MARGIN_C 5.0f

/* Runtime-adjustable configuration (editable via the on-screen Settings page
 * and persisted to /settings.cfg on the microSD card). Seeded from the
 * bring-up defaults in docs/firmware-plan.md "Confirmed Hardware Interfaces". */
typedef struct {
    float chamber_absolute_max_c;
    float plate_absolute_max_c;
    float bottom_plate_ceiling_c;
    float top_hysteresis_c;
    float bottom_hysteresis_c;
    uint32_t control_watchdog_timeout_ms;
    float runaway_min_rise_c;
    uint32_t runaway_window_ms;
} app_settings_t;

static app_settings_t s_settings = {
    .chamber_absolute_max_c = 260.0f,
    .plate_absolute_max_c = 200.0f,
    .bottom_plate_ceiling_c = 150.0f,
    .top_hysteresis_c = 2.0f,
    .bottom_hysteresis_c = 2.0f,
    .control_watchdog_timeout_ms = 2000,
    .runaway_min_rise_c = 2.0f,
    .runaway_window_ms = 30000,
};

static const ntc_params_t kPlateNtcParams = {
    .pulldown_resistor_ohm = 4700.0f,
    .nominal_resistance_ohm = 100000.0f,
    .nominal_temperature_c = 25.0f,
    .beta = 3950.0f,
};

static const heater_control_config_t kTopHeaterConfig = {
    .hysteresis_c = 2.0f,
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .min_power_percent = 0.0f,
    .max_power_percent = 100.0f,
};

static const heater_control_config_t kBottomHeaterConfig = {
    .hysteresis_c = 2.0f,
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .min_power_percent = 0.0f,
    .max_power_percent = 100.0f,
};

static TFT_eSPI tft;
/* Off-screen framebuffer used for flicker-free, tear-free updates: the whole
 * frame is drawn into this sprite and composited to the panel in a single
 * pushSprite(), so the screen is never cleared while it is being shown. */
static TFT_eSprite s_canvas = TFT_eSprite(&tft);

static reflow_profile_catalog_t s_catalog;
static profile_engine_t s_profile_engine;
static heater_control_t s_top_heater_ctrl;
static heater_control_t s_bottom_heater_ctrl;
static thermal_runaway_monitor_t s_top_runaway;
static thermal_runaway_monitor_t s_bottom_runaway;
static ui_local_controller_t s_ui_controller;

static uint32_t s_window_elapsed_ms = 0;
static uint32_t s_last_control_tick_ms = 0;
static uint32_t s_last_ui_poll_ms = 0;
static uint32_t s_last_phase_index = 0;
static float s_previous_phase_target_c = 0.0f;
static int s_have_last_phase = 0;

/* Rolling capture of the measured chamber temperature during a run, plotted as
 * the "actual" curve on the reflow graph alongside the target profile. Samples
 * are spaced evenly across the run so the whole cycle fits the graph width. */
#define TRACE_MAX_POINTS 240
static float s_trace_temp[TRACE_MAX_POINTS];
static int s_trace_count = 0;
static float s_trace_interval_s = 1.0f;
static uint32_t s_trace_accum_ms = 0;

static void reset_temperature_trace(void)
{
    s_trace_count = 0;
    s_trace_accum_ms = 0;
}

/* Called once per control tick. Starts a fresh trace when a run begins and
 * appends evenly time-spaced samples of the measured chamber temperature. */
static void update_temperature_trace(float chamber_c)
{
    static profile_engine_state_t prev_state = PROFILE_ENGINE_STATE_IDLE;
    profile_engine_state_t state = s_profile_engine.state;

    if (state == PROFILE_ENGINE_STATE_RUNNING && prev_state != PROFILE_ENGINE_STATE_RUNNING) {
        reset_temperature_trace();
        uint32_t total_s = 0;
        if (s_profile_engine.active_profile != NULL) {
            for (uint32_t i = 0; i < s_profile_engine.active_profile->phase_count; i++) {
                total_s += s_profile_engine.active_profile->phases[i].duration_s;
            }
        }
        if (total_s == 0) {
            total_s = TRACE_MAX_POINTS;
        }
        s_trace_interval_s = (float)total_s / (float)TRACE_MAX_POINTS;
        if (s_trace_interval_s < 0.1f) {
            s_trace_interval_s = 0.1f;
        }
        s_trace_temp[s_trace_count++] = chamber_c;
        s_trace_accum_ms = 0;
    } else if (state == PROFILE_ENGINE_STATE_RUNNING) {
        s_trace_accum_ms += CONTROL_TICK_MS;
        if (s_trace_count < TRACE_MAX_POINTS &&
            s_trace_accum_ms >= (uint32_t)(s_trace_interval_s * 1000.0f)) {
            s_trace_accum_ms = 0;
            s_trace_temp[s_trace_count++] = chamber_c;
        }
    }

    prev_state = state;
}

typedef struct {
    float chamber_temperature_c;
    int present; /* MAX31855 responded with a structurally valid frame */
    int fault;
    int fault_open_circuit;
    int fault_short_to_gnd;
    int fault_short_to_vcc;
} thermocouple_reading_t;

/** Reads the MAX31855 over a manual (read-only) SPI transaction. */
static int thermocouple_max31855_read(thermocouple_reading_t *out_reading)
{
    memset(out_reading, 0, sizeof(*out_reading));

    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_MAX31855_CS, LOW);
    uint32_t raw = 0;
    for (int i = 0; i < 4; i++) {
        raw = (raw << 8) | SPI.transfer(0x00);
    }
    digitalWrite(PIN_MAX31855_CS, HIGH);
    SPI.endTransaction();

    /* A genuine MAX31855 frame always has reserved bits D17 and D3 cleared and
     * is never all-zeros or all-ones. When no converter is connected, MISO
     * floats and reads back as one of those patterns; we flag that as
     * "sensor not present" so the system fails safe instead of trusting a
     * fabricated 0 C reading (critical: this is the loss-of-sensor guard). */
    int present = !(raw == 0x00000000UL || raw == 0xFFFFFFFFUL ||
                    (raw & (1UL << 17)) != 0 || (raw & (1UL << 3)) != 0);
    out_reading->present = present;

    int16_t raw_temp = (int16_t)(raw >> 18);
    if (raw_temp & 0x2000) {
        raw_temp |= 0xC000; /* sign-extend 14-bit value */
    }
    out_reading->chamber_temperature_c = raw_temp * 0.25f;

    out_reading->fault = (raw & (1UL << 16)) != 0;
    out_reading->fault_open_circuit = (raw & (1UL << 0)) != 0;
    out_reading->fault_short_to_gnd = (raw & (1UL << 1)) != 0;
    out_reading->fault_short_to_vcc = (raw & (1UL << 2)) != 0;

    /* Absent converter or a converter-reported thermocouple fault are both
     * unsafe to control on. */
    return (!present || out_reading->fault) ? -1 : 0;
}

/** Reads and converts the plate NTC divider, oversampling to reduce noise. */
static int plate_ntc_read(float *out_temperature_c, int *out_implausible)
{
    uint32_t sum = 0;
    for (int i = 0; i < NTC_OVERSAMPLE_COUNT; i++) {
        sum += analogRead(PIN_PLATE_NTC_ADC);
    }
    uint32_t raw = sum / NTC_OVERSAMPLE_COUNT;

    *out_implausible = ntc_reading_is_implausible(raw, ADC_MAX, 8);
    return ntc_convert_adc_to_celsius(raw, ADC_MAX, &kPlateNtcParams, out_temperature_c);
}

static void relay_output_set(bool top_on, bool bottom_on)
{
    digitalWrite(PIN_TOP_HEATER_RELAY, top_on ? HIGH : LOW);
    digitalWrite(PIN_BOTTOM_HEATER_RELAY, bottom_on ? HIGH : LOW);
}

static void relay_output_force_all_off(void)
{
    relay_output_set(false, false);
}

/** Loads firmware/profiles/reflow-profiles.yaml from the microSD card into
 * the profile catalog. Falls back to the built-in profile on any failure
 * (missing card, missing file, or parse error). */
static void load_profile_catalog(void)
{
    memset(&s_catalog, 0, sizeof(s_catalog));

    File file = SD.open("/reflow-profiles.yaml");
    if (file) {
        static char yaml_buffer[8192];
        size_t len = file.readBytes(yaml_buffer, sizeof(yaml_buffer) - 1);
        yaml_buffer[len] = '\0';
        file.close();

        if (profile_catalog_parse_yaml(yaml_buffer, &s_catalog) == 0 && s_catalog.profile_count > 0) {
            Serial.println("Loaded profile catalog from microSD");
            return;
        }
        Serial.println("microSD profile catalog parse failed; using built-in profile");
    } else {
        Serial.println("No microSD profile catalog found; using built-in profile");
    }

    memset(&s_catalog, 0, sizeof(s_catalog));
    s_catalog.profiles[0] = *built_in_profile_get();
    s_catalog.profile_count = 1;
}

static void play_notification_tone(ui_local_notification_t notification)
{
    switch (notification) {
    case UI_LOCAL_NOTIFICATION_PROFILE_CHANGED:
        tone(WIO_BUZZER, 1200, 60);
        break;
    case UI_LOCAL_NOTIFICATION_RUN_STARTED:
        tone(WIO_BUZZER, 1800, 120);
        break;
    case UI_LOCAL_NOTIFICATION_RUN_STOPPED:
        tone(WIO_BUZZER, 600, 120);
        break;
    case UI_LOCAL_NOTIFICATION_PHASE_CHANGED:
        tone(WIO_BUZZER, 1500, 80);
        break;
    case UI_LOCAL_NOTIFICATION_COMPLETE:
        tone(WIO_BUZZER, 2000, 300);
        break;
    case UI_LOCAL_NOTIFICATION_FAULT:
    case UI_LOCAL_NOTIFICATION_PROFILE_LOAD_FAILED:
        tone(WIO_BUZZER, 300, 500);
        break;
    case UI_LOCAL_NOTIFICATION_NONE:
    default:
        break;
    }
}

/* ------------------------------------------------------------------ *
 *  Polished, Miniware-style front panel UI (rendered into s_canvas).
 * ------------------------------------------------------------------ */

/* 16-bit RGB565 helper so the palette can be written in plain 0-255 RGB. */
static constexpr uint16_t rgb(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

/* Dark instrument theme with an amber "heat" accent. */
static const uint16_t COL_BG = rgb(8, 9, 12);
static const uint16_t COL_PANEL = rgb(22, 24, 30);
static const uint16_t COL_PANEL_EDGE = rgb(44, 48, 58);
static const uint16_t COL_GRID = rgb(38, 41, 50);
static const uint16_t COL_TEXT = rgb(236, 239, 245);
static const uint16_t COL_MUTED = rgb(126, 134, 148);
static const uint16_t COL_ACCENT = rgb(255, 140, 0);   /* amber - chamber/top heat */
static const uint16_t COL_BOTTOM = rgb(0, 190, 255);   /* cyan  - bottom plate heat */
static const uint16_t COL_OK = rgb(0, 210, 120);       /* green - running */
static const uint16_t COL_DONE = rgb(0, 170, 255);     /* blue  - complete */
static const uint16_t COL_FAULT = rgb(255, 70, 70);    /* red   - fault */

static const char *engine_state_label(profile_engine_state_t state)
{
    switch (state) {
    case PROFILE_ENGINE_STATE_RUNNING:
        return "RUNNING";
    case PROFILE_ENGINE_STATE_COMPLETE:
        return "DONE";
    case PROFILE_ENGINE_STATE_FAULT:
        return "FAULT";
    case PROFILE_ENGINE_STATE_IDLE:
    default:
        return "READY";
    }
}

static uint16_t engine_state_color(profile_engine_state_t state)
{
    switch (state) {
    case PROFILE_ENGINE_STATE_RUNNING:
        return COL_OK;
    case PROFILE_ENGINE_STATE_COMPLETE:
        return COL_DONE;
    case PROFILE_ENGINE_STATE_FAULT:
        return COL_FAULT;
    case PROFILE_ENGINE_STATE_IDLE:
    default:
        return COL_MUTED;
    }
}

static const char *primary_fault_name(uint32_t faults)
{
    if (faults & SAFETY_FAULT_THERMAL_RUNAWAY) return "THERMAL RUNAWAY";
    if (faults & SAFETY_FAULT_THERMOCOUPLE_OPEN_SHORT) return "THERMOCOUPLE OPEN/SHORT";
    if (faults & SAFETY_FAULT_THERMOCOUPLE_FAULT_BIT) return "THERMOCOUPLE FAULT";
    if (faults & SAFETY_FAULT_CHAMBER_OVER_TEMP) return "CHAMBER OVER-TEMP";
    if (faults & SAFETY_FAULT_PLATE_OVER_TEMP) return "PLATE OVER-TEMP";
    if (faults & SAFETY_FAULT_PLATE_ADC_IMPLAUSIBLE) return "PLATE SENSOR FAULT";
    if (faults & SAFETY_FAULT_CONTROL_WATCHDOG_TIMEOUT) return "CONTROL WATCHDOG";
    if (faults & SAFETY_FAULT_RELAY_COMMANDED_WHILE_IDLE) return "RELAY STATE FAULT";
    if (faults & SAFETY_FAULT_INVALID_PROFILE) return "INVALID PROFILE";
    return "SAFETY FAULT";
}

/* Draws a small degree ring followed by the unit letter, instrument style. */
static int draw_degree_unit(int x, int y_top, uint8_t font, char unit, uint16_t color)
{
    s_canvas.drawCircle(x + 3, y_top + 3, 3, color);
    s_canvas.drawCircle(x + 3, y_top + 3, 2, color);
    char u[2] = {unit, '\0'};
    s_canvas.setTextDatum(TL_DATUM);
    s_canvas.setTextColor(color, COL_BG);
    return s_canvas.drawString(u, x + 9, y_top, font);
}

/* Rounded "pill" badge with centered text (state indicator, tags). */
static void draw_pill(int x, int y, int w, int h, const char *label, uint16_t fill, uint16_t text_col)
{
    s_canvas.fillRoundRect(x, y, w, h, h / 2, fill);
    s_canvas.setTextDatum(MC_DATUM);
    s_canvas.setTextColor(text_col, fill);
    s_canvas.drawString(label, x + w / 2, y + h / 2 + 1, 2);
}

/* Horizontal power meter: label, rounded track, filled portion, percentage. */
static void draw_power_bar(int x, int y, int w, int h, const char *label, float percent, uint16_t color)
{
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    s_canvas.setTextDatum(ML_DATUM);
    s_canvas.setTextColor(COL_MUTED, COL_BG);
    s_canvas.drawString(label, x, y + h / 2, 2);

    const int track_x = x + 42;
    const int track_w = w - 42 - 52;
    s_canvas.fillRoundRect(track_x, y, track_w, h, h / 2, COL_PANEL);
    s_canvas.drawRoundRect(track_x, y, track_w, h, h / 2, COL_PANEL_EDGE);

    int fill_w = (int)((track_w - 2) * (percent / 100.0f) + 0.5f);
    if (fill_w > 0) {
        if (fill_w < h) fill_w = h; /* keep the rounded cap readable at low duty */
        s_canvas.fillRoundRect(track_x + 1, y + 1, fill_w, h - 2, (h - 2) / 2, color);
    }

    char pct[8];
    snprintf(pct, sizeof(pct), "%d%%", (int)(percent + 0.5f));
    s_canvas.setTextDatum(MR_DATUM);
    s_canvas.setTextColor(COL_TEXT, COL_BG);
    s_canvas.drawString(pct, x + w, y + h / 2, 2);
}

/* Live reflow curve: target profile polyline, grid, run progress marker, and
 * the current chamber temperature plotted against the setpoint curve. */
static void draw_reflow_curve(int gx, int gy, int gw, int gh, const profile_engine_t *engine, float chamber_c)
{
    s_canvas.fillRoundRect(gx, gy, gw, gh, 6, COL_PANEL);
    s_canvas.drawRoundRect(gx, gy, gw, gh, 6, COL_PANEL_EDGE);

    const int pad = 6;
    const int px = gx + pad;
    const int py = gy + pad;
    const int pw = gw - 2 * pad;
    const int ph = gh - 2 * pad;

    /* Light horizontal grid. */
    for (int i = 1; i < 3; i++) {
        int yy = py + (ph * i) / 3;
        s_canvas.drawFastHLine(px, yy, pw, COL_GRID);
    }

    const reflow_profile_t *profile = (engine != NULL) ? engine->active_profile : NULL;
    if (profile == NULL || profile->phase_count == 0) {
        return;
    }

    uint32_t total_s = 0;
    float max_temp = 1.0f;
    for (uint32_t i = 0; i < profile->phase_count; i++) {
        total_s += profile->phases[i].duration_s;
        if (profile->phases[i].target_chamber_c > max_temp) {
            max_temp = profile->phases[i].target_chamber_c;
        }
    }
    if (chamber_c > max_temp) max_temp = chamber_c;
    for (int i = 0; i < s_trace_count; i++) {
        if (s_trace_temp[i] > max_temp) max_temp = s_trace_temp[i];
    }
    const float temp_span = max_temp * 1.12f + 1.0f; /* headroom above the peak */
    if (total_s == 0) return;

    /* Map (time_s, temp_c) into the plot rectangle. */
    auto map_x = [&](float t_s) -> int { return px + (int)((pw - 1) * (t_s / (float)total_s)); };
    auto map_y = [&](float temp_c) -> int {
        float f = temp_c / temp_span;
        if (f < 0.0f) f = 0.0f;
        if (f > 1.0f) f = 1.0f;
        return py + ph - 1 - (int)((ph - 1) * f);
    };

    /* Target setpoint polyline (starts near ambient, steps through phases). */
    float t_accum = 0.0f;
    int prev_x = map_x(0.0f);
    int prev_y = map_y(25.0f);
    for (uint32_t i = 0; i < profile->phase_count; i++) {
        t_accum += (float)profile->phases[i].duration_s;
        int cx = map_x(t_accum);
        int cy = map_y(profile->phases[i].target_chamber_c);
        s_canvas.drawLine(prev_x, prev_y, cx, cy, COL_MUTED);
        s_canvas.drawLine(prev_x, prev_y + 1, cx, cy + 1, COL_MUTED);
        prev_x = cx;
        prev_y = cy;
    }

    /* Measured (actual) chamber temperature captured during the run, drawn as
     * the amber curve tracking the grey target profile. */
    if (s_trace_count > 1) {
        int ax = map_x(0.0f);
        int ay = map_y(s_trace_temp[0]);
        for (int i = 1; i < s_trace_count; i++) {
            float ts = (float)i * s_trace_interval_s;
            if (ts > (float)total_s) ts = (float)total_s;
            int cx = map_x(ts);
            int cy = map_y(s_trace_temp[i]);
            s_canvas.drawLine(ax, ay, cx, cy, COL_ACCENT);
            s_canvas.drawLine(ax, ay + 1, cx, cy + 1, COL_ACCENT);
            ax = cx;
            ay = cy;
        }
    }

    /* Run progress: elapsed time across completed phases plus the current one. */
    if (engine->state == PROFILE_ENGINE_STATE_RUNNING || engine->state == PROFILE_ENGINE_STATE_COMPLETE) {
        float elapsed_s = engine->elapsed_in_phase_s;
        for (uint32_t i = 0; i < engine->current_phase_index && i < profile->phase_count; i++) {
            elapsed_s += (float)profile->phases[i].duration_s;
        }
        if (elapsed_s > (float)total_s) elapsed_s = (float)total_s;

        int mx = map_x(elapsed_s);
        s_canvas.drawFastVLine(mx, py, ph, COL_PANEL_EDGE);

        int cy = map_y(chamber_c);
        s_canvas.fillCircle(mx, cy, 3, COL_ACCENT);
        s_canvas.drawCircle(mx, cy, 3, COL_TEXT);
    }
}

static void format_mmss(uint32_t seconds, char *out, size_t out_len)
{
    snprintf(out, out_len, "%lu:%02lu", (unsigned long)(seconds / 60), (unsigned long)(seconds % 60));
}

static void render_status(const ui_local_status_t *status, const profile_engine_t *engine, float chamber_c,
                           float plate_c, float target_c, float top_power_percent, float bottom_power_percent,
                           uint32_t faults, bool chamber_valid, bool plate_valid)
{
    /* Draw the whole frame off-screen, then blit it in one shot. Clearing and
     * redrawing happen in the hidden buffer, so the visible panel updates
     * atomically with no flash or scan-line tearing. */
    s_canvas.fillSprite(COL_BG);

    const int W = s_canvas.width();
    const bool fault = (faults != SAFETY_FAULT_NONE);

    /* ---- Header: profile name + state pill --------------------------- */
    s_canvas.setTextDatum(TL_DATUM);
    s_canvas.setTextColor(COL_TEXT, COL_BG);
    s_canvas.drawString(status->selected_profile_name ? status->selected_profile_name : "(no profile)", 8, 6, 2);

    const char *state_text = fault ? "FAULT" : engine_state_label(status->engine_state);
    const uint16_t state_col = fault ? COL_FAULT : engine_state_color(status->engine_state);
    draw_pill(W - 92, 4, 84, 20, state_text, state_col, COL_BG);

    s_canvas.drawFastHLine(0, 28, W, COL_PANEL_EDGE);

    /* ---- Hero: large chamber temperature ----------------------------- */
    s_canvas.setTextDatum(TL_DATUM);
    s_canvas.setTextColor(COL_MUTED, COL_BG);
    int nx = 10;
    int ny = 36;
    s_canvas.drawString("CHAMBER", nx + 2, ny - 12, 1);

    if (chamber_valid) {
        char num[8];
        snprintf(num, sizeof(num), "%d", (int)(chamber_c + 0.5f));
        s_canvas.setTextDatum(TL_DATUM);
        s_canvas.setTextColor(fault ? COL_FAULT : COL_ACCENT, COL_BG);
        int after = nx + s_canvas.drawString(num, nx, ny, 7); /* Font 7: 48px 7-segment */
        draw_degree_unit(after + 4, ny + 2, 4, 'C', COL_MUTED);
    } else {
        /* No converter / thermocouple fault: never display a fabricated value. */
        s_canvas.setTextDatum(TL_DATUM);
        s_canvas.setTextColor(COL_FAULT, COL_BG);
        s_canvas.drawString("Err", nx, ny + 8, 4);
        s_canvas.setTextColor(COL_MUTED, COL_BG);
        s_canvas.drawString("no sensor", nx + 78, ny + 16, 2);
    }

    /* Right column: target setpoint + active phase. */
    const int rx = 180;
    s_canvas.setTextColor(COL_MUTED, COL_BG);
    s_canvas.drawString("TARGET", rx, 34, 1);
    if (status->engine_state == PROFILE_ENGINE_STATE_RUNNING && target_c > 0.0f) {
        char tnum[8];
        snprintf(tnum, sizeof(tnum), "%d", (int)(target_c + 0.5f));
        s_canvas.setTextColor(COL_TEXT, COL_BG);
        int tx = rx + s_canvas.drawString(tnum, rx, 46, 4);
        draw_degree_unit(tx + 3, 48, 2, 'C', COL_MUTED);
    } else {
        s_canvas.setTextColor(COL_TEXT, COL_BG);
        s_canvas.drawString("--", rx, 46, 4);
    }

    s_canvas.setTextColor(COL_MUTED, COL_BG);
    s_canvas.drawString("PHASE", rx, 76, 1);
    s_canvas.setTextColor(COL_TEXT, COL_BG);
    s_canvas.drawString(status->active_phase_name ? status->active_phase_name : "-", rx, 88, 2);

    /* ---- Reflow curve ------------------------------------------------ */
    draw_reflow_curve(8, 110, W - 16, 62, engine, chamber_c);

    /* ---- Footer: heater power bars + plate temp / elapsed time -------- */
    draw_power_bar(8, 180, W - 16, 16, "TOP", top_power_percent, COL_ACCENT);
    draw_power_bar(8, 200, W - 16, 16, "BOT", bottom_power_percent, COL_BOTTOM);

    char plate_str[20];
    s_canvas.setTextDatum(ML_DATUM);
    if (plate_valid) {
        snprintf(plate_str, sizeof(plate_str), "PLATE %d C", (int)(plate_c + 0.5f));
        s_canvas.setTextColor(COL_MUTED, COL_BG);
        s_canvas.drawString(plate_str, 8, 228, 2);
    } else {
        s_canvas.setTextColor(COL_FAULT, COL_BG);
        s_canvas.drawString("PLATE Err", 8, 228, 2);
    }

    if (fault) {
        /* Fault banner takes over the footer-right for maximum legibility. */
        s_canvas.setTextDatum(MR_DATUM);
        s_canvas.setTextColor(COL_FAULT, COL_BG);
        s_canvas.drawString(primary_fault_name(faults), W - 8, 228, 2);
    } else if (engine != NULL && engine->active_profile != NULL &&
               (status->engine_state == PROFILE_ENGINE_STATE_RUNNING ||
                status->engine_state == PROFILE_ENGINE_STATE_COMPLETE)) {
        uint32_t total_s = 0;
        for (uint32_t i = 0; i < engine->active_profile->phase_count; i++) {
            total_s += engine->active_profile->phases[i].duration_s;
        }
        uint32_t elapsed_s = (uint32_t)engine->elapsed_in_phase_s;
        for (uint32_t i = 0; i < engine->current_phase_index && i < engine->active_profile->phase_count; i++) {
            elapsed_s += engine->active_profile->phases[i].duration_s;
        }
        if (elapsed_s > total_s) elapsed_s = total_s;
        char e[12], t[12], line[32];
        format_mmss(elapsed_s, e, sizeof(e));
        format_mmss(total_s, t, sizeof(t));
        snprintf(line, sizeof(line), "%s / %s", e, t);
        s_canvas.setTextDatum(MR_DATUM);
        s_canvas.setTextColor(COL_TEXT, COL_BG);
        s_canvas.drawString(line, W - 8, 228, 2);
    }

    s_canvas.pushSprite(0, 0);
}

/* ================================================================== *
 *  Interactive front panel: 5-way joystick navigation, Settings page,
 *  and on-device profile create/edit/save. Rendered into s_canvas.
 * ================================================================== */

/* Latest control-loop values, published by control_tick() for the renderer. */
static float s_disp_chamber_c = 0.0f;
static float s_disp_plate_c = 0.0f;
static float s_disp_target_c = 0.0f;
static float s_disp_top_power = 0.0f;
static float s_disp_bottom_power = 0.0f;
static uint32_t s_disp_faults = 0;
static int s_disp_chamber_valid = 0;
static int s_disp_plate_valid = 0;
static uint32_t s_last_render_ms = 0;

typedef enum {
    SCREEN_HOME = 0,
    SCREEN_MENU,
    SCREEN_PROFILE_LIST,
    SCREEN_SETTINGS,
    SCREEN_PROFILE_EDIT,
    SCREEN_NAME_EDIT,
    SCREEN_PINOUT,
} screen_t;

typedef enum {
    EV_NONE = 0,
    EV_UP,
    EV_DOWN,
    EV_LEFT,
    EV_RIGHT,
    EV_SELECT,
    EV_BACK,
    EV_RUN,
} input_event_t;

static screen_t s_screen = SCREEN_HOME;
static int s_menu_index = 0;
static int s_list_index = 0;
static int s_settings_index = 0;
static int s_edit_row = 0;
static int s_edit_scroll = 0;
static int s_edit_is_new = 0;
static int s_name_cursor = 0;
static reflow_profile_t s_edit_profile;

static const char *const kMenuItems[] = {"Run / Stop", "Select Profile", "Edit Profile", "New Profile",
                                         "Settings",   "Pinout",         "Save to SD"};
#define MENU_COUNT 7
#define SETTINGS_COUNT 8

/* Profile-editor field indices within a phase row block. */
#define EF_DURATION 0
#define EF_TARGET 1
#define EF_RAMP 2
#define EF_BMODE 3
#define EF_BLIMIT 4
#define EF_TOPEN 5
#define PHASE_FIELD_COUNT 6

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ---- microSD persistence ---------------------------------------------- */

static const char *bottom_mode_str(bottom_heater_mode_t m)
{
    switch (m) {
    case BOTTOM_HEATER_MODE_LIMITED:
        return "limited";
    case BOTTOM_HEATER_MODE_ENABLED:
        return "enabled";
    case BOTTOM_HEATER_MODE_OFF:
    default:
        return "off";
    }
}

/* Writes the whole catalog back to /reflow-profiles.yaml in the exact schema
 * accepted by profile_catalog_parse_yaml() (2-space indent steps, all six
 * phase fields present), so on-device edits reload verbatim on next boot. */
static bool save_catalog_to_sd(void)
{
    File f = SD.open("/reflow-profiles.yaml", FILE_WRITE);
    if (!f) {
        Serial.println("save catalog: open failed");
        return false;
    }
    char line[96];
    f.println("profiles:");
    for (uint32_t p = 0; p < s_catalog.profile_count; p++) {
        const reflow_profile_t *prof = &s_catalog.profiles[p];
        snprintf(line, sizeof(line), "  - name: %s", prof->name);
        f.println(line);
        f.println("    phases:");
        for (uint32_t i = 0; i < prof->phase_count; i++) {
            const profile_phase_t *ph = &prof->phases[i];
            snprintf(line, sizeof(line), "      - name: %s", ph->name);
            f.println(line);
            snprintf(line, sizeof(line), "        duration_s: %lu", (unsigned long)ph->duration_s);
            f.println(line);
            snprintf(line, sizeof(line), "        target_chamber_c: %.1f", ph->target_chamber_c);
            f.println(line);
            snprintf(line, sizeof(line), "        ramp_rate_c_per_s: %.2f", ph->ramp_rate_c_per_s);
            f.println(line);
            snprintf(line, sizeof(line), "        bottom_heater_mode: %s", bottom_mode_str(ph->bottom_heater_mode));
            f.println(line);
            snprintf(line, sizeof(line), "        bottom_plate_limit_c: %.1f", ph->bottom_plate_limit_c);
            f.println(line);
            snprintf(line, sizeof(line), "        top_heater_enabled: %s", ph->top_heater_enabled ? "true" : "false");
            f.println(line);
        }
    }
    f.close();
    return true;
}

static void save_settings_to_sd(void)
{
    File f = SD.open("/settings.cfg", FILE_WRITE);
    if (!f) {
        return;
    }
    char line[64];
    snprintf(line, sizeof(line), "chamber_max=%.1f", s_settings.chamber_absolute_max_c);
    f.println(line);
    snprintf(line, sizeof(line), "plate_max=%.1f", s_settings.plate_absolute_max_c);
    f.println(line);
    snprintf(line, sizeof(line), "plate_ceiling=%.1f", s_settings.bottom_plate_ceiling_c);
    f.println(line);
    snprintf(line, sizeof(line), "top_hyst=%.1f", s_settings.top_hysteresis_c);
    f.println(line);
    snprintf(line, sizeof(line), "bot_hyst=%.1f", s_settings.bottom_hysteresis_c);
    f.println(line);
    snprintf(line, sizeof(line), "watchdog_ms=%lu", (unsigned long)s_settings.control_watchdog_timeout_ms);
    f.println(line);
    snprintf(line, sizeof(line), "runaway_rise=%.1f", s_settings.runaway_min_rise_c);
    f.println(line);
    snprintf(line, sizeof(line), "runaway_win_ms=%lu", (unsigned long)s_settings.runaway_window_ms);
    f.println(line);
    f.close();
}

static void load_settings_from_sd(void)
{
    File f = SD.open("/settings.cfg");
    if (!f) {
        return;
    }
    static char buf[512];
    size_t n = f.readBytes(buf, sizeof(buf) - 1);
    buf[n] = '\0';
    f.close();

    for (char *line = strtok(buf, "\r\n"); line != NULL; line = strtok(NULL, "\r\n")) {
        char *eq = strchr(line, '=');
        if (eq == NULL) {
            continue;
        }
        *eq = '\0';
        const char *key = line;
        float v = strtof(eq + 1, NULL);
        if (strcmp(key, "chamber_max") == 0)
            s_settings.chamber_absolute_max_c = v;
        else if (strcmp(key, "plate_max") == 0)
            s_settings.plate_absolute_max_c = v;
        else if (strcmp(key, "plate_ceiling") == 0)
            s_settings.bottom_plate_ceiling_c = v;
        else if (strcmp(key, "top_hyst") == 0)
            s_settings.top_hysteresis_c = v;
        else if (strcmp(key, "bot_hyst") == 0)
            s_settings.bottom_hysteresis_c = v;
        else if (strcmp(key, "watchdog_ms") == 0)
            s_settings.control_watchdog_timeout_ms = (uint32_t)v;
        else if (strcmp(key, "runaway_rise") == 0)
            s_settings.runaway_min_rise_c = v;
        else if (strcmp(key, "runaway_win_ms") == 0)
            s_settings.runaway_window_ms = (uint32_t)v;
    }
}

/* Re-initialises the bang-bang heater controllers from the current hysteresis
 * settings, reusing the base gain templates for everything else. */
static void apply_heater_settings(void)
{
    heater_control_config_t top = kTopHeaterConfig;
    heater_control_config_t bot = kBottomHeaterConfig;
    top.hysteresis_c = s_settings.top_hysteresis_c;
    bot.hysteresis_c = s_settings.bottom_hysteresis_c;
    heater_control_init(&s_top_heater_ctrl, HEATER_CONTROL_MODE_BANG_BANG, &top);
    heater_control_init(&s_bottom_heater_ctrl, HEATER_CONTROL_MODE_BANG_BANG, &bot);
}

/* ---- Shared engine helpers -------------------------------------------- */

static bool engine_is_idle_or_complete(void)
{
    return s_profile_engine.state == PROFILE_ENGINE_STATE_IDLE ||
           s_profile_engine.state == PROFILE_ENGINE_STATE_COMPLETE;
}

/* Loads the currently selected catalog profile into the engine while idle so
 * the home screen graph reflects the selection before a run is started. */
static void sync_idle_engine_profile(void)
{
    if (!engine_is_idle_or_complete()) {
        return;
    }
    const reflow_profile_t *p = profile_catalog_get(&s_catalog, s_ui_controller.selected_profile_index);
    if (p != NULL) {
        profile_engine_load(&s_profile_engine, p);
        reset_temperature_trace();
    }
}

static void toggle_run(void)
{
    ui_local_notification_t n = ui_local_controller_handle_button(&s_ui_controller, UI_LOCAL_BUTTON_SELECT,
                                                                  &s_profile_engine, s_catalog.profiles);
    if (n != UI_LOCAL_NOTIFICATION_NONE) {
        play_notification_tone(n);
    }
}

/* ---- Input: 5-way joystick + top keys, with directional auto-repeat --- */

#define INPUT_REPEAT_DELAY_MS 380
#define INPUT_REPEAT_RATE_MS 110

static input_event_t poll_input_event(void)
{
    static const uint8_t pins[] = {WIO_5S_UP,   WIO_5S_DOWN, WIO_5S_LEFT, WIO_5S_RIGHT,
                                   WIO_5S_PRESS, WIO_KEY_A,   WIO_KEY_C};
    static const input_event_t evs[] = {EV_UP, EV_DOWN, EV_LEFT, EV_RIGHT, EV_SELECT, EV_BACK, EV_RUN};
    static const bool repeatable[] = {true, true, true, true, false, false, false};
    static bool held[7] = {false, false, false, false, false, false, false};
    static uint32_t next_repeat[7] = {0, 0, 0, 0, 0, 0, 0};
    const int N = 7;

    uint32_t now = millis();
    for (int i = 0; i < N; i++) {
        bool down = (digitalRead(pins[i]) == LOW);
        if (down) {
            if (!held[i]) {
                held[i] = true;
                next_repeat[i] = now + INPUT_REPEAT_DELAY_MS;
                return evs[i];
            }
            if (repeatable[i] && (int32_t)(now - next_repeat[i]) >= 0) {
                next_repeat[i] = now + INPUT_REPEAT_RATE_MS;
                return evs[i];
            }
        } else {
            held[i] = false;
        }
    }
    return EV_NONE;
}

/* ---- Profile-editor row model ----------------------------------------- */

static int edit_action_add(void) { return 1 + (int)s_edit_profile.phase_count * PHASE_FIELD_COUNT; }
static int edit_action_del(void) { return edit_action_add() + 1; }
static int edit_action_save(void) { return edit_action_add() + 2; }
static int edit_action_cancel(void) { return edit_action_add() + 3; }
static int edit_total_rows(void) { return edit_action_add() + 4; }

static void enter_editor_for_selected(bool as_new)
{
    const reflow_profile_t *base = profile_catalog_get(&s_catalog, s_ui_controller.selected_profile_index);
    if (base != NULL) {
        s_edit_profile = *base;
    } else {
        memset(&s_edit_profile, 0, sizeof(s_edit_profile));
    }

    if (as_new) {
        s_edit_is_new = 1;
        snprintf(s_edit_profile.name, sizeof(s_edit_profile.name), "custom-%u",
                 (unsigned)(s_catalog.profile_count + 1));
        if (s_edit_profile.phase_count == 0) {
            s_edit_profile.phase_count = 1;
            profile_phase_t *ph = &s_edit_profile.phases[0];
            memset(ph, 0, sizeof(*ph));
            strncpy(ph->name, "phase1", sizeof(ph->name) - 1);
            ph->duration_s = 60;
            ph->target_chamber_c = 150.0f;
            ph->ramp_rate_c_per_s = 1.0f;
            ph->bottom_heater_mode = BOTTOM_HEATER_MODE_OFF;
            ph->bottom_plate_limit_c = 0.0f;
            ph->top_heater_enabled = 1;
        }
    } else {
        s_edit_is_new = 0;
    }

    s_edit_row = 0;
    s_edit_scroll = 0;
    s_screen = SCREEN_PROFILE_EDIT;
}

static bool commit_edit_profile(void)
{
    if (!profile_engine_validate(&s_edit_profile)) {
        return false;
    }
    if (s_edit_is_new) {
        if (s_catalog.profile_count >= PROFILE_CATALOG_MAX_PROFILES) {
            return false;
        }
        s_catalog.profiles[s_catalog.profile_count] = s_edit_profile;
        s_catalog.profile_count++;
        ui_local_controller_set_profile_count(&s_ui_controller, s_catalog.profile_count);
        s_ui_controller.selected_profile_index = s_catalog.profile_count - 1;
    } else {
        s_catalog.profiles[s_ui_controller.selected_profile_index] = s_edit_profile;
    }
    sync_idle_engine_profile();
    save_catalog_to_sd();
    return true;
}

static void edit_adjust_field(int phase, int field, int dir)
{
    profile_phase_t *ph = &s_edit_profile.phases[phase];
    switch (field) {
    case EF_DURATION: {
        long v = (long)ph->duration_s + dir * 5;
        if (v < 1) v = 1;
        if (v > 3600) v = 3600;
        ph->duration_s = (uint32_t)v;
        break;
    }
    case EF_TARGET:
        ph->target_chamber_c = clampf(ph->target_chamber_c + dir * 5.0f, 0.0f, 300.0f);
        break;
    case EF_RAMP:
        ph->ramp_rate_c_per_s = clampf(ph->ramp_rate_c_per_s + dir * 0.1f, 0.0f, 10.0f);
        break;
    case EF_BMODE: {
        int m = (int)ph->bottom_heater_mode + dir;
        if (m < 0) m = 2;
        if (m > 2) m = 0;
        ph->bottom_heater_mode = (bottom_heater_mode_t)m;
        break;
    }
    case EF_BLIMIT:
        ph->bottom_plate_limit_c = clampf(ph->bottom_plate_limit_c + dir * 5.0f, 0.0f, 300.0f);
        break;
    case EF_TOPEN:
        ph->top_heater_enabled = !ph->top_heater_enabled;
        break;
    default:
        break;
    }
}

/* ---- Name editor ------------------------------------------------------ */

static const char NAME_CHARS[] = "abcdefghijklmnopqrstuvwxyz0123456789-";

static char name_cycle(char c, int dir)
{
    int n = (int)(sizeof(NAME_CHARS) - 1);
    int idx = 0;
    for (int i = 0; i < n; i++) {
        if (NAME_CHARS[i] == c) {
            idx = i;
            break;
        }
    }
    idx = (idx + dir + n) % n;
    return NAME_CHARS[idx];
}

/* ---- Per-screen event handling ---------------------------------------- */

static void ui_home_event(input_event_t ev)
{
    switch (ev) {
    case EV_LEFT: {
        ui_local_notification_t n = ui_local_controller_handle_button(
            &s_ui_controller, UI_LOCAL_BUTTON_PREVIOUS, &s_profile_engine, s_catalog.profiles);
        if (n != UI_LOCAL_NOTIFICATION_NONE) play_notification_tone(n);
        sync_idle_engine_profile();
        break;
    }
    case EV_RIGHT: {
        ui_local_notification_t n = ui_local_controller_handle_button(
            &s_ui_controller, UI_LOCAL_BUTTON_NEXT, &s_profile_engine, s_catalog.profiles);
        if (n != UI_LOCAL_NOTIFICATION_NONE) play_notification_tone(n);
        sync_idle_engine_profile();
        break;
    }
    case EV_SELECT:
        s_screen = SCREEN_MENU;
        s_menu_index = 0;
        break;
    default:
        break;
    }
}

static void ui_menu_event(input_event_t ev)
{
    switch (ev) {
    case EV_UP:
        s_menu_index = (s_menu_index - 1 + MENU_COUNT) % MENU_COUNT;
        break;
    case EV_DOWN:
        s_menu_index = (s_menu_index + 1) % MENU_COUNT;
        break;
    case EV_BACK:
    case EV_LEFT:
        s_screen = SCREEN_HOME;
        break;
    case EV_SELECT:
    case EV_RIGHT:
        switch (s_menu_index) {
        case 0:
            toggle_run();
            s_screen = SCREEN_HOME;
            break;
        case 1:
            s_list_index = (int)s_ui_controller.selected_profile_index;
            s_screen = SCREEN_PROFILE_LIST;
            break;
        case 2:
            enter_editor_for_selected(false);
            break;
        case 3:
            enter_editor_for_selected(true);
            break;
        case 4:
            s_settings_index = 0;
            s_screen = SCREEN_SETTINGS;
            break;
        case 5:
            s_screen = SCREEN_PINOUT;
            break;
        case 6: {
            bool ok = save_catalog_to_sd();
            tone(WIO_BUZZER, ok ? 2000 : 300, ok ? 120 : 300);
            break;
        }
        default:
            break;
        }
        break;
    default:
        break;
    }
}

static void ui_list_event(input_event_t ev)
{
    int n = (int)s_catalog.profile_count;
    if (n <= 0) {
        if (ev == EV_BACK || ev == EV_LEFT) s_screen = SCREEN_MENU;
        return;
    }
    switch (ev) {
    case EV_UP:
        s_list_index = (s_list_index - 1 + n) % n;
        break;
    case EV_DOWN:
        s_list_index = (s_list_index + 1) % n;
        break;
    case EV_BACK:
    case EV_LEFT:
        s_screen = SCREEN_MENU;
        break;
    case EV_SELECT:
    case EV_RIGHT:
        s_ui_controller.selected_profile_index = (uint32_t)s_list_index;
        sync_idle_engine_profile();
        play_notification_tone(UI_LOCAL_NOTIFICATION_PROFILE_CHANGED);
        s_screen = SCREEN_HOME;
        break;
    default:
        break;
    }
}

static void settings_adjust(int idx, int dir)
{
    switch (idx) {
    case 0:
        s_settings.chamber_absolute_max_c = clampf(s_settings.chamber_absolute_max_c + dir * 5.0f, 100.0f, 300.0f);
        break;
    case 1:
        s_settings.plate_absolute_max_c = clampf(s_settings.plate_absolute_max_c + dir * 5.0f, 50.0f, 250.0f);
        break;
    case 2:
        s_settings.bottom_plate_ceiling_c = clampf(s_settings.bottom_plate_ceiling_c + dir * 5.0f, 50.0f, 250.0f);
        break;
    case 3:
        s_settings.top_hysteresis_c = clampf(s_settings.top_hysteresis_c + dir * 0.5f, 0.5f, 10.0f);
        break;
    case 4:
        s_settings.bottom_hysteresis_c = clampf(s_settings.bottom_hysteresis_c + dir * 0.5f, 0.5f, 10.0f);
        break;
    case 5: {
        long v = (long)s_settings.control_watchdog_timeout_ms + dir * 250;
        if (v < 500) v = 500;
        if (v > 5000) v = 5000;
        s_settings.control_watchdog_timeout_ms = (uint32_t)v;
        break;
    }
    case 6:
        s_settings.runaway_min_rise_c = clampf(s_settings.runaway_min_rise_c + dir * 0.5f, 0.5f, 20.0f);
        break;
    case 7: {
        long v = (long)s_settings.runaway_window_ms + dir * 5000;
        if (v < 5000) v = 5000;
        if (v > 120000) v = 120000;
        s_settings.runaway_window_ms = (uint32_t)v;
        break;
    }
    default:
        break;
    }
}

static void ui_settings_event(input_event_t ev)
{
    switch (ev) {
    case EV_UP:
        s_settings_index = (s_settings_index - 1 + SETTINGS_COUNT) % SETTINGS_COUNT;
        break;
    case EV_DOWN:
        s_settings_index = (s_settings_index + 1) % SETTINGS_COUNT;
        break;
    case EV_LEFT:
        settings_adjust(s_settings_index, -1);
        break;
    case EV_RIGHT:
        settings_adjust(s_settings_index, +1);
        break;
    case EV_BACK:
        save_settings_to_sd();
        apply_heater_settings();
        tone(WIO_BUZZER, 1500, 60);
        s_screen = SCREEN_MENU;
        break;
    default:
        break;
    }
}

static void ui_edit_event(input_event_t ev)
{
    int total = edit_total_rows();
    switch (ev) {
    case EV_UP:
        s_edit_row = (s_edit_row - 1 + total) % total;
        break;
    case EV_DOWN:
        s_edit_row = (s_edit_row + 1) % total;
        break;
    case EV_BACK:
        s_screen = SCREEN_MENU;
        break;
    case EV_LEFT:
    case EV_RIGHT: {
        int dir = (ev == EV_RIGHT) ? +1 : -1;
        if (s_edit_row > 0 && s_edit_row < edit_action_add()) {
            int idx = s_edit_row - 1;
            edit_adjust_field(idx / PHASE_FIELD_COUNT, idx % PHASE_FIELD_COUNT, dir);
        }
        break;
    }
    case EV_SELECT:
        if (s_edit_row == 0) {
            s_name_cursor = (int)strlen(s_edit_profile.name);
            s_screen = SCREEN_NAME_EDIT;
        } else if (s_edit_row == edit_action_add()) {
            if (s_edit_profile.phase_count < PROFILE_ENGINE_MAX_PHASES) {
                profile_phase_t *last = &s_edit_profile.phases[s_edit_profile.phase_count - 1];
                profile_phase_t *np = &s_edit_profile.phases[s_edit_profile.phase_count];
                *np = *last;
                snprintf(np->name, sizeof(np->name), "phase%u", (unsigned)(s_edit_profile.phase_count + 1));
                s_edit_profile.phase_count++;
                tone(WIO_BUZZER, 1500, 40);
            }
        } else if (s_edit_row == edit_action_del()) {
            if (s_edit_profile.phase_count > 1) {
                s_edit_profile.phase_count--;
                if (s_edit_row >= edit_total_rows()) s_edit_row = edit_total_rows() - 1;
                tone(WIO_BUZZER, 700, 40);
            }
        } else if (s_edit_row == edit_action_save()) {
            if (commit_edit_profile()) {
                tone(WIO_BUZZER, 2000, 120);
                s_screen = SCREEN_HOME;
            } else {
                tone(WIO_BUZZER, 300, 300);
            }
        } else if (s_edit_row == edit_action_cancel()) {
            s_screen = SCREEN_MENU;
        }
        break;
    default:
        break;
    }
}

static void ui_name_event(input_event_t ev)
{
    int len = (int)strlen(s_edit_profile.name);
    int maxlen = (int)sizeof(s_edit_profile.name) - 1;
    switch (ev) {
    case EV_LEFT:
        if (s_name_cursor > 0) s_name_cursor--;
        break;
    case EV_RIGHT:
        if (s_name_cursor < len) s_name_cursor++;
        break;
    case EV_UP:
        if (s_name_cursor == len) {
            if (len < maxlen) {
                s_edit_profile.name[len] = NAME_CHARS[0];
                s_edit_profile.name[len + 1] = '\0';
            }
        } else {
            s_edit_profile.name[s_name_cursor] = name_cycle(s_edit_profile.name[s_name_cursor], +1);
        }
        break;
    case EV_DOWN:
        if (s_name_cursor < len) {
            s_edit_profile.name[s_name_cursor] = name_cycle(s_edit_profile.name[s_name_cursor], -1);
        }
        break;
    case EV_SELECT:
    case EV_BACK:
        if (s_edit_profile.name[0] == '\0') {
            strncpy(s_edit_profile.name, "custom", sizeof(s_edit_profile.name) - 1);
        }
        s_screen = SCREEN_PROFILE_EDIT;
        break;
    default:
        break;
    }
}

static void ui_pinout_event(input_event_t ev)
{
    if (ev == EV_BACK || ev == EV_LEFT || ev == EV_SELECT) {
        s_screen = SCREEN_MENU;
    }
}

static void ui_handle_event(input_event_t ev)
{
    if (ev == EV_NONE) {
        return;
    }

    /* The top-right key is a global run/stop: always stops a running profile
     * (safety), and quick-starts from the non-editor screens. */
    if (ev == EV_RUN) {
        if (s_profile_engine.state == PROFILE_ENGINE_STATE_RUNNING) {
            toggle_run();
            s_screen = SCREEN_HOME;
            return;
        }
        if (s_screen == SCREEN_HOME || s_screen == SCREEN_MENU || s_screen == SCREEN_PROFILE_LIST) {
            toggle_run();
            s_screen = SCREEN_HOME;
        }
        return;
    }

    switch (s_screen) {
    case SCREEN_HOME:
        ui_home_event(ev);
        break;
    case SCREEN_MENU:
        ui_menu_event(ev);
        break;
    case SCREEN_PROFILE_LIST:
        ui_list_event(ev);
        break;
    case SCREEN_SETTINGS:
        ui_settings_event(ev);
        break;
    case SCREEN_PROFILE_EDIT:
        ui_edit_event(ev);
        break;
    case SCREEN_NAME_EDIT:
        ui_name_event(ev);
        break;
    case SCREEN_PINOUT:
        ui_pinout_event(ev);
        break;
    default:
        break;
    }
}

/* ---- Per-screen rendering (all into s_canvas, pushed once) ------------ */

static void ui_begin_screen(const char *title)
{
    s_canvas.fillSprite(COL_BG);
    s_canvas.setTextDatum(TL_DATUM);
    s_canvas.setTextColor(COL_TEXT, COL_BG);
    s_canvas.drawString(title, 8, 6, 4);
    s_canvas.drawFastHLine(0, 34, s_canvas.width(), COL_PANEL_EDGE);
}

static void ui_footer_hint(const char *hint)
{
    s_canvas.setTextDatum(BL_DATUM);
    s_canvas.setTextColor(COL_MUTED, COL_BG);
    s_canvas.drawString(hint, 8, s_canvas.height() - 3, 1);
}

static void draw_list_row(int y, int h, const char *left, const char *right, bool selected, uint16_t right_col)
{
    int W = s_canvas.width();
    uint16_t bg = selected ? COL_PANEL : COL_BG;
    if (selected) {
        s_canvas.fillRoundRect(4, y, W - 8, h, 4, COL_PANEL);
        s_canvas.fillRect(4, y, 3, h, COL_ACCENT);
    }
    s_canvas.setTextDatum(ML_DATUM);
    s_canvas.setTextColor(selected ? COL_TEXT : COL_MUTED, bg);
    s_canvas.drawString(left, 14, y + h / 2, 2);
    if (right != NULL && right[0] != '\0') {
        s_canvas.setTextDatum(MR_DATUM);
        s_canvas.setTextColor(right_col, bg);
        s_canvas.drawString(right, W - 12, y + h / 2, 2);
    }
}

static void render_home(void)
{
    ui_local_status_t status;
    ui_local_controller_snapshot(&s_ui_controller, &s_profile_engine, s_catalog.profiles, &status);
    render_status(&status, &s_profile_engine, s_disp_chamber_c, s_disp_plate_c, s_disp_target_c, s_disp_top_power,
                  s_disp_bottom_power, s_disp_faults, s_disp_chamber_valid, s_disp_plate_valid);
}

static void render_menu(void)
{
    ui_begin_screen("Menu");
    const bool running = (s_profile_engine.state == PROFILE_ENGINE_STATE_RUNNING);
    int y = 40;
    const int step = 27;
    for (int i = 0; i < MENU_COUNT; i++) {
        const char *right = (i == 0) ? (running ? "Stop" : "Run") : "";
        draw_list_row(y, 24, kMenuItems[i], right, i == s_menu_index, COL_ACCENT);
        y += step;
    }
    ui_footer_hint("Up/Down move   Press select   A back   C run/stop");
    s_canvas.pushSprite(0, 0);
}

static void render_profile_list(void)
{
    ui_begin_screen("Profiles");
    int n = (int)s_catalog.profile_count;
    const int visible = 6;
    int top = 0;
    if (s_list_index >= visible) {
        top = s_list_index - visible + 1;
    }
    int y = 42;
    const int step = 30;
    for (int i = top; i < n && i < top + visible; i++) {
        char left[40];
        char right[16];
        snprintf(left, sizeof(left), "%s%s",
                 ((uint32_t)i == s_ui_controller.selected_profile_index) ? "* " : "  ",
                 s_catalog.profiles[i].name);
        snprintf(right, sizeof(right), "%u ph", (unsigned)s_catalog.profiles[i].phase_count);
        draw_list_row(y, 24, left, right, i == s_list_index, COL_MUTED);
        y += step;
    }
    ui_footer_hint("Press select   A back   (* = active)");
    s_canvas.pushSprite(0, 0);
}

static const char *const kSettingLabels[SETTINGS_COUNT] = {
    "Chamber max", "Plate max",      "Plate ceiling", "Top hysteresis",
    "Bottom hyst.", "Watchdog",      "Runaway rise",  "Runaway time"};

static void settings_value_str(int idx, char *out, size_t n)
{
    switch (idx) {
    case 0:
        snprintf(out, n, "%d C", (int)s_settings.chamber_absolute_max_c);
        break;
    case 1:
        snprintf(out, n, "%d C", (int)s_settings.plate_absolute_max_c);
        break;
    case 2:
        snprintf(out, n, "%d C", (int)s_settings.bottom_plate_ceiling_c);
        break;
    case 3:
        snprintf(out, n, "%.1f C", s_settings.top_hysteresis_c);
        break;
    case 4:
        snprintf(out, n, "%.1f C", s_settings.bottom_hysteresis_c);
        break;
    case 5:
        snprintf(out, n, "%lu ms", (unsigned long)s_settings.control_watchdog_timeout_ms);
        break;
    case 6:
        snprintf(out, n, "%.1f C", s_settings.runaway_min_rise_c);
        break;
    case 7:
        snprintf(out, n, "%lu s", (unsigned long)(s_settings.runaway_window_ms / 1000));
        break;
    default:
        out[0] = '\0';
        break;
    }
}

static void render_settings(void)
{
    ui_begin_screen("Settings");
    const int visible = 6;
    int top = 0;
    if (s_settings_index >= visible) {
        top = s_settings_index - visible + 1;
    }
    int y = 42;
    const int step = 30;
    for (int i = top; i < SETTINGS_COUNT && i < top + visible; i++) {
        char val[24];
        settings_value_str(i, val, sizeof(val));
        draw_list_row(y, 24, kSettingLabels[i], val, i == s_settings_index, COL_ACCENT);
        y += step;
    }
    ui_footer_hint("Left/Right adjust   A save & back");
    s_canvas.pushSprite(0, 0);
}

static void edit_row_text(int row, char *lbl, size_t ln, char *val, size_t vn, bool *is_action)
{
    *is_action = false;
    val[0] = '\0';
    if (row == 0) {
        snprintf(lbl, ln, "Name");
        snprintf(val, vn, "%s", s_edit_profile.name);
        return;
    }
    if (row < edit_action_add()) {
        int idx = row - 1;
        int p = idx / PHASE_FIELD_COUNT;
        int f = idx % PHASE_FIELD_COUNT;
        profile_phase_t *ph = &s_edit_profile.phases[p];
        static const char *const fname[] = {"Duration", "Target", "Ramp", "Bottom", "Plate lim", "Top heat"};
        snprintf(lbl, ln, "P%d %s", p + 1, fname[f]);
        switch (f) {
        case EF_DURATION:
            snprintf(val, vn, "%lu s", (unsigned long)ph->duration_s);
            break;
        case EF_TARGET:
            snprintf(val, vn, "%d C", (int)ph->target_chamber_c);
            break;
        case EF_RAMP:
            snprintf(val, vn, "%.1f C/s", ph->ramp_rate_c_per_s);
            break;
        case EF_BMODE:
            snprintf(val, vn, "%s", bottom_mode_str(ph->bottom_heater_mode));
            break;
        case EF_BLIMIT:
            snprintf(val, vn, "%d C", (int)ph->bottom_plate_limit_c);
            break;
        case EF_TOPEN:
            snprintf(val, vn, "%s", ph->top_heater_enabled ? "on" : "off");
            break;
        default:
            break;
        }
        return;
    }
    *is_action = true;
    if (row == edit_action_add())
        snprintf(lbl, ln, "+ Add phase");
    else if (row == edit_action_del())
        snprintf(lbl, ln, "- Delete last phase");
    else if (row == edit_action_save())
        snprintf(lbl, ln, "Save profile");
    else
        snprintf(lbl, ln, "Cancel");
}

static void render_profile_edit(void)
{
    char title[40];
    snprintf(title, sizeof(title), "Edit %s", s_edit_profile.name);
    ui_begin_screen(title);

    int total = edit_total_rows();
    const int visible = 7;
    const int rowH = 24;
    if (s_edit_row < s_edit_scroll) s_edit_scroll = s_edit_row;
    if (s_edit_row >= s_edit_scroll + visible) s_edit_scroll = s_edit_row - visible + 1;

    int y = 40;
    for (int i = s_edit_scroll; i < total && i < s_edit_scroll + visible; i++) {
        char lbl[40];
        char val[24];
        bool act = false;
        edit_row_text(i, lbl, sizeof(lbl), val, sizeof(val), &act);
        draw_list_row(y, rowH - 2, lbl, val, i == s_edit_row, act ? COL_OK : COL_ACCENT);
        y += rowH;
    }
    ui_footer_hint("U/D move  L/R change  Press name/action  A back");
    s_canvas.pushSprite(0, 0);
}

static void render_name_edit(void)
{
    ui_begin_screen("Profile name");
    int len = (int)strlen(s_edit_profile.name);
    const int chW = 16;
    const int startx = 16;
    const int y = 96;
    for (int i = 0; i <= len; i++) {
        char c = (i < len) ? s_edit_profile.name[i] : '_';
        bool cur = (i == s_name_cursor);
        int cx = startx + i * chW;
        if (cur) {
            s_canvas.fillRoundRect(cx - 2, y - 6, chW, 34, 3, COL_PANEL);
        }
        char s[2] = {c, '\0'};
        s_canvas.setTextDatum(TL_DATUM);
        s_canvas.setTextColor(cur ? COL_ACCENT : COL_TEXT, cur ? COL_PANEL : COL_BG);
        s_canvas.drawString(s, cx, y, 4);
    }
    ui_footer_hint("Up/Down change char   Left/Right move   Press done");
    s_canvas.pushSprite(0, 0);
}

/* Static wiring reference reflecting the firmware's current pin assignments
 * (Wio Terminal 40-pin header; BCM names with the equivalent RPi-style
 * physical header pin). See docs/hardware.md "External Wiring Summary". */
static void render_pinout(void)
{
    ui_begin_screen("Wiring");
    const int W = s_canvas.width();

    auto section = [&](const char *title, int yy) {
        s_canvas.setTextDatum(TL_DATUM);
        s_canvas.setTextColor(COL_ACCENT, COL_BG);
        s_canvas.drawString(title, 8, yy, 2);
    };
    auto row = [&](const char *label, const char *pin, int yy) {
        s_canvas.setTextDatum(TL_DATUM);
        s_canvas.setTextColor(COL_MUTED, COL_BG);
        s_canvas.drawString(label, 22, yy, 2);
        s_canvas.setTextDatum(TR_DATUM);
        s_canvas.setTextColor(COL_TEXT, COL_BG);
        s_canvas.drawString(pin, W - 12, yy, 2);
    };

    section("MAX31855 thermocouple (SPI)", 40);
    row("SCK", "BCM11 / pin23", 58);
    row("MISO", "BCM9 / pin21", 76);
    row("CS", "BCM8 / pin24", 94);

    section("Plate NTC", 116);
    row("ADC in", "A0", 134);

    section("Heaters", 156);
    row("Top SSR", "BCM23 / pin16", 174);
    row("Bottom relay", "BCM24 / pin18", 192);

    ui_footer_hint("A back   -   Wio Terminal 40-pin header");
    s_canvas.pushSprite(0, 0);
}

static void render_current_screen(void)
{
    switch (s_screen) {
    case SCREEN_HOME:
        render_home();
        break;
    case SCREEN_MENU:
        render_menu();
        break;
    case SCREEN_PROFILE_LIST:
        render_profile_list();
        break;
    case SCREEN_SETTINGS:
        render_settings();
        break;
    case SCREEN_PROFILE_EDIT:
        render_profile_edit();
        break;
    case SCREEN_NAME_EDIT:
        render_name_edit();
        break;
    case SCREEN_PINOUT:
        render_pinout();
        break;
    default:
        break;
    }
}

void setup()
{
    Serial.begin(115200);

    analogReadResolution(ADC_RESOLUTION_BITS);

    pinMode(PIN_MAX31855_CS, OUTPUT);
    digitalWrite(PIN_MAX31855_CS, HIGH);
    pinMode(PIN_TOP_HEATER_RELAY, OUTPUT);
    pinMode(PIN_BOTTOM_HEATER_RELAY, OUTPUT);
    relay_output_force_all_off(); /* fail-safe default-off */

    pinMode(WIO_KEY_A, INPUT_PULLUP);
    pinMode(WIO_KEY_B, INPUT_PULLUP);
    pinMode(WIO_KEY_C, INPUT_PULLUP);
    pinMode(WIO_5S_UP, INPUT_PULLUP);
    pinMode(WIO_5S_DOWN, INPUT_PULLUP);
    pinMode(WIO_5S_LEFT, INPUT_PULLUP);
    pinMode(WIO_5S_RIGHT, INPUT_PULLUP);
    pinMode(WIO_5S_PRESS, INPUT_PULLUP);
    pinMode(WIO_BUZZER, OUTPUT);

    SPI.begin();

    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);

    /* Allocate the off-screen framebuffer matching the (rotated) panel size.
     * 8-bit colour keeps it to ~76 KB, well within the SAMD51's 192 KB RAM. */
    s_canvas.setColorDepth(8);
    s_canvas.createSprite(tft.width(), tft.height());
    s_canvas.fillSprite(COL_BG);

    if (!SD.begin(SDCARD_SS_PIN, SDCARD_SPI, 4000000UL)) {
        Serial.println("microSD init failed; continuing without profile catalog card");
    }

    load_settings_from_sd();
    load_profile_catalog();

    apply_heater_settings();
    thermal_runaway_reset(&s_top_runaway);
    thermal_runaway_reset(&s_bottom_runaway);

    ui_local_controller_init(&s_ui_controller, s_catalog.profile_count, 0);

    const reflow_profile_t *initial_profile = profile_catalog_get(&s_catalog, 0);
    if (initial_profile == NULL || profile_engine_load(&s_profile_engine, initial_profile) != 0) {
        Serial.println("Initial profile failed validation; refusing to start");
        for (;;) {
            relay_output_force_all_off();
            delay(1000);
        }
    }

    s_last_control_tick_ms = millis();
    s_last_ui_poll_ms = s_last_control_tick_ms;
    s_last_render_ms = s_last_control_tick_ms;
}

static void control_tick(uint32_t ms_since_last)
{
    thermocouple_reading_t tc_reading;
    int tc_err = thermocouple_max31855_read(&tc_reading);

    float plate_temp_c = 0.0f;
    int plate_implausible = 0;
    int ntc_err = plate_ntc_read(&plate_temp_c, &plate_implausible);

    profile_engine_tick(&s_profile_engine, (float)CONTROL_TICK_MS / 1000.0f);

    const profile_phase_t *phase = profile_engine_current_phase(&s_profile_engine);

    if (!s_have_last_phase || s_profile_engine.current_phase_index != s_last_phase_index) {
        s_previous_phase_target_c = (phase != NULL) ? tc_reading.chamber_temperature_c : 0.0f;
        s_last_phase_index = s_profile_engine.current_phase_index;
        s_have_last_phase = 1;
    }

    const safety_limits_t safety_limits = {
        .chamber_absolute_max_c = s_settings.chamber_absolute_max_c,
        .plate_absolute_max_c = s_settings.plate_absolute_max_c,
        .control_watchdog_timeout_ms = s_settings.control_watchdog_timeout_ms,
    };

    safety_inputs_t inputs;
    memset(&inputs, 0, sizeof(inputs));
    inputs.thermocouple_fault_bit = (tc_err != 0) || tc_reading.fault;
    inputs.thermocouple_open_or_short =
        tc_reading.fault_open_circuit || tc_reading.fault_short_to_gnd || tc_reading.fault_short_to_vcc;
    inputs.plate_adc_implausible = (ntc_err != 0) || plate_implausible;
    inputs.chamber_temperature_c = tc_reading.chamber_temperature_c;
    inputs.plate_temperature_c = plate_temp_c;
    inputs.ms_since_last_control_tick = ms_since_last;
    inputs.system_is_idle_or_fault = (s_profile_engine.state == PROFILE_ENGINE_STATE_IDLE ||
                                       s_profile_engine.state == PROFILE_ENGINE_STATE_FAULT);
    inputs.active_profile_valid =
        (s_profile_engine.active_profile != NULL) && profile_engine_validate(s_profile_engine.active_profile);

    uint32_t faults = safety_evaluate(&inputs, &safety_limits);

    float top_power_percent = 0.0f;
    float bottom_power_percent = 0.0f;
    bool top_on = false;
    bool bottom_on = false;

    /* Whether each heater is energized AND still expected to be climbing toward
     * its setpoint; drives the open-loop / thermal-runaway guard below. */
    int top_heating_expected = 0;
    int bottom_heating_expected = 0;

    if (faults == SAFETY_FAULT_NONE && phase != NULL) {
        float target_c = profile_engine_current_target_c(&s_profile_engine, s_previous_phase_target_c);
        float dt_s = (float)CONTROL_TICK_MS / 1000.0f;

        if (phase->top_heater_enabled) {
            top_power_percent =
                heater_control_update(&s_top_heater_ctrl, tc_reading.chamber_temperature_c, target_c, dt_s);
            top_heating_expected = (top_power_percent > 0.0f) &&
                                   ((target_c - tc_reading.chamber_temperature_c) > RUNAWAY_ARM_MARGIN_C);
        } else {
            heater_control_reset(&s_top_heater_ctrl);
        }

        if (phase->bottom_heater_mode == BOTTOM_HEATER_MODE_OFF) {
            heater_control_reset(&s_bottom_heater_ctrl);
        } else {
            float bottom_target_c = phase->bottom_plate_limit_c;
            if (phase->bottom_heater_mode == BOTTOM_HEATER_MODE_LIMITED) {
                bottom_target_c = (bottom_target_c > s_settings.bottom_plate_ceiling_c)
                                      ? s_settings.bottom_plate_ceiling_c
                                      : bottom_target_c;
            }
            bottom_power_percent = heater_control_update(&s_bottom_heater_ctrl, plate_temp_c, bottom_target_c, dt_s);
            bottom_heating_expected =
                (bottom_power_percent > 0.0f) && ((bottom_target_c - plate_temp_c) > RUNAWAY_ARM_MARGIN_C);
        }

        uint32_t top_on_ms = relay_timing_on_ms(top_power_percent, RELAY_WINDOW_MS);
        uint32_t bottom_on_ms = relay_timing_on_ms(bottom_power_percent, RELAY_WINDOW_MS);

        top_on = relay_timing_should_energize(s_window_elapsed_ms, top_on_ms);
        bottom_on = relay_timing_should_energize(s_window_elapsed_ms, bottom_on_ms);
    }

    /* Open-loop / thermal-runaway guard: if a heater is driven while the
     * temperature is expected to climb but it stalls (heater/SSR failed open,
     * thermocouple detached, etc.), trip a fault and force everything off. */
    const thermal_runaway_config_t runaway_cfg = {
        .min_rise_c = s_settings.runaway_min_rise_c,
        .window_ms = s_settings.runaway_window_ms,
    };
    int top_runaway = thermal_runaway_update(&s_top_runaway, &runaway_cfg, top_heating_expected,
                                             tc_reading.chamber_temperature_c, CONTROL_TICK_MS);
    int bottom_runaway = thermal_runaway_update(&s_bottom_runaway, &runaway_cfg, bottom_heating_expected,
                                                plate_temp_c, CONTROL_TICK_MS);
    inputs.thermal_runaway = (top_runaway || bottom_runaway);

    inputs.relay_commanded_on = top_on || bottom_on;
    faults |= safety_evaluate(&inputs, &safety_limits);

    ui_local_notification_t notification = ui_local_controller_observe_engine(&s_ui_controller, &s_profile_engine, faults);
    if (notification != UI_LOCAL_NOTIFICATION_NONE) {
        play_notification_tone(notification);
    }

    if (faults != SAFETY_FAULT_NONE) {
        relay_output_force_all_off();
        heater_control_reset(&s_top_heater_ctrl);
        heater_control_reset(&s_bottom_heater_ctrl);
        top_power_percent = 0.0f;
        bottom_power_percent = 0.0f;
        profile_engine_force_fault(&s_profile_engine);
    } else {
        relay_output_set(top_on, bottom_on);
    }

    s_window_elapsed_ms += CONTROL_TICK_MS;
    if (s_window_elapsed_ms >= RELAY_WINDOW_MS) {
        s_window_elapsed_ms = 0;
    }

    /* Publish the latest values for the renderer (which runs independently of
     * the current screen, so live temperatures stay current everywhere). */
    s_disp_chamber_c = tc_reading.chamber_temperature_c;
    s_disp_plate_c = plate_temp_c;
    s_disp_top_power = top_power_percent;
    s_disp_bottom_power = bottom_power_percent;
    s_disp_faults = faults;
    s_disp_chamber_valid = (tc_err == 0);
    s_disp_plate_valid = (ntc_err == 0) && !plate_implausible;
    s_disp_target_c =
        (phase != NULL) ? profile_engine_current_target_c(&s_profile_engine, s_previous_phase_target_c) : 0.0f;

    update_temperature_trace(tc_reading.chamber_temperature_c);
}

void loop()
{
    uint32_t now = millis();

    if (now - s_last_ui_poll_ms >= UI_POLL_MS) {
        s_last_ui_poll_ms = now;
        input_event_t ev = poll_input_event();
        if (ev != EV_NONE) {
            ui_handle_event(ev);
        }
    }

    if (now - s_last_control_tick_ms >= CONTROL_TICK_MS) {
        uint32_t ms_since_last = now - s_last_control_tick_ms;
        s_last_control_tick_ms = now;
        control_tick(ms_since_last);
    }

    if (now - s_last_render_ms >= RENDER_MS) {
        s_last_render_ms = now;
        render_current_screen();
    }
}
