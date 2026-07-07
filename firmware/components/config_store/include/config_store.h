#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <stdint.h>

#include "core/heater_control.h"
#include "core/safety.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Safety limits (see core/safety.h). */
    float chamber_absolute_max_c;
    float plate_absolute_max_c;
    uint32_t control_watchdog_timeout_ms;

    /* Bottom plate assisted-heating ceiling, must stay below the hardware
     * 250 C plate cutoff. */
    float bottom_plate_ceiling_c;

    /* Relay time-proportional window duration. */
    uint32_t relay_window_ms;

    /* PID constants for top (chamber) and bottom (plate) control loops. */
    heater_control_config_t top_heater_config;
    heater_control_config_t bottom_heater_config;

    /* Index of the active profile in persistent storage. */
    uint32_t active_profile_index;
} system_config_t;

/** Populates config with conservative compile-time defaults. */
void config_store_defaults(system_config_t *config);

/** Initializes the NVS-backed config namespace. Must be called once at startup, after nvs_flash_init(). */
esp_err_t config_store_init(void);

/**
 * Loads configuration from NVS into *out_config. If no configuration has
 * been saved yet (or any value is missing/corrupt), the corresponding
 * fields fall back to config_store_defaults() values and ESP_OK is still
 * returned.
 */
esp_err_t config_store_load(system_config_t *out_config);

/** Persists the given configuration to NVS. */
esp_err_t config_store_save(const system_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_STORE_H */
