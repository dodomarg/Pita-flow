#include "config_store.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#define CONFIG_STORE_NAMESPACE "pf_config"
#define CONFIG_STORE_BLOB_KEY "system_cfg"

static const char *TAG = "config_store";

void config_store_defaults(system_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));

    /* Conservative safety defaults; see docs/firmware-plan.md "Configuration". */
    config->chamber_absolute_max_c = 260.0f;
    config->plate_absolute_max_c = 230.0f; /* below the 250 C hardware cutoff */
    config->control_watchdog_timeout_ms = 500;

    config->bottom_plate_ceiling_c = 150.0f;
    config->relay_window_ms = 1000;

    config->top_heater_config.hysteresis_c = 2.0f;
    config->top_heater_config.kp = 2.0f;
    config->top_heater_config.ki = 0.05f;
    config->top_heater_config.kd = 0.0f;
    config->top_heater_config.min_power_percent = 0.0f;
    config->top_heater_config.max_power_percent = 100.0f;

    config->bottom_heater_config.hysteresis_c = 2.0f;
    config->bottom_heater_config.kp = 2.0f;
    config->bottom_heater_config.ki = 0.05f;
    config->bottom_heater_config.kd = 0.0f;
    config->bottom_heater_config.min_power_percent = 0.0f;
    config->bottom_heater_config.max_power_percent = 100.0f;

    config->active_profile_index = 0;
}

esp_err_t config_store_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS partition due to %s", esp_err_to_name(err));
        err = nvs_flash_erase();
        if (err != ESP_OK) {
            return err;
        }
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t config_store_load(system_config_t *out_config)
{
    if (out_config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    system_config_t defaults;
    config_store_defaults(&defaults);
    *out_config = defaults;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(CONFIG_STORE_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        /* No saved config yet: defaults already populated above. */
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s, using defaults", esp_err_to_name(err));
        return ESP_OK;
    }

    system_config_t stored;
    size_t required_size = sizeof(stored);
    err = nvs_get_blob(handle, CONFIG_STORE_BLOB_KEY, &stored, &required_size);
    nvs_close(handle);

    if (err == ESP_OK && required_size == sizeof(stored)) {
        *out_config = stored;
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "nvs_get_blob failed: %s, using defaults", esp_err_to_name(err));
    }

    return ESP_OK;
}

esp_err_t config_store_save(const system_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(CONFIG_STORE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(handle, CONFIG_STORE_BLOB_KEY, config, sizeof(*config));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}
