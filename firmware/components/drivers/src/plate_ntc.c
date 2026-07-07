#include "drivers/plate_ntc.h"

#include <stdbool.h>

#include "core/ntc_convert.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#define PLATE_NTC_ADC_MAX_RAW 4095u /* 12-bit ADC on ESP32-C3 */
#define PLATE_NTC_FAULT_MARGIN_COUNTS 20u

static const char *TAG = "plate_ntc";
static adc_oneshot_unit_handle_t s_adc_handle;
static plate_ntc_config_t s_config;
static bool s_initialized = false;

esp_err_t plate_ntc_init(const plate_ntc_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_config = *config;
    if (s_config.oversample_count <= 0) {
        s_config.oversample_count = 8;
    }

    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_config, &s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc_handle, (adc_channel_t)s_config.adc_channel, &chan_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
        return err;
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t plate_ntc_read(float *out_temperature_c, int *out_implausible)
{
    if (out_temperature_c == NULL || out_implausible == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    *out_temperature_c = 0.0f;
    *out_implausible = 0;

    long sum = 0;
    for (int i = 0; i < s_config.oversample_count; i++) {
        int raw = 0;
        esp_err_t err = adc_oneshot_read(s_adc_handle, (adc_channel_t)s_config.adc_channel, &raw);
        if (err != ESP_OK) {
            return err;
        }
        sum += raw;
    }
    uint32_t avg_raw = (uint32_t)(sum / s_config.oversample_count);

    if (ntc_reading_is_implausible(avg_raw, PLATE_NTC_ADC_MAX_RAW, PLATE_NTC_FAULT_MARGIN_COUNTS)) {
        *out_implausible = 1;
        return ESP_OK;
    }

    float temperature_c = 0.0f;
    int rc = ntc_convert_adc_to_celsius(avg_raw, PLATE_NTC_ADC_MAX_RAW, &s_config.ntc_params, &temperature_c);
    if (rc != 0) {
        *out_implausible = 1;
        return ESP_OK;
    }

    *out_temperature_c = temperature_c;
    return ESP_OK;
}
