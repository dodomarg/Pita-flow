#include "drivers/relay_output.h"

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "relay_output";
static int s_gpio[RELAY_OUTPUT_COUNT];
static bool s_initialized = false;

esp_err_t relay_output_init(const relay_output_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_gpio[RELAY_OUTPUT_TOP_HEATER] = config->gpio_top_heater;
    s_gpio[RELAY_OUTPUT_BOTTOM_HEATER] = config->gpio_bottom_heater;

    for (int i = 0; i < RELAY_OUTPUT_COUNT; i++) {
        gpio_config_t io_config = {
            .pin_bit_mask = 1ULL << s_gpio[i],
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t err = gpio_config(&io_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "gpio_config failed for relay %d: %s", i, esp_err_to_name(err));
            return err;
        }
        /* Fail-safe default-off before enabling anything else. */
        gpio_set_level(s_gpio[i], 0);
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t relay_output_set(relay_output_id_t relay, int energize)
{
    if (!s_initialized || relay >= RELAY_OUTPUT_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }
    return gpio_set_level(s_gpio[relay], energize ? 1 : 0);
}

esp_err_t relay_output_force_all_off(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = ESP_OK;
    for (int i = 0; i < RELAY_OUTPUT_COUNT; i++) {
        esp_err_t rc = gpio_set_level(s_gpio[i], 0);
        if (rc != ESP_OK) {
            err = rc;
        }
    }
    return err;
}
