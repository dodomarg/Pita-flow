#ifndef DRIVERS_RELAY_OUTPUT_H
#define DRIVERS_RELAY_OUTPUT_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RELAY_OUTPUT_TOP_HEATER = 0,
    RELAY_OUTPUT_BOTTOM_HEATER = 1,
    RELAY_OUTPUT_COUNT,
} relay_output_id_t;

typedef struct {
    int gpio_top_heater;
    int gpio_bottom_heater;
} relay_output_config_t;

/**
 * Configures the relay GPIOs as outputs and forces both relays off
 * immediately (fail-safe default-off), per the firmware safety model.
 */
esp_err_t relay_output_init(const relay_output_config_t *config);

/** Directly commands a relay on/off. Used by the control task's time-proportional loop. */
esp_err_t relay_output_set(relay_output_id_t relay, int energize);

/** Forces all relay outputs off immediately. Safe to call from any context, including ISR-adjacent fault paths. */
esp_err_t relay_output_force_all_off(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_RELAY_OUTPUT_H */
