#ifndef DRIVERS_THERMOCOUPLE_MAX31855_H
#define DRIVERS_THERMOCOUPLE_MAX31855_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    spi_host_device_t spi_host;
    int pin_clk;
    int pin_miso;
    int pin_cs;
    int clock_speed_hz; /* MAX31855 supports up to 5 MHz. */
} thermocouple_max31855_config_t;

typedef struct {
    float chamber_temperature_c;
    float internal_reference_c;
    bool fault;
    bool fault_open_circuit;
    bool fault_short_to_gnd;
    bool fault_short_to_vcc;
} thermocouple_max31855_reading_t;

/** Initializes the SPI bus/device for the MAX31855. Must be called once at startup. */
esp_err_t thermocouple_max31855_init(const thermocouple_max31855_config_t *config);

/**
 * Performs a 32-bit SPI read and decodes it into a reading, including fault
 * bits. Returns ESP_OK if the SPI transaction itself succeeded (the reading
 * may still indicate a sensor fault via the fault fields, which the caller
 * must check).
 */
esp_err_t thermocouple_max31855_read(thermocouple_max31855_reading_t *out_reading);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_THERMOCOUPLE_MAX31855_H */
