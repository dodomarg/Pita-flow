#include "drivers/thermocouple_max31855.h"

#include <string.h>

#include "esp_log.h"

static const char *TAG = "max31855";
static spi_device_handle_t s_spi_device;
static bool s_initialized = false;

esp_err_t thermocouple_max31855_init(const thermocouple_max31855_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_bus_config_t bus_config = {
        .mosi_io_num = -1, /* MAX31855 is read-only over SPI. */
        .miso_io_num = config->pin_miso,
        .sclk_io_num = config->pin_clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4,
    };

    esp_err_t err = spi_bus_initialize(config->spi_host, &bus_config, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE means the bus was already initialized by
         * another driver sharing the same SPI host, which is acceptable. */
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = config->clock_speed_hz > 0 ? config->clock_speed_hz : 4000000,
        .mode = 0, /* MAX31855 uses SPI mode 0. */
        .spics_io_num = config->pin_cs,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };

    err = spi_bus_add_device(config->spi_host, &dev_config, &s_spi_device);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t thermocouple_max31855_read(thermocouple_max31855_reading_t *out_reading)
{
    if (out_reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t rx_buf[4] = {0};
    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));
    transaction.rxlength = 32; /* bits */
    transaction.rx_buffer = rx_buf;

    esp_err_t err = spi_device_polling_transmit(s_spi_device, &transaction);
    if (err != ESP_OK) {
        return err;
    }

    uint32_t raw = ((uint32_t)rx_buf[0] << 24) | ((uint32_t)rx_buf[1] << 16) |
                   ((uint32_t)rx_buf[2] << 8) | (uint32_t)rx_buf[3];

    memset(out_reading, 0, sizeof(*out_reading));

    out_reading->fault = (raw & (1u << 16)) != 0;
    out_reading->fault_open_circuit = (raw & (1u << 0)) != 0;
    out_reading->fault_short_to_gnd = (raw & (1u << 1)) != 0;
    out_reading->fault_short_to_vcc = (raw & (1u << 2)) != 0;

    /* Bits 31..18: 14-bit signed thermocouple temperature, 0.25 C/LSB. */
    int32_t tc_raw = (int32_t)(raw >> 18) & 0x3FFF;
    if (tc_raw & 0x2000) {
        tc_raw |= ~0x3FFF; /* sign-extend 14-bit value */
    }
    out_reading->chamber_temperature_c = (float)tc_raw * 0.25f;

    /* Bits 15..4: 12-bit signed internal (cold-junction) reference, 0.0625 C/LSB. */
    int32_t ref_raw = (int32_t)(raw >> 4) & 0xFFF;
    if (ref_raw & 0x800) {
        ref_raw |= ~0xFFF; /* sign-extend 12-bit value */
    }
    out_reading->internal_reference_c = (float)ref_raw * 0.0625f;

    return ESP_OK;
}
