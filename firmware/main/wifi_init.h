#ifndef MAIN_WIFI_INIT_H
#define MAIN_WIFI_INIT_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initializes Wi-Fi in station mode using CONFIG_PITA_FLOW_WIFI_SSID /
 * CONFIG_PITA_FLOW_WIFI_PASSWORD (see Kconfig.projbuild) and blocks until
 * either a connection is established or the connection attempt times out.
 * Returns ESP_OK if connected, ESP_ERR_TIMEOUT otherwise. Wi-Fi failures
 * must never block or affect the heater control task, so callers should
 * treat a non-ESP_OK return as "run without network" rather than a fatal
 * error.
 */
esp_err_t wifi_init_station_and_wait(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_WIFI_INIT_H */
