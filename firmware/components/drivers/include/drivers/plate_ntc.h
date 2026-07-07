#ifndef DRIVERS_PLATE_NTC_H
#define DRIVERS_PLATE_NTC_H

#include "core/ntc_convert.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int adc_channel; /* ADC1 channel number, e.g. 0 for GPIO0. */
    ntc_params_t ntc_params;
    int oversample_count; /* Number of raw samples to average, e.g. 8. */
} plate_ntc_config_t;

/** Initializes the ADC unit/channel used to read the plate NTC. */
esp_err_t plate_ntc_init(const plate_ntc_config_t *config);

/**
 * Reads and averages `oversample_count` raw ADC samples, converts the
 * result to Celsius, and reports whether the raw reading is implausible
 * (open/short) via *out_implausible. On implausible readings,
 * *out_temperature_c is left at 0.0f and the caller should treat the
 * sensor as faulted (see safety.h).
 */
esp_err_t plate_ntc_read(float *out_temperature_c, int *out_implausible);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_PLATE_NTC_H */
