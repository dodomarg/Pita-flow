#ifndef CORE_NTC_CONVERT_H
#define CORE_NTC_CONVERT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Parameters describing a resistor-divider NTC thermistor circuit and the
 * Beta-model characteristics of the thermistor itself.
 *
 * The divider is assumed to be: 3.3V -- ntc -- (adc node) -- pulldown_resistor_ohm -- gnd
 * (i.e. the NTC is the high side leg, connected to the supply, and the fixed
 * resistor is the low-side "pulldown" leg to ground). With this wiring,
 * higher temperature (lower NTC resistance) produces a higher raw ADC
 * reading. See docs/firmware-plan.md "Confirmed Hardware Interfaces" for
 * the recommended pulldown resistor value.
 */
typedef struct {
    float pulldown_resistor_ohm;   /* Fixed low-side (pulldown) divider resistor value */
    float nominal_resistance_ohm; /* NTC resistance at nominal_temperature_c (typically 100000 @ 25C) */
    float nominal_temperature_c;  /* Nominal temperature for nominal_resistance_ohm, typically 25.0 */
    float beta;                   /* Beta coefficient of the thermistor, e.g. 3950 */
} ntc_params_t;

/**
 * Converts a raw ADC reading (0..adc_max inclusive) taken at the node
 * between the high-side NTC and the low-side pulldown resistor into a
 * temperature in Celsius using the Beta parameter (simplified
 * Steinhart-Hart) equation.
 *
 * Returns 0 on success and writes the temperature to *out_temperature_c.
 * Returns -1 if adc_max is 0, params is NULL, out_temperature_c is NULL,
 * or the computed thermistor resistance is non-positive (implausible
 * reading, e.g. an open or shorted sensor).
 */
int ntc_convert_adc_to_celsius(uint32_t raw_adc,
                                uint32_t adc_max,
                                const ntc_params_t *params,
                                float *out_temperature_c);

/**
 * Returns 1 if the raw ADC reading is implausible (i.e. outside the range
 * that could correspond to a working sensor: an open-circuit NTC pulls the
 * node to ~0, a shorted NTC pulls the node to ~adc_max), 0 otherwise.
 * margin_counts defines how many raw ADC counts of headroom from the
 * 0/adc_max rails are still considered a fault.
 */
int ntc_reading_is_implausible(uint32_t raw_adc, uint32_t adc_max, uint32_t margin_counts);

#ifdef __cplusplus
}
#endif

#endif /* CORE_NTC_CONVERT_H */
