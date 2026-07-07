#include "core/ntc_convert.h"

#include <math.h>
#include <stddef.h>

int ntc_convert_adc_to_celsius(uint32_t raw_adc,
                                uint32_t adc_max,
                                const ntc_params_t *params,
                                float *out_temperature_c)
{
    if (params == NULL || out_temperature_c == NULL || adc_max == 0) {
        return -1;
    }

    if (raw_adc > adc_max) {
        raw_adc = adc_max;
    }

    /* Divider: 3.3V -- ntc -- node -- pulldown_resistor -- gnd
     * Vnode / Vcc = raw_adc / adc_max = R_pulldown / (R_ntc + R_pulldown)
     * => R_ntc = R_pulldown * (adc_max - raw_adc) / raw_adc
     */
    if (raw_adc == 0 || raw_adc >= adc_max) {
        /* Open (raw_adc == 0, division by zero) or shorted (raw_adc == adc_max,
         * would compute R_ntc == 0) sensor. */
        return -1;
    }

    float ratio = (float)(adc_max - raw_adc) / (float)raw_adc;
    float r_ntc = params->pulldown_resistor_ohm * ratio;

    if (r_ntc <= 0.0f) {
        return -1;
    }

    /* Beta equation:
     * 1/T = 1/T0 + (1/beta) * ln(R / R0)
     */
    const float kelvin_offset = 273.15f;
    float t0_kelvin = params->nominal_temperature_c + kelvin_offset;
    float inv_t = (1.0f / t0_kelvin) +
                  (1.0f / params->beta) * logf(r_ntc / params->nominal_resistance_ohm);

    if (inv_t <= 0.0f) {
        return -1;
    }

    float t_kelvin = 1.0f / inv_t;
    *out_temperature_c = t_kelvin - kelvin_offset;
    return 0;
}

int ntc_reading_is_implausible(uint32_t raw_adc, uint32_t adc_max, uint32_t margin_counts)
{
    if (adc_max == 0) {
        return 1;
    }
    if (margin_counts > adc_max) {
        margin_counts = adc_max;
    }
    if (raw_adc <= margin_counts) {
        return 1; /* Near/at zero rail: likely open-circuit NTC (pulled down) */
    }
    if (raw_adc >= (adc_max - margin_counts)) {
        return 1; /* Near/at full-scale rail: likely shorted NTC (pulled to supply) */
    }
    return 0;
}
