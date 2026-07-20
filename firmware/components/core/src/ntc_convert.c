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
        /* Open or shorted sensor (division by zero or R_ntc == 0 at a rail). */
        return -1;
    }

    float r_ntc;
    if (params->ntc_low_side) {
        /* Vcc -- R_pullup -- node -- NTC -- gnd
         * raw_adc / adc_max = R_ntc / (R_ntc + R_pullup)
         * => R_ntc = R_pullup * raw_adc / (adc_max - raw_adc)
         */
        r_ntc = params->pulldown_resistor_ohm * (float)raw_adc / (float)(adc_max - raw_adc);
    } else {
        /* Vcc -- ntc -- node -- R_pulldown -- gnd
         * raw_adc / adc_max = R_pulldown / (R_ntc + R_pulldown)
         * => R_ntc = R_pulldown * (adc_max - raw_adc) / raw_adc
         */
        r_ntc = params->pulldown_resistor_ohm * (float)(adc_max - raw_adc) / (float)raw_adc;
    }

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
        return 1; /* Near/at zero rail: open-circuit NTC leaves the node pulled to GND by the pulldown resistor */
    }
    if (raw_adc >= (adc_max - margin_counts)) {
        return 1; /* Near/at full-scale rail: shorted/near-zero-ohm NTC pulls the node up towards the 3.3V supply */
    }
    return 0;
}
