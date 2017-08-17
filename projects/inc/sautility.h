#pragma once
#include "saconfig.h"
#include "utility.h"

/* sensor and actuator utility functions */
namespace sa {
    // DAC driver must be started before this function can be used
    dacsample_t set_kollmorgen_reference(float reference, float max_ref_value) {
        // Calculate channel value based on DAC device params
        // regshift = 0 -> CH1 -> channel value 0
        // regshift = 16 -> CH2 -> channel value 1
        static const dacchannel_t channel = KOLLM_DAC->params->regshift/16;
        const float saturated_reference = util::clamp(reference,
                                                      -max_ref_value,
                                                      max_ref_value);
        const dacsample_t aout =
            saturated_reference*DAC_HALF_RANGE/max_ref_value
            + KOLLMORGEN_DAC_ZERO_OFFSET;
        dacPutChannelX(KOLLM_DAC, channel, aout);
        return aout;
    }

    // DAC driver must be started before this function can be used
    dacsample_t set_kollmorgen_torque(float reference_torque) {
        return set_kollmorgen_reference(reference_torque, MAX_KOLLMORGEN_TORQUE);
    }

    // DAC driver must be started before this function can be used
    dacsample_t set_kollmorgen_velocity(float reference_velocity) {
        return set_kollmorgen_reference(reference_velocity, MAX_KOLLMORGEN_VELOCITY);
    }

    float convert_adcsample(adcsample_t ain, adcsample_t adc_zero, float magnitude) {
        // Linear conversion of ADC sample. ADC samples are 12 bits.
        const int16_t shifted_value = static_cast<int16_t>(ain) - static_cast<int16_t>(adc_zero);
        return static_cast<float>(shifted_value)*magnitude/static_cast<float>(ADC_HALF_RANGE);
    }

    float get_kollmorgen_motor_torque(adcsample_t ain) {
        return convert_adcsample(ain, KOLLMORGEN_ADC_ZERO_OFFSET, MAX_KOLLMORGEN_TORQUE);
    }

    float get_kistler_sensor_torque(adcsample_t ain) {
        return convert_adcsample(ain, KISTLER_ADC_ZERO_OFFSET, MAX_KISTLER_TORQUE);
    }
} // sa namespace
