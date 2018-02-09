#pragma once
#include "saconfig.h"
#include "utility.h"
#include "analog.h"
#include <type_traits>

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

    template <typename T>
    float convert_adcsample(T ain, adcsample_t adc_zero, float magnitude) {
        // Linear conversion of ADC sample. ADC samples are unsigned int of
        // 12 bits by default. The type can be coerced by casting the type of
        // parameter 'ain'.
        using signed_t = typename std::conditional<
            std::is_unsigned<T>::value,
            std::make_signed<T>,
            std::common_type<T>>::type::type;
        const signed_t shifted_value = static_cast<signed_t>(ain) - static_cast<signed_t>(adc_zero);
        return static_cast<float>(shifted_value)*magnitude/static_cast<float>(ADC_HALF_RANGE);
    }

    template <size_t N>
    float get_kollmorgen_motor_torque(const Analog<N>& analog) {
        const adcsample_t ain = analog.get_kollmorgen_motor();
        return convert_adcsample(ain, KOLLMORGEN_ADC_ZERO_OFFSET, MAX_KOLLMORGEN_TORQUE);
    }

    template <size_t N>
    float get_kistler_sensor_torque(const Analog<N>& analog) {
        // Note that sensor sign is reversed.
        // Decreasing adcsample_t values result in _increasing_ torque.
        // Different constants are used for positive and negative torque based
        // on measured sensor behavior.
        const adcsample_t ain = analog.get_kistler_sensor();
        if (ain > KISTLER_ADC_ZERO_OFFSET_NEGATIVE) {
            return convert_adcsample(
                    ain, KISTLER_ADC_ZERO_OFFSET_NEGATIVE, MAX_KISTLER_TORQUE_NEGATIVE);
        } else if (ain < KISTLER_ADC_ZERO_OFFSET_POSITIVE) {
            return convert_adcsample(
                    ain, KISTLER_ADC_ZERO_OFFSET_POSITIVE, MAX_KISTLER_TORQUE_POSITIVE);
        } // else
        return 0.0f;
    }
} // sa namespace
