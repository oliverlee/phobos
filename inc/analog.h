#pragma once
#include "hal.h"
#include <array>

/*
 * ADC configuration options (set in halconf.h)
 * ADC_USE_WAIT FALSE - ADC to be used asynchronously
 * ADC_USE_MUTUAL_EXCLUSION FALSE - ADC to be used in a single thread
 * HAL_USE_GPT TRUE - Used to sample continuously at a periodic rate

 * ADC driver system settings (set in mcuconf.h)
 * STM32_ADC_USE_ADC1 TRUE
 * STM32_GPT_USE_TIM8 TRUE
 */

template <size_t N>
class Analog {
    public:
        static constexpr eventflags_t adc_eventflag_complete = EVENT_MASK(0);
        static constexpr eventflags_t adc_eventflag_error = EVENT_MASK(1);
        static event_source_t adc_event_source;

        Analog();
        void start(gptcnt_t sample_rate, bool use_events=false);
        void stop();

        // The get_adcX() methods perform an average of the ADC samples
        // contained in the conversion buffer.
        //
        // The numeric type of the result can be specified to prevent overflow
        // if this is a concern or if a different numeric type is desired
        // (e.g. float instead of adcsample_t).
        //
        // The divisor defaults to N to perform an average. This can be changed
        // to allow an increase of resolution if the ADC channels are
        // sufficiently oversampled.
        template <typename T = adcsample_t>
        T get_adc10(T divisor = N) const;
        template <typename T = adcsample_t>
        T get_adc11(T divisor = N) const;
        template <typename T = adcsample_t>
        T get_adc12(T divisor = N) const;
        template <typename T = adcsample_t>
        T get_adc13(T divisor = N) const;
#ifdef STATIC_SIMULATOR_CONFIG
        template <typename T = adcsample_t>
        inline T get_kollmorgen_motor(T divisor = N) const {
            return get_adc12<T>(divisor);
        }
        template <typename T = adcsample_t>
        inline T get_kistler_sensor(T divisor = N) const {
            return get_adc13<T>(divisor);
        }
#endif // STATIC_SIMULATOR_CONIG

    private:
        enum sensor_t: uint8_t {ADC10=0, ADC11, ADC12, ADC13};
#ifdef STATIC_SIMULATOR_CONFIG
        /* use channels ADC11, ADC12, ADC13 */
        static constexpr adc_channels_num_t m_adc_num_channels = 3;
#else // STATIC_SIMULATOR_CONFIG
        /* use channels ADC10, ADC11, ADC12 */
        static constexpr adc_channels_num_t m_adc_num_channels = 3;
#endif // STATIC_SIMULATOR_CONFIG
        static constexpr adc_channels_num_t m_adc_buffer_depth = N;
        static constexpr adc_channels_num_t m_adc_buffer_size =
            m_adc_num_channels * m_adc_buffer_depth;
        std::array<adcsample_t, m_adc_buffer_size> m_adc_buffer;
        template <typename T = adcsample_t>
        T average_adc_conversion_value(sensor_t channel, T divisor = N) const;

        static void adcerrorcallback(ADCDriver* adcp, adcerror_t err);
        static void adccallback(ADCDriver* adcp, adcsample_t* buffer, size_t n);

/*
 * ADC conversion group.
 * Mode:        Continuous, 15(10) samples of 3(2) channels, HW triggered by GPT8-TRGO.
 * Channels:    IN10, IN11, IN12 (IN12, IN13).
 *
 * The GPT clock frequency is set to 1 MHz and the sample frequency will be set
 * using an integer prescaler and may not match the input frequency.
 *
 * Conversion of 3 samples takes
 * (3 conversions)*((144 + 12) ADC clock cycles/conversion)
 * = 468 clocks
 * With STM32_ADCCLK_MAX = 36 MHz, this takes 13 us.
 */
        static constexpr ADCConversionGroup adcgrpcfg_events {
            true,                     /* circular */
            m_adc_num_channels,       /* num channels */
            adccallback,              /* end callback */
            adcerrorcallback,         /* error callback */
            0,                        /* CR1 */
            ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(14),        /* CR2 */
    #ifdef STATIC_SIMULATOR_CONFIG
            ADC_SMPR1_SMP_AN13(ADC_SAMPLE_15) |
                ADC_SMPR1_SMP_AN12(ADC_SAMPLE_15) |
                ADC_SMPR1_SMP_AN11(ADC_SAMPLE_15),
    #else // STATIC_SIMULATOR_CONFIG
            ADC_SMPR1_SMP_AN12(ADC_SAMPLE_15) |
                ADC_SMPR1_SMP_AN11(ADC_SAMPLE_15) |
                ADC_SMPR1_SMP_AN10(ADC_SAMPLE_15),
    #endif // STATIC_SIMULATOR_CONFIG
            0,                        /* SMPR2 */
            ADC_SQR1_NUM_CH(m_adc_num_channels),
            0,                        /* SQR2 */
    #ifdef STATIC_SIMULATOR_CONFIG
            ADC_SQR3_SQ3_N(ADC_CHANNEL_IN13) |
                ADC_SQR3_SQ2_N(ADC_CHANNEL_IN12) |
                ADC_SQR3_SQ1_N(ADC_CHANNEL_IN11)
    #else // STATIC_SIMULATOR_CONFIG
            ADC_SQR3_SQ3_N(ADC_CHANNEL_IN12) |
                ADC_SQR3_SQ2_N(ADC_CHANNEL_IN11) |
                ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10)
    #endif // STATIC_SIMULATOR_CONFIG
        };

        static constexpr ADCConversionGroup adcgrpcfg {
            true,                     /* circular */
            m_adc_num_channels,       /* num channels */
            nullptr,                  /* end callback */
            nullptr,                  /* error callback */
            0,                        /* CR1 */
            ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(14),        /* CR2 */
    #ifdef STATIC_SIMULATOR_CONFIG
            ADC_SMPR1_SMP_AN13(ADC_SAMPLE_15) |
                ADC_SMPR1_SMP_AN12(ADC_SAMPLE_15) |
                ADC_SMPR1_SMP_AN11(ADC_SAMPLE_15),
    #else // STATIC_SIMULATOR_CONFIG
            ADC_SMPR1_SMP_AN12(ADC_SAMPLE_15) |
                ADC_SMPR1_SMP_AN11(ADC_SAMPLE_15) |
                ADC_SMPR1_SMP_AN10(ADC_SAMPLE_15),
    #endif // STATIC_SIMULATOR_CONFIG
            0,                        /* SMPR2 */
            ADC_SQR1_NUM_CH(m_adc_num_channels),
            0,                        /* SQR2 */
    #ifdef STATIC_SIMULATOR_CONFIG
            ADC_SQR3_SQ3_N(ADC_CHANNEL_IN13) |
                ADC_SQR3_SQ2_N(ADC_CHANNEL_IN12) |
                ADC_SQR3_SQ1_N(ADC_CHANNEL_IN11)
    #else // STATIC_SIMULATOR_CONFIG
            ADC_SQR3_SQ3_N(ADC_CHANNEL_IN12) |
                ADC_SQR3_SQ2_N(ADC_CHANNEL_IN11) |
                ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10)
    #endif // STATIC_SIMULATOR_CONFIG
        };

/*
 * GPT8 configuration. This timer is used as trigger for the ADC.
 */
    static constexpr GPTConfig gpt8cfg1 {
      frequency:    1000000U,
      callback:     nullptr,
      cr2:          TIM_CR2_MMS_1,  /* MMS = 010 = TRGO on Update Event. */
      dier:         0U
    };
};

#include "analog.hh"
