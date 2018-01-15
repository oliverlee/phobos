#include "analog.h"
#include "ch.h"
#include <array>
#include <type_traits>

/*
 * ADC configuration options (set in halconf.h)
 * ADC_USE_WAIT FALSE - ADC to be used asynchronously
 * ADC_USE_MUTUAL_EXCLUSION FALSE - ADC to be used in a single thread
 * HAL_USE_GPT TRUE - Used to sample continuously at a periodic rate

 * ADC driver system settings (set in mcuconf.h)
 * STM32_ADC_USE_ADC1 TRUE
 * STM32_GPT_USE_TIM8 TRUE
 */

/*
 * ADC conversion group.
 * Mode:        Continuous, 8 samples of 3 channels, HW triggered by GPT8-TRGO.
 * Channels:    IN10, IN11, IN12.
 *
 * The GPT clock frequency is set to 1 MHz and the sample frequency will be set
 * using an integer prescaler and may not match the input frequency.
 *
 * TODO: move functions from unnamed namespace to private static methods
 */

namespace {
    const eventflags_t adc_eventflag_complete = EVENT_MASK(0);
    const eventflags_t adc_eventflag_error = EVENT_MASK(1);
    event_source_t adc_event_source;

    void adcerrorcallback(ADCDriver* adcp, adcerror_t err) {
        (void)adcp;
        (void)err;
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&adc_event_source, adc_eventflag_error);
        chSysUnlockFromISR();
    }
    void adccallback(ADCDriver* adcp, adcsample_t* buffer, size_t n) {
        (void)adcp;
        (void)buffer;
        (void)n;
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&adc_event_source, adc_eventflag_complete);
        chSysUnlockFromISR();
    }

    const ADCConversionGroup adcgrpcfg_events = {
        true,                     /* circular */
        Analog::num_channels(),   /* num channels */
        adccallback,              /* end callback */
        adcerrorcallback,         /* error callback */
        0,                        /* CR1 */
        ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(14),        /* CR2 */
#ifdef STATIC_SIMULATOR_CONFIG
        ADC_SMPR1_SMP_AN13(ADC_SAMPLE_144) |
            ADC_SMPR1_SMP_AN12(ADC_SAMPLE_144) |
            ADC_SMPR1_SMP_AN11(ADC_SAMPLE_144),
#else // STATIC_SIMULATOR_CONFIG
        ADC_SMPR1_SMP_AN12(ADC_SAMPLE_144) |
            ADC_SMPR1_SMP_AN11(ADC_SAMPLE_144) |
            ADC_SMPR1_SMP_AN10(ADC_SAMPLE_144),
#endif // STATIC_SIMULATOR_CONFIG
        0,                        /* SMPR2 */
        ADC_SQR1_NUM_CH(Analog::num_channels()),
        0, /* SQR2 */
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

    const ADCConversionGroup adcgrpcfg = {
        true,                     /* circular */
        Analog::num_channels(),   /* num channels */
        nullptr,                  /* end callback */
        nullptr,                  /* error callback */
        0,                        /* CR1 */
        ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(14),        /* CR2 */
#ifdef STATIC_SIMULATOR_CONFIG
        ADC_SMPR1_SMP_AN13(ADC_SAMPLE_144) |
            ADC_SMPR1_SMP_AN12(ADC_SAMPLE_144) |
            ADC_SMPR1_SMP_AN11(ADC_SAMPLE_144),
#else // STATIC_SIMULATOR_CONFIG
        ADC_SMPR1_SMP_AN12(ADC_SAMPLE_144) |
            ADC_SMPR1_SMP_AN11(ADC_SAMPLE_144) |
            ADC_SMPR1_SMP_AN10(ADC_SAMPLE_144),
#endif // STATIC_SIMULATOR_CONFIG
        0,                        /* SMPR2 */
        ADC_SQR1_NUM_CH(Analog::num_channels()),
        0, /* SQR2 */
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
     * Conversion of 3 samples takes
     * (3 conversions)*((144 + 12) ADC clock cycles/conversion)
     * = 468 clocks
     * With STM32_ADCCLK_MAX = 36 MHz, this takes 13 us.
     */

    /*
     * GPT8 configuration. This timer is used as trigger for the ADC.
     */
    const GPTConfig gpt8cfg1 = {
      frequency:    1000000U,
      callback:     nullptr,
      cr2:          TIM_CR2_MMS_1,  /* MMS = 010 = TRGO on Update Event. */
      dier:         0U
    };
} // namespace


Analog::Analog() : m_adc_buffer() { }

//void Analog::start(gptcnt_t sample_rate, bool use_events) {
void Analog::start(bool use_events) {
#ifdef STATIC_SIMULATOR_CONFIG
    /*
     * We manually toggle the Kistler torque sensor measurement line
     * since the output will not be valid unless measurement is enabled _after_
     * the torque sensor is powered on. The analog signal is read from ADC13.
     */
    palClearLine(LINE_TORQUE_MEAS_EN);
    chThdSleepMilliseconds(100);
    palSetLine(LINE_TORQUE_MEAS_EN);
#endif // STATIC_SIMULATOR_CONFIG
    gptStart(&GPTD8, &gpt8cfg1);
    adcStart(&ADCD1, nullptr);
    if (use_events) {
        adcStartConversion(&ADCD1, &adcgrpcfg_events,
                m_adc_buffer.data(), m_adc_buffer_depth);
    } else {
        adcStartConversion(&ADCD1, &adcgrpcfg,
                m_adc_buffer.data(), m_adc_buffer_depth);
    }

    constexpr gptcnt_t sample_rate = 8000; // 8 kHz
    gptStartContinuous(&GPTD8, gpt8cfg1.frequency/sample_rate);
}

void Analog::stop() {
    gptStopTimer(&GPTD8);
    adcStopConversion(&ADCD1);
    adcStop(&ADCD1);
    gptStop(&GPTD8);
}

adcsample_t Analog::average_adc_conversion_value(sensor_t channel) const {
#ifdef STATIC_SIMULATOR_CONFIG
    channel = static_cast<sensor_t>(static_cast<std::underlying_type_t<sensor_t>>(channel) - 1);
#endif // STATIC_SIMULATOR_CONFIG
    adcsample_t sum = m_adc_buffer[channel];
    for (unsigned int i = 1; i < m_adc_buffer_depth; ++i) {
        sum += m_adc_buffer[channel + i*m_adc_num_channels];
    }
    return sum / m_adc_buffer_depth;
}

float Analog::average_float_adc_conversion_value(sensor_t channel) const {
#ifdef STATIC_SIMULATOR_CONFIG
    channel = static_cast<sensor_t>(static_cast<std::underlying_type_t<sensor_t>>(channel) - 1);
#endif // STATIC_SIMULATOR_CONFIG
    adcsample_t sum = m_adc_buffer[channel];
    for (unsigned int i = 1; i < m_adc_buffer_depth; ++i) {
        sum += m_adc_buffer[channel + i*m_adc_num_channels];
    }
    return static_cast<float>(sum) / m_adc_buffer_depth;
}

adcsample_t Analog::get_adc10() const {
    //return m_adc_buffer[sensor_t::ADC10];
#ifdef STATIC_SIMULATOR_CONFIG
    return 0;
#else // STATIC_SIMULATOR_CONFIG
    return average_adc_conversion_value(sensor_t::ADC10);
#endif // STATIC_SIMULATOR_CONFIG
}

adcsample_t Analog::get_adc11() const {
    //return m_adc_buffer[sensor_t::ADC11];
    return average_adc_conversion_value(sensor_t::ADC11);
}

adcsample_t Analog::get_adc12() const {
    //return m_adc_buffer[sensor_t::ADC12];
    return average_adc_conversion_value(sensor_t::ADC12);
}

adcsample_t Analog::get_adc13() const {
    //return m_adc_buffer[sensor_t::ADC13];
#ifdef STATIC_SIMULATOR_CONFIG
    return average_adc_conversion_value(sensor_t::ADC13);
#else // STATIC_SIMULATOR_CONFIG
    return 0;
#endif // STATIC_SIMULATOR_CONFIG
}

float Analog::getf_adc10() const {
    //return m_adc_buffer[sensor_t::ADC10];
#ifdef STATIC_SIMULATOR_CONFIG
    return 0;
#else // STATIC_SIMULATOR_CONFIG
    return average_float_adc_conversion_value(sensor_t::ADC10);
#endif // STATIC_SIMULATOR_CONFIG
}

float Analog::getf_adc11() const {
    //return m_adc_buffer[sensor_t::ADC11];
    return average_float_adc_conversion_value(sensor_t::ADC11);
}

float Analog::getf_adc12() const {
    //return m_adc_buffer[sensor_t::ADC12];
    return average_float_adc_conversion_value(sensor_t::ADC12);
}

float Analog::getf_adc13() const {
    //return m_adc_buffer[sensor_t::ADC13];
#ifdef STATIC_SIMULATOR_CONFIG
    return average_float_adc_conversion_value(sensor_t::ADC13);
#else // STATIC_SIMULATOR_CONFIG
    return 0;
#endif // STATIC_SIMULATOR_CONFIG
}

adc_channels_num_t Analog::buffer_size() {
    return m_adc_buffer_size;
}

adc_channels_num_t Analog::num_channels() {
    return m_adc_num_channels;
}
