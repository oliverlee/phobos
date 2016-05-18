#include "analog.h"
#include "ch.h"
#include <array>

/*
 * ADC configuration options (set in halconf.h)
 * ADC_USE_WAIT FALSE - ADC to be used asynchronously
 * ADC_USE_MUTUAL_EXCLUSION FALSE - ADC to be used in a single thread

 * ADC driver system settings (set in mcuconf.h)
 * STM32_ADC_USE_ADC1 TRUE
 */

/*
 * ADC conversion group.
 * Mode:        One shot, 1 sample of 3 channels, SW triggered.
 * Channels:    IN10, IN11, IN12.
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

    static const ADCConversionGroup adcgrpcfg1 = {
        false,
        Analog::buffer_size(),
        adccallback,
        adcerrorcallback,
        0,                        /* CR1 */
        ADC_CR2_SWSTART,          /* CR2 */
        ADC_SMPR1_SMP_AN12(ADC_SAMPLE_3) |
        ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3) |
        ADC_SMPR1_SMP_AN10(ADC_SAMPLE_3),
        0,                        /* SMPR2 */
        ADC_SQR1_NUM_CH(Analog::buffer_size()),
        0,                        /* SQR2 */
        ADC_SQR3_SQ3_N(ADC_CHANNEL_IN12) |
        ADC_SQR3_SQ2_N(ADC_CHANNEL_IN11) |
        ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10)
    };
} // namespace


Analog::Analog() : m_adc_buffer() { }

void Analog::start() {
    adcStart(&ADCD1, nullptr);
    adcStartConversion(&ADCD1, &adcgrpcfg1, m_adc_buffer.data(), 1);
}

adcsample_t Analog::get_adc10() const {
    return m_adc_buffer[sensor_t::ADC10];
}

adcsample_t Analog::get_adc11() const {
    return m_adc_buffer[sensor_t::ADC11];
}

adcsample_t Analog::get_adc12() const {
    return m_adc_buffer[sensor_t::ADC12];
}

adc_channels_num_t Analog::buffer_size() {
    return m_adc_buffer_size;
}
