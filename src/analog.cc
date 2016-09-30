#include "analog.h"
#include "ch.h"
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

/*
 * ADC conversion group.
 * Mode:        Continuous, 1 sample of 3 channels, HW triggered by GPT8-TRGO.
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
        Analog::buffer_size(),    /* num channels */
        adccallback,              /* end callback */
        adcerrorcallback,         /* error callback */
        0,                        /* CR1 */
        ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(14),        /* CR2 */
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

    const ADCConversionGroup adcgrpcfg = {
        true,                     /* circular */
        Analog::buffer_size(),    /* num channels */
        nullptr,                  /* end callback */
        nullptr,                  /* error callback */
        0,                        /* CR1 */
        ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(14),        /* CR2 */
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

void Analog::start(gptcnt_t sample_rate, bool use_events) {
    gptStart(&GPTD8, &gpt8cfg1);
    adcStart(&ADCD1, nullptr);
    if (use_events) {
        adcStartConversion(&ADCD1, &adcgrpcfg_events, m_adc_buffer.data(), 1);
    } else {
        adcStartConversion(&ADCD1, &adcgrpcfg, m_adc_buffer.data(), 1);
    }
    gptStartContinuous(&GPTD8, gpt8cfg1.frequency/sample_rate);
}

void Analog::stop() {
    gptStopTimer(&GPTD8);
    adcStopConversion(&ADCD1);
    adcStop(&ADCD1);
    gptStop(&GPTD8);
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