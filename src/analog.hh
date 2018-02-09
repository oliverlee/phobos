#include <limits>
#include <type_traits>

/*
 * Member function definitions of Analog template class.
 * See analog.h for template class declaration.
 */

template <size_t N>
event_source_t Analog<N>::adc_event_source;

template <size_t N>
Analog<N>::Analog() : m_adc_buffer() { }

template <size_t N>
void Analog<N>::start(gptcnt_t sample_rate, bool use_events) {
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
    gptStartContinuous(&GPTD8, gpt8cfg1.frequency/sample_rate);
}

template <size_t N>
void Analog<N>::stop() {
    gptStopTimer(&GPTD8);
    adcStopConversion(&ADCD1);
    adcStop(&ADCD1);
    gptStop(&GPTD8);
}

template <size_t N>
template <typename T>
T Analog<N>::average_adc_conversion_value(sensor_t channel, T divisor) const {
#ifdef STATIC_SIMULATOR_CONFIG
    channel = static_cast<sensor_t>(static_cast<std::underlying_type_t<sensor_t>>(channel) - 1);
#endif // STATIC_SIMULATOR_CONFIG
    static_assert(4096 * N < std::numeric_limits<adcsample_t>::max(),
            "Template parameter N may result in ADC buffer sum overflow.");
    T sum = static_cast<T>(m_adc_buffer[channel]);
    for (unsigned int i = 1; i < m_adc_buffer_depth; ++i) {
        sum += static_cast<T>(m_adc_buffer[channel + i*m_adc_num_channels]);
    }
    return sum/divisor;
}

template <size_t N>
template <typename T>
T Analog<N>::get_adc10(T divisor) const {
    //return m_adc_buffer[sensor_t::ADC10];
#ifdef STATIC_SIMULATOR_CONFIG
    return 0;
#else // STATIC_SIMULATOR_CONFIG
    return average_adc_conversion_value(sensor_t::ADC10, divisor);
#endif // STATIC_SIMULATOR_CONFIG
}

template <size_t N>
template <typename T>
T Analog<N>::get_adc11(T divisor) const {
    //return m_adc_buffer[sensor_t::ADC11];
    return average_adc_conversion_value(sensor_t::ADC11, divisor);
}

template <size_t N>
template <typename T>
T Analog<N>::get_adc12(T divisor) const {
    //return m_adc_buffer[sensor_t::ADC12];
    return average_adc_conversion_value(sensor_t::ADC12, divisor);
}

template <size_t N>
template <typename T>
T Analog<N>::get_adc13(T divisor) const {
    //return m_adc_buffer[sensor_t::ADC13];
#ifdef STATIC_SIMULATOR_CONFIG
    return average_adc_conversion_value(sensor_t::ADC13, divisor);
#else // STATIC_SIMULATOR_CONFIG
    return 0;
#endif // STATIC_SIMULATOR_CONFIG
}

template <size_t N>
void Analog<N>::adcerrorcallback(ADCDriver* adcp, adcerror_t err) {
    (void)adcp;
    (void)err;
    chSysLockFromISR();
    chEvtBroadcastFlagsI(&Analog<N>::adc_event_source, adc_eventflag_error);
    chSysUnlockFromISR();
}

template <size_t N>
void Analog<N>::adccallback(ADCDriver* adcp, adcsample_t* buffer, size_t n) {
    (void)adcp;
    (void)buffer;
    (void)n;
    chSysLockFromISR();
    chEvtBroadcastFlagsI(&Analog<N>::adc_event_source, adc_eventflag_complete);
    chSysUnlockFromISR();
}

template <size_t N>
constexpr ADCConversionGroup Analog<N>::adcgrpcfg_events;

template <size_t N>
constexpr ADCConversionGroup Analog<N>::adcgrpcfg;

template <size_t N>
constexpr GPTConfig Analog<N>::gpt8cfg1;
