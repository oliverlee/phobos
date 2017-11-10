#pragma once
#include "hal.h"
#include <array>

class Analog {
    public:
#ifdef STATIC_SIMULATOR_CONFIG
        /* use channels ADC11, ADC12, ADC13 */
        static constexpr adc_channels_num_t m_ADC_NUM_CHANNELS = 3;
#else // STATIC_SIMULATOR_CONFIG
        /* use channels ADC10, ADC11, ADC12 */
        static constexpr adc_channels_num_t m_ADC_NUM_CHANNELS = 3;
#endif // STATIC_SIMULATOR_CONFIG
        static constexpr adc_channels_num_t m_ADC_BUFFER_DEPTH = 5;
        static constexpr adc_channels_num_t m_ADC_BUFFER_SIZE =
            m_ADC_NUM_CHANNELS * m_ADC_BUFFER_DEPTH;

        Analog();
        void start(gptcnt_t sample_rate, bool use_events=false, uint8_t oversample_rate=0);
        void stop();
        adcsample_t get_adc10() const;
        adcsample_t get_adc11() const;
        adcsample_t get_adc12() const;
        adcsample_t get_adc13() const;
        uint8_t oversample_rate() const;

    private:
        enum sensor_t: uint8_t {ADC10=0, ADC11, ADC12, ADC13};
        std::array<adcsample_t, m_ADC_BUFFER_SIZE> m_adc_buffer;
        uint8_t m_oversample_rate;
        adcsample_t average_adc_conversion_value(sensor_t channel) const;
        adcsample_t get_oversample_mean(sensor_t channel) const;
};
