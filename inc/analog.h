#pragma once
#include "hal.h"
#include <array>

class Analog {
    public:
        Analog();
        //void start(gptcnt_t sample_rate, bool use_events=false);
        void start(bool use_events=false);
        void stop();
        adcsample_t get_adc10() const;
        adcsample_t get_adc11() const;
        adcsample_t get_adc12() const;
        adcsample_t get_adc13() const;
        float getf_adc10() const;
        float getf_adc11() const;
        float getf_adc12() const;
        float getf_adc13() const;
        static adc_channels_num_t buffer_size();
        static adc_channels_num_t num_channels();

    private:
        enum sensor_t: uint8_t {ADC10=0, ADC11, ADC12, ADC13};
#ifdef STATIC_SIMULATOR_CONFIG
        /* use channels ADC11, ADC12, ADC13 */
        static constexpr adc_channels_num_t m_adc_num_channels = 3;
#else // STATIC_SIMULATOR_CONFIG
        /* use channels ADC10, ADC11, ADC12 */
        static constexpr adc_channels_num_t m_adc_num_channels = 3;
#endif // STATIC_SIMULATOR_CONFIG
        static constexpr adc_channels_num_t m_adc_buffer_depth = 8;
        static constexpr adc_channels_num_t m_adc_buffer_size =
            m_adc_num_channels * m_adc_buffer_depth;
        std::array<adcsample_t, m_adc_buffer_size> m_adc_buffer;
        adcsample_t average_adc_conversion_value(sensor_t channel) const;
        float average_float_adc_conversion_value(sensor_t channel) const;
};
