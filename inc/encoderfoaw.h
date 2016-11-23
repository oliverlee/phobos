#pragma once
#include "encoder.h"
#include "foaw.h"
#include "ch.h"

template <typename T, size_t N>
class EncoderFoaw: public Encoder {
    public:
        // TODO: GPT for higher sampling frequency instead of virtual timer?
        EncoderFoaw(GPTDriver* gptp, const EncoderConfig& config,
                    systime_t sample_period, T allowed_error);
        virtual void start() override;
        virtual void stop() override;
        T velocity() const;

    private:
        static constexpr size_t m_input_buffers_number = 4;
        static constexpr size_t m_input_buffers_size = sizeof(T);

        uint8_t m_ib[BQ_BUFFER_SIZE(m_input_buffers_number, m_input_buffers_size)];
        THD_WORKING_AREA(m_wa_sample_handler_thread, 128);
        Foaw<T, N> m_estimator;
        thread_t* m_sample_thread;
        input_buffers_queue_t m_ibqueue;
        mutable binary_semaphore_t m_buffer_semaphore;
        virtual_timer_t m_sample_timer;
        const systime_t m_sample_period;

        static void sample_callback(void* p);
        static void sample_ibqueue_handler(void* p);
};

#include "encoderfoaw.hh"
