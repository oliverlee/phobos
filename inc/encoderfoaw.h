#pragma once
#include "encoder.h"
#include "foaw.h"
#include "iqhandler.h"
#include "ch.h"

template <typename T, size_t N>
class EncoderFoaw: public Encoder {
    public:
        // TODO: GPT for higher sampling frequency instead of virtual timer?
        EncoderFoaw(GPTDriver* gptp, const EncoderConfig& config,
                    systime_t sample_period, T allowed_error);
        virtual void start() override;
        virtual void stop() override;
        T velocity();

    private:
        IQHandler<T, 4, N> m_iqhandler;
        Foaw<T, N> m_foaw;
        virtual_timer_t m_sample_timer;
        const systime_t m_sample_period;

        static void sample_callback(void* p);
};

#include "encoderfoaw.hh"
