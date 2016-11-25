#include "foaw.h"

/*
 * Member function definitions of EncoderFoaw template class.
 * See foawencoder.h for template class declaration.
 */
template <typename T, size_t N>
EncoderFoaw<T, N>::EncoderFoaw(GPTDriver* gptp, const EncoderConfig& config,
                               systime_t sample_period, T allowed_error) :
Encoder(gptp, config),
m_iqhandler(),
m_timer_period(sample_period),
m_sample_period(static_cast<T>(sample_period)/CH_CFG_ST_FREQUENCY),
m_allowed_error(allowed_error) {
    chDbgAssert(sample_period >= 1, "sample period too small");
}

template <typename T, size_t N>
void EncoderFoaw<T, N>::start() {
    Encoder::start();
    m_iqhandler.start();
    chSysLock();
    chVTSetI(&m_sample_timer, m_timer_period, sample_callback, static_cast<void*>(this));
    chSysUnlock();
}

template <typename T, size_t N>
void EncoderFoaw<T, N>::stop() {
    chVTReset(&m_sample_timer);
    Encoder::stop();
    m_iqhandler.stop();
}

template <typename T, size_t N>
T EncoderFoaw<T, N>::velocity() {
    m_iqhandler.wait();
    T vel = foaw::estimate_velocity(m_iqhandler.circular_buffer(),
            m_iqhandler.index(), m_sample_period, m_allowed_error);
    m_iqhandler.signal();
    return vel;
}

template <typename T, size_t N>
void EncoderFoaw<T, N>::sample_callback(void* p) {
    EncoderFoaw<T, N>* enc = static_cast<EncoderFoaw<T, N>*>(p);
    T count = static_cast<T>(enc->count());

    chSysLockFromISR();
    chVTSetI(&enc->m_sample_timer, enc->m_timer_period, sample_callback, p);
    enc->m_iqhandler.insertI(&count);
    chSysUnlockFromISR();
}
