/*
 * Member function definitions of EncoderFoaw template class.
 * See foawencoder.h for template class declaration.
 */
template <typename T, size_t N>
EncoderFoaw<T, N>::EncoderFoaw(GPTDriver* gptp, const EncoderConfig& config,
                               systime_t sample_period, T allowed_error) :
Encoder(gptp, config),
m_iqhandler(),
m_foaw(static_cast<T>(1.0)/sample_period, allowed_error),
m_sample_period(sample_period) {
    chDbgAssert(sample_period >= 1, "sample period too small");
}

template <typename T, size_t N>
void EncoderFoaw<T, N>::start() {
    m_foaw.reset();
    Encoder::start();
    m_iqhandler.start();
    chSysLock();
    chVTSetI(&m_sample_timer, m_sample_period, sample_callback, static_cast<void*>(this));
    chSchWakeupS(m_sample_thread, MSG_OK);
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
    T vel = m_foaw.estimate_velocity();
    m_iqhandler.signal();
    return vel;
}

template <typename T, size_t N>
void EncoderFoaw<T, N>::sample_callback(void* p) {
    EncoderFoaw<T, N>* enc = static_cast<EncoderFoaw<T, N>*>(p);
    T count = static_cast<T>(enc->count());

    chSysLockFromISR();
    chVTSetI(&enc->m_sample_timer, enc->m_sample_period, sample_callback, p);
    enc->m_iqhandler.insertI(&count);
    chSysUnlockFromISR();
}
