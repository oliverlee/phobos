/*
 * Member function definitions of EncoderFoaw template class.
 * See foawencoder.h for template class declaration.
 */
template <typename T, size_t N>
EncoderFoaw<T, N>::EncoderFoaw(GPTDriver* gptp, const EncoderConfig& config,
                               systime_t sample_period, T allowed_error) :
Encoder(gptp, config),
m_estimator(static_cast<T>(1.0)/sample_period, allowed_error),
m_sample_period(sample_period) {
    chDbgAssert(sample_period >= 1, "sample period too small");
}

template <typename T, size_t N>
void EncoderFoaw<T, N>::start() {
    Encoder::start();
    chVTSet(&m_sample_timer, m_sample_period, sample_callback, static_cast<void*>(this));
}

template <typename T, size_t N>
void EncoderFoaw<T, N>::stop() {
    chVTReset(&m_sample_timer);
    Encoder::stop();
}

template <typename T, size_t N>
T EncoderFoaw<T, N>::velocity() const {
    return m_estimator.estimate_velocity();
}

template <typename T, size_t N>
void EncoderFoaw<T, N>::sample_callback(void* p) {
    EncoderFoaw<T, N>* enc = static_cast<EncoderFoaw<T, N>*>(p);
    chSysLockFromISR();
    enc->m_estimator.add_position(static_cast<T>(enc->count()));
    chVTSetI(&enc->m_sample_timer, enc->m_sample_period, sample_callback, p);
    chSysUnlockFromISR();
}
