#include <cstring>

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

    chSysLock();
    m_sample_thread = chThdCreateI(m_wa_sample_handler_thread,
            sizeof(m_wa_sample_handler_thread), NORMALPRIO+1,
            sample_ibqueue_handler, this);
    chSysUnlock();

    ibqObjectInit(&m_ibqueue, false, m_ib, m_input_buffers_size,
            m_input_buffers_number, nullptr, nullptr);
    chBSemObjectInit(&m_buffer_semaphore, false);
}

template <typename T, size_t N>
void EncoderFoaw<T, N>::start() {
    m_estimator.reset();
    Encoder::start();

    chSysLock();
    ibqResetI(&m_ibqueue);
    chBSemResetI(&m_buffer_semaphore, false);
    chVTSetI(&m_sample_timer, m_sample_period, sample_callback, static_cast<void*>(this));
    chSchWakeupS(m_sample_thread, MSG_OK);
    chSysUnlock();
}

template <typename T, size_t N>
void EncoderFoaw<T, N>::stop() {
    chThdTerminate(m_sample_thread);

    chSysLock();
    ibqResetI(&m_ibqueue);
    chBSemResetI(&m_buffer_semaphore, false);
    chVTResetI(&m_sample_timer);
    chSysUnlock();

    Encoder::stop();
    chThdWait(m_sample_thread);
}

template <typename T, size_t N>
T EncoderFoaw<T, N>::velocity() const {
    chBSemWait(&m_buffer_semaphore);
    T vel = m_estimator.estimate_velocity();
    chBSemSignal(&m_buffer_semaphore);

    return vel;
}

template <typename T, size_t N>
void EncoderFoaw<T, N>::sample_callback(void* p) {
    EncoderFoaw<T, N>* enc = static_cast<EncoderFoaw<T, N>*>(p);
    T count = static_cast<T>(enc->count());

    chSysLockFromISR();
    uint8_t* buf = ibqGetEmptyBufferI(&enc->m_ibqueue);
    if (buf == nullptr) {
        /* sample input buffer queue needs to be resized or sample rate needs to be decrease */
        chSysHalt("input buffer queue full");
        // TODO: handle this case
    }
    std::memcpy(buf, &count, sizeof(T)); /* use memcpy in case R/W is not atomic */
    ibqPostFullBufferI(&enc->m_ibqueue, sizeof(T));
    chVTSetI(&enc->m_sample_timer, enc->m_sample_period, sample_callback, p);
    chSysUnlockFromISR();
}

template <typename T, size_t N>
void EncoderFoaw<T, N>::sample_ibqueue_handler(void* p) {
    EncoderFoaw<T, N>* enc = static_cast<EncoderFoaw<T, N>*>(p);

    /* this thread moves encoder counts into the circular queue when velocity is not being calculated */
    while (!chThdShouldTerminateX()) {
        msg_t msg = ibqGetFullBufferTimeout(&enc->m_ibqueue, TIME_INFINITE);
        if (msg == MSG_OK) {
            /* wait if a velocity calculation is ongoing */
            msg = chBSemWait(&enc->m_buffer_semaphore);

            /* Sem was reset so just release buffer */
            if (msg == MSG_RESET) {
                ibqReleaseEmptyBuffer(&enc->m_ibqueue);
                chThdYield();
                continue;
            }

            T* count = reinterpret_cast<T*>(enc->m_ibqueue.ptr);
            enc->m_estimator.add_position(*count);
            chBSemSignal(&enc->m_buffer_semaphore);

            ibqReleaseEmptyBuffer(&enc->m_ibqueue);
        }
        chThdYield();
    }
}
