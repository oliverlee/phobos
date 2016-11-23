#include <cstring>

/*
 * Member function definitions of IQHandler template class.
 * See iqhandler.h for template class declaration.
 */
template <typename T, size_t M, size_t N>
IQHandler<T, M, N>::IQHandler(iqcond_t cond_func) :
m_cond(cond_func) {
    static_assert(M < N, "Input queue must be smaller than circular buffer.");

    m_thread = chThdCreateI(m_wa_iqueue_handler_thread,
            sizeof(m_wa_iqueue_handler_thread), NORMALPRIO + 1,
            iqueue_handler, this);
    ibqObjectInit(&m_iqueue, false, m_iqueue_buffer, sizeof(T), M, nullptr, nullptr);
    chBSemObjectInit(&m_sem, false); /* set to not taken */
}

template <typename T, size_t M, size_t N>
void IQHandler<T, M, N>::start() {
    ibqResetI(&m_iqueue);
    chBSemResetI(&m_sem, false); /* set to not taken*/
    chThdStart(m_thread);
}

template <typename T, size_t M, size_t N>
void IQHandler<T, M, N>::stop() {
    chThdTerminate(m_thread);
    ibqResetI(&m_iqueue);
    chBSemResetI(&m_sem, true); /* set to taken */
    chThdWait(m_thread);
}

template <typename T, size_t M, size_t N>
void IQHandler<T, M, N>::wait() {
    chBSemWait(&m_sem);
}

template <typename T, size_t M, size_t N>
void IQHandler<T, M, N>::signal() {
    chBSemSignal(&m_sem);
}

template <typename T, size_t M, size_t N>
void IQHandler<T, M, N>::insertI(T* element) {
    chDbgCheckClassI();

    uint8_t* buf = ibqGetEmptyBufferI(&m_iqueue);
    if (buf == nullptr) {
        /* queue elements not consumed quickly enough */
        chSysHalt("input queue full");
        // TODO: handle this case
    }
    std::memcpy(buf, element, sizeof(T));
    ibqPostFullBufferI(&m_iqueue, sizeof(T));
}

template <typename T, size_t M, size_t N>
const std::array<T, N>& IQHandler<T, M, N>::circular_buffer() const {
    return m_circular_buffer;
}

template <typename T, size_t M, size_t N>
size_t IQHandler<T, M, N>::index() const {
    return m_buffer_index;
}

template <typename T, size_t M, size_t N>
void IQHandler<T, M, N>::iqueue_handler(void* p) {
    auto obj = static_cast<IQHandler<T, M, N>*>(p);
    while (!chThdShouldTerminateX()) {
        if (ibqGetFullBufferTimeout(&obj->m_iqueue, TIME_INFINITE) == MSG_OK) {
            iqcond_t condition = obj->m_cond; /* get condition to check if element should be inserted */
            if ((condition == nullptr) || (condition(p))) {
                if (chBSemWait(&obj->m_sem) == MSG_RESET) { /* wait for algorithm calculation to finish */
                    /* Sem reset due to start/stop */
                    ibqReleaseEmptyBuffer(&obj->m_iqueue);
                    chThdYield();
                    continue;
                }
                obj->insert_circular_buffer(reinterpret_cast<T*>(obj->m_iqueue.ptr));
                chBSemSignal(&obj->m_sem);
            }
            ibqReleaseEmptyBuffer(&obj->m_iqueue);
        }
        chThdYield();
    }
}

template <typename T, size_t M, size_t N>
void IQHandler<T, M, N>::insert_circular_buffer(T* element) {
    std::memcpy(&m_circular_buffer[m_buffer_index++], element, sizeof(T));
    if (m_buffer_index == N) {
        m_buffer_index = 0;
    }
}
