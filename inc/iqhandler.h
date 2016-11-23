#pragma once
#include <array>
#include <type_traits>
#include "ch.h"

/*
 * This is a utility class that handles sampled data. Frequently, data is
 * sampled in an interrupt service routine and then saved in a circular buffer,
 * where the data in the circular buffer is used as input to some algorithm.
 * However, without any queue or lock, the ISR can write to the circular buffer
 * algorithm execution. This class adds sampled data to an input queue, allowing
 * delayed insertion to the circular buffer, which may be necessary during
 * algorithm execution.
 *
 * T: type of element in queue/buffer
 * N: number of elements of input queue
 * M: number of elements of circular buffer
 */

typedef bool (*iqcond_t)(void *p);

template <typename T, size_t N, size_t M>
class IQHandler {
    public:
        IQHandler(iqcond_t cond_func=nullptr);
        void start(); /* start queue */
        void stop(); /* stop queue */
        void wait(); /* prevent queue access to circular buffer */
        void signal(); /* allow queue access to circular buffer */

        void insertI(T* element); /* insert element to input queue */
        const std::array<T, M>& circular_buffer() const; /* read access to circular buffer */
        size_t index() const; /* index of oldest element */

    private:
        std::array<T, M> m_circular_buffer; /* circular buffer to used by other threads */
        uint8_t m_iqueue_buffer[BQ_BUFFER_SIZE(N, sizeof(T))]; /* buffer (memory) used by input queue */
        THD_WORKING_AREA(m_wa_iqueue_handler_thread, 128);

        thread_t* m_thread;
        input_buffers_queue_t m_iqueue;
        binary_semaphore_t m_sem;
        iqcond_t m_cond;
        size_t m_buffer_index;

        static void iqueue_handler(void* p); /* thread function */
        void insert_circular_buffer(T* element); /* insert element to circular buffer */
};

#include "iqhandler.hh"
