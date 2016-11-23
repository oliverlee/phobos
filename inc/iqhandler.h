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
 */

/* forward declare template class for friend declaration of member function
 * M: polynomial order
 * N: encoder event buffer length
 * O: skip order
 */
template <size_t M, size_t N, size_t O>
class EncoderHots;

typedef bool (*iqcond_t)(void *p);

/*
 * T: type of element in queue/buffer
 * M: number of elements of input queue
 * N: number of elements of circular buffer
 */
template <typename T, size_t M, size_t N>
class IQHandler {
    public:
        IQHandler(iqcond_t cond_func=nullptr);
        void start(); /* start queue */
        void stop(); /* stop queue */
        void wait(); /* prevent queue access to circular buffer */
        void signal(); /* allow queue access to circular buffer */

        void insertI(T* element); /* insert element to input queue */
        const std::array<T, N>& circular_buffer() const; /* read access to circular buffer */
        size_t index() const; /* index of oldest element */

    private:
        std::array<T, N> m_circular_buffer; /* circular buffer to used by other threads */
        uint8_t m_iqueue_buffer[BQ_BUFFER_SIZE(M, sizeof(T))]; /* buffer (memory) used by input queue */
        THD_WORKING_AREA(m_wa_iqueue_handler_thread, 128);

        thread_t* m_thread;
        input_buffers_queue_t m_iqueue;
        binary_semaphore_t m_sem;
        iqcond_t m_cond;
        size_t m_buffer_index;

        static void iqueue_handler(void* p); /* thread function */
        void insert_circular_buffer(T* element); /* insert element to circular buffer */

        template <size_t A, size_t C>
        friend void EncoderHots<A, N, C>::ab_callback(EXTDriver*, expchannel_t);
};

#include "iqhandler.hh"
