#pragma once
#include <array>

/*
 * This class implements best fit First-Order Adaptive Windowing (FOAW) for
 * velocity estimation from a sampled position as described by:
 *
 * Janabi-Sharifi, Farrokh, Vincent Hayward, and C-SJ Chen.
 * "Discrete-time adaptive windowing for velocity estimation."
 * IEEE Transactions on control systems technology 8.6 (2000): 1003-1009.
 *
 * Note: Only the best fit variant is implemented (not end fit).
 *
 * In this implementation:
 *  T is the numeric type (assumed to be float or double)
 *  N is the maximum window size (size of buffer of sampled positions)
 */

template <typename T, size_t N>
class Foaw {
    public:
        Foaw(T sample_period, T allowed_error);
        void reset();
        void add_position(T x);
        T estimate_velocity() const;
        T sample_period() const;
        T allowed_error() const;

    private:
        std::array<T, N> m_positions; /* circular buffer for sampled positions */
        size_t m_position_index; /* index of oldest entry */
        mutable T m_velocity; /* last calculated velocity */
        T m_T; /* sample period */
        T m_d; /* allowed error between sample and best fit line */
};

#include "foaw.hh"
