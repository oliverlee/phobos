#include <cassert>
#include <cmath>

namespace foaw {

template <typename T, size_t N>
T estimate_velocity(const std::array<T, N>& circular_buffer, size_t oldest_index,
        T sample_period, T allowed_error, T overflow_value) {
    assert(sample_period > static_cast<T>(0));
    assert(allowed_error > static_cast<T>(0));
    assert(oldest_index < N);
    assert(overflow_value >= static_cast<T>(0));

    size_t newest_index = oldest_index - 1;
    if (oldest_index == 0) {
        newest_index = N - 1;
    }
    T x0 = circular_buffer[newest_index];
    bool overflow = false;
    bool done = false;

    T velocity = 0.0;
    for (int n = 1; n < static_cast<int>(N); ++n) {
        if (overflow_value > static_cast<T>(0)) {
            for (int i = 1; i <= n; ++i) {
                T* xp = const_cast<T*>(&circular_buffer[(newest_index - i + N) % N]);
                /*
                 * If overflow value is greater than 0, account for position
                 * being able to overflow or underflow. Check each new sample
                 * to determine if overflow/underflow has occurred and modify the
                 * buffer accordingly.
                 *
                 * This check only occurs to the last element in the inner loop to
                 * avoid checking a specific index multiple times.
                 */
                if (i == n) {
                    /*
                     * Assume a difference of 'overflow_value' cannot occurs between
                     * two adjacent samples. If so, the signal is not sampled fast
                     * enough.
                     */
                    const T dx = x0 - *xp;
                    if (std::abs(dx) > (overflow_value/2)) {
                        *xp += std::copysign(overflow_value, dx);
                        overflow = true;
                    }
                    x0 = *xp;
                }
            }
        }

        // use end fit FOAW
        const T yk = circular_buffer[newest_index];
        const T ykn = circular_buffer[(newest_index - n + N) % N];
        const T bn = (yk - ykn)/(n*sample_period);

        /*
         * The definition of 'an' in the paper doesn't make much sense so assume
         * line passes through last sampled position (which may not be the case
         * with a best fit line).
         */
        const T an = yk;
        for (int j = 1; j <= n; ++j) {
            const T ykj = an - bn*j*sample_period;
            const T x = circular_buffer[(newest_index - j + N) % N];
            const T error = x - ykj;
            if (std::abs(error) > allowed_error) {
                done = true;
                break;
            }

        }
        if (done) {
            break;
        }
        velocity = bn;
    }

    /* Restore buffer values if overflow occurred. */
    if (overflow) {
        for (unsigned int i = 0; i < N; ++i) {
            T* xp = const_cast<T*>(&circular_buffer[i]);
            if (*xp < static_cast<T>(0)) {
                *xp += overflow_value;
            } else if (*xp >= overflow_value) {
                *xp -= overflow_value;
            }
        }
    }
    return velocity;
}

} // namespace foaw

/*
 * Member function definitions of Foaw template class.
 * See foaw.h for template class declaration.
 */
template <typename T, size_t N>
Foaw<T, N>::Foaw(T sample_period, T allowed_error) :
    m_positions(),
    m_position_index(0),
    m_velocity(0.0),
    m_T(sample_period),
    m_d(allowed_error) { }

template <typename T, size_t N>
void Foaw<T, N>::reset() {
    m_positions.fill(0.0);
}

template <typename T, size_t N>
void Foaw<T, N>::add_position(T x) {
    m_positions[m_position_index++] = x;
    if (m_position_index == N) {
        m_position_index = 0;
    }
}

template <typename T, size_t N>
T Foaw<T, N>::estimate_velocity() {
    m_velocity = foaw::estimate_velocity(m_positions, m_position_index, m_T, m_d);
    return m_velocity;
}

template <typename T, size_t N>
T Foaw<T, N>::sample_period() const {
    return m_T;
}

template <typename T, size_t N>
T Foaw<T, N>::allowed_error() const {
    return m_d;
}
