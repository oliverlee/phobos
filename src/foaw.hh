#include <cassert>
#include <cmath>

namespace foaw {

template <typename T, size_t N>
T estimate_velocity(const std::array<T, N>& circular_buffer, size_t oldest_index, T sample_period, T allowed_error) {
    assert(sample_period > static_cast<T>(0));
    assert(allowed_error > static_cast<T>(0));
    assert(oldest_index < N);

    size_t newest_index = oldest_index - 1;
    if (oldest_index == 0) {
        newest_index = N - 1;
    }

    T velocity = 0.0;
    for (int n = 1; n < static_cast<int>(N); ++n) {
        T bn = 0.0;
        for (int i = 0; i <= n; ++i) {
            bn += (n - 2*i)*circular_buffer[(newest_index - i + N) % N];
        }
        bn /= sample_period*n*(n + 1)*(n + 2)/6;

        /*
         * The definition of 'an' in the paper doesn't make much sense so assume
         * line passes through last sampled position (which may not be the case
         * with a best fit line).
         */
        T an = circular_buffer[newest_index];
        for (int j = 1; j < n; ++j) {
            T ykj = an - bn * j * sample_period;
            T error = circular_buffer[(newest_index - j + N) % N] - ykj;
            if (std::abs(error) > allowed_error) {
                return velocity;
            }

        }
        velocity = bn;
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
