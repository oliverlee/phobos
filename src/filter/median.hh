#include <algorithm>

/*
 * Member function definitions of filter::Median template class.
 * See filter/median.h for template class declaration.
 */

namespace filter {

template <typename T, size_t N>
Median<T, N>::Median(T initial_value) :
m_data(),
m_sorted_data(),
m_data_index(0) {
    static_assert(N > 0, "Filter size must be greater than 0.");
    m_data.fill(initial_value);
    m_median = initial_value;
}

template <typename T, size_t N>
T Median<T, N>::output() const {
    return m_median;
}


template <typename T, size_t N>
T Median<T, N>::output(T input) {
    m_data[m_data_index++] = input;
    if (m_data_index == N) {
        m_data_index = 0;
    }

    m_sorted_data = m_data;
    std::nth_element(m_sorted_data.begin(),
            m_sorted_data.begin() + N/2,
            m_sorted_data.end());
    m_median = m_sorted_data[N/2];

    return output();
}

} // namespace filter
