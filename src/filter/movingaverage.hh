/*
 * Member function definitions of filter::MovingAverage template class.
 * See filter/movingaverage.h for template class declaration.
 */

namespace filter {

template <typename T, size_t N>
MovingAverage<T, N>::MovingAverage(T initial_value) :
m_data(),
m_data_index(0),
m_sum(0) {
    static_assert(N > 0, "Filter size must be greater than 0.");
    m_data.fill(initial_value);
    for (auto& value: m_data) {
        m_sum += value;
    }
}

template <typename T, size_t N>
T MovingAverage<T, N>::output() const {
    return m_sum / N;
}


template <typename T, size_t N>
T MovingAverage<T, N>::output(T input) {
    m_sum += -m_data[m_data_index] + input;
    m_data[m_data_index++] = input;
    if (m_data_index == N) {
        m_data_index = 0;
    }
    return output();
}

} // namespace filter
