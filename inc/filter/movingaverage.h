#pragma once
#include <array>

namespace filter {

template <typename T, size_t N>
class MovingAverage {
    public:
        MovingAverage(T initial_value=static_cast<T>(0));
        T output() const; // return average of data in buffer
        T output(T input); // add new value to buffer and return average

    private:
        std::array<T, N> m_data; // circular buffer containing samples
        size_t m_data_index; // index of oldest circular buffer element
        T m_sum; // sum of all elements in buffer
};

} // namespace filter

#include "filter/movingaverage.hh"
