#pragma once
#include <array>

namespace filter {

template <typename T, size_t N>
class Median {
    public:
        Median(T initial_value=static_cast<T>(0));
        T output() const; // return median of data in buffer
        T output(T input); // add new value to buffer and return average

    private:
        std::array<T, N> m_data; // circular buffer containing samples
        std::array<T, N> m_sorted_data; // buffer for in-place sorting when finding median
        size_t m_data_index; // index of oldest circular buffer element
        T m_median; // last calculated median
};

} // namespace filter

#include "filter/median.hh"
