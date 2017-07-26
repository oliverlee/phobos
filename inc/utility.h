#pragma once
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include "debug.h"
#include "encoder.h"
#include "encoderfoaw.h"

namespace util {
template <typename T>
T wrap(T angle) {
    angle = std::fmod(angle, boost::math::constants::two_pi<T>());
    if (angle >= boost::math::constants::pi<T>()) {
        angle -= boost::math::constants::two_pi<T>();
    }
    if (angle < -boost::math::constants::pi<T>()) {
        angle += boost::math::constants::two_pi<T>();
    }
    return angle;
}

/*
 * Compute the relative position p of x between x_0 and x_1 and return y at
 * the relative position p between y_0 and y_1. The relative position p
 * can be < 0 or > 1.
 */
template <typename T>
constexpr T numeric_map(T x, T x_0, T x_1, T y_0, T y_1) {
    // This implementation works for integers as long as
    // max(abs([ x, x_0, x_1 ])) * max(abs([ y_0, y_1 ])) fits in T.
    // Guarantees that the output y can equal y_1 when using floats/doubles.
    // This is equivalent to the standard form:
    // y = y_0 + (x - x_0)*(y_1 - y_0)/(x_1 - x_0)
    return (y_0*(x_1 - x) + y_1*(x - x_0))/(x_1 - x_0);
}

template <typename T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    // TODO: Replace this with std::clamp once gcc-arm-none-eabi supports it.
    debug_assert(hi > lo, "hi must be greater than lo");
    return std::min(std::max(v, lo), hi);
}

/*
 * Get angle from encoder count (enccnt_t is uint32_t)
 * Convert angle from enccnt_t (unsigned) to corresponding signed type and use negative
 * values for any count over half a revolution.
 */
template <typename T>
T encoder_count(const Encoder& encoder) {
    auto position = static_cast<std::make_signed<enccnt_t>::type>(encoder.count());
    auto rev = static_cast<std::make_signed<enccnt_t>::type>(encoder.config().counts_per_rev);
    if (position > rev / 2) {
        position -= rev;
    }
    return static_cast<T>(position) / rev * boost::math::constants::two_pi<T>();
}

template <typename T, size_t N>
T encoder_rate(const EncoderFoaw<T, N>& encoder) {
    auto rev = static_cast<std::make_signed<enccnt_t>::type>(encoder.config().counts_per_rev);
    return static_cast<T>(encoder.velocity()) / rev * boost::math::constants::two_pi<T>();
}
} // namespace util
