#pragma once
#include <boost/math/constants/constants.hpp>
#include "encoder.h"
#include "encoderfoaw.h"

namespace angle {
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
} // namespace angle
