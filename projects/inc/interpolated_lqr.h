#pragma once
#include <Eigen/Core>
#include "constants.h" // real_t
#include "discrete_linear.h"

namespace controller {
using real_t = model::real_t;

/*
 * This class is an interpolated LQR controller for a DiscreteLinear model.
 * It holds two precomputed feedback gain matrices and performs interpolation
 * between the two when calculating the feedback gain.
 */

template <typename T>
class InterpolatedLqr {
    static_assert(std::is_base_of<model::DiscreteLinearBase, T>::value, "Invalid template parameter type for InterpolatedLqr");
    public:
        using state_t = typename T::state_t;
        using input_t = typename T::input_t;
        using feedback_gain_t = typename Eigen::Matrix<real_t, T::m, T::n>;
        InterpolatedLqr(feedback_gain_t K0, feedback_gain_t K1) : m_K0(K0), m_Kd(K1 - K0) { }

        input_t control_calculate(const state_t& x, real_t interp) const { 
            const feedback_gain_t K = m_K0 + interp*m_Kd;
            return K*x;
        }

    private:
        const feedback_gain_t m_K0;
        const feedback_gain_t m_Kd;
};

} // namespace controller
