#include "haptic.h"
/* bicycle submodule imports */
#include "constants.h"

namespace haptic {

HandlebarStatic::HandlebarStatic(model::Bicycle& bicycle) :
    m_bicycle(bicycle) { }

/*
 * Simplified equations of motion are used to simulate the bicycle dynamics.
 * Roll/steer rate and acceleration terms are ignored resulting in:
 *      (g*K0 + v^2*K2) [phi  ] = [T_phi  ]
 *                      [delta] = [T_delta]
 *
 * As T_phi is zero, roll (phi) is dependent on steer (delta). Steer torque can
 * be determined solely from steer angle.
 *
 * The equation of motion governing the *physical* steering assembly is:
 *      I_delta * delta_dd = T_delta + T_m
 *
 * where I_delta is the moment of inertia of the physical steering assembly about the steer axis
 *       delta_dd is the steer angular acceleration determined from the bicycle model
 *       T_delta is the steer torque
 *       T_m is the feedback torque
 *
 * In this case, delta_dd is set to zero as well to simplify calculations resulting in:
 *      T_m = -T_delta
 */
model::real_t HandlebarStatic::feedback_torque(const model::Bicycle::state_t& x, const model::Bicycle::input_t& u) const {
    (void)u;
    const model::real_t v = m_bicycle.v();
    const model::Bicycle::second_order_matrix_t K = constants::g*m_bicycle.K0() + v*v*m_bicycle.K2();
    const uint8_t steer_index = static_cast<uint8_t>(model::Bicycle::state_index_t::steer_angle);

    return -(K(1, 1) - K(0, 1)*K(1, 0)/K(0, 0))*x[steer_index];
}

} //namespace haptic
