#include "haptic.h"
/* bicycle submodule imports */
#include "constants.h"
#include <type_traits>

namespace haptic {

Handlebar0::Handlebar0(model::Bicycle& bicycle) :
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
model::real_t Handlebar0::torque(const model::Bicycle::state_t& x, const model::Bicycle::input_t& u) const {
    (void)u;
    const model::real_t v = m_bicycle.v();
    const model::Bicycle::second_order_matrix_t K = constants::g*m_bicycle.K0() + v*v*m_bicycle.K2();

    return -(K(1, 1) - K(0, 1)*K(1, 0)/K(0, 0))*
        model::Bicycle::get_state_element(x, model::Bicycle::state_index_t::steer_angle);
}

Handlebar2::Handlebar2(model::Bicycle& bicycle, model::real_t moment_of_inertia) :
    m_bicycle(bicycle),
    m_I_delta(moment_of_inertia) { }

/*
 * The equations of motion for the Whipple model can be written as:
 *   M [phi_dd  ] + v*C1 [phi_d  ] + K [phi  ] = [T_phi  ]
 *     [delta_dd]        [delta_d]     [delta] = [T_delta]
 * where v*C1 is used to distinguish the "damping" matrix from the state
 * space output matrix C. In this simulation, T_phi defined to be 0 as there
 * is no way for the user nor the environment to supply a roll torque.
 *
 * The dynamics of the handlebar are governed by the following equation of
 * motion:
 *  I_delta * delta_dd = T_delta + T_m.
 *
 * Note: Positive torque and steer angle is clockwise as seen from the rider
 * looking down at the handlebars.
 *
 * In standard state space form, the equations of motion for the system are:
 *  [phi_d   ] = A [phi    ] + B [T_phi]
 *  [delta_d ]     [delta  ]     [T_delta]
 *  [phi_dd  ]     [phi_d  ]
 *  [delta_dd]     [delta_d]
 *
 * As we need an estimate of the steer angle acceleration, we use the last row
 * of A and B _with_ the assumption that the full state is available. As the
 * state vector is appended with yaw angle in this implementation, this must be
 * accounted for when calculating delta_dd.
 *
 * The output of this function is very susceptible to error/noise in the
 * state or noise in the input. For use with equipment, it is suggested to
 * filter the returned value.
 */
model::real_t Handlebar2::torque(const model::Bicycle::state_t& x, const model::Bicycle::input_t& u) const {
    static constexpr auto steer_rate_index =
        static_cast<typename std::underlying_type<model::Bicycle::state_index_t>::type>(
                model::Bicycle::state_index_t::steer_rate);

    model::real_t steer_acceleration = (m_bicycle.A().row(steer_rate_index)*x + m_bicycle.B().row(steer_rate_index)*u).value();
    return steer_acceleration*m_I_delta - model::Bicycle::get_input_element(u, model::Bicycle::input_index_t::steer_torque);
}

model::real_t Handlebar2::moment_of_inertia() const {
    return m_I_delta;
}
} //namespace haptic
