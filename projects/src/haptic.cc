#include "haptic.h"
/* bicycle submodule imports */
#include "constants.h"
#include <type_traits>

namespace haptic {

Handlebar0::Handlebar0(model_t& bicycle) :
    m_bicycle(bicycle) { }

/*
 * Simplified equations of motion are used to calculate the handlebar feedback torque.
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
real_t Handlebar0::torque(const model_t::state_t& x, const model_t::input_t& u) const {
    (void)u;
    const real_t v = m_bicycle.v();
    const model_t::second_order_matrix_t K = constants::g*m_bicycle.K0() + v*v*m_bicycle.K2();

    return -(K(1, 1) - K(0, 1)*K(1, 0)/K(0, 0))*
        model_t::get_state_element(x, model_t::state_index_t::steer_angle);
}

Handlebar1::Handlebar1(model_t& bicycle) :
    m_bicycle(bicycle) { }

/*
 * Simplified equations of motion are used to calculate the handlebar feedback torque.
 * Roll/steer acceleration terms.
 *
 * The equations of motion from the Whipple model are:
 *      [M_00 M_01] [phi_ddot  ] + v*C1 [phi_dot  ] + (g*K0 + v^2*K2) [phi  ] = [T_phi  ]
 *      [M_10 M_11] [delta_ddot]        [delta_dot]                   [delta] = [T_delta]
 *
 * We assume T_phi is zero, and set M_01 = M_10 to zero.
 *      [M_00    0] [phi_ddot  ] + v*C1 [phi_dot  ] + (g*K0 + v^2*K2) [phi  ] = [      0]
 *      [   0 M_11] [delta_ddot]        [delta_dot]                   [delta] = [T_delta]
 *
 * The equation of motion governing the *physical* steering assembly is:
 *      I_delta * delta_dd = T_delta + T_m
 *
 * where I_delta is the moment of inertia of the physical steering assembly about the steer axis
 *       delta_dd is the steer angular acceleration determined from the bicycle model
 *       T_delta is the steer torque
 *       T_m is the feedback torque
 * and I_delta = M_11
 *
 * Taking the bicycle steer equation and physical steering assembly equations to be equal and
 * setting the motor torque to encompass the remaining terms
 *      -T_m = M_10*phi_ddot + C_10*phi_dot + C_11*delta_dot + K_10*phi + K_11*delta
 *
 * and with simplifying zeros
 *      -T_m = C_10*phi_dot + C_11*delta_dot + K_10*phi + K_11*delta
 *
 * For more information, refer to: https://github.com/oliverlee/phobos/issues/161
 */
real_t Handlebar1::torque(const model_t::state_t& x, const model_t::input_t& u) const {
    (void)u;
    const real_t v = m_bicycle.v();
    const model_t::second_order_matrix_t K = constants::g*m_bicycle.K0() + v*v*m_bicycle.K2();
    const model_t::second_order_matrix_t C = v*m_bicycle.C1();

    return -(C(1, 0)*model_t::get_state_element(x, model_t::state_index_t::roll_rate) +
             C(1, 1)*model_t::get_state_element(x, model_t::state_index_t::steer_rate) +
             K(1, 0)*model_t::get_state_element(x, model_t::state_index_t::roll_angle) +
             K(1, 1)*model_t::get_state_element(x, model_t::state_index_t::steer_angle));
}

Handlebar2::Handlebar2(model_t& bicycle, real_t moment_of_inertia) :
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
real_t Handlebar2::torque(const model_t::state_t& x, const model_t::input_t& u) const {
    static constexpr auto steer_rate_index =
        static_cast<typename std::underlying_type<model_t::state_index_t>::type>(
                model_t::state_index_t::steer_rate);

    real_t steer_acceleration = (m_bicycle.A().row(steer_rate_index)*x + m_bicycle.B().row(steer_rate_index)*u).value();
    return steer_acceleration*m_I_delta - model_t::get_input_element(u, model_t::input_index_t::steer_torque);
}

real_t Handlebar2::moment_of_inertia() const {
    return m_I_delta;
}
} //namespace haptic
