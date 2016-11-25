#pragma once
#include <Eigen/Core>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>
#include "types.h"
#include "pose.pb.h"

namespace model {

// TODO: Define bicycle parent and subclasses so member functions:
// - set_moore_parameters()
// - integrate_auxiliary_state()
// - solve_constraint_pitch()
// - accessors
// do not need to be repeated.

class SimpleBicycle {
    public:
        using second_order_matrix_t = Eigen::Matrix<real_t, 2, 2>;

        /* Bicycle model parameters */
        static constexpr real_t default_fs = 200.0; // sample rate [Hz]
        static constexpr real_t default_dt = 1.0/default_fs; // sample time [s]
        static constexpr real_t default_v = 5.0; // forward speed [m/s]

        SimpleBicycle(real_t v = default_v, real_t dt = default_dt);

        void set_v(real_t v);
        void set_dt(real_t dt);
        void reset_pose();
        void update(real_t roll_torque_input, real_t steer_torque_input,
                    real_t yaw_angle_measurement, real_t steer_angle_measurement,
                    real_t rear_wheel_angle_measurement);

        const BicyclePoseMessage& pose() const; /* get most recently computed pose */
        real_t handlebar_feedback_torque() const; /* get most recently computed feedback torque */

        const second_order_matrix_t& M() const;
        const second_order_matrix_t& C1() const;
        const second_order_matrix_t& K0() const;
        const second_order_matrix_t& K2() const;
        const second_order_matrix_t& C() const;
        const second_order_matrix_t& K() const;
        real_t wheelbase() const;
        real_t trail() const;
        real_t steer_axis_tilt() const;
        real_t rear_wheel_radius() const;
        real_t front_wheel_radius() const;
        real_t v() const;
        real_t dt() const;

    private:
        using state_t = Eigen::Matrix<real_t, 4, 1>;
        using auxiliary_state_t = Eigen::Matrix<real_t, 4, 1>;
        enum state_index_t: uint8_t {
            roll_angle = 0,
            steer_angle,
            roll_rate,
            steer_rate,
        };
        enum auxiliary_state_index_t: uint8_t {
            x = 0,
            y,
            pitch_angle,
            yaw_angle,
        };

        second_order_matrix_t m_M;
        second_order_matrix_t m_C1;
        second_order_matrix_t m_K0;
        second_order_matrix_t m_K2;
        second_order_matrix_t m_C;
        second_order_matrix_t m_K;
        BicyclePoseMessage m_pose; /* visualization fields */
        state_t m_x; /*state */
        auxiliary_state_t m_x_aux; /* auxiliary state */
        real_t m_v; /* forward velocity */
        real_t m_dt; /* time step */
        real_t m_T_m; /* handlebar feedback torque*/
        real_t m_w; /* wheelbase */
        real_t m_c; /* trail */
        real_t m_lambda; /* steer axis tilt */
        real_t m_rr; /* rear wheel radius */
        real_t m_rf; /* front wheel radius */
        real_t m_d1; // Moore parameter. Luke calls this cR.
        real_t m_d2; // Moore parameter. Luke calls this ls.
        real_t m_d3; // Moore parameter. Luke calls this cF.

        boost::numeric::odeint::runge_kutta_dopri5<
            auxiliary_state_t, real_t, auxiliary_state_t, real_t,
            boost::numeric::odeint::vector_space_algebra> m_auxiliary_stepper;

        void set_moore_parameters();
        void set_pose();
        void update_state(real_t steer_angle_measurement);
        void update_feedback_torque();
        auxiliary_state_t integrate_auxiliary_state(const state_t& x, const auxiliary_state_t& x_aux);
        real_t solve_constraint_pitch(const state_t& x, real_t guess) const;
}; // class SimpleBicycle

// define simple member functions using inline
inline const BicyclePoseMessage& SimpleBicycle::pose() const {
    return m_pose;
}
inline real_t SimpleBicycle::handlebar_feedback_torque() const {
    return m_T_m;
}
inline const SimpleBicycle::second_order_matrix_t& SimpleBicycle::M() const {
    return m_M;
}
inline const SimpleBicycle::second_order_matrix_t& SimpleBicycle::C1() const {
    return m_C1;
}
inline const SimpleBicycle::second_order_matrix_t& SimpleBicycle::K0() const {
    return m_K0;
}
inline const SimpleBicycle::second_order_matrix_t& SimpleBicycle::K2() const {
    return m_K2;
}
inline const SimpleBicycle::second_order_matrix_t& SimpleBicycle::C() const {
    return m_C;
}
inline const SimpleBicycle::second_order_matrix_t& SimpleBicycle::K() const {
    return m_K;
}
inline real_t SimpleBicycle::wheelbase() const {
    return m_w;
}
inline real_t SimpleBicycle::trail() const {
    return m_c;
}
inline real_t SimpleBicycle::steer_axis_tilt() const {
    return m_lambda;
}
inline real_t SimpleBicycle::rear_wheel_radius() const {
    return m_rr;
}
inline real_t SimpleBicycle::front_wheel_radius() const {
    return m_rf;
}
inline real_t SimpleBicycle::v() const {
    return m_v;
}
inline real_t SimpleBicycle::dt() const {
    return m_dt;
}

} // namespace model
