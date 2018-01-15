#include <cmath>
#include <type_traits>
#include <boost/math/special_functions/round.hpp>
#include "bicycle/kinematic.h"
#include "kalman.h"
/*
 * Member function definitions of sim::Bicycle template class.
 * See simbicycle.h for template class declaration.
 */

namespace sim {

template <typename Model>
Bicycle<Model>::Bicycle(real_t v, real_t dt) :
m_model(v, dt),
m_full_state(full_state_t::Zero()),
m_pose(BicyclePoseMessage_init_zero),
m_input(input_t::Zero()),
m_measurement(measurement_t::Zero()) {
    chBSemObjectInit(&m_state_sem, false); // initialize to not taken
}

template <typename Model>
void Bicycle<Model>::set_v(real_t v)  {
    using namespace boost::math::policies;
    using quantize_policy = policy<rounding_error<ignore_error>>;
    static constexpr quantize_policy policy;

    real_t v_quantized = v_quantization_resolution *
        boost::math::round(v/v_quantization_resolution, policy);
    if (v_quantized != this->v()) {
        m_model.set_v_dt(v_quantized, m_model.dt());
    }
}

template <typename Model>
void Bicycle<Model>::set_dt(real_t dt)  {
    if (dt != this->dt()) {
        m_model.set_v_dt(m_model.v(), dt);
    }
}

template <typename Model>
void Bicycle<Model>::reset() {
    m_full_state = full_state_t::Zero();
}

template <typename Model>
void Bicycle<Model>::update_dynamics(real_t roll_torque_input, real_t steer_torque_input,
    real_t yaw_angle_measurement,
    real_t steer_angle_measurement,
    real_t rear_wheel_angle_measurement) {

    //  While the rear wheel angle measurement can be used to determine velocity,
    //  it is assumed that velocity is determined outside this class and passed as
    //  an input. This allows the velocity resolution, and thus the model update
    //  frequency, to be determined by a filter unrelated to the bicycle model.
    //
    //  This could also be used to determine the wheel angles, as we assume
    //  no-slip conditions. However, we calculate the wheel angles during the
    //  kinematic  update and solely from bicycle velocity.
    (void)rear_wheel_angle_measurement;

    input_t u = input_t::Zero();
    model_t::set_input_element(u, model_t::input_index_t::roll_torque, roll_torque_input);
    model_t::set_input_element(u, model_t::input_index_t::steer_torque, steer_torque_input);
    measurement_t z = measurement_t::Zero();
    model_t::set_output_element(z, model_t::output_index_t::yaw_angle, yaw_angle_measurement);
    model_t::set_output_element(z, model_t::output_index_t::steer_angle, steer_angle_measurement);

    update_dynamics(u, z);
}

template <typename Model>
void Bicycle<Model>::update_dynamics(input_t u, measurement_t z) {
    m_input = u;
    m_measurement = z;

    // copy full state before performing state update
    chBSemWait(&m_state_sem);
    full_state_t full_state_copy = m_full_state;
    chBSemSignal(&m_state_sem);

    full_state_t full_state_next = m_model.integrate_full_state(
            full_state_copy,
            m_input,
            m_model.dt(),
            m_measurement);

    chBSemWait(&m_state_sem);
    m_full_state = full_state_next;
    chBSemSignal(&m_state_sem);
}

template <typename Model>
void Bicycle<Model>::update_kinematics() {
    chBSemWait(&m_state_sem);
    full_state_t full_state_copy = m_full_state;
    chBSemSignal(&m_state_sem);

    // solve for pitch as this does not get integrated
    const real_t roll = model_t::get_full_state_element(full_state_copy, full_state_index_t::roll_angle);
    const real_t steer = model_t::get_full_state_element(full_state_copy, full_state_index_t::steer_angle);
    real_t pitch = model_t::get_full_state_element(full_state_copy, full_state_index_t::pitch_angle);
    pitch = m_model.solve_constraint_pitch(roll, steer, pitch);

    // update full state with newest computed pitch angle
    chBSemWait(&m_state_sem);
    model_t::set_full_state_element(m_full_state, full_state_index_t::pitch_angle, pitch);

    // mod rear wheel angle so it does not grow beyond all bounds
    model_t::set_full_state_element(m_full_state, full_state_index_t::rear_wheel_angle,
            std::fmod(model_t::get_full_state_element(full_state_copy, full_state_index_t::rear_wheel_angle),
                      constants::two_pi)),
    chBSemSignal(&m_state_sem);

    m_pose.timestamp = chVTGetSystemTime();
    m_pose.x = model_t::get_full_state_element(full_state_copy, full_state_index_t::x);
    m_pose.y = model_t::get_full_state_element(full_state_copy, full_state_index_t::y);
    m_pose.rear_wheel = model_t::get_full_state_element(full_state_copy, full_state_index_t::rear_wheel_angle);
    m_pose.pitch = pitch;
    m_pose.yaw = model_t::get_full_state_element(full_state_copy, full_state_index_t::yaw_angle);
    m_pose.roll = roll;
    m_pose.steer = steer;
}

template <typename Model>
const BicyclePoseMessage& Bicycle<Model>::pose() const {
    return m_pose;
}

template <typename Model>
const typename Bicycle<Model>::input_t& Bicycle<Model>::input() const {
    return m_input;
}

template <typename Model>
typename Bicycle<Model>::model_t& Bicycle<Model>::model() {
    return m_model;
};

template <typename Model>
const typename Bicycle<Model>::model_t& Bicycle<Model>::model() const {
    return m_model;
};

template <typename Model>
const typename Bicycle<Model>::second_order_matrix_t& Bicycle<Model>::M() const {
    return m_model.M();
}

template <typename Model>
const typename Bicycle<Model>::second_order_matrix_t& Bicycle<Model>::C1() const {
    return m_model.C1();
}

template <typename Model>
const typename Bicycle<Model>::second_order_matrix_t& Bicycle<Model>::K0() const {
    return m_model.K0();
}

template <typename Model>
const typename Bicycle<Model>::second_order_matrix_t& Bicycle<Model>::K2() const {
    return m_model.K2();
}

template <typename Model>
model::real_t Bicycle<Model>::wheelbase() const {
    return m_model.wheelbase();
}

template <typename Model>
model::real_t Bicycle<Model>::trail() const {
    return m_model.trail();
}

template <typename Model>
model::real_t Bicycle<Model>::steer_axis_tilt() const {
    return m_model.steer_axis_tilt();
}

template <typename Model>
model::real_t Bicycle<Model>::rear_wheel_radius() const {
    return m_model.rear_wheel_radius();
}

template <typename Model>
model::real_t Bicycle<Model>::front_wheel_radius() const {
    return m_model.front_wheel_radius();
}

template <typename Model>
model::real_t Bicycle<Model>::v() const {
    return m_model.v();
}

template <typename Model>
model::real_t Bicycle<Model>::dt() const {
    return m_model.dt();
}

template <typename Model>
const typename Bicycle<Model>::full_state_t& Bicycle<Model>::full_state() const {
    return m_full_state;
}

} // namespace sim
