#include <cmath>
#include <boost/math/special_functions/round.hpp>
/*
 * Member function definitions of sim::Bicycle template class.
 * See simbicycle.h for template class declaration.
 */

namespace sim {

template <typename Model>
Bicycle<Model>::Bicycle(real_t v, real_t dt) :
m_model(v, dt),
m_full_state(full_state_t::Zero()),
m_input(input_t::Zero()) {
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
        m_model.set_v(v_quantized);
    }
}

template <typename Model>
void Bicycle<Model>::set_dt(real_t dt)  {
    if (dt != this->dt()) {
        m_model.set_dt(dt);
    }
}

template <typename Model>
void Bicycle<Model>::reset() {
    m_full_state = full_state_t::Zero();
}

template <typename Model>
void Bicycle<Model>::update_dynamics(real_t roll_torque_input, real_t steer_torque_input) {
    input_t u = input_t::Zero();
    model_t::set_input_element(u, model_t::input_index_t::roll_torque, roll_torque_input);
    model_t::set_input_element(u, model_t::input_index_t::steer_torque, steer_torque_input);

    update_dynamics(u);
}

template <typename Model>
void Bicycle<Model>::update_dynamics(const input_t& u) {
    m_input = u;

    // copy full state before performing state update
    chBSemWait(&m_state_sem);
    full_state_t full_state_copy = m_full_state;
    chBSemSignal(&m_state_sem);

    full_state_t full_state_next = m_model.integrate_full_state(
            full_state_copy,
            m_input,
            m_model.dt());

    chBSemWait(&m_state_sem);
    m_full_state = full_state_next;
    chBSemSignal(&m_state_sem);
}

template <typename Model>
void Bicycle<Model>::update_dynamics(const input_t& u, const measurement_t& z) {
    m_input = u;

    // copy full state before performing state update
    chBSemWait(&m_state_sem);
    full_state_t full_state_copy = m_full_state;
    chBSemSignal(&m_state_sem);

    full_state_t full_state_next = m_model.integrate_full_state(
            full_state_copy,
            m_input,
            m_model.dt(),
            z);

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
}

template <typename Model>
const typename Bicycle<Model>::input_t& Bicycle<Model>::input() const {
    return m_input;
}

template <typename Model>
const typename Bicycle<Model>::full_state_t& Bicycle<Model>::full_state() const {
    return m_full_state;
}

template <typename Model>
typename Bicycle<Model>::state_t Bicycle<Model>::state() const {
    return model::Bicycle::get_state_part(m_full_state);
}

template <typename Model>
typename Bicycle<Model>::auxiliary_state_t Bicycle<Model>::auxiliary_state() const {
    return model::Bicycle::get_auxiliary_state_part(m_full_state);
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

} // namespace sim
