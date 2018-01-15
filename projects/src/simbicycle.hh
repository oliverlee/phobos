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

#define OBSERVER_FUNCTION(return_type) \
    template <typename T> \
    typename std::enable_if<std::is_base_of<observer::ObserverBase, T>::value, return_type>::type
#define NULL_OBSERVER_FUNCTION(return_type) \
    template <typename T> \
    typename std::enable_if<!std::is_base_of<observer::ObserverBase, T>::value, return_type>::type
#define BICYCLE_TYPE Bicycle<Model, Observer>

template <typename Model, typename Observer> template <typename T>
Bicycle<Model, Observer>::Bicycle(typename std::enable_if<std::is_base_of<observer::ObserverBase, T>::value, real_t>::type v, real_t dt) :
m_model(v, dt),
m_observer(m_model),
m_full_state(full_state_t::Zero()),
m_pose(BicyclePoseMessage_init_zero),
m_input(input_t::Zero()),
m_measurement(measurement_t::Zero()) {
    // Note: User must initialize Kalman matrices in application.
    chBSemObjectInit(&m_state_sem, false); // initialize to not taken

static_assert((!std::is_same<Model, model::BicycleKinematic>::value) ||
              std::is_same<Observer, std::nullptr_t>::value,
              "Only std::nullptr_t observer type can be used with BicycleKinematic model type.");
}

template <typename Model, typename Observer> template <typename T>
Bicycle<Model, Observer>::Bicycle(typename std::enable_if<!std::is_base_of<observer::ObserverBase, T>::value, real_t>::type v, real_t dt) :
m_model(v, dt),
m_observer(nullptr),
m_full_state(full_state_t::Zero()),
m_pose(BicyclePoseMessage_init_zero),
m_input(input_t::Zero()),
m_measurement(measurement_t::Zero()) {
    // Note: User must initialize Kalman matrices in application.
    chBSemObjectInit(&m_state_sem, false); // initialize to not taken

static_assert((!std::is_same<Model, model::BicycleKinematic>::value) ||
              std::is_same<Observer, std::nullptr_t>::value,
              "Only std::nullptr_t observer type can be used with BicycleKinematic model type.");
}

template <typename Model, typename Observer>
void Bicycle<Model, Observer>::set_v(real_t v)  {
    using namespace boost::math::policies;
    using quantize_policy = policy<rounding_error<ignore_error>>;
    static constexpr quantize_policy policy;

    real_t v_quantized = v_quantization_resolution *
        boost::math::round(v/v_quantization_resolution, policy);
    if (v_quantized != this->v()) {
        m_model.set_v_dt(v_quantized, m_model.dt());
    }
}

template <typename Model, typename Observer>
void Bicycle<Model, Observer>::set_dt(real_t dt)  {
    if (dt != this->dt()) {
        m_model.set_v_dt(m_model.v(), dt);
    }
}

template <typename Model, typename Observer>
OBSERVER_FUNCTION(void) Bicycle<Model, Observer>::reset() {
    m_observer.reset();
    m_full_state = full_state_t::Zero();
}
template <typename Model, typename Observer>
NULL_OBSERVER_FUNCTION(void) Bicycle<Model, Observer>::reset() {
    m_full_state = full_state_t::Zero();
}

template <typename Model, typename Observer>
void Bicycle<Model, Observer>::update_dynamics(real_t roll_torque_input, real_t steer_torque_input,
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
    //  kinematic update and solely from bicycle velocity.
    (void)rear_wheel_angle_measurement;

    input_t u = input_t::Zero();
    model_t::set_input_element(u, model_t::input_index_t::roll_torque, roll_torque_input);
    model_t::set_input_element(u, model_t::input_index_t::steer_torque, steer_torque_input);
    measurement_t z = measurement_t::Zero();
    model_t::set_output_element(z, model_t::output_index_t::yaw_angle, yaw_angle_measurement);
    model_t::set_output_element(z, model_t::output_index_t::steer_angle, steer_angle_measurement);

    update_dynamics(u, z);
}

template <typename Model, typename Observer>
void Bicycle<Model, Observer>::update_dynamics(input_t u, measurement_t z) {
    m_input = u;
    m_measurement = z;

    // do full state update which is observer specific
    chBSemWait(&m_state_sem);
    full_state_t full_state_copy = m_full_state;
    chBSemSignal(&m_state_sem);

    full_state_t full_state_next = do_full_state_update(full_state_copy);

    chBSemWait(&m_state_sem);
    m_full_state = full_state_next;
    chBSemSignal(&m_state_sem);
}

template <typename Model, typename Observer>
void Bicycle<Model, Observer>::update_kinematics() {
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

template <typename Model, typename Observer>
OBSERVER_FUNCTION(void) Bicycle<Model, Observer>::prime_observer() {
    // define a non-zero initial state, this is selected arbitrarily for now
    state_t x = state_t::Zero();
    model::Bicycle::set_state_element(x, model_t::state_index_t::steer_angle, 0.1f); // in radians

    static constexpr real_t observer_prime_period = 3.0; // seconds
    real_t t = 0;
    while (t < observer_prime_period) {
        x = m_model.update_state(x);
        m_observer.update_state(input_t::Zero(), m_model.calculate_output(x));
        t += m_model.dt();
    }
}
template <typename Model, typename Observer>
NULL_OBSERVER_FUNCTION(void) Bicycle<Model, Observer>::prime_observer() {
        // do nothing
}

template <typename Model, typename Observer>
const BicyclePoseMessage& Bicycle<Model, Observer>::pose() const {
    return m_pose;
}

template <typename Model, typename Observer>
const typename Bicycle<Model, Observer>::input_t& Bicycle<Model, Observer>::input() const {
    return m_input;
}

template <typename Model, typename Observer>
typename Bicycle<Model, Observer>::model_t& Bicycle<Model, Observer>::model() {
    return m_model;
};

template <typename Model, typename Observer>
const typename Bicycle<Model, Observer>::model_t& Bicycle<Model, Observer>::model() const {
    return m_model;
};

template <typename Model, typename Observer>
typename Bicycle<Model, Observer>::observer_t& Bicycle<Model, Observer>::observer() {
    return m_observer;
};

template <typename Model, typename Observer>
const typename Bicycle<Model, Observer>::observer_t& Bicycle<Model, Observer>::observer() const {
    return m_observer;
};

template <typename Model, typename Observer>
const typename Bicycle<Model, Observer>::second_order_matrix_t& Bicycle<Model, Observer>::M() const {
    return m_model.M();
}

template <typename Model, typename Observer>
const typename Bicycle<Model, Observer>::second_order_matrix_t& Bicycle<Model, Observer>::C1() const {
    return m_model.C1();
}

template <typename Model, typename Observer>
const typename Bicycle<Model, Observer>::second_order_matrix_t& Bicycle<Model, Observer>::K0() const {
    return m_model.K0();
}

template <typename Model, typename Observer>
const typename Bicycle<Model, Observer>::second_order_matrix_t& Bicycle<Model, Observer>::K2() const {
    return m_model.K2();
}

template <typename Model, typename Observer>
model::real_t Bicycle<Model, Observer>::wheelbase() const {
    return m_model.wheelbase();
}

template <typename Model, typename Observer>
model::real_t Bicycle<Model, Observer>::trail() const {
    return m_model.trail();
}

template <typename Model, typename Observer>
model::real_t Bicycle<Model, Observer>::steer_axis_tilt() const {
    return m_model.steer_axis_tilt();
}

template <typename Model, typename Observer>
model::real_t Bicycle<Model, Observer>::rear_wheel_radius() const {
    return m_model.rear_wheel_radius();
}

template <typename Model, typename Observer>
model::real_t Bicycle<Model, Observer>::front_wheel_radius() const {
    return m_model.front_wheel_radius();
}

template <typename Model, typename Observer>
model::real_t Bicycle<Model, Observer>::v() const {
    return m_model.v();
}

template <typename Model, typename Observer>
model::real_t Bicycle<Model, Observer>::dt() const {
    return m_model.dt();
}

template <typename Model, typename Observer>
const typename Bicycle<Model, Observer>::full_state_t& Bicycle<Model, Observer>::full_state() const {
    return m_full_state;
}

template <typename Model, typename Observer>
OBSERVER_FUNCTION(typename BICYCLE_TYPE::full_state_t) Bicycle<Model, Observer>::do_full_state_update(const full_state_t& full_state) {
    // The auxiliary states _must_ also be integrated at the same time as the
    // dynamic state. After the observer update, we "merge" dynamic states together.
    full_state_t state_full = m_model.integrate_full_state(
            full_state, m_input, m_model.dt(), m_measurement);

    m_observer.update_state(m_input, m_measurement);

    if (!m_observer.state().allFinite()) {
        chSysHalt("state elements with non finite values");
    }

    // Merge observer and model states
    //
    // We simply copy the observer state estimate to the full state vector
    // this may result in accumulated error in the auxiliary states but
    // convergence of the observer estimate should keep it low.
    // Also, we have no way to correct auxiliary states as there are no sensors
    // to measure them and that's because they are _purely_ virtual.
    return model_t::make_full_state(model_t::get_auxiliary_state_part(state_full),
                                    m_observer.state());
}

template <typename Model, typename Observer>
NULL_OBSERVER_FUNCTION(typename BICYCLE_TYPE::full_state_t) Bicycle<Model, Observer>::do_full_state_update(const full_state_t& full_state) {
    // REMOVE ME: No limiting is done but that is removed in a different commit
    return m_model.integrate_full_state(full_state, m_input, m_model.dt(), m_measurement);
}

} // namespace sim
