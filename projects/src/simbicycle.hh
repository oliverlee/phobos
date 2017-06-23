#include <cmath>
#include <type_traits>
#include <boost/math/special_functions/round.hpp>
#include "bicycle/kinematic.h"
#include "oracle.h"
#include "kalman.h"
/*
 * Member function definitions of sim::Bicycle template class.
 * See simbicycle.h for template class declaration.
 */

namespace sim {

template <typename Model, typename Observer>
Bicycle<Model, Observer>::Bicycle(real_t v, real_t dt) :
m_model(v, dt),
m_observer(m_model),
m_full_state(full_state_t::Zero()),
m_pose(BicyclePoseMessage_init_zero),
m_input(input_t::Zero()) {
    // Note: User must initialize Kalman matrices in application.
    // TODO: Do this automatically with default values?
    chBSemObjectInit(&m_state_sem, false); // initialize to not taken

static_assert((!std::is_same<Model, model::BicycleKinematic>::value) ||
              std::is_same<Observer, observer::Oracle<model::BicycleKinematic>>::value,
              "Only Oracle observer type can be used with BicycleKinematic model type.");
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
void Bicycle<Model, Observer>::reset() {
    m_observer.reset();
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
    //  kinematic  update and solely from bicycle velocity.
    (void)rear_wheel_angle_measurement;

    m_input = input_t::Zero();
    measurement_t measurement = measurement_t::Zero();

    model_t::set_input_element(m_input, model_t::input_index_t::roll_torque, roll_torque_input);
    model_t::set_input_element(m_input, model_t::input_index_t::steer_torque, steer_torque_input);
    model_t::set_output_element(measurement, model_t::output_index_t::yaw_angle, yaw_angle_measurement);
    model_t::set_output_element(measurement, model_t::output_index_t::steer_angle, steer_angle_measurement);

    // If we are below the critical speed, roll angle and roll rate are set to zero before _and_ after integration
    observer_t& obs = m_observer;
    auto zero_roll_states = [&obs = obs]() {
        state_t x = obs.state();
        model_t::set_state_element(x, model_t::state_index_t::roll_angle, static_cast<real_t>(0.0));
        model_t::set_state_element(x, model_t::state_index_t::roll_rate, static_cast<real_t>(0.0));
        obs.set_state(x);
    };
    if (m_model.v() < v_critical) {
        zero_roll_states();
    }


    // The auxiliary states _must_ also be integrated at the same time as the
    // dynamic state. After the observer update, we "merge" dynamic states together.
    chBSemWait(&m_state_sem);
    full_state_t full_state_copy = m_full_state;
    chBSemSignal(&m_state_sem);
    full_state_t full_state_next = m_model.integrate_full_state(
            full_state_copy, m_input, m_model.dt(), measurement);
    m_observer.update_state(m_input, measurement);

    if (m_model.v() < v_critical) {
        zero_roll_states();
    }

    if (!m_observer.state().allFinite()) {
        chSysHalt("");
    }

    // Merge observer and model states
    //
    // We simply copy the observer state estimate to the full state vector
    // this may result in accumulated error in the auxiliary states but
    // convergence of the observer estimate should keep it low.
    // Also, we have no way to correct auxiliary states as there are no sensors
    // to measure them and that's because they are _purely_ virtual.
    //
    // TODO: improve this
    chBSemWait(&m_state_sem);
    m_full_state = model_t::make_full_state(
            model_t::get_auxiliary_state_part(full_state_next),
            m_observer.state());
    chBSemSignal(&m_state_sem);
}

template <typename Model, typename Observer>
void Bicycle<Model, Observer>::update_kinematics() {
    chBSemWait(&m_state_sem);
    full_state_t x = m_full_state;
    chBSemSignal(&m_state_sem);

    // solve for pitch as this does not get integrated
    const real_t roll = model_t::get_full_state_element(x, full_state_index_t::roll_angle);
    const real_t steer = model_t::get_full_state_element(x, full_state_index_t::steer_angle);
    real_t pitch = model_t::get_full_state_element(x, full_state_index_t::pitch_angle);
    pitch = m_model.solve_constraint_pitch(roll, steer, pitch);

    // update full state with newest computed pitch angle
    chBSemWait(&m_state_sem);
    model_t::set_full_state_element(m_full_state, full_state_index_t::pitch_angle, pitch);

    // mod rear wheel angle so it does not grow beyond all bounds
    model_t::set_full_state_element(m_full_state, full_state_index_t::rear_wheel_angle,
            std::fmod(model_t::get_full_state_element(x, full_state_index_t::rear_wheel_angle),
                      constants::two_pi)),
    chBSemSignal(&m_state_sem);

    m_pose.timestamp = chVTGetSystemTime();
    m_pose.x = model_t::get_full_state_element(x, full_state_index_t::x);
    m_pose.y = model_t::get_full_state_element(x, full_state_index_t::y);
    m_pose.rear_wheel = model_t::get_full_state_element(x, full_state_index_t::rear_wheel_angle);
    m_pose.pitch = pitch;
    m_pose.yaw = model_t::get_full_state_element(x, full_state_index_t::yaw_angle);
    m_pose.roll = roll;
    m_pose.steer = steer;
}

template <typename Model, typename Observer>
void Bicycle<Model, Observer>::prime_observer() {
    if (std::is_same<Observer, typename observer::Kalman<Model>>::value) {
        // define a non-zero initial state, this is selected arbitrarily for now
        state_t x = state_t::Zero();
        model::Bicycle::set_state_element(x, model_t::state_index_t::steer_angle, 0.1f); // in radians

        float t = 0;
        while (t < observer_prime_period) {
            x = m_model.update_state(x);
            m_observer.update_state(input_t::Zero(), m_model.calculate_output(x));
            t += m_model.dt();
        }
    }
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

} // namespace sim
