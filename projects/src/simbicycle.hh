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

template <typename Model, typename Observer, typename Haptic>
Bicycle<Model, Observer, Haptic>::Bicycle(real_t v, real_t dt,
        real_t upper_assembly_inertia_virtual,
        real_t lower_assembly_inertia_physical) :
m_model(v, dt),
m_observer(m_model),
m_inertia_upper_virtual(m_model, upper_assembly_inertia_virtual),
m_inertia_lower_physical(m_model, lower_assembly_inertia_physical),
m_state_full(full_state_t::Zero()),
m_pose(BicyclePoseMessage_init_zero) {
    // Note: User must initialize Kalman matrices in application.
    // TODO: Do this automatically with default values?
    chBSemObjectInit(&m_state_sem, false); // initialize to not taken

static_assert((!std::is_same<Model, model::BicycleKinematic>::value) ||
              std::is_same<Haptic, haptic::HandlebarStatic>::value,
              "Only HandlebarStatic haptic type can be used with BicycleKinematic model type.");
static_assert((!std::is_same<Model, model::BicycleKinematic>::value) ||
              std::is_same<Observer, observer::Oracle<model::BicycleKinematic>>::value,
              "Only Oracle observer type can be used with BicycleKinematic model type.");
}

template <typename Model, typename Observer, typename Haptic>
void Bicycle<Model, Observer, Haptic>::set_v(real_t v)  {
    using namespace boost::math::policies;
    using quantize_policy = policy<rounding_error<ignore_error>>;
    static constexpr quantize_policy policy;

    real_t v_quantized = v_quantization_resolution *
        boost::math::round(v/v_quantization_resolution, policy);
    if (v_quantized != this->v()) {
        m_model.set_v_dt(v_quantized, m_model.dt());
    }
}

template <typename Model, typename Observer, typename Haptic>
void Bicycle<Model, Observer, Haptic>::set_dt(real_t dt)  {
    if (dt != this->dt()) {
        m_model.set_v_dt(m_model.v(), dt);
    }
}

template <typename Model, typename Observer, typename Haptic>
void Bicycle<Model, Observer, Haptic>::reset() {
    m_observer.reset();
}

template <typename Model, typename Observer, typename Haptic>
void Bicycle<Model, Observer, Haptic>::update_dynamics(real_t roll_torque_measurement,
        real_t steer_torque_measurement, real_t yaw_angle_measurement,
        real_t steer_angle_measurement, real_t rear_wheel_angle_measurement) {
    // While the rear wheel angle measurement can be used to determine velocity,
    // it is assumed that velocity is determined outside this class and passed as
    // an input. This allows the velocity resolution, and thus the model update
    // frequency, to be determined by a filter unrelated to the bicycle model.
    //
    // The rear wheel angle measurement also could be used to determine the
    // wheel angles. Instead we assume no-slip conditions and calculate the wheel
    // angle during the kinematic update and solely from bicycle velocity.
    (void)rear_wheel_angle_measurement; // velocity is calculated and updated externally

    input_t input = input_t::Zero();
    measurement_t measurement = measurement_t::Zero();

    // Convert roll, steer torque from physical to virtual.
    const float upper_inertia_torque = m_inertia_upper_virtual.torque(m_observer.state());
    // steer_torque_measurement sign is correct when considering the lower assembly but needs
    // to be flipped when considering the upper assembly.
    const float steer_torque = upper_inertia_torque + steer_torque_measurement;
    const float roll_torque = roll_torque_measurement;

    model_t::set_input_element(input, model_t::input_index_t::roll_torque, roll_torque);
    model_t::set_input_element(input, model_t::input_index_t::steer_torque, steer_torque);
    model_t::set_output_element(measurement, model_t::output_index_t::yaw_angle, yaw_angle_measurement);
    model_t::set_output_element(measurement, model_t::output_index_t::steer_angle, steer_angle_measurement);

    // The auxiliary states _must_ also be integrated at the same time as the
    // dynamic state. After the observer update, we "merge" dynamic states together.
    full_state_t state_full = m_model.integrate_full_state(
            m_state_full, input, m_model.dt(), measurement);
    m_observer.update_state(input, measurement);

    if (!m_observer.state().allFinite()) {
        chSysHalt("");
    }

    { // limit allowed bicycle state
        state_t x = m_observer.state();

        auto limit_state_element = [&x](model::Bicycle::state_index_t index, real_t limit) {
            const real_t value = model::Bicycle::get_state_element(x, index);
            if (std::abs(value) > limit) {
                model::Bicycle::set_state_element(x, index, std::copysign(limit, value));
            }
        };

        real_t roll_angle = model_t::get_state_element(m_observer.state(),
                model_t::state_index_t::roll_angle);
        if (std::abs(roll_angle) > constants::pi) {
            // state normalization limits angles to the range [-2*pi, 2*pi]
            roll_angle += std::copysign(constants::two_pi, -1*roll_angle);
        }

        limit_state_element(model_t::state_index_t::roll_rate, roll_rate_limit);
        limit_state_element(model_t::state_index_t::steer_rate, steer_rate_limit);

        m_observer.set_state(x);
    }

    m_input = input;
    //m_T_m = m_inertia_lower_physical.torque(m_observer.state(), input) + steer_torque_measurement;
    m_T_m = 3.1816*m_inertia_lower_physical.torque(m_observer.state(), input) + steer_torque_measurement;

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
    m_state_full = model_t::make_full_state(
            model_t::get_auxiliary_state_part(state_full),
            m_observer.state());
    chBSemSignal(&m_state_sem);
}

template <typename Model, typename Observer, typename Haptic>
void Bicycle<Model, Observer, Haptic>::update_kinematics() {
    chBSemWait(&m_state_sem);
    full_state_t x = m_state_full;
    chBSemSignal(&m_state_sem);

    // solve for pitch as this does not calculated by integration in update_dynamics()
    const real_t roll = model_t::get_full_state_element(x, full_state_index_t::roll_angle);
    const real_t steer = model_t::get_full_state_element(x, full_state_index_t::steer_angle);
    real_t pitch = model_t::get_full_state_element(x, full_state_index_t::pitch_angle);
    pitch = m_model.solve_constraint_pitch(roll, steer, pitch);

    // update full state with newest computed pitch angle
    chBSemWait(&m_state_sem);
    model_t::set_full_state_element(m_state_full, full_state_index_t::pitch_angle, pitch);
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

template <typename Model, typename Observer, typename Haptic>
void Bicycle<Model, Observer, Haptic>::prime_observer() {
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

template <typename Model, typename Observer, typename Haptic>
const BicyclePoseMessage& Bicycle<Model, Observer, Haptic>::pose() const {
    return m_pose;
}

template <typename Model, typename Observer, typename Haptic>
model::real_t Bicycle<Model, Observer, Haptic>::handlebar_feedback_torque() const {
    return m_T_m;
}

template <typename Model, typename Observer, typename Haptic>
const typename Bicycle<Model, Observer, Haptic>::input_t& Bicycle<Model, Observer, Haptic>::input() const {
    return m_input;
}

template <typename Model, typename Observer, typename Haptic>
typename Bicycle<Model, Observer, Haptic>::model_t& Bicycle<Model, Observer, Haptic>::model() {
    return m_model;
};

template <typename Model, typename Observer, typename Haptic>
const typename Bicycle<Model, Observer, Haptic>::model_t& Bicycle<Model, Observer, Haptic>::model() const {
    return m_model;
};

template <typename Model, typename Observer, typename Haptic>
typename Bicycle<Model, Observer, Haptic>::observer_t& Bicycle<Model, Observer, Haptic>::observer() {
    return m_observer;
};

template <typename Model, typename Observer, typename Haptic>
const typename Bicycle<Model, Observer, Haptic>::observer_t& Bicycle<Model, Observer, Haptic>::observer() const {
    return m_observer;
};

template <typename Model, typename Observer, typename Haptic>
const typename Bicycle<Model, Observer, Haptic>::haptic_t& Bicycle<Model, Observer, Haptic>::inertia_upper_virtual() const {
    return m_inertia_upper_virtual;
};

template <typename Model, typename Observer, typename Haptic>
const typename Bicycle<Model, Observer, Haptic>::haptic_t& Bicycle<Model, Observer, Haptic>::inertia_lower_physical() const {
    return m_inertia_lower_physical;
};

template <typename Model, typename Observer, typename Haptic>
const typename Bicycle<Model, Observer, Haptic>::second_order_matrix_t& Bicycle<Model, Observer, Haptic>::M() const {
    return m_model.M();
}

template <typename Model, typename Observer, typename Haptic>
const typename Bicycle<Model, Observer, Haptic>::second_order_matrix_t& Bicycle<Model, Observer, Haptic>::C1() const {
    return m_model.C1();
}

template <typename Model, typename Observer, typename Haptic>
const typename Bicycle<Model, Observer, Haptic>::second_order_matrix_t& Bicycle<Model, Observer, Haptic>::K0() const {
    return m_model.K0();
}

template <typename Model, typename Observer, typename Haptic>
const typename Bicycle<Model, Observer, Haptic>::second_order_matrix_t& Bicycle<Model, Observer, Haptic>::K2() const {
    return m_model.K2();
}

template <typename Model, typename Observer, typename Haptic>
model::real_t Bicycle<Model, Observer, Haptic>::wheelbase() const {
    return m_model.wheelbase();
}

template <typename Model, typename Observer, typename Haptic>
model::real_t Bicycle<Model, Observer, Haptic>::trail() const {
    return m_model.trail();
}

template <typename Model, typename Observer, typename Haptic>
model::real_t Bicycle<Model, Observer, Haptic>::steer_axis_tilt() const {
    return m_model.steer_axis_tilt();
}

template <typename Model, typename Observer, typename Haptic>
model::real_t Bicycle<Model, Observer, Haptic>::rear_wheel_radius() const {
    return m_model.rear_wheel_radius();
}

template <typename Model, typename Observer, typename Haptic>
model::real_t Bicycle<Model, Observer, Haptic>::front_wheel_radius() const {
    return m_model.front_wheel_radius();
}

template <typename Model, typename Observer, typename Haptic>
model::real_t Bicycle<Model, Observer, Haptic>::v() const {
    return m_model.v();
}

template <typename Model, typename Observer, typename Haptic>
model::real_t Bicycle<Model, Observer, Haptic>::dt() const {
    return m_model.dt();
}

template <typename Model, typename Observer, typename Haptic>
const typename Bicycle<Model, Observer, Haptic>::full_state_t& Bicycle<Model, Observer, Haptic>::full_state() const {
    return m_state_full;
}

} // namespace sim
