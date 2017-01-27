#include <cmath>
#include <boost/math/special_functions/round.hpp>
/*
 * Member function definitions of sim::Bicycle template class.
 * See simbicycle.h for template class declaration.
 */

namespace sim {

template <typename T, typename U, typename V>
Bicycle<T, U, V>::Bicycle(real_t v, real_t dt, real_t steer_inertia) :
m_model(v, dt),
m_observer(m_model),
m_haptic(m_model, steer_inertia),
m_state_full(full_state_t::Zero()),
m_pose() {
    // Note: User must initialize Kalman matrices in application.
    // TODO: Do this automatically with default values?
    chBSemObjectInit(&m_state_sem, false); /* initialize to not taken */
}


template <typename T, typename U, typename V>
void Bicycle<T, U, V>::set_v(real_t v)  {
    using namespace boost::math::policies;
    using quantize_policy = policy<rounding_error<ignore_error>>;
    static constexpr quantize_policy policy;

    real_t v_quantized = v_quantization_resolution *
        boost::math::round(v/v_quantization_resolution, policy);
    if (v_quantized != this->v()) {
        m_model.set_v_dt(v_quantized, m_model.dt());
    }
}

template <typename T, typename U, typename V>
void Bicycle<T, U, V>::set_dt(real_t dt)  {
    if (dt != this->dt()) {
        m_model.set_v_dt(m_model.v(), dt);
    }
}

template <typename T, typename U, typename V>
void Bicycle<T, U, V>::reset() {
    m_observer.reset();
}

template <typename T, typename U, typename V>
void Bicycle<T, U, V>::update_dynamics(real_t roll_torque_input, real_t steer_torque_input,
    real_t yaw_angle_measurement,
    real_t steer_angle_measurement,
    real_t rear_wheel_angle_measurement) {
    /*
     * While the rear wheel angle measurement can be used to determine velocity,
     * it is assumed that velocity is determined outside this class and passed as
     * an input. This allows the velocity resolution, and thus the model update
     * frequency, to be determined by a filter unrelated to the bicycle model.
     *
     * While this also could be used to determine the wheel angles, we assume
     * no-slip conditions and calculate the wheel angles during the kinematic
     * update and solely from bicycle velocity.
     */
    (void)rear_wheel_angle_measurement;
    constexpr uint8_t roll_torque_index = static_cast<uint8_t>(model_t::input_index_t::roll_torque);
    constexpr uint8_t steer_torque_index = static_cast<uint8_t>(model_t::input_index_t::steer_torque);
    constexpr uint8_t yaw_angle_index = static_cast<uint8_t>(model_t::output_index_t::yaw_angle);
    constexpr uint8_t steer_angle_index = static_cast<uint8_t>(model_t::output_index_t::steer_angle);

    input_t input = input_t::Zero();
    measurement_t measurement = measurement_t::Zero();

    input[roll_torque_index] = roll_torque_input;
    input[steer_torque_index] = steer_torque_input;
    measurement[yaw_angle_index] = yaw_angle_measurement;
    measurement[steer_angle_index] = steer_angle_measurement;

    /*
     * The auxiliary states _must_ also be integrated at the same time as the
     * dynamic state. After the observer update, we "merge" dynamic states together.
     */
    m_state_full = m_model.integrate_full_state(m_state_full, input, m_model.dt());
    m_observer.update_state(input, measurement);

    if (!m_observer.state().allFinite()) {
        chSysHalt("");
    }

    { /* limit allowed bicycle state */
        state_t x = m_observer.state();

        static constexpr auto roll_angle_index = static_cast<uint8_t>(model_t::state_index_t::roll_angle);
        static constexpr auto steer_angle_index = static_cast<uint8_t>(model_t::state_index_t::steer_angle);
        static constexpr auto roll_rate_index = static_cast<uint8_t>(model_t::state_index_t::roll_rate);
        static constexpr auto steer_rate_index = static_cast<uint8_t>(model_t::state_index_t::steer_rate);

        auto limit_state_element = [&](auto index_type, real_t limit) {
            const uint8_t index = static_cast<uint8_t>(index_type);
            real_t value = m_observer.state()[index];
            if (std::abs(value) > limit) {
                x[index] = std::copysign(limit, value);
            }
        };

        real_t roll_angle = m_observer.state()[roll_angle_index];
        if (std::abs(roll_angle) > constants::pi) {
            /* state normalization limits angles to the range [-2*pi, 2*pi] */
            roll_angle += std::copysign(constants::two_pi, -1*roll_angle);
        }

        if (roll_angle > roll_angle_limit) {
            x[roll_angle_index] = std::copysign(roll_angle_limit, roll_angle);
            x[steer_angle_index] = steer_angle_measurement;
            x[roll_rate_index] = get_state_element(m_state_full, full_state_index_t::roll_rate);
            x[steer_rate_index] = get_state_element(m_state_full, full_state_index_t::steer_rate);
        } else {
            limit_state_element(model_t::state_index_t::roll_rate, roll_rate_limit);
            limit_state_element(model_t::state_index_t::steer_rate, steer_rate_limit);
        }

        m_observer.set_state(x);
    }

    m_T_m = m_haptic.feedback_torque(m_observer.state(), input);

    /*
     * Merge observer and model states
     *
     * We simply copy the observer state estimate to the full state vector
     * this may result in accumulated error in the auxiliary states but
     * convergence of the observer estimate should keep it low.
     * Also, we have no way to correct auxiliary states as there are no sensors
     * to measure them and that's because they are _purely_ virtual.
     *
     * TODO: improve this
     */
    chBSemWait(&m_state_sem);
    m_state_full = model_t::make_full_state(
            model_t::get_auxiliary_state_part(m_state_full),
            m_observer.state());
    chBSemSignal(&m_state_sem);
}

template <typename T, typename U, typename V>
void Bicycle<T, U, V>::update_kinematics() {
    chBSemWait(&m_state_sem);
    full_state_t x = m_state_full;
    chBSemSignal(&m_state_sem);

    // solve for pitch as this does not get integrated
    const real_t roll = get_state_element(x, full_state_index_t::roll_angle);
    const real_t steer = get_state_element(x, full_state_index_t::steer_angle);
    real_t pitch = get_state_element(x, full_state_index_t::pitch_angle);
    pitch = m_model.solve_constraint_pitch(roll, steer, pitch);

    // update full state with newest computed pitch angle
    chBSemWait(&m_state_sem);
    m_state_full[get_state_element_index(full_state_index_t::pitch_angle)] = pitch;
    chBSemSignal(&m_state_sem);

    m_pose.timestamp = chVTGetSystemTime();
    m_pose.x = get_state_element(x, full_state_index_t::x);
    m_pose.y = get_state_element(x, full_state_index_t::y);
    m_pose.rear_wheel = get_state_element(x, full_state_index_t::rear_wheel_angle);
    m_pose.pitch = pitch;
    m_pose.yaw = get_state_element(x, full_state_index_t::yaw_angle);
    m_pose.roll = roll;
    m_pose.steer = steer;
}


template <typename T, typename U, typename V>
const BicyclePoseMessage& Bicycle<T, U, V>::pose() const {
    return m_pose;
}

template <typename T, typename U, typename V>
model::real_t Bicycle<T, U, V>::handlebar_feedback_torque() const {
    return m_T_m;
}

template <typename T, typename U, typename V>
const typename Bicycle<T, U, V>::model_t& Bicycle<T, U, V>::model() const {
    return m_model;
};

template <typename T, typename U, typename V>
const typename Bicycle<T, U, V>::observer_t& Bicycle<T, U, V>::observer() const {
    return m_observer;
};

template <typename T, typename U, typename V>
const typename Bicycle<T, U, V>::second_order_matrix_t& Bicycle<T, U, V>::M() const {
    return m_model.M();
}

template <typename T, typename U, typename V>
const typename Bicycle<T, U, V>::second_order_matrix_t& Bicycle<T, U, V>::C1() const {
    return m_model.C1();
}

template <typename T, typename U, typename V>
const typename Bicycle<T, U, V>::second_order_matrix_t& Bicycle<T, U, V>::K0() const {
    return m_model.K0();
}

template <typename T, typename U, typename V>
const typename Bicycle<T, U, V>::second_order_matrix_t& Bicycle<T, U, V>::K2() const {
    return m_model.K2();
}

template <typename T, typename U, typename V>
model::real_t Bicycle<T, U, V>::wheelbase() const {
    return m_model.wheelbase();
}

template <typename T, typename U, typename V>
model::real_t Bicycle<T, U, V>::trail() const {
    return m_model.trail();
}

template <typename T, typename U, typename V>
model::real_t Bicycle<T, U, V>::steer_axis_tilt() const {
    return m_model.steer_axis_tilt();
}

template <typename T, typename U, typename V>
model::real_t Bicycle<T, U, V>::rear_wheel_radius() const {
    return m_model.rear_wheel_radius();
}

template <typename T, typename U, typename V>
model::real_t Bicycle<T, U, V>::front_wheel_radius() const {
    return m_model.front_wheel_radius();
}

template <typename T, typename U, typename V>
model::real_t Bicycle<T, U, V>::v() const {
    return m_model.v();
}

template <typename T, typename U, typename V>
model::real_t Bicycle<T, U, V>::dt() const {
    return m_model.dt();
}


template <typename T, typename U, typename V>
typename Bicycle<T, U, V>::index_type Bicycle<T, U, V>::get_state_element_index(Bicycle<T, U, V>::full_state_index_t field) {
    return static_cast<index_type>(field);
}

template <typename T, typename U, typename V>
model::real_t Bicycle<T, U, V>::get_state_element(const Bicycle<T, U, V>::full_state_t& state,
        Bicycle<T, U, V>::full_state_index_t field) {
    return state[get_state_element_index(field)];
}

} // namespace sim
