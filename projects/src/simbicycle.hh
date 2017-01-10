/*
 * Member function definitions of sim::Bicycle template class.
 * See simbicycle.h for template class declaration.
 */

namespace sim {

template <typename T, typename U>
Bicycle<T, U>::Bicycle(real_t v, real_t dt) :
m_model(v, dt),
m_observer(m_model),
m_dstate(state_t::Zero()),
m_kstate(auxiliary_state_t::Zero()),
m_pose() {
    // Note: User must initialize Kalman matrices in application.
    // TODO: Do this automatically with default values?
    chBSemObjectInit(&m_kstate_sem, false); /* initialize to not taken */
}


template <typename T, typename U>
void Bicycle<T, U>::set_v(real_t v)  {
    m_model.set_v_dt(v, m_model.dt());
}

template <typename T, typename U>
void Bicycle<T, U>::set_dt(real_t dt)  {
    m_model.set_v_dt(m_model.v(), dt);
}

template <typename T, typename U>
void Bicycle<T, U>::reset() {
    m_observer.reset();
}

template <typename T, typename U>
void Bicycle<T, U>::update_dynamics(real_t roll_torque_input, real_t steer_torque_input,
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

    /* prevent observer state change at beginning of update_kinematics() */
    // TODO: maybe pass semaphore to update_state to take right before setting
    //       observer state?
    chBSemWait(&m_kstate_sem);
    m_observer.update_state(input, measurement);
    m_T_m = 0;
    chBSemSignal(&m_kstate_sem);
}

template <typename T, typename U>
void Bicycle<T, U>::update_kinematics() {
    /* prevent copy of observer state during update_dynamics() */
    chBSemWait(&m_kstate_sem);
    m_dstate = m_observer.state();
    chBSemSignal(&m_kstate_sem);

    m_kstate = m_model.update_auxiliary_state(m_dstate, m_kstate);

    m_pose.timestamp = chVTGetSystemTime();
    m_pose.x = get_state_element(full_state_index_t::x);
    m_pose.y = get_state_element(full_state_index_t::y);
    m_pose.rear_wheel = get_state_element(full_state_index_t::rear_wheel_angle);
    m_pose.pitch = get_state_element(full_state_index_t::pitch_angle);
    m_pose.yaw = get_state_element(full_state_index_t::yaw_angle);
    m_pose.roll = get_state_element(full_state_index_t::roll_angle);
    m_pose.steer = get_state_element(full_state_index_t::steer_angle);
}


template <typename T, typename U>
const BicyclePoseMessage& Bicycle<T, U>::pose() const {
    return m_pose;
}

template <typename T, typename U>
model::real_t Bicycle<T, U>::handlebar_feedback_torque() const {
    // TODO: implement handlebar feedback torque
    return 0;
}

template <typename T, typename U>
typename Bicycle<T, U>::model_t& Bicycle<T, U>::model() const {
    return m_model;
};

template <typename T, typename U>
const typename Bicycle<T, U>::second_order_matrix_t& Bicycle<T, U>::M() const {
    return m_model.M();
}

template <typename T, typename U>
const typename Bicycle<T, U>::second_order_matrix_t& Bicycle<T, U>::C1() const {
    return m_model.C1();
}

template <typename T, typename U>
const typename Bicycle<T, U>::second_order_matrix_t& Bicycle<T, U>::K0() const {
    return m_model.K0();
}

template <typename T, typename U>
const typename Bicycle<T, U>::second_order_matrix_t& Bicycle<T, U>::K2() const {
    return m_model.K2();
}

template <typename T, typename U>
model::real_t Bicycle<T, U>::wheelbase() const {
    return m_model.wheelbase();
}

template <typename T, typename U>
model::real_t Bicycle<T, U>::trail() const {
    return m_model.trail();
}

template <typename T, typename U>
model::real_t Bicycle<T, U>::steer_axis_tilt() const {
    return m_model.steer_axis_tilt();
}

template <typename T, typename U>
model::real_t Bicycle<T, U>::rear_wheel_radius() const {
    return m_model.rear_wheel_radius();
}

template <typename T, typename U>
model::real_t Bicycle<T, U>::front_wheel_radius() const {
    return m_model.front_wheel_radius();
}

template <typename T, typename U>
model::real_t Bicycle<T, U>::v() const {
    return m_model.v();
}

template <typename T, typename U>
model::real_t Bicycle<T, U>::dt() const {
    return m_model.dt();
}

template <typename T, typename U>
model::real_t Bicycle<T, U>::get_state_element(full_state_index_t field) {
    /*
     * a field may be a state_t element or an auxiliary_state_t element
     * depending on what model is used.
     */
    if (m_model.auxiliary_state_field(field)) {
        return m_kstate[static_cast<uint8_t>(field)];
    } else {
        uint8_t i = static_cast<uint8_t>(field) -
            static_cast<uint8_t>(model_t::auxiliary_state_index_t::number_of_types);
        return m_dstate[i];
    }
}

} // namespace sim
