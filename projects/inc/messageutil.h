#pragma once
#include "simulation.pb.h"
#include "bicycle/bicycle.h"
#include "kalman.h"
#include "simbicycle.h"

namespace message {
    void set_config_message(ConfigMessage* pb, EnumProjectType e);

    using bicycle_t = model::Bicycle;
    void set_bicycle_state(BicycleStateMessage* pb, const bicycle_t::state_t& x);
    void set_bicycle_auxiliary_state(BicycleAuxiliaryStateMessage* pb, const bicycle_t::auxiliary_state_t& x);
    void set_bicycle_input(BicycleInputMessage* pb, const bicycle_t::input_t& u);
    void set_state_matrix(StateMatrixMessage* pb, const bicycle_t::state_matrix_t& m);
    void set_input_matrix(InputMatrixMessage* pb, const bicycle_t::input_matrix_t& m);
    void set_output_matrix(OutputMatrixMessage* pb, const bicycle_t::output_matrix_t& m);
    void set_feedthrough_matrix(FeedthroughMatrixMessage* pb, const bicycle_t::feedthrough_matrix_t& m);
    void set_second_order_matrix(SecondOrderMatrixMessage* pb, const bicycle_t::second_order_matrix_t& m);
    void set_symmetric_state_matrix(SymmetricStateMatrixMessage* pb, const bicycle_t::state_matrix_t& m);
    void set_bicycle_canonical(BicycleModelMessage* pb, const bicycle_t& b);
    void set_bicycle_discrete_time_state_space(BicycleModelMessage* pb, const bicycle_t& b);

    /* functions for different observer variants */
    template <typename observer_t>
    typename std::enable_if<std::is_same<observer_t, typename observer::Kalman<typename observer_t::model_t>>::value, void>::type
    set_symmetric_output_matrix(SymmetricOutputMatrixMessage* pb, const typename observer_t::measurement_noise_covariance_t& m);

    template <typename observer_t>
    typename std::enable_if<!std::is_same<observer_t, typename observer::Kalman<typename observer_t::model_t>>::value, void>::type
    set_symmetric_output_matrix(SymmetricOutputMatrixMessage* pb, const typename observer_t::measurement_noise_covariance_t& m);

    template <typename observer_t>
    typename std::enable_if<std::is_same<observer_t, typename observer::Kalman<typename observer_t::model_t>>::value, void>::type
    set_observer_gain_matrix(KalmanGainMatrixMessage* pb, const typename observer_t::kalman_gain_t& m);

    template <typename observer_t>
    typename std::enable_if<!std::is_same<observer_t, typename observer::Kalman<typename observer_t::model_t>>::value, void>::type
    set_observer_gain_matrix(KalmanGainMatrixMessage* pb, const typename observer_t::kalman_gain_t& m);

    template <typename observer_t>
    typename std::enable_if<std::is_same<observer_t, typename observer::Kalman<typename observer_t::model_t>>::value, void>::type
    set_kalman_noise_covariances(BicycleKalmanMessage* pb, const observer_t& k);

    template <typename observer_t>
    typename std::enable_if<!std::is_same<observer_t, typename observer::Kalman<typename observer_t::model_t>>::value, void>::type
    set_kalman_noise_covariances(BicycleKalmanMessage* pb, const observer_t& k);

    template <typename observer_t>
    typename std::enable_if<std::is_same<observer_t, typename observer::Kalman<typename observer_t::model_t>>::value, void>::type
    set_kalman_gain(BicycleKalmanMessage* pb, const observer_t& k);

    template <typename observer_t>
    typename std::enable_if<!std::is_same<observer_t, typename observer::Kalman<typename observer_t::model_t>>::value, void>::type
    set_kalman_gain(BicycleKalmanMessage* pb, const observer_t& k);

    void set_simulation_sensors(SimulationMessage* pb,
            uint32_t measured_steer_torque, uint32_t measured_motor_torque,
            uint32_t steer_encoder_count, uint32_t rear_wheel_encoder_count);
    void set_simulation_actuators(SimulationMessage* pb,
            uint32_t commanded_feedback_velocity);
    void set_simulation_timing(SimulationMessage* pb,
            uint32_t computation_time, uint32_t transmission_time);

    template <typename simbicycle_t>
    void set_simulation_state(SimulationMessage* pb, const simbicycle_t& b);
    template <typename simbicycle_t>
    void set_simulation_auxiliary_state(SimulationMessage* pb, const simbicycle_t& b);
    template <typename simbicycle_t>
    void set_simulation_pose(SimulationMessage* pb, const simbicycle_t& b);

    template <typename simbicycle_t>
    void set_simulation_full_model(SimulationMessage* pb, const simbicycle_t& b);

    /* functions for different observer variants */
    template <typename simbicycle_t>
    typename std::enable_if<std::is_same<typename simbicycle_t::observer_t, typename observer::Kalman<typename simbicycle_t::model_t>>::value, void>::type
    set_simulation_full_model_observer(SimulationMessage* pb, const simbicycle_t& b);

    template <typename simbicycle_t>
    typename std::enable_if<!std::is_same<typename simbicycle_t::observer_t, typename observer::Kalman<typename simbicycle_t::model_t>>::value, void>::type
    set_simulation_full_model_observer(SimulationMessage* pb, const simbicycle_t& b);
} // namespace message

namespace message {

template <typename observer_t>
typename std::enable_if<std::is_same<observer_t, typename observer::Kalman<typename observer_t::model_t>>::value, void>::type
set_symmetric_output_matrix(SymmetricOutputMatrixMessage* pb, const typename observer_t::measurement_noise_covariance_t& m) {
    // TODO: autogenerate in case state size changes in the future
    auto p = pb->m;
    auto d = m.data();
    std::memcpy(p, d, 2*sizeof(pb->m[0]));
    p += 2;
    d += 2 + 1;
    std::memcpy(p, d, 1*sizeof(pb->m[0]));
    pb->m_count = sizeof(pb->m)/sizeof(pb->m[0]);
}

template <typename observer_t>
typename std::enable_if<!std::is_same<observer_t, typename observer::Kalman<typename observer_t::model_t>>::value, void>::type
set_symmetric_output_matrix(SymmetricOutputMatrixMessage* pb, const typename observer_t::measurement_noise_covariance_t& m) {
    // no-op
    (void)pb;
    (void)m;
}

template <typename observer_t>
typename std::enable_if<std::is_same<observer_t, typename observer::Kalman<typename observer_t::model_t>>::value, void>::type
set_observer_gain_matrix(KalmanGainMatrixMessage* pb, const typename observer_t::kalman_gain_t& m) {
    std::memcpy(pb->m, m.data(), sizeof(pb->m));
    pb->m_count = sizeof(pb->m)/sizeof(pb->m[0]);
}

template <typename observer_t>
typename std::enable_if<!std::is_same<observer_t, typename observer::Kalman<typename observer_t::model_t>>::value, void>::type
set_observer_gain_matrix(KalmanGainMatrixMessage* pb, const typename observer_t::kalman_gain_t& m) {
    // no-op
    (void)pb;
    (void)m;
}

template <typename observer_t>
typename std::enable_if<std::is_same<observer_t, typename observer::Kalman<typename observer_t::model_t>>::value, void>::type
set_kalman_noise_covariances(BicycleKalmanMessage* pb, const observer_t& k) {
    set_symmetric_state_matrix(&pb->process_noise_covariance, k.Q());
    set_symmetric_output_matrix<observer_t>(&pb->measurement_noise_covariance, k.R());
    pb->has_process_noise_covariance = true;
    pb->has_measurement_noise_covariance = true;
}

template <typename observer_t>
typename std::enable_if<!std::is_same<observer_t, typename observer::Kalman<typename observer_t::model_t>>::value, void>::type
set_kalman_noise_covariances(BicycleKalmanMessage* pb, const observer_t& k) {
    // no-op
    (void)pb;
    (void)k;
}

template <typename observer_t>
typename std::enable_if<std::is_same<observer_t, typename observer::Kalman<typename observer_t::model_t>>::value, void>::type
set_kalman_gain(BicycleKalmanMessage* pb, const observer_t& k) {
    set_symmetric_state_matrix(&pb->error_covariance, k.P());
    set_observer_gain_matrix<observer_t>(&pb->kalman_gain, k.K());
    pb->has_error_covariance = true;
    pb->has_kalman_gain = true;
}

template <typename observer_t>
typename std::enable_if<!std::is_same<observer_t, typename observer::Kalman<typename observer_t::model_t>>::value, void>::type
set_kalman_gain(BicycleKalmanMessage* pb, const observer_t& k) {
    // no-op
    (void)pb;
    (void)k;
}

template <typename simbicycle_t>
void set_simulation_state(SimulationMessage* pb, const simbicycle_t& b) {
    set_bicycle_state(&pb->state, bicycle_t::get_state_part(b.full_state()));
    pb->has_state = true;
}

template <typename simbicycle_t>
void set_simulation_auxiliary_state(SimulationMessage* pb, const simbicycle_t& b) {
    set_bicycle_auxiliary_state(&pb->auxiliary_state, bicycle_t::get_auxiliary_state_part(b.full_state()));
    pb->has_auxiliary_state = true;
}

template <typename simbicycle_t>
void set_simulation_pose(SimulationMessage* pb, const simbicycle_t& b) {
    pb->pose = b.pose();
    pb->has_pose = true;
}

template <typename simbicycle_t>
void set_simulation_full_model(SimulationMessage* pb, const simbicycle_t& b) {
    set_simulation_state(pb, b);
    set_simulation_auxiliary_state(pb, b);

    set_bicycle_canonical(&pb->model, b.model());
    set_bicycle_discrete_time_state_space(&pb->model, b.model());
    pb->has_model = true;
}

template <typename simbicycle_t>
typename std::enable_if<std::is_same<typename simbicycle_t::observer_t, typename observer::Kalman<typename simbicycle_t::model_t>>::value, void>::type
set_simulation_full_model_observer(SimulationMessage* pb, const simbicycle_t& b) {
    set_simulation_full_model(pb, b);

    // TODO: this assumes observer is kalman type
    set_kalman_noise_covariances<typename simbicycle_t::observer_t>(&pb->kalman, b.observer());
    set_kalman_gain<typename simbicycle_t::observer_t>(&pb->kalman, b.observer());
    pb->has_kalman = true;
}

template <typename simbicycle_t>
typename std::enable_if<!std::is_same<typename simbicycle_t::observer_t, typename observer::Kalman<typename simbicycle_t::model_t>>::value, void>::type
set_simulation_full_model_observer(SimulationMessage* pb, const simbicycle_t& b) {
    set_simulation_full_model(pb, b);
}

} // namespace message
