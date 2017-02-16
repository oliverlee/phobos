#pragma once
#include "simulation.pb.h"
#include "bicycle/bicycle.h"
#include "kalman.h"
#include "simbicycle.h"

namespace message {
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

    template <typename kalman_t>
    void set_symmetric_output_matrix(SymmetricOutputMatrixMessage* pb,
            const typename kalman_t::measurement_noise_covariance_t& m);
    template <typename kalman_t>
    void set_kalman_gain_matrix(KalmanGainMatrixMessage* pb,
            const typename kalman_t::kalman_gain_t& m);
    template <typename kalman_t>
    void set_kalman_noise_covariances(BicycleKalmanMessage* pb,
            const kalman_t& k);
    template <typename kalman_t>
    void set_kalman_gain(BicycleKalmanMessage* pb,
            const kalman_t& k);

    template <typename simbicycle_t>
    void set_simulation_initial_values(SimulationMessage* pb, const simbicycle_t& b);
    template <typename simbicycle_t>
    void set_simulation_loop_values(SimulationMessage* pb, const simbicycle_t& b,
            float measured_steer_torque, float measured_motor_torque,
            float encoder_count, float commanded_feedback_torque);

    void set_simulation_timing(SimulationMessage* pb, uint32_t computation_time, uint32_t transmission_time);
} // namespace message

namespace message {

template <typename kalman_t>
void set_symmetric_output_matrix(SymmetricOutputMatrixMessage* pb,
        const typename kalman_t::measurement_noise_covariance_t& m) {
    // TODO: autogenerate in case state size changes in the future
    auto p = pb->m;
    auto d = m.data();
    std::memcpy(p, d, 2*sizeof(pb->m[0]));
    p += 2;
    d += 2 + 1;
    std::memcpy(p, d, 1*sizeof(pb->m[0]));
    pb->m_count = sizeof(pb->m)/sizeof(pb->m[0]);
}

template <typename kalman_t>
void set_kalman_gain_matrix(KalmanGainMatrixMessage* pb,
        const typename kalman_t::kalman_gain_t& m) {
    std::memcpy(pb->m, m.data(), sizeof(pb->m));
    pb->m_count = sizeof(pb->m)/sizeof(pb->m[0]);
}

template <typename kalman_t>
inline void set_kalman_noise_covariances(BicycleKalmanMessage* pb, const kalman_t& k) {
    set_symmetric_state_matrix(&pb->process_noise_covariance, k.Q());
    set_symmetric_output_matrix<kalman_t>(&pb->measurement_noise_covariance, k.R());
    pb->has_process_noise_covariance = true;
    pb->has_measurement_noise_covariance = true;
}

template <typename kalman_t>
inline void set_kalman_gain(BicycleKalmanMessage* pb, const kalman_t& k) {
    set_symmetric_state_matrix(&pb->error_covariance, k.P());
    set_kalman_gain_matrix<kalman_t>(&pb->kalman_gain, k.K());
    pb->has_error_covariance = true;
    pb->has_kalman_gain = true;
}

template <typename simbicycle_t>
void set_simulation_initial_values(SimulationMessage* pb, const simbicycle_t& b) {
    set_bicycle_state(&pb->state, bicycle_t::get_state_part(b.full_state()));
    pb->has_state = true;

    set_bicycle_auxiliary_state(&pb->auxiliary_state, bicycle_t::get_auxiliary_state_part(b.full_state()));
    pb->has_auxiliary_state = true;

    set_bicycle_canonical(&pb->model, b.model());
    set_bicycle_discrete_time_state_space(&pb->model, b.model());
    pb->has_model = true;

    // TODO: this assumes observer is kalman type
    set_kalman_noise_covariances<typename simbicycle_t::observer_t>(&pb->kalman, b.observer());
    set_kalman_gain<typename simbicycle_t::observer_t>(&pb->kalman, b.observer());
    pb->has_kalman = true;
}

template <typename simbicycle_t>
void set_simulation_loop_values(SimulationMessage* pb, const simbicycle_t& b,
        uint32_t measured_steer_torque, uint32_t measured_motor_torque,
        uint32_t steer_encoder_count, uint32_t rear_wheel_encoder_count,
        uint32_t commanded_feedback_torque) {
    pb->sensors.kistler_measured_torque = measured_steer_torque;
    pb->sensors.kollmorgen_actual_torque = measured_motor_torque;
    pb->sensors.steer_encoder_count = steer_encoder_count;
    pb->sensors.rear_wheel_encoder_count = rear_wheel_encoder_count;
    pb->sensors.has_rear_wheel_encoder_count = true;
    pb->has_sensors = true;

    pb->actuators.kollmorgen_command_torque = commanded_feedback_torque;
    pb->has_actuators = true;

    // TODO: input is repeated from sensors, should this still be recorded?
    // This differs from torque input as these are the converted float values
    // as opposed to the unsigned integer values of the sensors.
    //set_bicycle_input(&pb->input, b.u());
    //pb->has_input = true;

    set_bicycle_state(&pb->state, bicycle_t::get_state_part(b.full_state()));
    pb->has_state = true;

    set_bicycle_auxiliary_state(&pb->auxiliary_state, bicycle_t::get_auxiliary_state_part(b.full_state()));
    pb->has_auxiliary_state = true;

    // TODO: add missing measurement z?
    // This differs from measurement input as these are the converted float values
    // as opposed to the unsigned integer values of the sensors.

    // TODO: Should pose be included in this message or should it be transmitted
    // in a separate message to Unity?
    //pb->pose = b.pose();
    //pb->has_pose = true;

    // TODO: Stop writing once this stops changing. This assumes observer is of Kalman type
    set_kalman_gain(&pb->kalman, b.observer());
    pb->has_kalman = true;
}

} // namespace message
