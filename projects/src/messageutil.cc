#include "messageutil.h"
#include <cstring>

namespace message {

void set_bicycle_state(BicycleStateMessage* pb, const bicycle_t::state_t& x) {
    std::memcpy(pb->x, x.data(), sizeof(pb->x));
    pb->x_count = sizeof(pb->x)/sizeof(pb->x[0]);
}

void set_bicycle_input(BicycleInputMessage* pb, const bicycle_t::input_t& u) {
    std::memcpy(pb->u, u.data(), sizeof(pb->u));
    pb->u_count = sizeof(pb->u)/sizeof(pb->u[0]);
}

void set_state_matrix(StateMatrixMessage* pb, const bicycle_t::state_matrix_t& m) {
    std::memcpy(pb->m, m.data(), sizeof(pb->m));
    pb->m_count = sizeof(pb->m)/sizeof(pb->m[0]);
}

void set_input_matrix(InputMatrixMessage* pb, const bicycle_t::input_matrix_t& m) {
    std::memcpy(pb->m, m.data(), sizeof(pb->m));
    pb->m_count = sizeof(pb->m)/sizeof(pb->m[0]);
}

void set_output_matrix(OutputMatrixMessage* pb, const bicycle_t::output_matrix_t& m) {
    std::memcpy(pb->m, m.data(), sizeof(pb->m));
    pb->m_count = sizeof(pb->m)/sizeof(pb->m[0]);
}

void set_feedthrough_matrix(FeedthroughMatrixMessage* pb, const bicycle_t::feedthrough_matrix_t& m) {
    std::memcpy(pb->m, m.data(), sizeof(pb->m));
    pb->m_count = sizeof(pb->m)/sizeof(pb->m[0]);
}

void set_second_order_matrix(SecondOrderMatrixMessage* pb, const bicycle_t::second_order_matrix_t& m) {
    std::memcpy(pb->m, m.data(), sizeof(pb->m));
    pb->m_count = sizeof(pb->m)/sizeof(pb->m[0]);
}

void set_symmetric_state_matrix(SymmetricStateMatrixMessage* pb, const bicycle_t::state_matrix_t& m) {
    // TODO: autogenerate in case state size changes in the future
    auto p = pb->m;
    auto d = m.data(); // (0, 0)
    std::memcpy(p, d, 5*sizeof(pb->m[0]));
    p += 5;
    d += 5 + 1; // (6/5, 6%5) = (1, 1)
    std::memcpy(p, d, 4*sizeof(pb->m[0]));
    p += 4;
    d += 5 + 1; // (12/5, 12%5) = (2, 2)
    std::memcpy(p, d, 3*sizeof(pb->m[0]));
    p += 3;
    d += 5 + 1; // (18/5, 18%5) = (3, 3)
    std::memcpy(p, d, 2*sizeof(pb->m[0]));
    p += 2;
    d += 5 + 1; // (24/5, 24%5) = (3, 3)
    std::memcpy(p, d, 1*sizeof(pb->m[0]));
    pb->m_count = sizeof(pb->m)/sizeof(pb->m[0]);
}

void set_symmetric_output_matrix(SymmetricOutputMatrixMessage* pb, const kalman_t::measurement_noise_covariance_t& m) {
    // TODO: autogenerate in case state size changes in the future
    auto p = pb->m;
    auto d = m.data();
    std::memcpy(p, d, 2*sizeof(pb->m[0]));
    p += 2;
    d += 2 + 1;
    std::memcpy(p, d, 1*sizeof(pb->m[0]));
    pb->m_count = sizeof(pb->m)/sizeof(pb->m[0]);
}

void set_kalman_gain_matrix(KalmanGainMatrixMessage* pb, const kalman_t::kalman_gain_t& m) {
    std::memcpy(pb->m, m.data(), sizeof(pb->m));
    pb->m_count = sizeof(pb->m)/sizeof(pb->m[0]);
}

void set_bicycle_canonical(BicycleModelMessage* pb, const bicycle_t& b) {
    set_second_order_matrix(&pb->M, b.M());
    set_second_order_matrix(&pb->C1, b.C1());
    set_second_order_matrix(&pb->K0, b.K0());
    set_second_order_matrix(&pb->K2, b.K2());
    pb->has_M = true;
    pb->has_C1 = true;
    pb->has_K0 = true;
    pb->has_K2 = true;
}

void set_bicycle_discrete_time_state_space(BicycleModelMessage* pb, const bicycle_t& b) {
    pb->v = b.v();
    pb->dt = b.dt();
    pb->has_v = true;
    pb->has_dt = true;

    set_state_matrix(&pb->A, b.Ad());
    set_input_matrix(&pb->B, b.Bd());
    set_output_matrix(&pb->C, b.Cd());
    set_feedthrough_matrix(&pb->D, b.Dd());
    pb->has_A = true;
    pb->has_B = true;
    pb->has_C = true;
    pb->has_D = true;
}

void set_kalman_noise_covariances(BicycleKalmanMessage* pb, const kalman_t& k) {
    set_symmetric_state_matrix(&pb->process_noise_covariance, k.Q());
    set_symmetric_output_matrix(&pb->measurement_noise_covariance, k.R());
    pb->has_process_noise_covariance = true;
    pb->has_measurement_noise_covariance = true;
}

void set_kalman_gain(BicycleKalmanMessage* pb, const kalman_t& k) {
    set_symmetric_state_matrix(&pb->error_covariance, k.P());
    set_kalman_gain_matrix(&pb->kalman_gain, k.K());
    pb->has_error_covariance = true;
    pb->has_kalman_gain = true;
}

void set_clustril_initial_values(ClustrilMessage* pb, const VirtualBicycle& b) {
    message::set_bicycle_state(&pb->state, b.x());
    pb->has_state = true;

    set_bicycle_canonical(&pb->model, b.model());
    set_bicycle_discrete_time_state_space(&pb->model, b.model());
    pb->has_model = true;

    set_kalman_noise_covariances(&pb->kalman, b.kalman());
    set_kalman_gain(&pb->kalman, b.kalman());
    pb->has_kalman = true;
}

void set_clustril_loop_values(ClustrilMessage* pb, const VirtualBicycle& b,
        float measured_steer_torque, float measured_motor_torque,
        float encoder_count, float commanded_feedback_torque) {
    pb->sensors.kistler_measured_torque = measured_steer_torque;
    pb->sensors.kollmorgen_actual_torque = measured_motor_torque;
    pb->sensors.steer_encoder_count = encoder_count;
    pb->has_sensors = true;

    pb->actuators.kollmorgen_command_torque = commanded_feedback_torque;
    pb->has_actuators = true;

    set_bicycle_input(&pb->input, b.u());
    pb->has_input = true;

    set_bicycle_state(&pb->state, b.x());
    pb->has_state = true;

    // TODO: add missing measurement z?

    pb->pose = b.pose();
    pb->has_pose = true;

    // TODO: stop writing once this stop changing
    set_kalman_gain(&pb->kalman, b.kalman());
    pb->has_kalman = true;
}

} // namespace message
