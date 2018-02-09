#include "messageutil.h"
#include "gitsha1.h"
#include <cstring>

namespace message {

void set_bicycle_state(BicycleStateMessage* pb, const bicycle_t::state_t& x) {
    std::memcpy(pb->x, x.data(), sizeof(pb->x));
    pb->x_count = sizeof(pb->x)/sizeof(pb->x[0]);
}

void set_bicycle_auxiliary_state(BicycleAuxiliaryStateMessage* pb, const bicycle_t::auxiliary_state_t& x) {
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

void set_simulation_gitsha1(SimulationMessage* pb) {
    std::memcpy(pb->gitsha1.f, g_GITSHA1, sizeof(pb->gitsha1.f)*sizeof(pb->gitsha1.f[0]));
    pb->has_gitsha1 = true;
}

void set_simulation_sensors(SimulationMessage* pb,
        uint32_t measured_steer_torque, uint32_t measured_motor_torque,
        uint32_t steer_encoder_count, uint32_t rear_wheel_encoder_count) {
    pb->sensors.kistler_measured_torque = measured_steer_torque;
    pb->sensors.kollmorgen_actual_torque = measured_motor_torque;
    pb->sensors.steer_encoder_count = steer_encoder_count;
    pb->sensors.rear_wheel_encoder_count = rear_wheel_encoder_count;
    pb->sensors.has_rear_wheel_encoder_count = true;
    pb->has_sensors = true;
}

void set_simulation_actuators(SimulationMessage* pb,
        uint32_t commanded_torque) {
    pb->actuators.kollmorgen_command_torque = commanded_torque;
    pb->has_actuators = true;
}

void set_simulation_timing(SimulationMessage* pb,
        uint32_t computation_time, uint32_t transmission_time) {
    pb->timing.computation = computation_time;
    pb->timing.has_computation = true;
    pb->timing.transmission = transmission_time;
    pb->timing.has_transmission = true;
    pb->has_timing = true;
}

void set_simulation_feedback(SimulationMessage* pb,
        float torque, float error, float error_derivative) {
    pb->controller.feedback.torque = torque;
    pb->controller.feedback.has_torque = true;
    pb->controller.feedback.error = error;
    pb->controller.feedback.has_error = true;
    pb->controller.feedback.error_derivative = error_derivative;
    pb->controller.feedback.has_error_derivative = true;
    pb->controller.has_feedback = true;
    pb->has_controller = true;
}

void set_simulation_feedforward(SimulationMessage* pb,
        float torque, float acceleration) {
    pb->controller.feedforward.torque = torque;
    pb->controller.feedforward.has_torque = true;
    pb->controller.feedforward.acceleration = acceleration;
    pb->controller.feedforward.has_acceleration = true;
    pb->controller.has_feedforward = true;
    pb->has_controller = true;
}

} // namespace message
