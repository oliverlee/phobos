#pragma once
#include "simulation.pb.h"
#include "bicycle/bicycle.h"
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

    void set_simulation_gitsha1(SimulationMessage* pb);
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
} // namespace message

namespace message {

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
    set_simulation_gitsha1(pb);

    set_simulation_state(pb, b);
    set_simulation_auxiliary_state(pb, b);

    set_bicycle_canonical(&pb->model, b.model());
    set_bicycle_discrete_time_state_space(&pb->model, b.model());
    pb->has_model = true;
}

} // namespace message
