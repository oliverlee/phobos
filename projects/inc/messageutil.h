#pragma once
#include "simulation.pb.h"
#include "build.pb.h"
#include "pose.pb.h"
#include "bicycle/bicycle.h"
#include "simbicycle.h"
#include <cstring>

namespace message {
    /**
     There are utility functions to simplify storing of data.
     */
    void set_build_config(pbBuildConfig* pb);

    void set_state_matrix(pbStateMatrix* pb, const model::Bicycle::state_matrix_t& m);
    void set_input_matrix(pbInputMatrix* pb, const model::Bicycle::input_matrix_t& m);
    void set_output_matrix(pbOutputMatrix* pb, const model::Bicycle::output_matrix_t& m);
    void set_feedthrough_matrix(pbFeedthroughMatrix* pb, const model::Bicycle::feedthrough_matrix_t& m);
    void set_second_order_matrix(pbSecondOrderMatrix* pb, const model::Bicycle::second_order_matrix_t& m);

    void set_model_state(pbModelState* pb, const model::Bicycle::state_t& m);
    void set_model_input(pbModelInput* pb, const model::Bicycle::input_t& m);
    void set_model_auxiliary_state(pbModelAuxiliaryState* pb, const model::Bicycle::auxiliary_state_t& m);

    void set_model_state_space(pbModelStateSpace* pb, const model::Bicycle& bicycle);

    template <typename PbType, typename EigenType>
    void set_matrix(PbType* pb, const EigenType& m);

    template <typename Model>
    void set_simulation_model(pbSimulation* pb, const sim::Bicycle<Model>& sim);

    template <typename Model>
    void set_pose(pbPose* pb, const sim::Bicycle<Model>& sim);
} // namespace message


namespace message {

namespace impl {
template <typename PbType, typename EigenType>
void set_matrix(PbType* pb, const EigenType& m) {
    static_assert(
            sizeof(pb->data) == (
                sizeof(typename EigenType::Scalar) *
                EigenType::RowsAtCompileTime *
                EigenType::ColsAtCompileTime),
            "Number of matrix elements do not match.");

    std::memcpy(pb->data, m.data(), sizeof(pb->data));
}
}

template <typename Model>
void set_simulation_model(pbSimulation* pb, const sim::Bicycle<Model>& sim) {
    set_model_state_space(&pb->model, sim.model());
    pb->model_v = sim.v();
    set_model_state(&pb->state, sim.state());
    set_model_input(&pb->input, sim.input());
    set_model_auxiliary_state(&pb->auxiliary_state, sim.auxiliary_state());
}

template <typename Model>
void set_pose(pbPose* pb, const sim::Bicycle<Model>& sim) {
    pb->x_position = model::Bicycle::get_full_state_element(
            sim.full_state(),
            model::Bicycle::full_state_index_t::x);
    pb->y_position = model::Bicycle::get_full_state_element(
            sim.full_state(),
            model::Bicycle::full_state_index_t::y);
    pb->yaw_angle = model::Bicycle::get_full_state_element(
            sim.full_state(),
            model::Bicycle::full_state_index_t::yaw_angle);
    pb->roll_angle = model::Bicycle::get_full_state_element(
            sim.full_state(),
            model::Bicycle::full_state_index_t::yaw_angle);
    pb->pitch_angle = model::Bicycle::get_full_state_element(
            sim.full_state(),
            model::Bicycle::full_state_index_t::pitch_angle);
    pb->steer_angle = model::Bicycle::get_full_state_element(
            sim.full_state(),
            model::Bicycle::full_state_index_t::steer_angle);
    pb->rear_wheel_angle = model::Bicycle::get_full_state_element(
            sim.full_state(),
            model::Bicycle::full_state_index_t::rear_wheel_angle);
}

} // namespace message
