#pragma once
#include "messages.pb.h"
#include "bicycle.h"
#include "kalman.h"
#include "virtualbicycle.h"

namespace message {
    using bicycle_t = model::Bicycle;
    using kalman_t = observer::Kalman<bicycle_t>;

    void set_state_matrix(StateMatrix* pb, const bicycle_t::state_matrix_t& m);
    void set_input_matrix(InputMatrix* pb, const bicycle_t::input_matrix_t& m);
    void set_output_matrix(OutputMatrix* pb, const bicycle_t::output_matrix_t& m);
    void set_feedthrough_matrix(FeedthroughMatrix* pb, const bicycle_t::feedthrough_matrix_t& m);
    void set_second_order_matrix(SecondOrderMatrix* pb, const bicycle_t::second_order_matrix_t& m);
    void set_symmetric_state_matrix(SymmetricStateMatrix* pb, const bicycle_t::state_matrix_t& m);
    void set_symmetric_output_matrix(SymmetricOutputMatrix* pb, const kalman_t::measurement_noise_covariance_t& m);
    void set_kalman_gain_matrix(KalmanGainMatrix* pb, const kalman_t::kalman_gain_t& m);

    void set_bicycle_state(BicycleState* pb, const bicycle_t::state_t& x);
    void set_bicycle_input(BicycleInput* pb, const bicycle_t::input_t& u);

    void set_bicycle_canonical(BicycleModel* pb, const bicycle_t& b);
    void set_bicycle_discrete_time_state_space(BicycleModel* pb, const bicycle_t& b);
    void set_kalman_noise_covariances(Kalman* pb, const kalman_t& k);
    void set_kalman_gain(Kalman* pb, const kalman_t& k);

    void set_clustril_initial_values(ClustrilMessage* pb, const VirtualBicycle& b);
    void set_clustril_loop_values(ClustrilMessage* pb, const VirtualBicycle& b,
            float measured_steer_torque, float measured_motor_torque,
            float encoder_count, float commanded_feedback_torque);
} // namespace message
