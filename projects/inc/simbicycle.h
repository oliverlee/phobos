#pragma once
#include "ch.h"
#include "pose.pb.h"
#include "saconfig.h"
#include "haptic.h"
// bicycle submodule imports
#include "bicycle/bicycle.h"
#include "observer.h"
#include "constants.h" // rad/deg constants, real_t
#include "parameters.h"

#include <type_traits>

namespace sim {

/*
 * This template class simulates a bicycle model (template argument Model)
 * with an observer type (template argument Observer) and a handlebar feedback type Haptic
 *  - incorporates fields necessary for visualization, such as wheel angle
 *  - provides a single interface for using different bicycle models
 *  - allows simulation of dynamics and kinematics separately
 */
template <typename Model, typename Observer, typename Haptic = haptic::null_t>
class Bicycle {
    static_assert(std::is_base_of<model::Bicycle, Model>::value,
            "Invalid template parameter type for sim::Bicycle");
    static_assert(std::is_base_of<observer::ObserverBase, Observer>::value,
            "Invalid template parameter type for sim::Bicycle");
    static_assert(std::is_base_of<haptic::HandlebarBase, Haptic>::value,
            "Invalid template parameter type for sim::Bicycle");

    public:
        using model_t = Model;
        using observer_t = Observer;
        using haptic_t = Haptic; // right now we only have handlebar feedback
                                 // with pedaling feedback to be implemented
        using real_t = model::real_t;
        using second_order_matrix_t = typename model_t::second_order_matrix_t;
        using state_t = typename model_t::state_t;
        using auxiliary_state_t = typename model_t::auxiliary_state_t;
        using full_state_t = typename model_t::full_state_t;
        using input_t = typename model_t::input_t;
        using measurement_t = typename model_t::output_t;
        using full_state_index_t = typename model_t::full_state_index_t;

        // default bicycle model parameters
        static constexpr real_t default_fs = 200.0; // sample rate, Hz
        static constexpr real_t default_dt = 1.0/default_fs; // sample period, s
        static constexpr real_t default_v = 5.0; // forward speed, m/s
        static constexpr real_t default_steer_inertia = sa::FULL_ASSEMBLY_INERTIA; // kg-m^2
        static constexpr real_t v_quantization_resolution = 0.1; // m/s
        static constexpr real_t roll_rate_limit = 1e10; // rad
        static constexpr real_t steer_rate_limit = 1e10; // rad
        static constexpr real_t observer_prime_period = 3.0; // seconds

        Bicycle(real_t v = default_v, real_t dt = default_dt, real_t steer_inertia = default_steer_inertia);

        void set_v(real_t v);
        void set_dt(real_t dt);
        void reset();
        void update_dynamics(real_t roll_torque_input, // update bicycle internal state
                real_t steer_torque_input,             // and handlebar feedback torque
                real_t yaw_angle_measurement,
                real_t steer_angle_measurement,
                real_t steer_rate_measurement,
                real_t rear_wheel_angle_measurement);
        void update_kinematics(); // update bicycle pose
        void prime_observer(); // perform observer specific initialization routine

        const BicyclePoseMessage& pose() const; // get most recently computed pose
        real_t handlebar_feedback_torque() const; // get most recently computed feedback torque
        const input_t& input() const; // get most recently computed input
        const full_state_t& full_state() const; // get most recently computed full state

        // common bicycle model member variables
        model_t& model();
        const model_t& model() const;
        observer_t& observer();
        const observer_t& observer() const;
        const second_order_matrix_t& M() const;
        const second_order_matrix_t& C1() const;
        const second_order_matrix_t& K0() const;
        const second_order_matrix_t& K2() const;
        real_t wheelbase() const;
        real_t trail() const;
        real_t steer_axis_tilt() const;
        real_t rear_wheel_radius() const;
        real_t front_wheel_radius() const;
        real_t v() const;
        real_t dt() const;

    private:
        model_t m_model; // bicycle model object
        observer_t m_observer; // observer object
        haptic_t m_haptic; // handlebar feedback calculation object
        full_state_t m_state_full; // auxiliary + dynamic state
        BicyclePoseMessage m_pose; // Unity visualization message
        input_t m_input; // bicycle model input vector
        binary_semaphore_t m_state_sem; // bsem for synchronizing kinematics update
        real_t m_T_m; // handlebar feedback torque
};

} // namespace sim

#include "simbicycle.hh"
