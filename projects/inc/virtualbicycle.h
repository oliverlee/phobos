#pragma once
#include "bicycle/whipple.h"
#include "kalman.h"
#include "pose.pb.h"
#include "constants.h"

class VirtualBicycle {
    public:
        using bicycle_t = model::BicycleWhipple;
        using kalman_t = observer::Kalman<bicycle_t>;

        /* Bicycle model parameters */
        static constexpr float default_fs = 200.0f; // sample rate [Hz]
        static constexpr float default_dt = 1.0f/default_fs; // sample time [s]
        static constexpr float default_v = 5.0f; // forward speed [m/s]

        /* Kalman filter variance values */
        static constexpr float default_sigma0 = 1 * constants::as_radians; // yaw angle measurement noise variance
        static constexpr float default_sigma1 = 0.008 * constants::as_radians; // steer angle measurement noise variance

        VirtualBicycle(float v = default_v, float dt = default_dt,
                       float sigma0 = default_sigma0, float sigma1 = default_sigma1);
        void update(float roll_torque_input, float steer_torque_input,
                    float yaw_angle_measurement, float steer_angle_measurement);
        uint8_t encode_and_stuff_pose();

        /* const accessors */
        const bicycle_t::state_t& x() const; /* get most recent state (estimate) computed */
        const bicycle_t::input_t& u() const; /* get most recent input used */
        const bicycle_t::output_t& z() const; /* get most recent measurement used */
        const bicycle_t::auxiliary_state_t& x_aux() const; /* get most recent auxiliary state (estimate) computed */
        const BicyclePoseMessage& pose() const; /* get most recent pose computed */
        const bicycle_t& model() const; /* get current bicycle model */
        const kalman_t& kalman() const; /* get current Kalman model */
        const uint8_t* pose_buffer() const; /* get pointer to buffer with pose data */
        uint8_t pose_buffer_size() const; /* size of data in pose buffer */

    private:
        bicycle_t m_bicycle;
        kalman_t m_kalman;
        //bicycle_t::state_t m_x; /* yaw angle, roll angle, steer angle, roll rate, steer rate */
        bicycle_t::input_t m_u; /* roll torque, steer torque */
        bicycle_t::output_t m_z; /* yaw angle, steer angle */
        bicycle_t::auxiliary_state_t m_x_aux; /* rear contact x, rear contact y, pitch angle */
        BicyclePoseMessage m_pose; /* visualization fields */
        uint8_t m_pose_size; /* size of encoded and framed pose in bytes */
};
