#include "simsteerassembly.h"
#include <cmath> // std::abs

namespace sim {
SteerAssembly::SteerAssembly(model::real_t inertia, model::real_t dt, model::real_t q) :
    m_model(inertia, dt),
    m_kalman(m_model,
             kalman_t::state_t::Zero(),
             (kalman_t::process_noise_covariance_t() <<
              dt*dt*dt*dt/4, dt*dt*dt/2,
                 dt*dt*dt/2,      dt*dt).finished() * q,
             (kalman_t::measurement_noise_covariance_t() <<
              steer_angle_measurement_noise_variance,                                     0,
                                                   0, steer_rate_measurement_noise_variance).finished(),
             kalman_t::error_covariance_t::Zero()) { }

const imp::SteerAssemblyModel& SteerAssembly::model() const {
    return m_model;
}

SteerAssembly::kalman_t& SteerAssembly::observer() {
    return m_kalman;
}

const SteerAssembly::kalman_t& SteerAssembly::observer() const {
    return m_kalman;
}

void SteerAssembly::update_state_estimate(
        model::real_t torque_input,
        model::real_t steer_angle_measurement,
        model::real_t steer_rate_measurement) {
    const kalman_t::input_t u =
        (kalman_t::input_t() << torque_input).finished();
    const kalman_t::measurement_t z =
        (kalman_t::measurement_t() << steer_angle_measurement,
                                    steer_rate_measurement).finished();

    m_kalman.time_update(u);

    if (std::abs(steer_rate_measurement) > steer_rate_sensor_limit) {
        // gyro sensor cannot measure angular rates above 1.7 rad/s
        // and signal will saturate
        static constexpr model::real_t r0 = steer_angle_measurement_noise_variance;
        static constexpr model::real_t r1 = steer_rate_measurement_noise_variance;
        m_kalman.measurement_update(
                z,
                (kalman_t::measurement_noise_covariance_t() <<
                 r0,     0,
                  0, 10*r1).finished());
    } else {
        m_kalman.measurement_update(z);
    }
}

model::real_t SteerAssembly::velocity() const {
    return m_kalman.x()[1];
}

} // namespace sim
