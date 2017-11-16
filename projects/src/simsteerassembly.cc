#include "simsteerassembly.h"
#include <cmath> // std::abs

namespace sim {
SteerAssembly::SteerAssembly(model::real_dt dt, model::real_t inertia, model::real_t damping, model::real_t q) :
    m_model(dt, inertia, damping),
    m_kalman(m_model,
             kalman_t::state_t::Zero(),
             (kalman_t::process_noise_covariance_t() <<
              dt*dt*dt*dt/4, dt*dt*dt/2,
                 dt*dt*dt/2,      dt*dt).finished() * q,
             (kalman_t::measurement_noise_covariance_t() <<
              steer_angle_measurement_noise_variance).finished(),
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
        model::real_t steer_angle_measurement) {
    const kalman_t::input_t u =
        (kalman_t::input_t() << torque_input).finished();
    const kalman_t::measurement_t z =
        (kalman_t::measurement_t() << steer_angle_measurement).finished();

    m_kalman.time_update(u);
    m_kalman.measurement_update(z);
}

model::real_t SteerAssembly::velocity() const {
    return m_kalman.x()[1];
}

} // namespace sim
