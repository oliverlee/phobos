#include "virtualbicycle.h"
#include "angle.h"
#include "packet/serialize.h"
#include "packet/framing.h"

#include "parameters.h"

#include <array>


namespace {
    std::array<uint8_t, BicyclePose_size> serialize_buffer;
    std::array<uint8_t, BicyclePose_size + packet::framing::FRAME_STUFF_OVERHEAD> frame_buffer;
} // namespace

VirtualBicycle::VirtualBicycle(float v, float dt, float sigma0, float sigma1) :
m_bicycle(v , dt),
m_kalman(m_bicycle, /* bicycle model used in Kalman filter */
        parameters::defaultvalue::kalman::Q(dt), /* process noise cov */
        (kalman_t::measurement_noise_covariance_t() << /* measurement noise cov */
         sigma0,      0,
              0, sigma1).finished(),
        bicycle_t::state_t::Zero(), /* initial state estimate */
        std::pow(sigma0, 2)*bicycle_t::state_matrix_t::Identity()), /* error cov */ // FIXME
        m_pose_size(0) {
    m_u.setZero();
    m_z.setZero();
    m_x_aux.setZero();

    /* use an initial guess of 30 degrees for pitch */
    m_x_aux[2] = m_bicycle.solve_constraint_pitch(m_kalman.x(), 30 * constants::as_radians);

    m_pose = BicyclePose_init_zero;
}

void VirtualBicycle::update(float roll_torque_input, float steer_torque_input, /* u[0], u[1] */
                       float yaw_angle_measurement, float steer_angle_measurement) { /* z[0], z[1] */
    m_u << roll_torque_input, steer_torque_input;
    m_z << yaw_angle_measurement, steer_angle_measurement;

    m_kalman.time_update(m_u);
    m_kalman.measurement_update(m_z);

    m_x_aux = m_bicycle.x_aux_next(m_kalman.x(), m_x_aux);

    m_pose.timestamp = 1; // FIXME when running at a different rate
    m_pose.x = m_x_aux[0];
    m_pose.y = m_x_aux[1];
    m_pose.pitch = m_x_aux[2];
    m_pose.yaw = m_kalman.x()[0];
    m_pose.roll = m_kalman.x()[1];
    m_pose.steer = m_kalman.x()[2];
}

/* WARNING: this member function is not thread safe with multiple VirtualBicycle objects */
uint8_t VirtualBicycle::encode_and_stuff_pose() {
    uint8_t bytes_written = packet::serialize::encode(m_pose, serialize_buffer.data(), serialize_buffer.size());
    packet::framing::stuff(serialize_buffer.data(), frame_buffer.data(), bytes_written);

    m_pose_size = bytes_written + packet::framing::FRAME_STUFF_OVERHEAD;
    return m_pose_size;
}

const VirtualBicycle::bicycle_t::state_t& VirtualBicycle::x() const {
    return m_kalman.x();
}

const VirtualBicycle::bicycle_t::input_t& VirtualBicycle::u() const {
    return m_u;
}

const VirtualBicycle::bicycle_t::output_t& VirtualBicycle::z() const {
    return m_z;
}

const VirtualBicycle::bicycle_t::auxiliary_state_t& VirtualBicycle::x_aux() const {
    return m_x_aux;
}

const BicyclePose& VirtualBicycle::pose() const {
    return m_pose;
}

const VirtualBicycle::bicycle_t& VirtualBicycle::model() const {
    return m_bicycle;
}

const VirtualBicycle::kalman_t& VirtualBicycle::kalman() const {
    return m_kalman;
}

const uint8_t* VirtualBicycle::pose_buffer() const {
    return frame_buffer.data();
}

uint8_t VirtualBicycle::pose_buffer_size() const {
    return m_pose_size;
}
