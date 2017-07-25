/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"

#include "analog.h"
#include "encoder.h"

#include "gitsha1.h"
#include "blink.h"
#include "usbconfig.h"
#include "saconfig.h"
#include "utility.h"

#include "parameters.h"

#include "discrete_linear.h"
#include "kalman.h"
#include <Eigen/Cholesky>
#include <unsupported/Eigen/MatrixFunctions>

namespace {
    constexpr systime_t loop_period = MS2ST(1);
    constexpr float dt = static_cast<float>(ST2MS(loop_period))/1000.0f; // seconds
    constexpr float k = 31.4f; // N-m/rad
    constexpr float m_u = sa::UPPER_ASSEMBLY_INERTIA_PHYSICAL; // kg-m^2
    //constexpr float m_l = 0.00592455; // kg-m^2
    constexpr float m_l = sa::FULL_ASSEMBLY_INERTIA_WITH_WEIGHT - m_u;
    constexpr float m = 0.1; // kg-m^2
    //constexpr float k_torque = 0.742f; // [Nm/Arms] motor torque constant
    //constexpr float k_p = 0.396; // [Arms/(rad/s)] velocity loop proportional gain

    // sensors
    Analog analog;
    Encoder encoder_steer(sa::RLS_ROLIN_ENC, sa::RLS_ROLIN_ENC_INDEX_CFG);

    dacsample_t set_handlebar_reference(float reference) {
#if defined(LIMTOC_VELOCITY_MODE)
        constexpr float MAX_REF_VALUE = sa::MAX_KOLLMORGEN_VELOCITY;
#else // defined(LIMTOC_VELOCITY_MODE)
        constexpr float MAX_REF_VALUE = sa::MAX_KOLLMORGEN_TORQUE;
#endif // defined(LIMTOC_VELOCITY_MODE)
        const float saturated_reference = util::clamp(reference, -MAX_REF_VALUE, MAX_REF_VALUE);
        const dacsample_t aout = saturated_reference/MAX_REF_VALUE*sa::DAC_HALF_RANGE + sa::DAC_HALF_RANGE;
        dacPutChannelX(sa::KOLLM_DAC, 0, aout); // TODO: don't hardcode zero but find the DAC channel constant
        return aout;
    }

    // This is a simple mass-spring system so the equation of motion is:
    //   m*x_ddot = -k*x
    // and the state space formulation is
    // [x_dot ] = [  0    1][x    ] + [   0] u
    // [x_ddot]   [ -k/m  0][x_dot]   [ 1/m]
#if defined(LIMTOC_VELOCITY_MODE)
    class MassSpring final : public model::DiscreteLinear<2, 1, 1, 0> {
        public:
            MassSpring(model::real_t mass,
                       model::real_t k,
                       model::real_t dt) :
            m_mass(mass), m_k(k), m_dt(dt) {
                using discretization_matrix_t = Eigen::Matrix<model::real_t, n + m, n + m>;
                discretization_matrix_t AT = discretization_matrix_t::Zero();
                AT(0, 1) = 1.0f;
                AT(1, 0) = -m_k/m_mass;
                AT(1, 2) = 1.0f/m_mass;
                AT *= m_dt;

                discretization_matrix_t T = AT.exp();
#if !defined(NDEBUG)
                static const model::real_t discretization_precision =
#endif
                    Eigen::NumTraits<model::real_t>::dummy_precision();
                assert((T.bottomLeftCorner<m, n>().isZero(discretization_precision)) &&
                       (T.bottomRightCorner<m, m>().isIdentity(discretization_precision)));
                m_Ad = T.topLeftCorner<n, n>();
                m_Bd = T.topRightCorner<n, m>();

                m_Cd.setZero();
                m_Cd << 1, 1;
            }
            virtual state_t update_state(
                    const state_t& x,
                    const input_t& u=input_t::Zero(),
                    const measurement_t& z=measurement_t::Zero()) const override {
                (void)z;
                return m_Ad*x + m_Bd*u;
            }
            virtual output_t calculate_output(const state_t& x, const input_t& u) const override {
                (void)u;
                return m_Cd*x;
            }
            virtual const state_matrix_t& Ad() const override {
                return m_Ad;
            }
            virtual const input_matrix_t& Bd() const override {
                return m_Bd;
            }
            virtual const output_matrix_t& Cd() const override {
                return m_Cd;
            }
            virtual const feedthrough_matrix_t& Dd() const override {
                return m_Dd;
            }
            virtual model::real_t dt() const override {
                return m_dt;
            }
            virtual state_t normalize_state(const state_t& x) const override {
                return x;
            }
            virtual output_t normalize_output(const output_t& y) const override {
                return y;
            }
            model::real_t mass() const {
                return m_mass;
            }
            model::real_t k() const {
                return m_k;
            }

        private:
            model::real_t m_mass;
            model::real_t m_k;
            model::real_t m_dt;
            state_matrix_t m_A;
            state_matrix_t m_Ad;
            input_matrix_t m_Bd;
            output_matrix_t m_Cd;
            const feedthrough_matrix_t m_Dd = feedthrough_matrix_t::Zero();
    };

    class EncoderModel final : public model::DiscreteLinear<3, 0, 1, 0> {
        public:
            EncoderModel(model::real_t dt) : m_dt(dt) {
                m_Ad.setZero();
                m_Ad << 1.0f,   dt, dt*dt/2.0f,
                        0.0f, 1.0f,         dt,
                        0.0f, 0.0f,       1.0f;
                m_Cd.setZero();
                m_Cd << 1.0f, 0.0f, 0.0f;
            }
            virtual state_t update_state(
                    const state_t& x,
                    const input_t& u=input_t::Zero(),
                    const measurement_t& z=measurement_t::Zero()) const override {
                (void)u;
                (void)z;
                return m_Ad*x;
            }
            virtual output_t calculate_output(const state_t& x, const input_t& u) const override {
                (void)u;
                return m_Cd*x;
            }
            virtual const state_matrix_t& Ad() const override {
                return m_Ad;
            }
            virtual const input_matrix_t& Bd() const override {
                return m_Bd;
            }
            virtual const output_matrix_t& Cd() const override {
                return m_Cd;
            }
            virtual const feedthrough_matrix_t& Dd() const override {
                return m_Dd;
            }
            virtual model::real_t dt() const override {
                return m_dt;
            }
            virtual state_t normalize_state(const state_t& x) const override {
                return x;
            }
            virtual output_t normalize_output(const output_t& y) const override {
                return y;
            }
        private:
            model::real_t m_dt;
            state_matrix_t m_Ad;
            const input_matrix_t m_Bd = input_matrix_t::Zero();
            output_matrix_t m_Cd;
            const feedthrough_matrix_t m_Dd = feedthrough_matrix_t::Zero();
    };
    using KalmanMassSpring = observer::Kalman<MassSpring>;
    using KalmanEncoder = observer::Kalman<EncoderModel>;

    constexpr size_t window_size = 55;
    constexpr size_t velocity_polynomial_order = 3;
    using sample_vector_t = Eigen::Matrix<float, window_size, 1>;
    using velocity_polynomial_coeff_t = Eigen::Matrix<float, velocity_polynomial_order + 1, 1>;
    using vandermonde_matrix_t = Eigen::Matrix<float, window_size, velocity_polynomial_order + 1>;

    sample_vector_t velocity_samples; // buffer containing velocity samples
                                      // x[0] in most recent
                                      // x[n - 1] is least recent
    sample_vector_t inertia_torque_samples; // buffer containing reference samples
                                            // x[0] in most recent
                                            // x[n - 1] is least recent
    velocity_polynomial_coeff_t velocity_polynomial_coeffs;
    vandermonde_matrix_t polynomial_time_matrix;
    Eigen::LDLT<Eigen::Matrix<float,
        velocity_polynomial_order + 1, velocity_polynomial_order + 1>> velocity_pt_ldlt;

#else // defined(LIMTOC_VELOCITY_MODE)
    float get_torque_reference(float angle) {
        return -k*angle;
    }
#endif // defined(LIMTOC_VELOCITY_MODE)
} // namespace

/*
 * Application entry point.
 */
int main(void) {

    //
    // System initializations.
    // - HAL initialization, this also initializes the configured device drivers
    //   and performs the board-specific initializations.
    // - Kernel initialization, the main() function becomes a thread and the
    //   RTOS is active.
    halInit();
    chSysInit();

    // Initialize a serial-over-USB CDC driver.
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    // Activate the USB driver and then the USB bus pull-up on D+.
    // Note, a delay is inserted in order to not have to disconnect the cable
    // after a reset.
    board_usb_lld_disconnect_bus();   //usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    board_usb_lld_connect_bus();      //usbConnectBus(serusbcfg.usbp);

    // create the blink thread 
    chBlinkThreadCreateStatic();

    // Start sensors.
    // Encoder:
    //   Initialize encoder driver 5 on pins PA0, PA1 (EXT2-4, EXT2-8).
    //   Pins for encoder driver 3 are already set in board.h.
    palSetLineMode(LINE_TIM5_CH1, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_FLOATING);
    palSetLineMode(LINE_TIM5_CH2, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_FLOATING);
    encoder_steer.start();
    analog.start(10000); /* trigger ADC conversion at 10 kHz */

    // Start DAC1 driver and set output pin as analog as suggested in Reference Manual.
    // The default line configuration is OUTPUT_OPENDRAIN_PULLUP for SPI1_ENC1_NSS
    // and must be changed to use as analog output.
    palSetLineMode(LINE_DAC0_REF, PAL_MODE_INPUT_ANALOG);
    dacStart(sa::KOLLM_DAC, sa::KOLLM_DAC_CFG);
    set_handlebar_reference(0.0f);

    // Set torque measurement enable line low.
    // The output of the Kistler torque sensor is not valid until after a falling edge
    // on the measurement line and it is held low. The 'LINE_TORQUE_MEAS_EN' line is
    // reversed due to NPN switch Q1.
    palClearLine(LINE_TORQUE_MEAS_EN);
    chThdSleepMilliseconds(1);
    palSetLine(LINE_TORQUE_MEAS_EN);

#if defined(LIMTOC_VELOCITY_MODE)
    constexpr float q = 1000;
    constexpr float r = 0.00001 * 0.00001 * constants::as_radians * constants::as_radians;

    MassSpring model(m, k, dt);
    KalmanMassSpring kalman_mass_spring(model);
    kalman_mass_spring.set_Q((KalmanMassSpring::process_noise_covariance_t() <<
                dt*dt*dt*dt/4, dt*dt*dt/2,
                   dt*dt*dt/2,      dt*dt).finished() * q);
    kalman_mass_spring.set_R((KalmanMassSpring::measurement_noise_covariance_t() <<
                r,      0,
                0, 1000*r).finished());

    EncoderModel encoder_model(dt);
    KalmanEncoder kalman_encoder(encoder_model);
    kalman_encoder.set_Q((KalmanEncoder::process_noise_covariance_t() <<
                dt*dt*dt*dt*dt/20.0f, dt*dt*dt*dt/8.0f, dt*dt*dt/6.0f,
                    dt*dt*dt*dt/8.0f,    dt*dt*dt/3.0f,    dt*dt/2.0f,
                       dt*dt*dt/6.0f,       dt*dt/2.0f,            dt).finished() * q);
    kalman_encoder.set_R((KalmanEncoder::measurement_noise_covariance_t() <<
                r).finished());

    // set last column
    polynomial_time_matrix.rightCols<1>().setOnes();
    // set penultimate column
    polynomial_time_matrix.col(polynomial_time_matrix.cols() - 2).setLinSpaced(
            0.0f, -dt*(polynomial_time_matrix.rows() - 1));
    // set columns n-3 to 0
    for (int32_t j = polynomial_time_matrix.cols() - 3; j >= 0; --j) {
        polynomial_time_matrix.col(j) = polynomial_time_matrix.col(j + 1).cwiseProduct(
                polynomial_time_matrix.col(polynomial_time_matrix.cols() - 2));
    }

    // compute Cholesky decomposition
    velocity_pt_ldlt.compute(polynomial_time_matrix.transpose() * polynomial_time_matrix);
#endif // defined(LIMTOC_VELOCITY_MODE)

    // Normal main() thread activity
    systime_t deadline = chVTGetSystemTime();
    while (true) {
        const float kistler_torque = util::adc_to_Nm(analog.get_adc12(),
                sa::KISTLER_ADC_ZERO_OFFSET, sa::MAX_KISTLER_TORQUE);
        const float motor_torque = util::adc_to_Nm(analog.get_adc13(),
                sa::KOLLMORGEN_ADC_ZERO_OFFSET, sa::MAX_KOLLMORGEN_TORQUE);
        const float steer_angle = util::encoder_count<float>(encoder_steer);

        // generate motor command to simulate a spring
#if defined(LIMTOC_VELOCITY_MODE)
        // update our encoder state estimate
        {
            KalmanEncoder::measurement_t z = KalmanEncoder::measurement_t::Zero();
            z << steer_angle;
            kalman_encoder.update_state(KalmanEncoder::input_t::Zero(), z);
        }

        // update polynomial fit for velocity samples and calculate
        // smoothed velocity and acceleration
        // computing least squares solution of
        //   Ax = b
        // is equivalent to solving the normal equation
        //   A'Ax = A'b
        // where
        //   A: polynomial_time_matrix
        //   x: polynomial_coeffs
        //   b: velocity_samples
        velocity_samples[0] = kalman_encoder.x()[1];
        velocity_polynomial_coeffs = velocity_pt_ldlt.solve(
                polynomial_time_matrix.transpose() * velocity_samples);
        const float polyfit_velocity = velocity_polynomial_coeffs.tail<1>()[0]; // evaluated at time 0
        const float polyfit_acceleration = velocity_polynomial_coeffs.tail<2>()[0]; // evaluated at time 0
        const float torque_measurement_acceleration =
            (kistler_torque + motor_torque)/ m_l;

        // polyfit feedback reference to smooth it
        inertia_torque_samples[0] = m_u * torque_measurement_acceleration;
        velocity_polynomial_coeffs = velocity_pt_ldlt.solve(
                polynomial_time_matrix.transpose() * inertia_torque_samples);
        const float inertia_torque = velocity_polynomial_coeffs.tail<1>()[0];

        // update mass spring estimate with 'measurement' and
        // predict next velocity using steer torque
        {
            kalman_mass_spring.measurement_update(
                    (KalmanMassSpring::measurement_t() <<
                     kalman_encoder.x()[0], polyfit_velocity).finished());

            const float steer_torque = (-kistler_torque + inertia_torque);
            kalman_mass_spring.time_update(
                    (KalmanMassSpring::input_t() <<
                     steer_torque).finished());
        }

        //const float feedback_reference = kalman_mass_spring.x()[1];
        const float feedback_reference = 0;
        //const float feedback_reference = 1.0f * std::sin(
        //        constants::two_pi * ST2S(static_cast<float>(chVTGetSystemTime())));
#else // defined(LIMTOC_VELOCITY_MODE)
        const float feedback_reference = get_torque_reference(steer_angle);
#endif  // defined(LIMTOC_VELOCITY_MODE)
        set_handlebar_reference(feedback_reference);

#if defined(LIMTOC_VELOCITY_MODE)
        printf("%u, %10.5f, %10.5f, %10.5f, %10.5f, %10.5f, %10.5f\r\n",
               chSysGetRealtimeCounterX(), kistler_torque, motor_torque, feedback_reference,
               kalman_encoder.x()[0], polyfit_velocity, polyfit_acceleration);

        // FIXME copy values for now and implement a circular buffer later
        std::memmove(velocity_samples.data() + 1,
                     velocity_samples.data(),
                     sizeof(decltype(velocity_samples)::value_type) * (velocity_samples.size() - 1));
        std::memmove(inertia_torque_samples.data() + 1,
                     inertia_torque_samples.data(),
                     sizeof(decltype(inertia_torque_samples)::value_type) * (inertia_torque_samples.size() - 1));
#else // defined(LIMTOC_VELOCITY_MODE)
        printf("[%u] kistler: %8.3f Nm\tkollmorgen: %8.3f Nm\tsteer: %8.3f rad\r\n",
               chSysGetRealtimeCounterX(), kistler_torque, motor_torque, steer_angle);
#endif // defined(LIMTOC_VELOCITY_MODE)
        deadline = chThdSleepUntilWindowed(deadline, deadline + loop_period);
    }
}
