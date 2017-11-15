#pragma once
#include "discrete_linear.h"
#include "kalman.h"

namespace imp {
    class SteerAssemblyModel final : public model::DiscreteLinear<2, 1, 2, 0> {
        public:
            SteerAssemblyModel(model::real_t inertia, model::real_t dt) :
                m_m(inertia),
                m_dt(dt),
                m_Ad((state_matrix_t() << 1, dt,
                                          0,  1).finished()),
                m_Bd((input_matrix_t() << dt*dt/inertia/2,
                                          dt/inertia).finished()) { }
            virtual state_t update_state(const state_t& x,
                                         const input_t& u,
                                         const measurement_t& z) const override {
                (void)z;
                return m_Ad*x + m_Bd*u;
            }
            virtual output_t calculate_output(const state_t& x,
                                              const input_t& u) const override {
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
            const model::real_t m_m;
            const model::real_t m_dt;
            const state_matrix_t m_Ad;
            const input_matrix_t m_Bd;
            const output_matrix_t m_Cd = output_matrix_t::Identity();
            const feedthrough_matrix_t m_Dd = feedthrough_matrix_t::Zero();
    };
} // namespace imp

namespace sim {
    class SteerAssembly {
        public:
            using kalman_t = observer::Kalman<imp::SteerAssemblyModel>;

            // gyro sensor cannot measure angular rates above 1.7 rad/s
            // and signal will saturate
            static constexpr model::real_t
                steer_rate_sensor_limit = 1.7 * 0.95;
            // measured steer angle and rate variance
            // steer angle measurement noise variance is reduced due to
            // a 0.03 sec time delay in steer rate measurement
            static constexpr model::real_t
                steer_angle_measurement_noise_variance = 2.03049e-6 / 10;
            static constexpr model::real_t
                steer_rate_measurement_noise_variance = 0.02604;

            SteerAssembly(model::real_t inertia,
                          model::real_t dt,
                          model::real_t q = 200);
            const imp::SteerAssemblyModel& model() const;
            kalman_t& observer();
            const kalman_t& observer() const;
            void update_state_estimate(
                    model::real_t torque_input,
                    model::real_t steer_angle_measurement,
                    model::real_t steer_rate_measurement);
            model::real_t velocity() const;

        private:
            imp::SteerAssemblyModel m_model;
            kalman_t m_kalman;
    };
} // namespace sim
