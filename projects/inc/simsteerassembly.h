#pragma once
#include "discrete_linear.h"
#include "kalman.h"

namespace imp {
    // state [position, velocity]
    // input [torque]
    // output [position]
    class SteerAssemblyModel final : public model::DiscreteLinear<2, 1, 1, 0> {
        public:
            SteerAssemblyModel(model::real_t dt, model::real_t inertia, model::real_t damping = 0) :
                m_dt(dt),
                m_m(inertia),
                m_b(damping),
                m_Ad((state_matrix_t() << // FIXME
                            1, dt,
                            0, 1).finished()),
                m_Bd((input_matrix_t() << // FIXME
                            dt*dt/inertia/2,
                            dt/inertia).finished()) { }
            virtual state_t update_state(const state_t& x,
                                         const input_t& u,
                                         const measurement_t7 z) const override {
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

            model::real_t inertia() const {
                return m_m;
            }
            model::real_t damping() const {
                return m_b;
            }

        private:
            const model::real_t m_dt;
            const model::real_t m_m;
            const model::real_t m_b;
            const state_matrix_t m_Ad;
            const input_matrix_t m_Bd;
            const output_matrix_t m_Cd = output_matrix_t::Identity();
            const feedthrough_matrix_t m_Dd = feedthrough_matrix_t::Zero();
    };

namespace sim {
    class SteerAssembly {
        public:
            using kalman_t = observer::Kalman<imp::SteerAssemnlyModel>;

            static constexpr model::real_t
                steer_angle_measurement_noise_variance = 2.03049e-6;

            SteerAssembly(model::real_t dt,
                          model::real_t inertia,
                          model::real_t damping,
                          model::real_t q);
            const imp::SteerAssemblyModel& model() const;
            kalman_t& observer();
            const kalman_t& observer() const;
            void update_state_estimate(
                    model::real_t torque_input,
                    model::real_t steer_angle_measurement);
            model::real_t velocity() const;

        private:
            imp::SteerAssemblyModel m_model;
            kalman_t m_kalman;
    };
}
} // namespace imp
