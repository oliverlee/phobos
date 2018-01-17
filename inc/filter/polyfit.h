#pragma once
#include "debug.h"
#include <Eigen/Core>
#include <Eigen/QR>

template <size_t N, size_t K>
class Polyfit {
    static_assert(K > 0, "Polynomial order must greater than 0.");
    static_assert(N >= K - 1,
            "Sample size is too small for polynomial order.");
    public:
        static constexpr size_t buffer_length = N;
        static constexpr size_t polynomial_order = K;

        Polyfit(float sample_time);
        void update(float sample);
        Eigen::Matrix<float, K + 1, 1> polynomial() const;
        float polynomial_coefficient(size_t order) const;

    private:
        float m_buffer[N];
        Eigen::Map<Eigen::Matrix<float, N, 1>> m_y;
        Eigen::Matrix<float, N, K + 1> m_A; // memory is used by decomposition
        Eigen::ColPivHouseholderQR<Eigen::Ref<Eigen::Matrix<float, N, K + 1>>> m_A_qr;
};

template <size_t N, size_t K>
Polyfit<N, K>::Polyfit(float sample_time) :
    m_y(m_buffer),
    m_A_qr(m_A)
{
    debug_check(sample_time > 0.0f);

    m_y.setZero();

    // set A to
    // [0^K            0^(K-1)          ... 0        1]
    // [dt^K           dt^(K-1)         ... dt       1]
    // [...                                        ...]
    // [((N-1)*dt)^(K) ((N-1)*dt)^(K-1) ... (N-1)*dt 1]
    m_A.template rightCols<1>().setOnes();
    for (unsigned int i = 0; i < N; ++i) {
        m_A(i, K) = -static_cast<float>(i)*sample_time;
    }
    for (unsigned int i = 0; i < K - 1; ++i) {
        m_A.col(K - 2 - i) = m_A.col(K - 1 - i).cwiseProduct(m_A.col(K - 1));
    }

    m_A_qr.compute(m_A);
}

template <size_t N, size_t K>
void Polyfit<N, K>::update(float sample) {
    // shift buffer samples
    std::memcpy(m_buffer, &m_buffer[1],
            sizeof(m_buffer) - sizeof(&m_buffer[0]));
    // save most recent sample
    m_buffer[0] = sample;
}

template <size_t N, size_t K>
Eigen::Matrix<float, K + 1, 1> Polyfit<N, K>::polynomial() const {
    return m_A_qr.solve(m_y);
}

template <size_t N, size_t K>
float Polyfit<N, K>::polynomial_coefficient(size_t order) const {
    debug_check(order <= K);

    return polynomial()(K - order);
}
