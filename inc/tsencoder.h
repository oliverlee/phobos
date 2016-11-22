#pragma once
#include "ch.h"
#include "hal.h"
#include <array>
#include <Eigen/Core>

/*
 * This class implements higher-order encoder time-stamping (HOTS) as described by:
 * Merry, van de Molen, Steinbuch. Optimal higher-order encoder time-stamping.
 * In: Mechatronics, Vol. 23(2013), No. 5, p. 481-490
 * to estimate angular position, velocity, and acceleration.
 *
 * Note: The delay option has not been implemented.
 */

using tsenccnt_t = int32_t;
using polycoeff_t = float;

struct TSEncoderConfig {
    /*
     * GPIO for encoder channels A, B, Z should be set in board.h.
     * ioline_t a; IO line for encoder A channel
     * ioline_t b; IO line for encoder B channel
     * ioline_t z; IO line for encoder Z (index) channel
     *
     * The EXT driver is used as each encoder event requires the encoder count
     * and time stamp to be saved, requiring the use of an interrupt. Although
     * a timer can be set in encoder mode, as is done with the Encoder class,
     * there is no simple way to interrupt on each encoder event in order to
     * record the count and time stamp.
     */
    ioline_t a; /* IO line for encoder A channel */
    ioline_t b; /* IO line for encoder B channel */
    ioline_t z; /* IO line for encoder Z channel or PAL_NOLINE if not used */
    tsenccnt_t counts_per_rev; /* encoder counts per revolution */
};

template <size_t M, size_t N, size_t O>
class TSEncoder {
    public:
        static constexpr unsigned int m = M; /* polynomial order */
        static constexpr unsigned int n = N; /* encoder event buffer length */
        static constexpr unsigned int o = O; /* skip order */

        enum class state_t {
            STOP, READY
        };
        enum class index_t {
            NONE, NOTFOUND, FOUND
        };

        TSEncoder(const TSEncoderConfig& config);
        void start();
        void stop();
        state_t state() const;
        index_t index() const volatile;
        void update_polynomial_fit();
        void update_estimate_time(rtcnt_t tc);
        polycoeff_t position() const;
        polycoeff_t velocity() const;
        polycoeff_t acceleration() const;

    private:
        using event_t = std::pair<rtcnt_t, tsenccnt_t>;
        enum class position_result_t {
            NONE, ESTIMATED, ADJUSTED
        };

        mutable std::array<event_t, N> m_events; /* circular buffer for encoder events */
        mutable size_t m_event_index; /* index of oldest entry */
        mutable size_t m_skip_order_counter; /* skip order counter */
        Eigen::Matrix<polycoeff_t, N, M + 1> m_A; /* time stamp matrix */
        Eigen::Matrix<polycoeff_t, M + 1, 1> m_P; /* polynomial coefficients */
        Eigen::Matrix<tsenccnt_t, N, 1> m_B; /* position vector */
        Eigen::Matrix<polycoeff_t, M + 1, 1> m_T; /* time vector */
        rtcnt_t m_t0; /* polynomial zero time */
        polycoeff_t m_alpha; /* time scaling factor */
        const TSEncoderConfig m_config;
        tsenccnt_t m_count; /* encoder count */
        rtcnt_t m_tc; /* estimate time */
        state_t m_state;
        index_t m_index;
        mutable position_result_t m_position_result;
        virtual_timer_t m_event_deadline_timer;
        static constexpr systime_t m_event_deadline = MS2ST(10); /* observed event deadline */

        static void ab_callback(EXTDriver* extp, expchannel_t channel);
        static void index_callback(EXTDriver* extp, expchannel_t channel);
        static void add_event_callback(void* p);
        void add_event(rtcnt_t t, tsenccnt_t x, bool use_skip) const;
        void add_deadline_event() const;
};

#include "tsencoder.hh"
