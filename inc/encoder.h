#pragma once
#include "hal.h"

using enccnt_t = gptcnt_t;

struct EncoderConfig {
    enum class filter_t : uint8_t {
        CAPTURE_1 = 0, CAPTURE_2, CAPTURE_4, CAPTURE_8,
        CAPTURE_12, CAPTURE_16, CAPTURE_24, CAPTURE_32,
        CAPTURE_48, CAPTURE_64, CAPTURE_80, CAPTURE_96,
        CAPTURE_128, CAPTURE_160, CAPTURE_192, CAPTURE_256
    };
    /*
     * GPIO for encoder channels A, B should be set in board.h.
     * ioline_t a; IO line for encoder A channel
     * ioline_t b; IO line for encoder B channel
     */
    ioline_t z; /* IO line for encoder index (Z) channel or PAL_NOLINE if not used */
    enccnt_t count; /* encoder counts per revolution */
    filter_t filter; /* minimum consecutive clock cycles for a valid transition */
};

class Encoder {
    public:
        enum class state_t {
            STOP, READY
        };

        enum class index_t {
            NONE, NOTFOUND, FOUND
        };

        Encoder(GPTDriver* gptp, const EncoderConfig& config);
        /* TODO: configure reset on external interrupt from index channel */
        void start();
        void stop();
        void set_count(enccnt_t count);
        enccnt_t count() volatile;
        bool direction() volatile;
        const EncoderConfig& config();
        state_t state();
        index_t index();

    private:
        GPTDriver* m_gptp;
        const GPTConfig m_gptconfig;
        const EncoderConfig m_config;
        state_t m_state;
        index_t m_index;
};
