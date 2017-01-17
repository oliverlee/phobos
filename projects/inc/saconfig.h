#pragma once
#include "hal.h"
#include "encoder.h"

/* sensor and actuator configuration constants */
namespace sa {

constexpr GPTDriver* RLS_ROLIN_ENC = &GPTD5;

constexpr EncoderConfig RLS_ROLIN_ENC_CFG = {
    .z = PAL_NOLINE, /* no index channel */
    .counts_per_rev = 152000,
    .filter = EncoderConfig::filter_t::CAPTURE_64, /* 64 / 42 MHz (TIM5 on APB1) = 1.52 us for valid edge */
    .z_count = 0
};

constexpr EncoderConfig RLS_ROLIN_ENC_INDEX_CFG = {
    .z = PAL_LINE(GPIOA, GPIOA_PIN2),
    .counts_per_rev = 152000,
    .filter = EncoderConfig::filter_t::CAPTURE_64, /* 64 / 42 MHz (TIM5 on APB1) = 1.52 us for valid edge */
    .z_count = 18536
};


constexpr float REAR_WHEEL_RADIUS = 0.3f; /* m */
constexpr float ROLLER_TO_REAR_WHEEL_RATIO = 1.0f/6; /* rear wheel radius = 6 * roller radius */

constexpr GPTDriver* RLS_GTS35_ENC = &GPTD3;

constexpr EncoderConfig RLS_GTS35_ENC_CFG = {
    .z = PAL_NOLINE, /* no index channel */
    .counts_per_rev = 48 * 4 * 6,
    .filter = EncoderConfig::filter_t::CAPTURE_256, /* 256 / 42 MHz (TIM3 on APB1) = 6.09 us for valid edge */
    .z_count = 0
};

constexpr float MAX_KISTLER_TORQUE = 50.0f; /* maximum measured steer torque, N-m */

/*
 * The voltage output of the Kistler torque sensor is ±10V. With the 12-bit ADC,
 * resolution for LSB is 4.88 mV/bit or 12.2 mNm/bit.
 */
constexpr float MAX_KOLLMORGEN_TORQUE = 11.5f; /* max torque at 1.00 Arms/V, N-m */

constexpr DACDriver* KOLLM_DAC = &DACD1;

constexpr DACConfig dac1cfg1 = {
     .init       = 2047U, // max value is 4095 (12-bit)
     .datamode   = DAC_DHRM_12BIT_RIGHT
};
constexpr const DACConfig* KOLLM_DAC_CFG = &dac1cfg1;

// TODO: measure the real value
constexpr float STEER_ASSEMBLY_INERTIA = 0.1f; /* moment of inertia of steering assembly about the steer axis, kg-m^2 */

} // namespace
