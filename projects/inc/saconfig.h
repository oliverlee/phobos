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
constexpr float MAX_KOLLMORGEN_TORQUE = 10.78f; /* max torque at 1.00 Arms/V, N-m */
constexpr dacsample_t KOLLMORGEN_DAC_ZERO_OFFSET = 2048 - 125; /* DAC value for zero torque, found experimentally */

constexpr DACDriver* KOLLM_DAC = &DACD1;

constexpr DACConfig dac1cfg1 = {
     .init       = 2047U, // max value is 4095 (12-bit)
     .datamode   = DAC_DHRM_12BIT_RIGHT
};
constexpr const DACConfig* KOLLM_DAC_CFG = &dac1cfg1;

/* moment of inertia of steering assembly about the steer axis, kg-m^2 */
constexpr float STEER_ASSEMBLY_INERTIA_WITH_WEIGHT = 0.1942f; /* with 1 kg weight plate on each side of cross bar*/
constexpr float STEER_ASSEMBLY_INERTIA_WITHOUT_WEIGHT = 0.0828f; /* without weight plates */
constexpr float STEER_ASSEMBLY_INERTIA = STEER_ASSEMBLY_INERTIA_WITH_WEIGHT; /* default configuration */

/* moment of inertia of the handlebars and steering column above the torque sensor, about the steer axis 
 * determined from scripts/calculate_handlebar_inertia.py */
constexpr float HANDLEBAR_INERTIA = 0.0413; /* kg-m^2 */

} // namespace
