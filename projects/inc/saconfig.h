#pragma once
#include "hal.h"
#include "encoder.h"

/* sensor and actuator configuration constants */
namespace sa {

constexpr GPTDriver* RLS_ENC = &GPTD5;

constexpr EncoderConfig RLS_ENC_CFG = {
    .z = PAL_NOLINE, /* no index channel */
    .counts_per_rev = 152000,
    .filter = EncoderConfig::filter_t::CAPTURE_64 /* 64 * 42 MHz (TIM3 on APB1) = 1.52 us for valid edge */
};

constexpr EncoderConfig RLS_ENC_INDEX_CFG = {
    .z = PAL_LINE(GPIOA, GPIOA_PIN2),
    .counts_per_rev = 152000,
    .filter = EncoderConfig::filter_t::CAPTURE_64 /* 64 * 42 MHz (TIM3 on APB1) = 1.52 us for valid edge */
};

constexpr float MAX_KISTLER_TORQUE = 50.0f; /* maximum measured steer torque */

/*
 * The voltage output of the Kistler torque sensor is ±10V. With the 12-bit ADC,
 * resolution for LSB is 4.88 mV/bit or 12.2 mNm/bit.
 */
constexpr float MAX_KOLLMORGEN_TORQUE = 11.5f; /* max torque at 1.00 Arms/V */

constexpr DACDriver* KOLLM_DAC = &DACD1;

constexpr DACConfig dac1cfg1 = {
     .init       = 2047U, // max value is 4095 (12-bit)
     .datamode   = DAC_DHRM_12BIT_RIGHT
};
constexpr const DACConfig* KOLLM_DAC_CFG = &dac1cfg1;

} // namespace
