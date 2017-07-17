#pragma once
#include "hal.h"
#include "encoder.h"

/* sensor and actuator configuration constants */
namespace sa {

constexpr adcsample_t ADC_HALF_RANGE = (1 << 12)/2; // ADC is 12-bit
constexpr dacsample_t DAC_HALF_RANGE = (1 << 12)/2; // DAC is 12-bit

constexpr GPTDriver* RLS_ROLIN_ENC = &GPTD5;

constexpr EncoderConfig RLS_ROLIN_ENC_CFG = {
    .z = PAL_NOLINE, // no index channel
    .counts_per_rev = 152000,
    .filter = EncoderConfig::filter_t::CAPTURE_64, // 64 / 42 MHz (TIM5 on APB1) = 1.52 us for valid edge
    .z_count = 0
};

constexpr EncoderConfig RLS_ROLIN_ENC_INDEX_CFG = {
    .z = PAL_LINE(GPIOA, GPIOA_PIN2),
    .counts_per_rev = 152000,
    .filter = EncoderConfig::filter_t::CAPTURE_64, // 64 / 42 MHz (TIM5 on APB1) = 1.52 us for valid edge
    .z_count = 18407
};


constexpr float REAR_WHEEL_RADIUS = 0.3f; // m
constexpr float ROLLER_TO_REAR_WHEEL_RATIO = 1.0f/6; // rear wheel radius = 6 * roller radius

constexpr GPTDriver* RLS_GTS35_ENC = &GPTD3;

constexpr EncoderConfig RLS_GTS35_ENC_CFG = {
    .z = PAL_NOLINE, // no index channel
    .counts_per_rev = 48 * 4 * 6,
    .filter = EncoderConfig::filter_t::CAPTURE_256, // 256 / 42 MHz (TIM3 on APB1) = 6.09 us for valid edge
    .z_count = 0
};

/*
 * The voltage output of the Kistler torque sensor is ±10V. With the 12-bit ADC,
 * resolution for LSB is 4.88 mV/bit or 12.2 mNm/bit.
 */
constexpr float MAX_KISTLER_TORQUE = 50.0f; // maximum measured steer torque, N-m
constexpr adcsample_t KISTLER_ADC_ZERO_OFFSET = 2043; // ADC value for zero torque, found experimentally

constexpr float MAX_KOLLMORGEN_VELOCITY = 3*1.74533f; // max velocity of 300 deg/s in rad/s
constexpr float MAX_KOLLMORGEN_TORQUE = 10.78125f; // max torque at 1.50 Arms/V, N-m
constexpr adcsample_t KOLLMORGEN_ADC_ZERO_OFFSET = 2026; // ADC value for zero torque, found experimentally

constexpr DACDriver* KOLLM_DAC = &DACD1;

constexpr DACConfig dac1cfg1 = {
     .init       = DAC_HALF_RANGE,
     .datamode   = DAC_DHRM_12BIT_RIGHT
};
constexpr const DACConfig* KOLLM_DAC_CFG = &dac1cfg1;

/*
 * Moment of inertia of full steering assembly about the steer axis, kg-m^2
 */
constexpr float FULL_ASSEMBLY_INERTIA_WITH_WEIGHT = 0.1942f; // with 1 kg weight plate on each side of cross bar
constexpr float FULL_ASSEMBLY_INERTIA_WITHOUT_WEIGHT = 0.0828f; // without weight plates
constexpr float FULL_ASSEMBLY_INERTIA = FULL_ASSEMBLY_INERTIA_WITH_WEIGHT; // default configuration

/*
 * For the moment of inertia of the upper assembly, we use both virtual and physical values.
 * See ICSC2017 abstract for details.
 * The virtual term is determined from scripts/calculate_handlebar_inertia.py
 * The physical term is determined from an experiment measuring the oscillation period.
 */
constexpr float UPPER_ASSEMBLY_INERTIA_VIRTUAL = 0.1314; // kg-m^2
constexpr float UPPER_ASSEMBLY_INERTIA_PHYSICAL = 0.0413; // kg-m^2
} // namespace
