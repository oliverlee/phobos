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
#include "sautility.h"
#include "printf.h"

#include "parameters.h"

#include <cstdarg>

namespace {
    constexpr systime_t dt = MS2ST(1); // milliseconds converted to system ticks
    // actual EOM
    //             m_l*x_dd + b_l*x_d = -T_s + T_a
    // desired EOM
    //  m*_l*x_dd + b*_l*x_d + k*_l*x = -T_s
    //
    // T_s negated due to sign convention
    // upper assembly damping is neglected as inertia dominates
    // m*_l = m* - m_u
    // b*_l = b*
    // k*_l = k*
    constexpr float m_upper = 0.03294f; // kg-m^2, experimentally determined least squares estimate
    constexpr float m_lower = 0.002202f; // kg-m^2, experimentally determined least squares estimate
    constexpr float b_lower = 0.09902f; // N-m/(m/s), experimentally determined least squares estimate

    constexpr float k_star = 0.0f; // N-m/rad, desired spring stiffness
    constexpr float b_star = 0.0f; // N-m/(m/s), desired damping coefficient
    constexpr float m_star = 0.07; // kg-m^2, desired full assembly virtual inertia
    constexpr float m_star_lower = m_star - m_upper; // kg-m^2, desired lower assembly virtual inertia

    // sensors
    Analog analog;
    Encoder encoder_steer(sa::RLS_ROLIN_ENC, sa::RLS_ROLIN_ENC_INDEX_CFG);

    int printfq(const char *fmt, ...) {
        va_list ap;
        int formatted_bytes;
        static char serial_str[128];

        va_start(ap, fmt);
        formatted_bytes = chvsnprintf(serial_str, sizeof(serial_str)/sizeof(serial_str[0]), fmt, ap);
        va_end(ap);

        return usbTransmit(SDU1.config->usbp, SDU1.config->bulk_in, (const uint8_t*)serial_str, formatted_bytes);
    }
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
    palSetLineMode(LINE_KOLLM_ACTL_TORQUE, PAL_MODE_INPUT_ANALOG);
    dacStart(sa::KOLLM_DAC, sa::KOLLM_DAC_CFG);
    sa::set_kollmorgen_torque(0.0f);

    // Set torque measurement enable line low.
    // The output of the Kistler torque sensor is not valid until after a falling edge
    // on the measurement line and it is held low. The 'LINE_TORQUE_MEAS_EN' line is
    // reversed due to NPN switch Q1.
    palClearLine(LINE_TORQUE_MEAS_EN);
    chThdSleepMilliseconds(1);
    palSetLine(LINE_TORQUE_MEAS_EN);

    // Block USB device until ready
    while (!((SDU1.config->usbp->state == USB_ACTIVE) && (SDU1.state == SDU_READY))) {
        chThdSleepMilliseconds(10);
    }

    printfq("running limtoc %.7s, m* = %8.3f, b* = %8.3f, k* = %8.3f\r\n",
            g_GITSHA1, m_star, b_star, k_star);
    printfq("fields are:\n\r");
    printfq("realtime_counter, kistler_torque, motor_torque, steer_angle, feedback_torque\r\n");
    systime_t deadline = chVTGetSystemTime();

    // Normal main() thread activity
    while (true) {
        const float kistler_torque = sa::get_kistler_sensor_torque(analog.get_adc12());
        const float motor_torque = sa::get_kollmorgen_motor_torque(analog.get_adc13());
        const float steer_angle = util::encoder_count<float>(encoder_steer);
        const float steer_velocity = 0; // TODO: implement velocity estimation

        // calculate actuator torque for virtual mass/damping/spring
        constexpr float m = m_lower/m_star_lower - 1;
        constexpr float b = -b_star*m_lower/m_star_lower + b_lower;
        constexpr float k = -k_star*m_lower/m_star_lower;
        const float torque_actuator = m*-kistler_torque + b*steer_velocity + k*steer_angle;
        sa::set_kollmorgen_torque(torque_actuator);

        printfq("%u, %10.5f, %10.5f, %10.5f, %10.5f\r\n",
               chSysGetRealtimeCounterX(),
               kistler_torque,      // measurement
               motor_torque,        // measurement
               steer_angle,         // measurement
               torque_actuator);    // command
        deadline = chThdSleepUntilWindowed(deadline, deadline + dt);
    }
}
