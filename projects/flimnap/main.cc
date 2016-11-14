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
#include "packet/frame.h"

#include "gitsha1.h"
#include "angle.h"
#include "blink.h"
#include "virtualbicycle.h"
#include "usbconfig.h"
#include "saconfig.h"

#include "parameters.h"

#include <array>

namespace {
    /* sensors */
    Analog analog;
    Encoder encoder(sa::RLS_ENC, sa::RLS_ENC_INDEX_CFG);

    struct __attribute__((__packed__)) pose_t {
        float x; /* m */
        float y; /* m */
        float pitch; /* rad */
        float yaw; /* rad */
        float roll; /* rad */
        float steer; /* rad */
        float v; /* m/s      * Wheel angle and radius are not considered here.
                             * Computation must occur during visualization */
        uint8_t timestamp;  /* Converted from system ticks to milliseconds
                             * and contains only the least significant bits */
    }; /* 29 bytes */

    pose_t pose = {};

    using namespace packet::frame;
    std::array<uint8_t, sizeof(pose_t) + PACKET_OVERHEAD> packet_buffer;
} // namespace

/*
 * Application entry point.
 */
int main(void) {

    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();

    /*
     * Initialize a serial-over-USB CDC driver.
     */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     * Activate the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    board_usb_lld_disconnect_bus();   //usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    board_usb_lld_connect_bus();      //usbConnectBus(serusbcfg.usbp);

    /* create the blink thread and print state monitor */
    chBlinkThreadCreateStatic();

    /*
     * Start sensors.
     * Encoder:
     *   Initialize encoder driver 5 on pins PA0, PA1 (EXT2-4, EXT2-8).
     */
    palSetLineMode(LINE_TIM5_CH1, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_FLOATING);
    palSetLineMode(LINE_TIM5_CH2, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_FLOATING);
    encoder.start();
    analog.start(1000); /* trigger ADC conversion at 1 kHz */

    /*
     * Set torque measurement enable line low.
     * The output of the Kistler torque sensor is not valid until after a falling edge
     * on the measurement line and it is held low. The 'LINE_TORQUE_MEAS_EN' line is
     * reversed due to NPN switch Q1.
     */
    palClearLine(LINE_TORQUE_MEAS_EN);
    chThdSleepMilliseconds(1);
    palSetLine(LINE_TORQUE_MEAS_EN);

    /*
     * Start DAC1 driver and set output pin as analog as suggested in Reference Manual.
     * The default line configuration is OUTPUT_OPENDRAIN_PULLUP  for SPI1_ENC1_NSS
     * and must be changed to use as analog output.
     */
    palSetLineMode(LINE_KOLLM_ACTL_TORQUE, PAL_MODE_INPUT_ANALOG);
    dacStart(sa::KOLLM_DAC, sa::KOLLM_DAC_CFG);

    /*
     * Initialize bicycle with default parameters, dt = 0.005 s, v = 5 m/s.
     */
    VirtualBicycle bicycle;

    /* transmit git sha information, block until receiver is ready */
    uint8_t bytes_written = packet::frame::stuff(g_GITSHA1, packet_buffer.data(), 7);
    obqWriteTimeout(&SDU1.obqueue, packet_buffer.data(), bytes_written, TIME_INFINITE);

    /*
     * Normal main() thread activity, in this demo it simulates the bicycle
     * dynamics in real-time (roughly).
     */
    while (true) {
        systime_t starttime = chVTGetSystemTime();

        constexpr float roll_torque = 0.0f;

        /* get sensor measurements */
        float steer_torque = static_cast<float>(analog.get_adc12()*2.0f*sa::MAX_KISTLER_TORQUE/4096 -
                sa::MAX_KISTLER_TORQUE);
        float motor_torque = static_cast<float>(
                analog.get_adc13()*2.0f*sa::MAX_KOLLMORGEN_TORQUE/4096 -
                sa::MAX_KOLLMORGEN_TORQUE);
        float steer_angle = angle::encoder_count<float>(encoder);

        (void)motor_torque; /* remove build warning for unused variable */

        /* yaw angle, just use previous state value */
        float yaw_angle = angle::wrap(bicycle.pose().yaw);

        /* set steer torque/angle for a simple simulation */
        steer_angle = angle::wrap(bicycle.pose().steer);
        steer_torque = 0.0f;

        /* simulate bicycle */
        bicycle.update(roll_torque, steer_torque, yaw_angle, steer_angle);

        /* generate an example torque output for testing */
        float feedback_torque = 10.0f * std::sin(
                constants::two_pi * ST2S(static_cast<float>(starttime)));
        dacsample_t aout = static_cast<dacsample_t>(
                (feedback_torque/21.0f * 2048) + 2048); /* reduce output to half of full range */
        dacPutChannelX(sa::KOLLM_DAC, 0, aout);

        pose = pose_t{}; /* reset pose to zero */
        pose.x = bicycle.pose().x;
        pose.y = bicycle.pose().y;
        pose.pitch = bicycle.pose().pitch;
        pose.yaw = bicycle.pose().yaw;
        pose.roll = bicycle.pose().roll;
        pose.steer = bicycle.pose().steer;
        pose.v = bicycle.model().v();
        pose.timestamp = static_cast<decltype(pose.timestamp)>(ST2MS(chVTGetSystemTime()));
        /* TODO: examine difference between chVTGetSystemTime() calls */

        bytes_written = packet::frame::stuff(&pose, packet_buffer.data(), sizeof(pose));
        obqWriteTimeout(&SDU1.obqueue, packet_buffer.data(), bytes_written, TIME_INFINITE);

        systime_t dt = MS2ST(static_cast<systime_t>(1000*bicycle.model().dt()));
        systime_t sleeptime = dt + starttime - chVTGetSystemTime();
        if (sleeptime >= dt) {
            chDbgAssert(false, "deadline missed");
            continue;
        }
        chThdSleep(sleeptime);
    }
}
