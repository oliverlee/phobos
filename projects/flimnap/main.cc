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
#include "encoderfoaw.h"
#include "packet/frame.h"
#include "filter/movingaverage.h"

#include "gitsha1.h"
#include "angle.h"
#include "blink.h"
#include "usbconfig.h"
#include "saconfig.h"

#include "parameters.h"

#include <algorithm>
#include <array>

#include "bicycle/whipple.h" /* whipple bicycle model */
#include "oracle.h" /* oracle observer */
#include "haptic.h" /* handlebar feedback */
#include "simbicycle.h"

namespace {
    using bicycle_t = sim::Bicycle<model::BicycleWhipple,
                                   observer::Oracle<model::BicycleWhipple>,
                                   haptic::HandlebarStatic>;

    /* sensors */
    Analog analog;
    Encoder encoder_steer(sa::RLS_ROLIN_ENC, sa::RLS_ROLIN_ENC_INDEX_CFG);
    EncoderFoaw<float, 32> encoder_rear_wheel(sa::RLS_GTS35_ENC,
                                              sa::RLS_GTS35_ENC_CFG,
                                              MS2ST(1), 3.0f);
    filter::MovingAverage<float, 5> velocity_filter;

    /* transmission */
    struct __attribute__((__packed__)) pose_t {
        float x; /* m */
        float y; /* m */
        float pitch; /* rad */
        float yaw; /* rad */
        float roll; /* rad */
        float steer; /* rad */
        float rear_wheel; /* rad */
        float v; /* m/s */
        uint16_t computation_time; /* time to acquire sensor data,
                                    * update bicycle model,
                                    * and command actuators */
        uint16_t timestamp;  /* system ticks
                              * (normally 10 kHz, see CH_CFG_ST_FREQUENCY) */
    }; /* 36 bytes */
    pose_t pose = {};
    std::array<uint8_t, sizeof(pose_t) + packet::frame::PACKET_OVERHEAD> packet_buffer;

    /* kinematics loop */
    constexpr systime_t kinematics_loop_time = US2ST(8333); /* update kinematics at 120 Hz */

    /* dynamics loop */
    constexpr model::real_t dynamics_period = 0.005; /* 5 ms -> 200 Hz */
    time_measurement_t dynamics_time_measurement;

    void set_handlebar_torque(float handlebar_torque) {
        int32_t saturated_value = (handlebar_torque/sa::MAX_KOLLMORGEN_TORQUE * 2048) + 2048;
        saturated_value = std::min<int32_t>(std::max<int32_t>(saturated_value, 0), 4096);
        dacsample_t aout = static_cast<dacsample_t>(saturated_value);
        dacPutChannelX(sa::KOLLM_DAC, 0, aout);
    }

    void set_pose(const bicycle_t& bicycle, uint32_t computation_time_measurement) {
        pose = pose_t{}; /* reset pose to zero */

        pose.timestamp = bicycle.pose().timestamp;
        pose.x = bicycle.pose().x;
        pose.y = bicycle.pose().y;
        pose.rear_wheel = angle::wrap(bicycle.pose().rear_wheel);
        pose.pitch = angle::wrap(bicycle.pose().pitch);
        pose.yaw = angle::wrap(bicycle.pose().yaw);
        pose.roll = angle::wrap(bicycle.pose().roll);
        pose.steer = angle::wrap(bicycle.pose().steer);
        pose.v = bicycle.v();
        pose.computation_time = static_cast<uint16_t>(RTC2US(STM32_SYSCLK,
                    computation_time_measurement));
    }

    void transmit_gitsha1() {
        size_t bytes_written = packet::frame::stuff(g_GITSHA1, packet_buffer.data(), 7);
        while ((SDU1.config->usbp->state != USB_ACTIVE) || (SDU1.state != SDU_READY)) {
            chThdSleepMilliseconds(10);
        }
        usbTransmit(SDU1.config->usbp, SDU1.config->bulk_in, packet_buffer.data(), bytes_written);
    }

    void update_and_transmit_kinematics(bicycle_t& bicycle) {
        bicycle.update_kinematics();
        /* time measurement value is cast down to uint16_t as we don't need all 32 bits */
        set_pose(bicycle, dynamics_time_measurement.last);

        /*
         * TODO: Pose message should be transmitted asynchronously.
         * After starting transmission, the simulation should start to calculate pose for the next timestep.
         * This is currently not necessary given the observed timings.
         */
        size_t bytes_written = packet::frame::stuff(&pose, packet_buffer.data(), sizeof(pose));
        if ((SDU1.config->usbp->state == USB_ACTIVE) && (SDU1.state == SDU_READY)) {
            usbTransmit(SDU1.config->usbp, SDU1.config->bulk_in, packet_buffer.data(), bytes_written);
        }
    }

    THD_WORKING_AREA(wa_kinematics_thread, 2048);
    THD_FUNCTION(kinematics_thread, arg) {
        bicycle_t& bicycle = *static_cast<bicycle_t*>(arg);

        chRegSetThreadName("kinematics");
        while (true) {
            systime_t starttime = chVTGetSystemTime();
            update_and_transmit_kinematics(bicycle);
            systime_t sleeptime = kinematics_loop_time + starttime - chVTGetSystemTime();
            if (sleeptime >= kinematics_loop_time) {
                continue;
            } else {
                chThdSleep(sleeptime);
            }
        }
    }
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
     *   Pins for encoder driver 3 are already set in board.h.
     */
    palSetLineMode(LINE_TIM5_CH1, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_FLOATING);
    palSetLineMode(LINE_TIM5_CH2, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_FLOATING);
    encoder_steer.start();
    encoder_rear_wheel.start();
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
     * Initialize bicycle. Velocity doesn't matter as we immediately use the measured value.
     * TODO: change observer to Kalman
     */
    bicycle_t bicycle(0.0, dynamics_period);

    /* transmit git sha information, block until receiver is ready */
    transmit_gitsha1();

    /*
     * Normal main() thread activity, in this demo it simulates the bicycle
     * dynamics in real-time (roughly).
     */
    chTMObjectInit(&dynamics_time_measurement);
    chThdCreateStatic(wa_kinematics_thread, sizeof(wa_kinematics_thread), NORMALPRIO - 1,
            kinematics_thread, static_cast<void*>(&bicycle));
    while (true) {
        systime_t starttime = chVTGetSystemTime();
        chTMStartMeasurementX(&dynamics_time_measurement);
        constexpr float roll_torque = 0.0f;

        /* get sensor measurements */
        const float steer_torque = static_cast<float>(analog.get_adc12()*2.0f*sa::MAX_KISTLER_TORQUE/4096 -
                sa::MAX_KISTLER_TORQUE);
        const float motor_torque = static_cast<float>(
                analog.get_adc13()*2.0f*sa::MAX_KOLLMORGEN_TORQUE/4096 -
                sa::MAX_KOLLMORGEN_TORQUE);
        const float steer_angle = angle::encoder_count<float>(encoder_steer);
        const float rear_wheel_angle = -angle::encoder_count<float>(encoder_rear_wheel);
        const float v = velocity_filter.output(
                -sa::REAR_WHEEL_RADIUS*(angle::encoder_rate(encoder_rear_wheel)));
        (void)motor_torque; /* remove build warning */

        /* yaw angle, just use previous state value */
        const float yaw_angle = angle::wrap(bicycle.pose().yaw);

        /* simulate bicycle */
        bicycle.set_v(v);
        bicycle.update_dynamics(roll_torque, steer_torque, yaw_angle, steer_angle, rear_wheel_angle);

        /* generate handlebar torque output */
        set_handlebar_torque(bicycle.handlebar_feedback_torque());
        chTMStopMeasurementX(&dynamics_time_measurement);

        const systime_t looptime = MS2ST(static_cast<systime_t>(1000*bicycle.dt()));
        const systime_t sleeptime = looptime + starttime - chVTGetSystemTime();
        if (sleeptime >= looptime) {
            //chDbgAssert(false, "deadline missed");
            continue;
        } else {
            chThdSleep(sleeptime);
        }
    }
}
