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
#include "packet/serialize.h"
#include "pose.pb.h"
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
#include "kalman.h" /* kalman filter observer */
#include "haptic.h" /* handlebar feedback */
#include "simbicycle.h"

namespace {
    using bicycle_t = sim::Bicycle<model::BicycleWhipple,
                                   observer::Kalman<model::BicycleWhipple>,
                                   haptic::HandlebarDynamic>;

    /* sensors */
    Analog analog;
    Encoder encoder_steer(sa::RLS_ROLIN_ENC, sa::RLS_ROLIN_ENC_INDEX_CFG);
    EncoderFoaw<float, 32> encoder_rear_wheel(sa::RLS_GTS35_ENC,
                                              sa::RLS_GTS35_ENC_CFG,
                                              MS2ST(1), 3.0f);
    filter::MovingAverage<float, 5> velocity_filter;

    /* transmission */
    std::array<uint8_t, BicyclePoseMessage_size + packet::frame::PACKET_OVERHEAD> encode_buffer;
    std::array<uint8_t, BicyclePoseMessage_size + packet::frame::PACKET_OVERHEAD> packet_buffer;

    /* kinematics loop */
    constexpr systime_t kinematics_loop_time = US2ST(8333); /* update kinematics at 120 Hz */

    /* dynamics loop */
    constexpr model::real_t dynamics_period = 0.005; /* 5 ms -> 200 Hz */
    time_measurement_t dynamics_time_measurement;

    void set_handlebar_torque(float handlebar_torque) {
        float limit_torque = handlebar_torque;
        if (std::abs(limit_torque) > 3.0f) {
            limit_torque = std::copysign(3.0f, limit_torque);
        }

        int32_t saturated_value = (limit_torque/sa::MAX_KOLLMORGEN_TORQUE * 2048) + 2048;
        saturated_value = std::min<int32_t>(std::max<int32_t>(saturated_value, 0), 4096);
        dacsample_t aout = static_cast<dacsample_t>(saturated_value);
        dacPutChannelX(sa::KOLLM_DAC, 0, aout);
    }

    void transmit_gitsha1() {
        size_t bytes_written = packet::frame::stuff(g_GITSHA1, encode_buffer.data(), 7);
        while ((SDU1.config->usbp->state != USB_ACTIVE) || (SDU1.state != SDU_READY)) {
            chThdSleepMilliseconds(10);
        }
        usbTransmit(SDU1.config->usbp, SDU1.config->bulk_in, encode_buffer.data(), bytes_written);
    }

    void update_and_transmit_kinematics(bicycle_t& bicycle) {
        bicycle.update_kinematics();
        /*
         * TODO: Pose message should be transmitted asynchronously.
         * After starting transmission, the simulation should start to calculate pose for the next timestep.
         * This is currently not necessary given the observed timings.
         */
        size_t bytes_encoded = packet::serialize::encode_delimited(bicycle.pose(),
                encode_buffer.data(), encode_buffer.size());
        size_t bytes_written = packet::frame::stuff(
                encode_buffer.data(), packet_buffer.data(), bytes_encoded);
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
     */
    bicycle_t bicycle(0.0, dynamics_period);

    /*
     * set Kalman filter matrices
     * We start with steer angle equal to the measurement and all other state elements at zero.
     */
    {
        using model_t = bicycle_t::model_t;
        model_t::state_t x0 = model_t::state_t::Zero();
        model_t::set_state_element(x0, model_t::state_index_t::steer_angle,
                angle::encoder_count<float>(encoder_steer));
        bicycle.observer().set_x(x0);
        bicycle.observer().set_Q(parameters::defaultvalue::kalman::Q(dynamics_period));
        bicycle.observer().set_R(parameters::defaultvalue::kalman::R);
    }

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
        const float yaw_angle = bicycle.pose().yaw;

        /* simulate bicycle */
        if (v > 1.0f) {
            bicycle.set_v(v);
        } else {
            bicycle.set_v(5.0f);
        }
        bicycle.update_dynamics(roll_torque, 0.0f, yaw_angle, steer_angle, rear_wheel_angle);

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
