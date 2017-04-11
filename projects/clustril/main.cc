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

#include "gitsha1.h"
#include "filesystem.h"
#include "saconfig.h"
#include "utility.h"

#include "packet/serialize.h"
#include "simulation.pb.h"
#include "messageutil.h"

#include "parameters.h"

#include <array>

#include "bicycle/whipple.h" /* whipple bicycle model */
#include "kalman.h" /* kalman observer */
#include "haptic.h" /* handlebar feedback */
#include "simbicycle.h"

namespace {
    using model_t = model::BicycleWhipple;
    using observer_t = observer::Kalman<model_t>;
    using haptic_t = haptic::HandlebarStatic;
    using bicycle_t = sim::Bicycle<model_t, observer_t, haptic_t>;

    /* sensors */
    Analog analog;
    Encoder encoder_steer(sa::RLS_ROLIN_ENC, sa::RLS_ROLIN_ENC_INDEX_CFG);
    EncoderFoaw<float, 32> encoder_rear_wheel(sa::RLS_GTS35_ENC, sa::RLS_GTS35_ENC_CFG, MS2ST(1), 3.0f);

    static constexpr SimulationMessage simulation_message_zero = SimulationMessage_init_zero;
    SimulationMessage sample;

    std::array<uint8_t, SimulationMessage_size> encode_buffer;

    dacsample_t set_handlebar_velocity(float velocity) {
        // limit velocity to a maximum magnitude of 100 deg/s
        // input is in units of rad/s
        const float saturated_velocity = util::clamp(velocity, -sa::MAX_KOLLMORGEN_VELOCITY, sa::MAX_KOLLMORGEN_VELOCITY);
        const dacsample_t aout = saturated_velocity/sa::MAX_KOLLMORGEN_VELOCITY*sa::DAC_HALF_RANGE + sa::DAC_HALF_RANGE;
        dacPutChannelX(sa::KOLLM_DAC, 0, aout); // TODO: don't hardcode zero but find the DAC channel constant
        return aout;
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

    filesystem::init();

    /*
     * Start sensors.
     * Encoder:
     *   Initialize encoder driver 5 on pins PA0, PA1 (EXT2-4, EXT2-8).
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

    bicycle_t bicycle(5.0f, 1.0f/200); /* (v [m/s], dt [s]) */

    // FIXME: initialize Kalman matrices here

    /* write firmware gitsha1, bicycle and Kalman settings to file */
    rtcnt_t bicycle_simulation_time = chSysGetRealtimeCounterX();

    sample = simulation_message_zero;
    sample.timestamp = bicycle_simulation_time;
    message::set_simulation_full_model_observer(&sample, bicycle);

    /*
     * Normal main() thread activity, in this demo it simulates the bicycle
     * dynamics in real-time (roughly).
     */
    bool delete_file = true;
    while (true) {
        chEvtDispatch(filesystem::sdc_eventhandlers, chEvtWaitOneTimeout(ALL_EVENTS, MS2ST(5)));

        // TODO: write to file in a separate thread
        if (filesystem::ready()) {
            const char* filename = "test.txt";
            UINT bytes_written;
            FIL f;
            FRESULT res;

            if (delete_file) {
                res = f_unlink(filename);
                chDbgCheck((res == FR_OK) || (res ==  FR_NO_FILE) || (res = FR_NO_PATH));
                delete_file = false;
            }

            res = f_open(&f, filename, FA_WRITE | FA_OPEN_ALWAYS);
            chDbgAssert(res == FR_OK, "file open failed");

            res = f_lseek(&f, f_size(&f));
            chDbgAssert(res == FR_OK, "file seek failed");

            uint8_t bytes_encoded = packet::serialize::encode_delimited(
                    sample, encode_buffer.data(), encode_buffer.size());
            res = f_write(&f, encode_buffer.data(), bytes_encoded, &bytes_written);
            chDbgAssert(res == FR_OK, "file write failed");

            res = f_close(&f);
            chDbgAssert(res == FR_OK, "file close failed");
        } else {
            continue;
        }

        constexpr float roll_torque = 0;

        /* get torque measurements */
        const float steer_torque = static_cast<float>(
                analog.get_adc12()*2.0f*sa::MAX_KISTLER_TORQUE/(sa::ADC_HALF_RANGE*2) -
                sa::MAX_KISTLER_TORQUE);
        const float motor_torque = static_cast<float>(
                analog.get_adc13()*2.0f*sa::MAX_KOLLMORGEN_TORQUE/(sa::ADC_HALF_RANGE*2) -
                sa::MAX_KOLLMORGEN_TORQUE);
        (void)motor_torque; /* this isn't currently used */

        /* get angle measurements */
        const float yaw_angle = util::wrap(bicycle.pose().yaw);
        const float steer_angle = util::encoder_count<float>(encoder_steer);
        const float rear_wheel_angle = -util::encoder_count<float>(encoder_rear_wheel);

        /* observer time/measurement update (~80 us with real_t = float) */
        bicycle.update_dynamics(roll_torque, steer_torque, yaw_angle, steer_angle, rear_wheel_angle);
        bicycle.update_kinematics();

        const float desired_velocity = model_t::get_state_element(bicycle.observer().state(),
                model_t::state_index_t::steer_rate);
        const dacsample_t handlebar_velocity_dac = set_handlebar_velocity(desired_velocity);

        sample = simulation_message_zero;
        sample.timestamp = chSysGetRealtimeCounterX();
        message::set_simulation_sensors(&sample,
                analog.get_adc12(), analog.get_adc13(),
                encoder_steer.count(), encoder_rear_wheel.count());
        message::set_simulation_actuators(&sample, handlebar_velocity_dac);

        // TODO: fix sleep amount
        chThdSleepMilliseconds(static_cast<systime_t>(1000*bicycle.dt()));
    }
}
