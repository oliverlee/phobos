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

#include "blink.h"
#include "usbconfig.h"
#include "printf.h"

#include "bicycle.h"
#include "kalman.h"
#include "parameters.h"

#include "analog.h"
#include "encoder.h"
#include <boost/math/constants/constants.hpp>

#include <type_traits>


namespace {
    using bicycle_t = model::Bicycle;
    using kalman_t = observer::Kalman<bicycle_t>;

    const float fs = 200; // sample rate [Hz]
    const float dt = 1.0/fs; // sample time [s]
    const float v0 = 5.0; // forward speed [m/s]

    /* Kalman filter variance values */
    const float sigma0 = 1 * constants::as_radians; // yaw angle measurement noise variance
    const float sigma1 = 0.008 * constants::as_radians; // steer angle measurement noise variance

    bicycle_t::input_t u; /* roll torque, steer torque */
    bicycle_t::state_t x; /* yaw angle, roll angle, steer angle, roll rate, steer rate */
    bicycle_t::output_t z; /* yaw angle, steer angle */

    /* sensors */
    Analog analog;
    Encoder encoder(&GPTD5, /* CH1, CH2 connected to PA0, PA1 and NOT enabled by board.h */
            {PAL_NOLINE, /* no index channel */
             152000, /* counts per revolution */
             EncoderConfig::filter_t::CAPTURE_64}); /* 64 * 42 MHz (TIM3 on APB1) = 1.52 us
                                                     * for valid edge */

     const float max_kistler_torque = 25.0f; /* maximum measured steer torque */
     /*
      * The voltage output of the Kistler torque sensor is ±10V. With the 12-bit ADC,
      * resolution for LSB is 4.88 mV/bit or 12.2 mNm/bit.
      */
     const float max_kollmorgen_torque = 10.0f; /* max torque at 1.00 Arms/V */

    const DACConfig dac1cfg1 = {
        .init       = 2047U, // max value is 4095 (12-bit)
        .datamode   = DAC_DHRM_12BIT_RIGHT
    };

    model::real_t wrap_angle(model::real_t angle) {
        /*
         * Angle magnitude is assumed to small enough to NOT require multiple
         * additions/subtractions of 2pi.
         */
        if (angle >= boost::math::constants::pi<float>()) {
            angle -= boost::math::constants::two_pi<float>();
        }
        if (angle < -boost::math::constants::pi<float>()) {
            angle += boost::math::constants::two_pi<float>();
        }
        return angle;
    }

     float rad_to_deg(float angle) {
         return angle * 360 / boost::math::constants::two_pi<float>();
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

    /* create the blink thread */
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
    dacStart(&DACD1, &dac1cfg1);

    /* initialize bicycle model and states */
    model::Bicycle bicycle(v0, dt);
    x.setZero();

    /* initialize Kalman filter */
    kalman_t kalman(bicycle,
            parameters::defaultvalue::kalman::Q(dt), /* process noise cov */
            (kalman_t::measurement_noise_covariance_t() <<
             sigma0,      0,
                  0, sigma1).finished(),
            bicycle_t::state_t::Zero(), /* set initial state estimate to zero */
            std::pow(x[1]/2, 2) * bicycle_t::state_matrix_t::Identity()); /* error cov */

    /*
     * Normal main() thread activity, in this demo it simulates the bicycle
     * dynamics in real-time (roughly).
     */
    rtcnt_t kalman_update_time = 0;
    while (true) {
        u.setZero(); /* set both roll and steer torques to zero */
        u[1] = static_cast<float>(analog.get_adc12()*2.0f*max_kistler_torque/4096 -
                max_kistler_torque);
        float motor_torque = static_cast<float>(
                analog.get_adc13()*2.0f*max_kollmorgen_torque/4096 -
                max_kollmorgen_torque);

        /* set measurement vector */
        z[0] = x[0]; /* yaw angle, just use previous state value */

        /*
         * Steer angle, read from encoder (enccnt_t is uint32_t)
         * Convert angle from enccnt_t (unsigned) to corresponding signed type and use negative
         * values for any count over half a revolution.
         */
        auto position = static_cast<std::make_signed<enccnt_t>::type>(encoder.count());
        auto rev = static_cast<std::make_signed<enccnt_t>::type>(encoder.config().counts_per_rev);
        if (position > rev / 2) {
            position -= rev;
        }
        z[1] = static_cast<float>(position) / rev *
                boost::math::constants::two_pi<float>();

        /* observer time/measurement update (~80 us with real_t = float) */
        kalman_update_time = chSysGetRealtimeCounterX();
        kalman.time_update(u);
        kalman.measurement_update(z);
        x = kalman.x();
        x[0] = wrap_angle(x[0]);
        x[1] = wrap_angle(x[1]);
        x[2] = wrap_angle(x[2]);
        kalman_update_time = chSysGetRealtimeCounterX() - kalman_update_time;

        /* generate an example torque output for testing */
        float torque = 10.0f * std::sin(
                boost::math::constants::two_pi<float>() *
                ST2S(static_cast<float>(chVTGetSystemTime())));
        dacsample_t aout = static_cast<dacsample_t>(
                (torque/21.0f * 2048) + 2048); /* reduce output to half of full range */
        dacPutChannelX(&DACD1, 0, aout);

        printf("DAC output:\t%d\r\n", aout);
        printf("sensors:\t%0.3f\t%0.3f\t%0.3f\t%0.3f\r\n",
                u[1], motor_torque, rad_to_deg(z[0]), rad_to_deg(z[1]));
        printf("state:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\r\n",
                x[0], x[1], x[2], x[3], x[4]);
        printf("kalman update time: %U us\r\n",
                RTC2US(STM32_SYSCLK, kalman_update_time));
        chThdSleepMilliseconds(static_cast<systime_t>(1000*dt));
    }
}
