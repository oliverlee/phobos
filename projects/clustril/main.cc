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
#include "chprintf.h"

#include "blink.h"
#include "usbconfig.h"
#include "bicycle.h"
#include "kalman.h"
#include "parameters.h"

#include "encoder.h"
#include <boost/math/constants/constants.hpp>

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
    Encoder encoder(&GPTD5, /* CH1, CH2 connected to PA0, PA1 and NOT enabled by board.h */
            {PAL_NOLINE, /* no index channel */
             152000, /* counts per revolution */
             EncoderConfig::filter_t::CAPTURE_64}); /* 64 * 42 MHz (TIM3 on APB1) = 1.52 us
                                                     * for valid edge */
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
     * Initializes a serial-over-USB CDC driver.
     */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    board_usb_lld_disconnect_bus();   //usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    board_usb_lld_connect_bus();      //usbConnectBus(serusbcfg.usbp);

    /* create the example thread */
    chBlinkThreadCreateStatic();

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
        u.setZero();
        u[1] = 0; /* steer torque, read from torque sensor */

        /* set measurement vector */
        z[0] = x[0]; /* yaw angle, just use previous state value */

        /* steer angle, read from encoder (enccnt_t is uint32_t) */
        z[1] = static_cast<float>(encoder.count() /
                boost::math::constants::two_pi<float>());
        if (z[1] >= boost::math::constants::pi<float>()) {
            z[1] -= boost::math::constants::pi<float>();
        }

        /* observer time/measurement update (~512 us) */
        kalman_update_time = chSysGetRealtimeCounterX();
        kalman.time_update(u);
        kalman.measurement_update(z);
        x = kalman.x();
        kalman_update_time = chSysGetRealtimeCounterX() - kalman_update_time;

        chprintf((BaseSequentialStream*)&SDU1,
                "error:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\r\n",
                x[0], x[1], x[2], x[3], x[4]);
        chprintf((BaseSequentialStream*)&SDU1,
                "kalman update time: %d us\r\n",
                RTC2US(STM32_SYSCLK, kalman_update_time));
        chThdSleepMilliseconds(static_cast<systime_t>(1000*dt));
    }
}
