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

#include "usbconfig.h"
#include "bicycle.h"
#include "kalman.h"
#include "parameters.h"

namespace {
    using bicycle_t = model::Bicycle;
    using kalman_t = observer::Kalman<bicycle_t>;

    const float fs = 200; // sample rate [Hz]
    const float dt = 1.0/fs; // sample time [s]
    const float v0 = 5.0; // forward speed [m/s]

    /* Kalman filter variance values */
    const float sigma0 = 1 * constants::as_radians; // yaw angle measurement noise variance
    const float sigma1 = 0.008 * constants::as_radians; // steer angle measurement noise variance

    /* create a periodic steer torque pulse disturbance */
    const float disturbance_period = 5.0; // [s]
    const float disturbance_duty_cycle = 1.0; // [s]
    const float disturbance_magnitude = 2.0; // [N-m]

    bicycle_t::input_t u; /* roll torque, steer torque */
    bicycle_t::state_t x; /* yaw angle, roll angle, steer angle, roll rate, steer rate */
    bicycle_t::state_t x_hat; /* state estimate */
    bicycle_t::auxiliary_state_t aux; /* rear contact x, rear contact y, pitch angle */
    bicycle_t::auxiliary_state_t aux_hat; /* auxiliary state estimate */

    /*
     * This is a periodic thread that does absolutely nothing except flashing
     * a LED.
     */
    THD_WORKING_AREA(waThread1, 128);
    THD_FUNCTION(Thread1, arg) {
        (void)arg;

        chRegSetThreadName("blinker");
        while (true) {
            palToggleLine(LINE_LED);
            if (SDU1.config->usbp->state == USB_ACTIVE) {
                chThdSleepMilliseconds(100);
            } else {
                chThdSleepMilliseconds(1000);
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
    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

    /* initialize bicycle model and states */
    model::Bicycle bicycle(v0, dt);
    x << 0, 0, 10, 10, 0; /* define in degrees */
    x *= constants::as_radians; /* convert degrees to radians */
    aux.setZero();
    aux[2] = bicycle.solve_constraint_pitch(x, 0); /* solve for initial pitch angle */
    aux_hat = aux;

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
    uint32_t disturbance_counter = static_cast<uint32_t>(disturbance_period * fs);
    rtcnt_t dt = 0;
    while (true) {
        dt = chSysGetRealtimeCounterX(); // measure computation/transmit time
        u.setZero();
        if (--disturbance_counter < static_cast<uint32_t>(disturbance_duty_cycle * fs)) {
            u[1] = disturbance_magnitude; /* N-m, steer torque */
        }
        if (disturbance_counter <= 0) {
            disturbance_counter = static_cast<uint32_t>(disturbance_period * fs);
        }

        /* advance bicycle model */
        x = bicycle.x_next(x, u);

        /* observer time/measurement update */
        kalman.time_update(u);
        kalman.measurement_update(bicycle.y(x)); /* use noiseless bicycle system output */
        x_hat = kalman.x();

        /* update auxiliary state and auxiliary state estimate */
        aux = bicycle.x_aux_next(x, aux);
        aux_hat = bicycle.x_aux_next(x_hat, aux_hat);

        dt = chSysGetRealtimeCounterX() - start;

        chprintf((BaseSequentialStream*)&SDU1,
                "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\r\n",
                aux[0], aux[1], aux[2], x[0], x[1], x[2], x[3], x[4]);
        chprintf((BaseSequentialStream*)&SDU1,
                "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\r\n",
                aux_hat[0], aux_hat[1], aux_hat[2], x_hat[0],
                x_hat[1], x_hat[2], x_hat[3], x_hat[4]);
        chprintf((BaseSequentialStream*)&SDU1,
                "loop time was: %d us\r\n", RTC2US(STM32_SYSCLK, dt));
        chThdSleepMilliseconds(static_cast<systime_t>(1000*dt));
    }
}
