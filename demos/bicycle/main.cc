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

#include "usbconfig.h"
#include "blink.h"
#include "printf.h"

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

    /* create the blink thread */
    chBlinkThreadCreateStatic();

    /* initialize bicycle model and states */
    model::Bicycle bicycle(v0, dt);
    //x << 0, 0, 0, 0, 0; /* define in degrees */
    //x *= constants::as_radians; /* convert degrees to radians */
    x.setZero();
    aux.setZero();
    aux[2] = bicycle.solve_constraint_pitch(x, 0); /* solve for initial pitch angle */
    aux_hat = aux;

    /* initialize Kalman filter */
    kalman_t kalman(bicycle,
            bicycle_t::state_t::Zero(), /* set initial state estimate to zero */
            parameters::defaultvalue::kalman::Q(dt), /* process noise cov */
            (kalman_t::measurement_noise_covariance_t() <<
             sigma0,      0,
                  0, sigma1).finished(),
            std::pow(x[1]/2, 2) * bicycle_t::state_matrix_t::Identity()); /* error cov */

    /*
     * Normal main() thread activity, in this demo it simulates the bicycle
     * dynamics in real-time (roughly).
     *
     * The boost integrator requires variable computation time and also results in
     * the loop computation exceeding 1 ms. Furthermore, as the auxiliary states are
     * not included in the Kalman filter, initial error in auxiliary states will not
     * converge to zero.
     */
    uint32_t disturbance_counter = static_cast<uint32_t>(disturbance_period * fs);
    rtcnt_t state_update_time = 0;
    rtcnt_t kalman_update_time = 0;
    rtcnt_t aux_update_time = 0;
    while (true) {
        u.setZero();
        if (--disturbance_counter < static_cast<uint32_t>(disturbance_duty_cycle * fs)) {
            u[1] = disturbance_magnitude; /* N-m, steer torque */
        }
        if (disturbance_counter <= 0) {
            disturbance_counter = static_cast<uint32_t>(disturbance_period * fs);
        }

        /* advance bicycle model (~30 us) */
        state_update_time = chSysGetRealtimeCounterX();
        x = bicycle.update_state(x, u);
        state_update_time = chSysGetRealtimeCounterX() - state_update_time;

        /* observer time/measurement update (~512 us) */
        kalman_update_time = chSysGetRealtimeCounterX();
        kalman.time_update(u);
        kalman.measurement_update(bicycle.calculate_output(x)); /* use noiseless bicycle system output */
        x_hat = kalman.x();
        kalman_update_time = chSysGetRealtimeCounterX() - kalman_update_time;

        /* update auxiliary state and auxiliary state estimate (~2.4 ms - ~9.5 ms) */
        aux_update_time = chSysGetRealtimeCounterX();
        aux = bicycle.update_auxiliary_state(x, aux);
        aux_hat = bicycle.update_auxiliary_state(x_hat, aux_hat);
        aux_update_time = chSysGetRealtimeCounterX() - aux_update_time;

        printf("error:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\r\n",
                aux_hat[0] - aux[0],
                aux_hat[1] - aux[1],
                aux_hat[2] - aux[2],
                x_hat[0] - x[0],
                x_hat[1] - x[1],
                x_hat[2] - x[2],
                x_hat[3] - x[3],
                x_hat[4] - x[4]);
        printf("state update time: %d us\r\n",
                RTC2US(STM32_SYSCLK, state_update_time));
        printf("kalman update time: %d us\r\n",
                RTC2US(STM32_SYSCLK, kalman_update_time));
        printf("aux update time: %d us\r\n",
                RTC2US(STM32_SYSCLK, aux_update_time));
        chThdSleepMilliseconds(static_cast<systime_t>(1000*dt));
    }
}
