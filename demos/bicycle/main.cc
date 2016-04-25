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
#include "parameters.h"

namespace {
    using bicycle_t = model::Bicycle;

    const float fs = 200; // sample rate [Hz]
    const float dt = 1.0/fs; // sample time [s]
    const float v0 = 5.0; // forward speed [m/s]

    bicycle_t::state_t x; /* yaw angle, roll angle, steer angle, roll rate, steer rate */
    bicycle_t::auxiliary_state_t aux; /* rear contact x, rear contact y, pitch angle */

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

    /*
     * Creates the example thread.
     */
    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

    /*
     * Initialize bicycle model and states
     */
    model::Bicycle bicycle(v0, dt);
    x << 0, 0, 10, 10, 0; /* define in degrees */
    x *= constants::as_radians; /* convert degrees to radians */
    aux.setZero();
    aux[2] = bicycle.solve_constraint_pitch(x, 0); /* solve for initial pitch angle */

    /*
     * Normal main() thread activity, in this demo it simulates the bicycle dynamics in real-time (roughly).
     */
    while (true) {
        x = bicycle.x_next(x);
        aux = bicycle.x_aux_next(x, aux);
        chprintf((BaseSequentialStream*)&SDU1,
                "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\r\n",
                aux[0], aux[1], aux[2], x[0], x[1], x[2], x[3], x[4]);
        chThdSleepMilliseconds(static_cast<systime_t>(1000*dt));
    }
}
