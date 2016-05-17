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

#include <Eigen/Core>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "blink.h"
#include "usbconfig.h"

namespace {
    using real_t = float;
    const unsigned int n = 4;
    const real_t matrix_scale_limit = 100.0f;
    using matrix_t = Eigen::Matrix<real_t, n, n>;
    const matrix_t A((matrix_t() <<
                1.0f, 0.9f, 0.8f, 0.7f,
                0.6f, 0.5f, 0.4f, 0.3f,
                0.2f, 0.1f, 0.09f, 0.08f,
                0.07f, 0.06f, 0.05f, 0.04f).finished());

    const systime_t loop_time = MS2ST(1); // loop at 1 kHz
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
     * Creates the blink thread.
     */
    chBlinkThreadCreateStatic();

    /*
     * Normal main() thread activity. In this demo it simulates multiplies a matrix.
     */
    uint32_t i = 0;
    matrix_t B = matrix_t::Identity();
    rtcnt_t start = 0;
    rtcnt_t dt = 0;
    systime_t deadline = chVTGetSystemTimeX();

    while (true) {
        // Use RTC directly due to a bug in last time computation in TM module.
        start = chSysGetRealtimeCounterX(); // measure computation/transmit time
        deadline += loop_time; // next deadline
        B = A * B;

        /*
         * Rescale matrix entries if too large or too small.
         */
        real_t* data_A = const_cast<real_t*>(A.data());
        real_t* data_B = const_cast<real_t*>(B.data());
        for (int i = 0; i < A.size(); ++i, ++data_A, ++data_B) {
            real_t a = *data_A;
            real_t b = *data_B;
            if ((b > 100*a) || (b < a/100)) {
                *data_B = a;
            }
        }

        /*
         * Transmit loop time and matrix values.
         */
        if (SDU1.config->usbp->state == USB_ACTIVE || SDU1.state == SDU_READY) {
            chprintf((BaseSequentialStream*)&SDU1,
                    "iteration %d: %d us\r\n", ++i, RTC2US(STM32_SYSCLK, dt));
            real_t* data = B.data();
            for (int i = 0; i < B.size(); ++i) {
                chprintf((BaseSequentialStream*)&SDU1, "%0.2f", *data++);
                if ((i % 4) == 3) {
                    chprintf((BaseSequentialStream*)&SDU1, "\r\n");
                } else {
                    chprintf((BaseSequentialStream*)&SDU1, ", ");
                }
            }
            chprintf((BaseSequentialStream*)&SDU1, "\r\n");
        }
        dt = chSysGetRealtimeCounterX() - start;

        /*
         * Sleep thread until next deadline if it hasn't already been missed.
         */
        bool miss = false;
        chSysLock();
        systime_t sleep_time = deadline - chVTGetSystemTimeX();
        if ((sleep_time > (systime_t)0) and (sleep_time < loop_time)) {
            chThdSleepS(sleep_time);
        } else {
            miss = true;
        }
        chSysUnlock();

        /*
         * If deadline missed, wait until button is pressed before continuing.
         */
        if (miss) {
            chprintf((BaseSequentialStream*)&SDU1,
                    "loop time was: %d us\r\n", RTC2US(STM32_SYSCLK, dt));
            chprintf((BaseSequentialStream*)&SDU1, "Press button to continue.\r\n");
            while (palReadLine(LINE_BUTTON)) { /* Button is active LOW. */
                chThdSleepMilliseconds(10);
            }
            deadline = chVTGetSystemTimeX();
        }
    }
}
