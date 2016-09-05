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

/*
 * DAC configuration options (set in halconf.h)
 * DAC_USE_WAIT                FALSE
 * DAC_USE_MUTUAL_EXCLUSION    FALSE
 *
 * DAC driver system settings (set in mcuconf.h)
 * STM32_DAC_DUAL_MODE                 FALSE
 * STM32_DAC_USE_DAC1_CH1              TRUE
 * STM32_DAC_USE_DAC1_CH2              FALSE
 */

/*
 * DAC conversion group is not used.
 */

namespace {
    const DACConfig dac1cfg1 = {
        .init       = 2047U, // max value is 4095 (12-bit)
        .datamode   = DAC_DHRM_12BIT_RIGHT
    };

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
     * Start DAC1 driver and set output pin as analog as suggested in Reference Manual.
     */
    palSetLineMode(LINE_KOLLM_ACTL_TORQUE, PAL_MODE_INPUT_ANALOG);
    dacStart(&DACD1, &dac1cfg1);

    /*
     * Create the LED blink thread.
     */
    chBlinkThreadCreateStatic(NORMALPRIO-1);

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
     * Normal main() thread activity. Set DAC conversion at roughly 1 kHz.
     */
    dacsample_t aout1 = 0;
    while (true) {
        aout1 = (++aout1) % 4095;
        dacPutChannelX(&DACD1, 0, aout1);
        if (SDU1.config->usbp->state == USB_ACTIVE) {
            chprintf((BaseSequentialStream*)&SDU1,
                    "dac: %d\r\n", aout1);
        }
        chThdSleep(loop_time);
    }
}
