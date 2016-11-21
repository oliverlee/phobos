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

#include "tsencoder.h"

namespace {
    const systime_t loop_time = MS2ST(100); /* loop at 10 Hz */
    using encoder_t = TSEncoder<5, 10, 3>;
    encoder_t encoder({
            LINE_TIM5_CH1,
            LINE_TIM5_CH2,
            PAL_LINE(GPIOA, GPIOA_PIN2), /* GPIOA_PIN2 is unused */
            152000, /* counts per revolution */
             });

    THD_WORKING_AREA(waSerialThread, 8192);
    THD_FUNCTION(SerialThread, arg) {
        (void)arg;
        chRegSetThreadName("serial");

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

        while (true) {
            if ((SDU1.config->usbp->state == USB_ACTIVE) || (SDU1.state == SDU_READY)) {
                encoder.update_polynomial_fit();
                encoder.update_estimate_time(chSysGetRealtimeCounterX());
                printf("index: %u\tpos: %8.3f\tvel: %8.3f\tacc: %8.3f\r\n",
                        encoder.index() == encoder_t::index_t::FOUND,
                        encoder.position(),
                        encoder.velocity(),
                        encoder.acceleration());
            }
            chThdSleep(loop_time);
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
     * Initializes the encoder driver 5 on pins PA0, PA1 (EXT2-4, EXT2-8).
     * Remove R19 to disable button functionality if necessary.
     * Configure index gpio PA2 (EXT2-7).
     */
    palSetLineMode(LINE_TIM5_CH1, PAL_STM32_MODE_INPUT | PAL_STM32_PUPDR_FLOATING);
    palSetLineMode(LINE_TIM5_CH2, PAL_STM32_MODE_INPUT | PAL_STM32_PUPDR_FLOATING);
    palSetLineMode(PAL_LINE(GPIOA, GPIOA_PIN2), PAL_STM32_MODE_INPUT | PAL_STM32_PUPDR_FLOATING);
    encoder.start();

    /*
     * Creates the LED blink and USB serial threads.
     */
    chBlinkThreadCreateStatic(NORMALPRIO-1);
    chThdCreateStatic(waSerialThread, sizeof(waSerialThread), NORMALPRIO+1,
            SerialThread, nullptr);

    /*
     * Normal main() thread activity.
     */
    while (true) {
        chThdSleep(MS2ST(50));
    }
}
