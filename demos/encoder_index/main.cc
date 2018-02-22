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
#include "usbcfg.h"
#include "encoder.h"
#include "printf.h"

namespace {
    const systime_t loop_time = MS2ST(100); /* loop at 10 Hz */
    Encoder encoder(&GPTD5, /* CH1, CH2 connected to PA0, PA1 and NOT enabled by board.h */
            {PAL_LINE(GPIOA, GPIOA_PIN2), /* GPIOA_PIN2 is available */
             152000, /* counts per revolution */
             EncoderConfig::filter_t::CAPTURE_64, /* 64 * 42 MHz (TIM3 on APB1) = 1.52 us for valid edge */
             0});
} // namespace

static THD_WORKING_AREA(waSerialThread, 256);
static THD_FUNCTION(SerialThread, arg) {
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
        if (SDU1.config->usbp->state == USB_ACTIVE) {
            printf("%d\tindex: ", encoder.count());
            if (encoder.index() == Encoder::index_t::FOUND) {
                printf("FOUND\r\n");
            } else {
                printf("NOTFOUND\r\n");
            }
        }
        chThdSleep(loop_time);
    }
}

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
    palSetLineMode(LINE_TIM5_CH1, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_FLOATING);
    palSetLineMode(LINE_TIM5_CH2, PAL_MODE_ALTERNATE(2) | PAL_STM32_PUPDR_FLOATING);
    palSetLineMode(encoder.config().z, PAL_STM32_MODE_INPUT | PAL_STM32_PUPDR_FLOATING);

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
