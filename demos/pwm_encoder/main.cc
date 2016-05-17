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
#include "encoder.h"

namespace {
    const systime_t loop_time = MS2ST(100); /* serial loop at 10 Hz */
    Encoder encoder(&GPTD3, /* CH1, CH2 connected to PC6, PC7 and enabled by board.h */
            {PAL_NOLINE, /* no index channel */
             1024, /* counts per revolution */
             EncoderConfig::filter_t::CAPTURE_64}); /* 64 * 42 MHz (TIM3 on APB1) = 1.52 us for valid edge */

    const uint32_t pwm_freq = 1000000; /* 1 MHz PWM clock frequency */
    const pwmcnt_t pwm_period = 1000; /* period is 10000 ticks (1 kHz) */

    PWMConfig pwmcfg = {
        pwm_freq,
        pwm_period,
        nullptr,
        {
            {PWM_OUTPUT_ACTIVE_HIGH, nullptr},
            {PWM_OUTPUT_DISABLED, nullptr},
            {PWM_OUTPUT_DISABLED, nullptr},
            {PWM_OUTPUT_DISABLED, nullptr}
        },
        0,
        0
    };

    /*
     * GPT8 configuration. This timer is used as trigger to start the PWM signals.
     */
    void start_pwm_cb(GPTDriver* gptp);
    const GPTConfig gptcfg8 = {
        frequency:    pwm_freq,
        callback:     start_pwm_cb,
        cr2:          0,
        dier:         0U
    };

    /*
     * Start PWM drivers 1/4 period out of phase to simulate an encoder.
     */
    void start_pwm_cb(GPTDriver* gptp) {
        (void)gptp;

        chSysLockFromISR();
        gptStartOneShotI(&GPTD8, pwm_period/4);
        if (PWMD1.state == PWM_STOP) {
            PWMD1.config = &pwmcfg;
            PWMD1.period = pwmcfg.period;
            pwm_lld_start(&PWMD1);
            PWMD1.enabled = 0;
            PWMD1.state = PWM_READY;
        } else if (PWMD4.state == PWM_STOP) {
            PWMD4.config = &pwmcfg;
            PWMD4.period = pwmcfg.period;
            pwm_lld_start(&PWMD4);
            PWMD4.enabled = 0;
            PWMD4.state = PWM_READY;
        } else {
            pwmEnableChannelI(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5000));
            pwmEnableChannelI(&PWMD4, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, 5000));
            gptStopTimerI(&GPTD8);
        }
        chSysUnlockFromISR();
    }
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
            chprintf((BaseSequentialStream*)&SDU1, "%d\r\n", encoder.count());
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
     * Initializes the encoder driver 3 on pins PC6, PC7
     * Initializes the PWM driver 1 and 4, starting PWMD4 1/4 period after PWMD1.
     * GPIOA8 is the PWMD1 output.
     * GPIOB6 is the PWMD4 output.
     */
    encoder.start();
    gptStart(&GPTD8, &gptcfg8);
    gptStartOneShot(&GPTD8, gptcfg8.frequency);

    palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1));
    palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(2));

    /*
     * Creates the LED blink and USB serial threads.
     */
    chBlinkThreadCreateStatic(NORMALPRIO-1);
    chThdCreateStatic(waSerialThread, sizeof(waSerialThread), NORMALPRIO+1,
            SerialThread, nullptr);

    /*
     * Normal main() thread activity. In this demo it does nothing.
     */
    while (true) {
        chThdSleep(MS2ST(500));
    }
}
