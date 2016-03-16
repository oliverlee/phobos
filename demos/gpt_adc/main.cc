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

#include <array>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "usbcfg.h"

namespace {
    thread_t* main_thread;

    const eventmask_t adc_eventmask_complete = EVENT_MASK(0);
    const eventmask_t adc_eventmask_error = EVENT_MASK(1);

    void adcerrorcallback(ADCDriver* adcp, adcerror_t err) {
        (void)adcp;
        (void)err;
        chSysLockFromISR();
        chEvtSignalI(main_thread, adc_eventmask_complete);
        chSysUnlockFromISR();
    }
    void adccallback(ADCDriver* adcp, adcsample_t* buffer, size_t n) {
        (void)adcp;
        (void)buffer;
        (void)n;
        chSysLockFromISR();
        chEvtSignalI(main_thread, adc_eventmask_complete);
        chSysUnlockFromISR();
    }

    std::array<adcsample_t, 3> adc_buffer;
    /*
     * ADC conversion group.
     * Mode:        Continuous, 1 sample of 3 channels, HW triggered by GPT8-TRGO.
     * Channels:    IN10, IN11, IN12.
     */
    static const ADCConversionGroup adcgrpcfg1 = {
        true,
        adc_buffer.size(),
        adccallback,
        adcerrorcallback,
        0,                        /* CR1 */
        ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(14),        /* CR2 */
        ADC_SMPR1_SMP_AN12(ADC_SAMPLE_3) | ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3) |
        ADC_SMPR1_SMP_AN10(ADC_SAMPLE_3),
        0,                        /* SMPR2 */
        ADC_SQR1_NUM_CH(adc_buffer.size()),
        0,                        /* SQR2 */
        ADC_SQR3_SQ3_N(ADC_CHANNEL_IN12) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN11) |
        ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10)
    };

    /*
     * GPT8 configuration. This timer is used as trigger for the ADC.
     */
    const GPTConfig gpt8cfg1 = {
      frequency:    1000000U,
      callback:     nullptr,
      cr2:          TIM_CR2_MMS_1,  /* MMS = 010 = TRGO on Update Event. */
      dier:         0U
    };

    /*
     * This is a periodic thread that simply flashes an LED and allows visual
     * inspection to see that the program has not halted.
     */
    THD_WORKING_AREA(wa_thread_led, 128);
    THD_FUNCTION(thread_led, arg) {
        (void)arg;
        chRegSetThreadName("led");
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

    main_thread = chThdGetSelfX();

    /*
     * Starting GPT8 driver, it is used for triggering the ADC.
     */
    gptStart(&GPTD8, &gpt8cfg1);

    /*
     * ADC init
     */
    adcStart(&ADCD1, nullptr);

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
     * Creates the LED blink thread.
     */
    chThdCreateStatic(wa_thread_led, sizeof(wa_thread_led), NORMALPRIO,
            thread_led, nullptr);

    /*
     * Starts an ADC continuous conversion triggered at a frequency of 1 kHz
     * (1000 * 1 MHz).
     */
    adcStartConversion(&ADCD1, &adcgrpcfg1, adc_buffer.data(), 1);
    gptStartContinuous(&GPTD8, 1000);

    /*
     * Normal main() thread activity. In this demo it sends ADC values over serial.
     */
    while (true) {
        eventmask_t evt = chEvtWaitAny(ALL_EVENTS);

        if (SDU1.config->usbp->state == USB_ACTIVE) {
            if (evt & adc_eventmask_error) {
                chprintf((BaseSequentialStream*)&SDU1,
                        "ERROR in ADC conversion.\r\n");
            }
            if (evt & adc_eventmask_complete) {
                chprintf((BaseSequentialStream*)&SDU1,
                        "%d\t%d\t%d\r\n",
                        adc_buffer[0],
                        adc_buffer[1],
                        adc_buffer[2]);
            }
        }
    }
}
