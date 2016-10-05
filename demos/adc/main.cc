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

#include "blink.h"
#include "printf.h"
#include "usbconfig.h"

/*
 * ADC configuration options (set in halconf.h)
 * ADC_USE_WAIT FALSE - ADC to be used asynchronously
 * ADC_USE_MUTUAL_EXCLUSION FALSE - ADC to be used in a single thread

 * ADC driver system settings (set in mcuconf.h)
 * STM32_ADC_USE_ADC1 TRUE
 */

/*
 * ADC conversion group.
 * Mode:        One shot, 1 sample of 3 channels, SW triggered.
 * Channels:    IN10, IN11, IN12.
 */

namespace {
    const eventflags_t adc_eventflag_complete = EVENT_MASK(0);
    const eventflags_t adc_eventflag_error = EVENT_MASK(1);
    event_source_t adc_event_source;

    void adcerrorcallback(ADCDriver* adcp, adcerror_t err) {
        (void)adcp;
        (void)err;
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&adc_event_source, adc_eventflag_error);
        chSysUnlockFromISR();
    }
    void adccallback(ADCDriver* adcp, adcsample_t* buffer, size_t n) {
        (void)adcp;
        (void)buffer;
        (void)n;
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&adc_event_source, adc_eventflag_complete);
        chSysUnlockFromISR();
    }

    std::array<adcsample_t, 3> adc_buffer;
    static const ADCConversionGroup adcgrpcfg1 = {
        false,
        adc_buffer.size(),
        adccallback,
        adcerrorcallback,
        0,                        /* CR1 */
        ADC_CR2_SWSTART,          /* CR2 */
        ADC_SMPR1_SMP_AN12(ADC_SAMPLE_3) |
        ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3) |
        ADC_SMPR1_SMP_AN10(ADC_SAMPLE_3),
        0,                        /* SMPR2 */
        ADC_SQR1_NUM_CH(adc_buffer.size()),
        0,                        /* SQR2 */
        ADC_SQR3_SQ3_N(ADC_CHANNEL_IN12) |
        ADC_SQR3_SQ2_N(ADC_CHANNEL_IN11) |
        ADC_SQR3_SQ1_N(ADC_CHANNEL_IN10)
    };

    const systime_t loop_time = MS2ST(1); // loop at 1 kHz
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

    event_listener_t adc_listener;
    chEvtRegisterMaskWithFlags(&adc_event_source, &adc_listener, EVENT_MASK(0),
            adc_eventflag_error | adc_eventflag_complete);

    while (true) {
        chEvtWaitAny(ALL_EVENTS);
        eventflags_t flags = chEvtGetAndClearFlags(&adc_listener);

        if (SDU1.config->usbp->state == USB_ACTIVE) {
            if (flags & adc_eventflag_error) {
                printf("ERROR in ADC conversion.\r\n");
            }
            if (flags & adc_eventflag_complete) {
                printf("%d\t%d\t%d\r\n",
                        adc_buffer[0],
                        adc_buffer[1],
                        adc_buffer[2]);
            }
        }
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
     * ADC init
     */
    adcStart(&ADCD1, nullptr);

    /*
     * Event Source init
     */
    chEvtObjectInit(&adc_event_source);

    /*
     * Creates the LED blink and USB serial threads.
     */
    chBlinkThreadCreateStatic(NORMALPRIO-1);
    chThdCreateStatic(waSerialThread, sizeof(waSerialThread), NORMALPRIO+1,
            SerialThread, nullptr);

    /*
     * Normal main() thread activity. In this demo it starts ADC conversion at roughly 1 kHz.
     */
    while (true) {
        adcStartConversion(&ADCD1, &adcgrpcfg1, adc_buffer.data(), 1);
        chThdSleep(loop_time);
    }
}
