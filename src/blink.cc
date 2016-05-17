#include "blink.h"
#include "usbconfig.h"

namespace {
    THD_WORKING_AREA(wa_blink_thread, 128);
    THD_FUNCTION(blink_thread, arg) {
        (void)arg;

        chRegSetThreadName("blink");
        while (true) {
            palToggleLine(LINE_LED); /* defined for OLIMEX STM32-H405 */
            if (SDU1.config->usbp->state == USB_ACTIVE) {
                chThdSleepMilliseconds(100);
            } else {
                chThdSleepMilliseconds(1000);
            }
        }
    }
} // namespace

thread_t* chBlinkThreadCreateStatic(tprio_t priority) {
    return chThdCreateStatic(wa_blink_thread, sizeof(wa_blink_thread), priority,
            blink_thread, nullptr);
}
