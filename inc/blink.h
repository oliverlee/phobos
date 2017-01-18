#pragma once
#include "ch.h"
#include "hal.h"

/*
 * This creates a thread in a static memory area that blinks the LED. The blink
 * rate increases when the USB state is USB_ACTIVE.
 */
thread_t* chBlinkThreadCreateStatic(tprio_t priority=LOWPRIO);
