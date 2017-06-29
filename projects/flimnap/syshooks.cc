#include "syshooks.h"
#include "hal.h"

void disable_motor(void) {
    palClearLine(LINE_MOTOR1_EN);
}
