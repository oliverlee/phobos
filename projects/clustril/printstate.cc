#include "printstate.h"
#include "extconfig.h"
#include <type_traits>

namespace {
    using printst_ul_t = std::underlying_type_t<printst_t>;
    printst_t state = printst_t::NONE;
    systime_t t_prev = 0;
    constexpr systime_t deadtime = MS2ST(500);

    constexpr printst_ul_t LAST_STATE = static_cast<printst_ul_t>(printst_t::NONE) + 1;

    void monitor_callback(EXTDriver* extp, expchannel_t channel) {
        (void)extp;
        (void)channel;
        osalSysLockFromISR();
        systime_t t = chVTGetSystemTimeX();
        if ((t - t_prev) > deadtime) {
            state = static_cast<printst_t>((static_cast<printst_ul_t>(state) + 1) % LAST_STATE);
            t_prev = t;
        }
        osalSysUnlockFromISR();
    }
} // namespace

void enablePrintStateMonitor() {
    osalDbgAssert((extp->state == EXT_STOP) || (extp->state == EXT_ACTIVE),
            "invalid state");
    if (extp->state == EXT_STOP) {
        extStartI(extp);
    }
    extChannelEnableSetModeI(extp, LINE_BUTTON, EXT_CH_MODE_FALLING_EDGE, /* button is active low */
            monitor_callback, nullptr);
}

printst_t getPrintState() {
    return state;
}
