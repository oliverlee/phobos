#pragma once

#if defined(CH_CFG_TIME_QUANTUM)
    // We are compiling for ChibiOS
    #include "osal.h"
    #define debug_assert(condition, remark) osalDbgAssert(condition, remark)
    #define debug_check(condition) osalDbgCheck(condition)
#else
    // We are not compiling for ChibiOS
    #include <cassert>
    #define debug_assert(condition, remark) assert(condition)
    #define debug_check(condition) assert(condition)
#endif
