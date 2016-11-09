#pragma once

#include "ch.h"
#include "ff.h"

namespace filesystem {
    extern const evhandler_t sdc_eventhandlers[];

    void init();
    bool ready();
} // filesystem
