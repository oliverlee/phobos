#pragma once
#include "ch.h"
#include "hal.h"

enum printst_t: uint8_t {VERSION=0, NORMAL, NONE};
void enablePrintStateMonitor();
printst_t getPrintState();
