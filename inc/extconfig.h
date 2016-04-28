#pragma once
#include "hal.h"

#if HAL_USE_EXT
extern EXTConfig extconfig;
void extChannelDisableClearModeI(EXTDriver* extp, uint32_t pad);
void extStopIfChannelsDisabledI(EXTDriver* extp);
#endif
