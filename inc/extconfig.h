#pragma once
#include "hal.h"
#include <array>

#if HAL_USE_EXT
extern EXTConfig extconfig;
extern EXTDriver* extp;
extern std::array<void*, EXT_MAX_CHANNELS> ext_map;
void extChannelDisableClearModeI(EXTDriver* extp, uint32_t pad);
void extStopIfChannelsDisabledI(EXTDriver* extp);
#endif
