#pragma once
#include "hal.h"

#if HAL_USE_EXT
extern EXTDriver* extp;

void extStartI(EXTDriver* extp);
void extChannelEnableSetModeI(EXTDriver* extp, ioline_t line, uint32_t edge_mode,
        extcallback_t callback, void* callback_object);
void extChannelDisableClearModeI(EXTDriver* extp, uint32_t pad);
void extStopIfChannelsDisabledI(EXTDriver* extp);
void* extGetChannelCallbackObject(expchannel_t channel);
#endif
