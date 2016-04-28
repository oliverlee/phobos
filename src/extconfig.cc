#include "extconfig.h"
#include <algorithm>

#if HAL_USE_EXT
EXTConfig extconfig = EXTConfig(); /* value initialization sets EXT_CH_MODE to zero for all channels */

void extChannelDisableClearModeI(EXTDriver* extp, uint32_t pad) {
        extenc_map[pad] = nullptr;
        extconfig.channels[pad] = EXTChannelConfig{EXT_CH_MODE_DISABLED, nullptr};
        extSetChannelModeI(extp, pad, &extconfig.channels[pad]);
        extChannelDisableI(extp, pad);
}

void extStopIfChannelsDisabledI(EXTDriver* extp) {
    if (std::all_of(std::begin(extp->config->channels), std::end(extp->config->channels),
                [](EXTChannelConfig config)->bool {
                    return ((config.mode & EXT_CH_MODE_EDGES_MASK) == EXT_CH_MODE_DISABLED);
                })) {
        ext_lld_stop(extp);
        extp->state = EXT_STOP;
    }
}
#endif
