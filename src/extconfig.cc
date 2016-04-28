#include "extconfig.h"
#include <algorithm>
#include <array>
#include "osal.h"

#if HAL_USE_EXT
EXTDriver* extp = &EXTD1;

namespace {
    EXTConfig extconfig = EXTConfig(); /* value initialization sets EXT_CH_MODE to zero for all channels */
    std::array<void*, EXT_MAX_CHANNELS> ext_map = std::array<void*, EXT_MAX_CHANNELS>();
} // namespace

void extStartI(EXTDriver* extp) {
    osalDbgCheckClassI();

    extp->config = &extconfig;
    ext_lld_start(extp);
    extp->state = EXT_ACTIVE;
}

void extChannelEnableSetModeI(EXTDriver* extp, ioline_t line, uint32_t edge_mode,
        extcallback_t callback, void* callback_object) {
    osalDbgCheckClassI();

    uint32_t pad = PAL_PAD(line);
    stm32_gpio_t* port = PAL_PORT(line);
    osalDbgAssert((extp->state == EXT_ACTIVE), "EXT driver not active");
    osalDbgAssert(((extp->config->channels[pad].mode & EXT_CH_MODE_EDGES_MASK) == EXT_CH_MODE_DISABLED),
            "channel already in use");
    osalDbgAssert(((port == GPIOA) || (port == GPIOB) || (port == GPIOC) ||
                (port == GPIOD) || (port == GPIOE) || (port == GPIOF) ||
                (port == GPIOG) || (port == GPIOH) || (port == GPIOI)),
            "invalid port"); /* check if port is available in STM32/LLD/EXTIv1 driver */
    osalDbgAssert(((edge_mode == EXT_CH_MODE_RISING_EDGE) ||
                (edge_mode == EXT_CH_MODE_FALLING_EDGE) ||
                (edge_mode == EXT_CH_MODE_BOTH_EDGES)),
            "invalid edge mode");
    uint32_t ext_mode_port;
    if (port == GPIOA) {
        ext_mode_port = EXT_MODE_GPIOA;
    } else if (port == GPIOB) {
        ext_mode_port = EXT_MODE_GPIOB;
    } else if (port == GPIOC) {
        ext_mode_port = EXT_MODE_GPIOC;
    } else if (port == GPIOD) {
        ext_mode_port = EXT_MODE_GPIOD;
    } else if (port == GPIOE) {
        ext_mode_port = EXT_MODE_GPIOE;
    } else if (port == GPIOF) {
        ext_mode_port = EXT_MODE_GPIOF;
    } else if (port == GPIOG) {
        ext_mode_port = EXT_MODE_GPIOG;
    } else if (port == GPIOH) {
        ext_mode_port = EXT_MODE_GPIOH;
    } else { /* port == GPIOI */
        ext_mode_port = EXT_MODE_GPIOI;
    }

    ext_map[pad] = callback_object;
    extconfig.channels[pad] = EXTChannelConfig{edge_mode | ext_mode_port, callback};
    extSetChannelModeI(extp, pad, &extconfig.channels[pad]);
    extChannelEnableI(extp, pad);
}

void extChannelDisableClearModeI(EXTDriver* extp, uint32_t pad) {
    osalDbgCheckClassI();

    ext_map[pad] = nullptr;
    extconfig.channels[pad] = EXTChannelConfig{EXT_CH_MODE_DISABLED, nullptr};
    extSetChannelModeI(extp, pad, &extconfig.channels[pad]);
    extChannelDisableI(extp, pad);
}

void extStopIfChannelsDisabledI(EXTDriver* extp) {
    osalDbgCheckClassI();

    if (std::all_of(std::begin(extp->config->channels), std::end(extp->config->channels),
                [](EXTChannelConfig config)->bool {
                    return ((config.mode & EXT_CH_MODE_EDGES_MASK) == EXT_CH_MODE_DISABLED);
                })) {
        ext_lld_stop(extp);
        extp->state = EXT_STOP;
    }
}

void* extGetChannelCallbackObject(expchannel_t channel) {
    return ext_map[channel];
}
#endif /* HAL_USE_EXT */
