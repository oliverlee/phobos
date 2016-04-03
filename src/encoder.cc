#include <algorithm>
#include <array>
#include "osal.h"
#include "encoder.h"
#include "extcfg.h"

namespace {
    EXTDriver* extp = &EXTD1;
    std::array<stm32_tim_t*, EXT_MAX_CHANNELS> exttim_map{{}}; /* initialized to nullptr */
} // namespace

Encoder::Encoder(GPTDriver* gptp, const EncoderConfig& config) :
    m_gptp(gptp),
    m_gptconfig{gptp->clock, nullptr, 0U, 0U},
    m_config(config),
    m_state(state_t::STOP),
    m_index(index_t::NONE) { }

void Encoder::start() {
    osalDbgCheck(m_gptp != nullptr);

    osalSysLock();
    osalDbgAssert((m_gptp->state == GPT_STOP) || (m_gptp->state == GPT_READY), "invalid state");
    osalDbgAssert(((m_config.count <= TIM_CNT_CNT) ||
                !IS_TIM_32B_COUNTER_INSTANCE(reinterpret_cast<TIM_TypeDef*>(m_gptp->tim))),
            "invalid count");

    m_gptp->config = &m_gptconfig;
    gpt_lld_start(m_gptp); /* configure and activate peripheral */
    m_gptp->tim->SMCR = TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0; /* encoder mode 3, count on both TI1FP2 and TI2FP1 edges */
    m_gptp->tim->CCER = 0U; /* rising edge polarity */
    m_gptp->tim->CCMR1 = TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC1S_0; /* no prescaler, IC2 -> TI2, IC1 -> TI1 */
    uint16_t filter = static_cast<uint16_t>(m_config.filter);
    m_gptp->tim->CCMR1 |= (filter << 12) | (filter << 4); /* set input capture filters */
    gpt_lld_start_timer(m_gptp, m_config.count); /* set ARR and start counting */

    m_gptp->state = GPT_READY;
    m_state = state_t::READY;
    if (m_config.z == PAL_NOLINE) {
        m_index = index_t::NONE;
    } else {
        m_index = index_t::NOTFOUND;
        uint32_t pad = PAL_PAD(m_config.z);
        stm32_gpio_t* port = PAL_PORT(m_config.z);
        osalDbgAssert((extp->state == EXT_STOP) || (extp->state == EXT_ACTIVE),
                "invalid state");
        osalDbgAssert(((extp->config->channels[pad].mode & EXT_CH_MODE_EDGES_MASK)
                    == EXT_CH_MODE_DISABLED),
                "channel already in use");
        osalDbgAssert(((port == GPIOA) || (port == GPIOB) || (port == GPIOC) ||
                    (port == GPIOD) || (port == GPIOE) || (port == GPIOF) ||
                    (port == GPIOG) || (port == GPIOH) || (port == GPIOI)),
                "invalid port"); /* port is available in STM32/LLD/EXTIv1 driver */

        auto callback = [](EXTDriver* extp, expchannel_t channel)->void {
            (void)extp;
            osalSysLockFromISR();
            exttim_map[channel]->CNT = 0U;
            osalSysUnlockFromISR();
        };
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

        if (extp->state == EXT_STOP) {
            /* execute same steps as extStart() as extStartI() does not exist */
            extp->config = extconfig;
            ext_lld_start(extp);
            extp->state = EXT_ACTIVE;
        }
        exttim_map[pad] = m_gptp->tim;
        extconfig.channels[pad] = EXTChannelConfig{EXT_CH_MODE_RISING_EDGE | ext_mode_port, callback};
        extSetChannelModeI(extp, pad, &extconfig.channels[pad]);
        extChannelEnableI(extp, pad);
    }
    osalSysUnlock();
}

void Encoder::stop() {
    osalDbgCheck(m_gptp != nullptr);

    osalSysLock();
    osalDbgAssert((m_gptp->state == GPT_STOP) || (m_gptp->state == GPT_READY),
                  "invalid state");
    gpt_lld_stop(m_gptp);
    m_gptp->state = GPT_STOP;
    m_state = state_t::STOP;
    m_index = index_t::NONE;
    if (m_config.z != PAL_NOLINE) {
        uint32_t pad = PAL_PAD(m_config.z);
        exttim_map[pad] = nullptr;
        extconfig.channels[pad] = EXTChannelConfig{EXT_CH_MODE_DISABLED, nullptr};
        extSetChannelModeI(extp, pad, &extconfig.channels[pad]);
        extChannelDisableI(extp, pad);
        if (std::all_of(std::begin(extp->config->channels), std::end(extp->config->channels),
                    [](EXTChannelConfig config)->bool {
                        return ((config.mode & EXT_CH_MODE_EDGES_MASK) == EXT_CH_MODE_DISABLED);
                    })) {
            ext_lld_stop(extp);
            extp->state = EXT_STOP;
        }
    }
    osalSysUnlock();
}

void Encoder::set_count(gptcnt_t count) {
    osalDbgCheck(m_state == state_t::READY);
    if (!IS_TIM_32B_COUNTER_INSTANCE(reinterpret_cast<TIM_TypeDef*>(m_gptp->tim))) {
        count &= TIM_CNT_CNT;
    }
    m_gptp->tim->CNT = count;
}

gptcnt_t Encoder::count() volatile {
    osalDbgCheck(m_state == state_t::READY);
    return static_cast<gptcnt_t>(m_gptp->tim->CNT);
}

bool Encoder::direction() volatile {
    osalDbgCheck(m_state == state_t::READY);
    return static_cast<bool>(m_gptp->tim->CR1 & TIM_CR1_DIR);
}

const EncoderConfig& Encoder::config() {
    return m_config;
}

Encoder::state_t Encoder::state() {
    return m_state;
}

Encoder::index_t Encoder::index() volatile {
    return m_index;
}
