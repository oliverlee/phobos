#include "encoder.h"
#include <array>
#include "osal.h"
#if HAL_USE_GPT
#if HAL_USE_EXT
#include "extconfig.h"
#endif /* HAL_USE_EXT */

#if HAL_USE_EXT
namespace {
    EXTDriver* extp = &EXTD1;
    std::array<Encoder*, EXT_MAX_CHANNELS> extenc_map{{}}; /* initialized to nullptr */
} // namespace
#endif /* HAL_USE_EXT */

Encoder::Encoder(GPTDriver* gptp, const EncoderConfig& config) :
    m_gptp(gptp),
    m_gptconfig{STM32_TIMCLK1, nullptr, 0U, 0U},
    m_config(config),
    m_state(state_t::STOP),
    m_index(index_t::NONE) {
#if defined(GPTD1)
    if (gptp == &GPTD1) {
        m_gptconfig.frequency = STM32_TIMCLK2;
    }
#endif
#if defined(GPTD8)
    if (gptp == &GPTD8) {
        m_gptconfig.frequency = STM32_TIMCLK2;
    }
#endif
#if defined(GPTD9)
    if (gptp == &GPTD9) {
        m_gptconfig.frequency = STM32_TIMCLK2;
    }
#endif
#if defined(GPTD11)
    if (gptp == &GPTD11) {
        m_gptconfig.frequency = STM32_TIMCLK2;
    }
#endif
}

void Encoder::start() {
    osalDbgCheck(m_gptp != nullptr);

    osalSysLock();
    osalDbgAssert((m_gptp->state == GPT_STOP) || (m_gptp->state == GPT_READY), "invalid state");
    osalDbgAssert(((m_config.counts_per_rev <= TIM_CNT_CNT) ||
                IS_TIM_32B_COUNTER_INSTANCE(reinterpret_cast<TIM_TypeDef*>(m_gptp->tim))),
            "invalid count");

    m_gptp->config = &m_gptconfig;
    gpt_lld_start(m_gptp); /* configure and activate peripheral */
    m_gptp->tim->SMCR = TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0; /* encoder mode 3, count on both TI1FP2 and TI2FP1 edges */
    m_gptp->tim->CCER = 0U; /* rising edge polarity */
    m_gptp->tim->CCMR1 = TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC1S_0; /* no prescaler, IC2 -> TI2, IC1 -> TI1 */
    uint16_t filter = static_cast<uint16_t>(m_config.filter);
    m_gptp->tim->CCMR1 |= (filter << 12) | (filter << 4); /* set input capture filters */
    gpt_lld_start_timer(m_gptp, m_config.counts_per_rev); /* set ARR and start counting */

    m_gptp->state = GPT_READY;
    m_state = state_t::READY;
    if (m_config.z == PAL_NOLINE) {
        m_index = index_t::NONE;
    } else {
        m_index = index_t::NOTFOUND;
#if HAL_USE_EXT
        uint32_t pad = PAL_PAD(m_config.z);
        stm32_gpio_t* port = PAL_PORT(m_config.z);
        osalDbgAssert((extp->state == EXT_STOP) || (extp->state == EXT_ACTIVE),
                "invalid state");
        osalDbgAssert(((extp->state == EXT_STOP) || /* check if channel is disabled only when EXT_ACTIVE */
                    ((extp->config->channels[pad].mode & EXT_CH_MODE_EDGES_MASK) == EXT_CH_MODE_DISABLED)),
                "channel already in use");
        osalDbgAssert(((port == GPIOA) || (port == GPIOB) || (port == GPIOC) ||
                    (port == GPIOD) || (port == GPIOE) || (port == GPIOF) ||
                    (port == GPIOG) || (port == GPIOH) || (port == GPIOI)),
                "invalid port"); /* port is available in STM32/LLD/EXTIv1 driver */
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
            extp->config = &extconfig;
            ext_lld_start(extp);
            extp->state = EXT_ACTIVE;
        }
        extenc_map[pad] = this;
        extconfig.channels[pad] = EXTChannelConfig{EXT_CH_MODE_RISING_EDGE | ext_mode_port, callback};
        extSetChannelModeI(extp, pad, &extconfig.channels[pad]);
        extChannelEnableI(extp, pad);
#endif
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
#if HAL_USE_EXT
    if (m_config.z != PAL_NOLINE) {
        extChannelDisableClearModeI(extp, PAL_PAD(m_config.z));
        extStopIfChannelsDisabledI(extp);
    }
#endif
    osalSysUnlock();
}

void Encoder::set_count(gptcnt_t count) {
    osalDbgCheck(m_state == state_t::READY);
    if (!IS_TIM_32B_COUNTER_INSTANCE(reinterpret_cast<TIM_TypeDef*>(m_gptp->tim))) {
        count &= TIM_CNT_CNT;
    }
    m_gptp->tim->CNT = count;
}

gptcnt_t Encoder::count() const volatile {
    osalDbgCheck(m_state == state_t::READY);
    return static_cast<gptcnt_t>(m_gptp->tim->CNT);
}

bool Encoder::direction() const volatile {
    osalDbgCheck(m_state == state_t::READY);
    return static_cast<bool>(m_gptp->tim->CR1 & TIM_CR1_DIR);
}

const EncoderConfig& Encoder::config() const {
    return m_config;
}

Encoder::state_t Encoder::state() const {
    return m_state;
}

Encoder::index_t Encoder::index() const volatile {
    return m_index;
}

#if HAL_USE_EXT
void Encoder::callback(EXTDriver* extp, expchannel_t channel) {
    (void)extp;
    osalSysLockFromISR();
    Encoder* enc = extenc_map[channel];
    enc->m_gptp->tim->CNT = 0U;
    enc->m_index = index_t::FOUND;
    extChannelDisableClearModeI(extp, PAL_PAD(enc->m_config.z));
    osalSysUnlockFromISR();
};
#endif /* HAL_USE_EXT */
#endif /* HAL_USE_GPT */
