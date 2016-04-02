#include "osal.h"
#include "encoder.h"

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
                !(IS_TIM_32B_COUNTER_INSTANCE(reinterpret_cast<TIM_TypeDef*>(m_gptp->tim)))),
            "invalid count");

    m_gptp->config = &m_gptconfig;
    gpt_lld_start(m_gptp); /* configure and activate peripheral */
    m_gptp->tim->SMCR = TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0; /* encoder mode 3, count on both TI1FP2 and TI2FP1 edges */
    m_gptp->tim->CCER = 0; /* rising edge polarity */
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
        /* TODO: setup external interrupt trigger */
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
    osalSysUnlock();
}

void Encoder::set_count(gptcnt_t count) {
    osalDbgCheck(m_state == state_t::READY);
    if (IS_TIM_32B_COUNTER_INSTANCE(reinterpret_cast<TIM_TypeDef*>(m_gptp->tim))) {
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
