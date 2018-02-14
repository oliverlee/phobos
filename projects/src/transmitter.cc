#include "transmitter.h"
#include "txrx_common.h"
#include "usbconfig.h"
#include <cstring>

// This define can be useful when sizing message mailbox and memory pools
#define ASSERT_TX_MEMORY_LIMIT TRUE

namespace message {
Transmitter::Transmitter() :
m_thread(nullptr),
m_serialized_size(0),
m_encoded_size(0) {
    chMBObjectInit(
            &m_mailbox,
            m_mailbox_buffer.data(),
            m_mailbox_buffer.size());
    chPoolLoadArray(
            &m_simulation_pool,
            static_cast<void*>(m_simulation_pool_buffer.data()),
            m_simulation_pool_buffer.size());
    chPoolLoadArray(
            &m_smallgroup_pool,
            static_cast<void*>(m_smallgroup_pool_buffer.data()),
            m_smallgroup_pool_buffer.size());

    // SDU driver must already be running.
    chDbgAssert(((SDU1.config->usbp->state == USB_ACTIVE) &&
                (SDU1.state == SDU_READY)),
            "SDU1 must be running.");
}

void Transmitter::start(tprio_t priority) {
    chDbgAssert(priority < STM32_USB_OTG_THREAD_PRIO,
            "Transmitter thread priority must be lower than STM32_USB_OTG_THREAD_PRIO");
    chDbgAssert(m_thread == nullptr, "Transmitter cannot start if already running");

    m_thread = chThdCreateStatic(m_wa_transmitter_thread,
            sizeof(m_wa_transmitter_thread), priority,
            transmitter_thread_function, this);
}

pbSimulation* Transmitter::alloc_simulation_message() {
    void* msg = chPoolAlloc(&m_simulation_pool);

#if ASSERT_TX_MEMORY_LIMIT
    chDbgAssert(msg != nullptr, "Increase Transmitter pbSimulation pool size.");
#endif

    return static_cast<pbSimulation*>(msg);
}

pbSmallMessageGroup* Transmitter::alloc_smallgroup_message() {
    void* msg = chPoolAlloc(&m_smallgroup_pool);

#if ASSERT_TX_MEMORY_LIMIT
    chDbgAssert(msg != nullptr, "Increase Transmitter pbSmallMessagGroup pool size,");
#endif

    return static_cast<pbSmallMessageGroup*>(msg);
}

msg_t Transmitter::transmit(pbSimulation* msg) {
    msg_t m = reinterpret_cast<msg_t>(msg);

    chDbgAssert(from_pool(msg, m_simulation_pool_buffer),
            "'msg' pointer does not originate from Transmitter managed memory");

    msg_t status = chMBPost(&m_mailbox, m, TIME_IMMEDIATE);
#if ASSERT_TX_MEMORY_LIMIT
    chDbgAssert(status == MSG_OK, "Increase Transmitter mailbox size");
#endif

    return status;
}

msg_t Transmitter::transmit(pbSmallMessageGroup* msg) {
    msg_t m = reinterpret_cast<msg_t>(msg);

    chDbgAssert(from_pool(msg, m_smallgroup_pool_buffer),
            "'msg' pointer does not originate from Transmitter managed memory");

    msg_t status = chMBPost(&m_mailbox, m, TIME_IMMEDIATE);
#if ASSERT_TX_MEMORY_LIMIT
    chDbgAssert(status == MSG_OK, "Increase Transmitter mailbox size");
#endif

    return status;
}

void Transmitter::free(pbSimulation* msg) {
    chDbgAssert(from_pool(msg, m_simulation_pool_buffer),
            "'msg' pointer does not originate from Transmitter managed memory");

    chPoolFree(&m_simulation_pool, static_cast<void*>(msg));
}

void Transmitter::free(pbSmallMessageGroup* msg) {
    chDbgAssert(from_pool(msg, m_smallgroup_pool_buffer),
            "'msg' pointer does not originate from Transmitter managed memory");

    chPoolFree(&m_smallgroup_pool, static_cast<void*>(msg));
}

void Transmitter::serialize_message(msg_t msg) {
    m_wrapper = pbTxMaster_init_zero;

    // TODO: Should we set the timestamp when the message is created?
    m_wrapper.timestamp = chVTGetSystemTime();

    if (from_pool(msg, m_simulation_pool_buffer)) {
        pbSimulation* p = reinterpret_cast<pbSimulation*>(msg);

        m_wrapper.which_value = pbTxMaster_simulation_data_tag;
        m_wrapper.value.simulation_data = *p;

        free(p);
    } else if (from_pool(msg, m_smallgroup_pool_buffer)) {
        pbSmallMessageGroup* p = reinterpret_cast<pbSmallMessageGroup*>(msg);

        // We have reserved tag values in pbSmallMessageGroup so they match
        // those of pbTxMaster
        m_wrapper.which_value = p->which_value;
        std::memcpy(&m_wrapper.value, &p->value, sizeof(p->value));

        free(p);
    } else {
        chSysHalt("'msg' pointer does not originate from Transmitter managed memory");
    }

   m_serialized_size = packet::serialize::encode_delimited(
            m_wrapper, m_serialize_buffer.data(), m_serialize_buffer.size());
}

void Transmitter::encode_packet() {
    const cobs::EncodeResult result = cobs::encode(
            m_serialize_buffer.data(),
            m_serialized_size,
            m_packet_buffer.data(),
            m_packet_buffer.size());
    m_serialized_size = 0;

    switch (result.status) {
        case cobs::EncodeResult::Status::OK: {
            m_encoded_size = result.produced;
            return;
        }
        case cobs::EncodeResult::Status::WRITE_OVERFLOW: {
            // This shouldn't happen as we size the buffer appropriately.
            chSysHalt("cobs encoded package cannot fit in m_serialize_buffer");
        }
        default: {
            chSysHalt("Unknown EncodeResult::Status");
        }
    }
}

void Transmitter::transmitter_thread_function(void* p) {
    auto self = static_cast<Transmitter*>(p);

    chRegSetThreadName("transmitter");

    usbep_t ep = SDU1.config->bulk_in;
    USBDriver* usbp = SDU1.config->usbp;

    while (!chThdShouldTerminateX()) {
        msg_t msg;
        const msg_t rdymsg = chMBFetch(&self->m_mailbox, &msg, MS2ST(200));

        if (rdymsg == MSG_OK) {
           self->serialize_message(msg);
        } else {
            // No message available so check for terminate flag.
            continue;
        }

        self->encode_packet();

        chSysLock();

        if ((usbGetDriverStateI(usbp) != USB_ACTIVE) || (SDU1.state != SDU_READY)) {
            chSysHalt("SDU driver not ready.");
        }

        if (usbGetTransmitStatusI(usbp, ep)) {
            msg = chThdSuspendTimeoutS(
                    &usbp->epc[ep]->in_state->thread,
                    MS2ST(200));

            if (msg == MSG_TIMEOUT) {
                chSysHalt("Previous USB transmission it taking too long.");
            }
        }

        usbStartTransmitI(
                usbp,
                ep,
                self->m_packet_buffer.data(),
                self->m_encoded_size);

        chSysUnlock();
    }
}

} // namespace message
