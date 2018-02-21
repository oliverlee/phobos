#include "transmitter.h"
#include "txrx_common.h"
#include "hal.h"
#include "usbcfg.h"
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
            &m_smallpackage_pool,
            static_cast<void*>(m_smallpackage_pool_buffer.data()),
            m_smallpackage_pool_buffer.size());

    chSysLock();

    chDbgCheck(usbGetDriverStateI(serusbcfg.usbp) == USB_STOP);

    // USB driver data received callback not used
    serusbcfg.usbp->in_params[serusbcfg.bulk_in - 1U] = nullptr;

    chSysUnlock();
}

void Transmitter::start(tprio_t priority) {
    chDbgAssert(priority < STM32_USB_OTG_THREAD_PRIO,
            "Transmitter thread priority must be lower than STM32_USB_OTG_THREAD_PRIO");
    chDbgAssert(m_thread == nullptr, "Transmitter cannot start if already running");

    usbStart(); // start USB device if not already active

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

pbTxSmallPackage* Transmitter::alloc_smallpackage_message() {
    void* msg = chPoolAlloc(&m_smallpackage_pool);

#if ASSERT_TX_MEMORY_LIMIT
    chDbgAssert(msg != nullptr, "Increase Transmitter pbSmallMessagGroup pool size,");
#endif

    return static_cast<pbTxSmallPackage*>(msg);
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

msg_t Transmitter::transmit(pbTxSmallPackage* msg) {
    msg_t m = reinterpret_cast<msg_t>(msg);

    chDbgAssert(from_pool(msg, m_smallpackage_pool_buffer),
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

void Transmitter::free(pbTxSmallPackage* msg) {
    chDbgAssert(from_pool(msg, m_smallpackage_pool_buffer),
            "'msg' pointer does not originate from Transmitter managed memory");

    chPoolFree(&m_smallpackage_pool, static_cast<void*>(msg));
}

void Transmitter::serialize_message(msg_t msg) {
    m_package = pbTxPackage_init_zero;

    // TODO: Should we set the timestamp when the message is created?
    m_package.timestamp = chVTGetSystemTime();

    if (from_pool(msg, m_simulation_pool_buffer)) {
        pbSimulation* p = reinterpret_cast<pbSimulation*>(msg);

        m_package.which_value = pbTxPackage_simulation_data_tag;
        m_package.value.simulation_data = *p;

        free(p);
    } else if (from_pool(msg, m_smallpackage_pool_buffer)) {
        pbTxSmallPackage* p = reinterpret_cast<pbTxSmallPackage*>(msg);

        // We have reserved tag values in pbTxSmallPackage so they match
        // those of pbTxPackage
        m_package.which_value = p->which_value;
        std::memcpy(&m_package.value, &p->value, sizeof(p->value));

        free(p);
    } else {
        chSysHalt("'msg' pointer does not originate from Transmitter managed memory");
    }

   m_serialized_size = packet::serialize::encode_delimited(
            m_package, m_serialize_buffer.data(), m_serialize_buffer.size());
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
    Transmitter* txp = static_cast<Transmitter*>(p);

    chRegSetThreadName("transmitter");

    usbep_t ep = serusbcfg.bulk_in;
    USBDriver* usbp = serusbcfg.usbp;

    while (!chThdShouldTerminateX()) {
        msg_t msg;
        const msg_t rdymsg = chMBFetch(&txp->m_mailbox, &msg, MS2ST(200));

        if (rdymsg == MSG_OK) {
           txp->serialize_message(msg);
        } else {
            // No message available so check for terminate flag.
            continue;
        }

        txp->encode_packet();

        chSysLock();

        if (usbGetDriverStateI(usbp) != USB_ACTIVE) {
            chSysHalt("USB driver not ready.");
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
                txp->m_packet_buffer.data(),
                txp->m_encoded_size);

        chSysUnlock();
    }
}

} // namespace message
