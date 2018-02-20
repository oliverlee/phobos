#include "receiver.h"
#include "usbcfg.h"
#include <cstring>

// This define can be useful when sizing message mailbox and memory pools
#define ASSERT_RX_MEMORY_LIMIT TRUE

namespace message {

void Receiver::data_received_callback(USBDriver* usbp, usbep_t ep) {
    Receiver* rxp = static_cast<Receiver*>(usbp->out_params[ep - 1U]);
    chDbgAssert(rxp != nullptr, "Receiver is not valid.");

    chSysLockFromISR();

    // Signal that data is available
    chEvtSignalI(rxp->m_thread, EVENT_DATA_AVAILABLE);

    // Set number of received bytes
    rxp->m_packet_buffer_offset += usbGetReceiveTransactionSizeX(
            serusbcfg.usbp,
            serusbcfg.bulk_out);

    chSysUnlockFromISR();
}

void Receiver::receiver_thread_function(void* p) {
    Receiver* rxp = static_cast<Receiver*>(p);

    chRegSetThreadName("receiver");

    auto start_receive = [rxp]() {
            chSysLock();

            // Check if the USB driver is in the correct state
            if (usbGetDriverStateI(serusbcfg.usbp) != USB_ACTIVE) {
                chSysHalt("USB driver not ready.");
            }

            usbStartReceiveI(
                    serusbcfg.usbp,
                    serusbcfg.bulk_out,
                    rxp->m_packet_buffer.data(),
                    ((rxp->m_packet_buffer.size() - rxp->m_packet_buffer_offset) /
                     USB_ENDPOINT_MAX_PACKET_SIZE));

            chSysUnlock();
    };

    start_receive();

    while(!chThdShouldTerminateX()) {
        // Wait for data available event
        // Timeout is used to periodically check if thread should terminate.
        const eventmask_t event = chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(200));

        if (event == 0) { // timeout
            continue;
        }

        if (event & EVENT_DATA_AVAILABLE) {
            rxp->decode_packet();

            start_receive();
        } else {
            chSysHalt("Unhandled thread event.");
        }
    }
}

Receiver::Receiver() :
m_thread(nullptr),
m_packet_buffer_offset(0) {
    chMBObjectInit(
            &m_mailbox,
            m_mailbox_buffer.data(),
            m_mailbox_buffer.size());
    chPoolLoadArray(
            &m_receive_pool,
            static_cast<void*>(m_receive_pool_buffer.data()),
            m_receive_pool_buffer.size());

    // Here we use a SerialUSB driver to receive data. Ideally the user
    // would be able to use another device as well (e.g. Serial and UARTx)
    // The initialization, start, callback, and thread functions would have to be
    // updated to handle alternate devices.
    //
    // While the SerialUSB driver is used, data sending and receiving is
    // handled by interacting with the USB driver directly to avoid delays
    // associated with buffering and reduce data copying.
    chSysLock();

    chDbgCheck(usbGetDriverStateI(serusbcfg.usbp) == USB_STOP);

    // We need to set USBDriver in_params to be used with
    // Receiver::data_available_callback.
    serusbcfg.usbp->out_params[serusbcfg.bulk_out - 1U] = this;
    if (serusbcfg.int_in > 0U) {
        serusbcfg.usbp->in_params[serusbcfg.int_in - 1U]  = this;
    }

    chSysUnlock();
}

void Receiver::start(tprio_t priority) {
    chDbgAssert(priority < STM32_USB_OTG_THREAD_PRIO,
            "Receiver thread priority must be lower than STM32_USB_OTG_THREAD_PRIO");
    chDbgAssert(m_thread == nullptr, "Receiver cannot start if already running");

    usbStart(); // start USB device if not already active

    m_thread = chThdCreateStatic(m_wa_receiver_thread,
            sizeof(m_wa_receiver_thread), priority,
            receiver_thread_function, this);
}

const Receiver::Result Receiver::get(systime_t time) {
    msg_t msg = 0;
    const msg_t rdymsg = chMBFetch(&m_mailbox, &msg, time);

    if ((rdymsg == MSG_TIMEOUT) || (rdymsg == MSG_RESET)) {
        return Result { rdymsg, reinterpret_cast<pbRxMaster*>(msg) };
    }

    return Result { rdymsg, reinterpret_cast<pbRxMaster*>(msg) };
}

void Receiver::free(const pbRxMaster* msg) {
    chDbgAssert(from_pool(msg, m_receive_pool_buffer),
            "'msg' pointer does not originate from Receiver managed memory");

    chPoolFree(&m_receive_pool, static_cast<void*>(const_cast<pbRxMaster*>(msg)));
}

void Receiver::deserialize_message(index_t message_size) {
    void* msg = chPoolAlloc(&m_receive_pool);

    chDbgAssert(from_pool(msg, m_receive_pool_buffer),
            "'msg' pointer does not originate from Receiver managed memory");

    if (packet::serialize::decode_delimited(m_deserialize_buffer.data(),
                static_cast<pbRxMaster*>(msg),
                message_size)) {
        if (chMBPost(&m_mailbox, reinterpret_cast<msg_t>(msg), TIME_IMMEDIATE) == MSG_OK) {
            return;
#if ASSERT_RX_MEMORY_LIMIT
        } else {
            chSysHalt("Increase Receiver mailbox size.");
#endif
        }

#if ASSERT_RX_MEMORY_LIMIT
    } else {
        chSysHalt("Increase Receiver pool size.");
#endif
    }

    // Free the message we just allocated since deserialization or posting failed.
    chPoolFree(&m_receive_pool, msg);
}


void Receiver::decode_packet() {
    index_t offset = 0;

    while (m_packet_buffer_offset > offset) {
        const cobs::DecodeResult result = cobs::decode(
                m_packet_buffer.data() + offset,
                m_packet_buffer_offset,
                m_deserialize_buffer.data(),
                m_deserialize_buffer.size());

        switch (result.status) {
            case cobs::DecodeResult::Status::OK: {
                // We could return here and deserialize in different function
                // which would allow us to start receiving again immediately.
                // However, we would have no way to tell if another complete
                // packet is contained within the packet buffer.
                deserialize_message(result.produced);
                offset += result.consumed;
                continue;
            }
            case cobs::DecodeResult::Status::WRITE_OVERFLOW: {
                // Package too large to decode, possibly due to packet loss.
                break;
            }
            case cobs::DecodeResult::Status::READ_OVERFLOW: {
                // Check if packet buffer is full
                chDbgCheck(m_packet_buffer_offset >= offset);

                // If index_t is uint8_t, we need to promote to do math.
                // Automatic promotion will convert to int.
                if ((static_cast<size_t>(m_packet_buffer_offset) - static_cast<size_t>(offset))
                        > (m_packet_buffer.size()/2U)) {
                    // Possibly receiving messages it should not
                    chDbgAssert(false,
                            "cobs encoded package cannot fit in m_packet_buffer");

                    // Ignore all encoded data and wait for next packet.
                    offset = 0;
                    m_packet_buffer_offset = 0;
                }

                // Otherwise wait for more data
                break;
            }
            case cobs::DecodeResult::Status::UNEXPECTED_ZERO: {
                // Resume decoding at next byte
                offset += result.consumed;
                continue;
            }
            default: {
                chSysHalt("Unknown DecodeResult::Status");
            }
        }
    }

    // Shove remaining bytes to start of packet buffer
    chDbgCheck(m_packet_buffer_offset >= offset);
    std::memmove(
            m_packet_buffer.data(),
            m_packet_buffer.data() + offset,
            m_packet_buffer_offset - offset);
    m_packet_buffer_offset -= offset;
}

} //namespace message
