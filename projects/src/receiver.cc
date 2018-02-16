#include "receiver.h"
#include "txrx_common.h"
#include <cstring>
#include <functional>

// This define can be useful when sizing message mailbox and memory pools
#define ASSERT_RX_MEMORY_LIMIT TRUE

namespace message {

void Receiver::usb_event(USBDriver *usbp, usbevent_t event) {
    switch (event) {
    case USB_EVENT_ADDRESS:
        return;
    case USB_EVENT_CONFIGURED:
        chSysLockFromISR();

        // Enables the endpoints specified into the configuration.
        // Note, this callback is invoked from an ISR so I-Class functions
        // must be used.
        usbInitEndpointI(usbp, USBD1_DATA_REQUEST_EP, &Receiver::ep1config);
        usbInitEndpointI(usbp, USBD1_INTERRUPT_REQUEST_EP, &ep2config);

        chSysUnlockFromISR();
        return;
    case USB_EVENT_RESET:
    case USB_EVENT_UNCONFIGURED:
    case USB_EVENT_SUSPEND:
        return;
    case USB_EVENT_WAKEUP:
        return;
    case USB_EVENT_STALLED:
        return;
    }
    return;
}

void Receiver::data_received_callback(USBDriver* usbp, usbep_t ep) {
    Receiver* self = static_cast<Receiver*>(usbp->out_params[ep - 1U]);
    chDbgAssert(self == nullptr, "Receiver is not valid.");

    chSysLockFromISR();

    // Signal that data is available
    chEvtSignalI(self->m_thread, EVENT_DATA_AVAILABLE);

    // Set number of received bytes
    self->m_packet_buffer_offset += usbGetReceiveTransactionSizeX(
            serusbcfg.usbp,
            serusbcfg.bulk_out);

    chSysUnlockFromISR();
}

void Receiver::receiver_thread_function(void* p) {
    auto self = static_cast<Receiver*>(p);

    chRegSetThreadName("receiver");

    auto start_receive = [self]() {
            chSysLock();

            // Check if the USB driver is in the correct state
            if (usbGetDriverStateI(serusbcfg.usbp) != USB_ACTIVE) {
                chSysHalt("USB driver not ready.");
            }

            usbStartReceiveI(
                    serusbcfg.usbp,
                    serusbcfg.bulk_out,
                    self->m_packet_buffer.data(),
                    ((self->m_packet_buffer.size() - self->m_packet_buffer_offset) /
                     ep1config.out_maxsize));

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
            self->decode_packet();

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

    chSysLock();
    chDbgCheck(usbGetDriverStateI(serusbcfg.usbp) == USB_STOP);

    // We need to set USBDriver in_params to be used with
    // Receiver::data_available_callback.
    serusbcfg.usbp->in_params[serusbcfg.bulk_in - 1U] = nullptr;
    serusbcfg.usbp->out_params[serusbcfg.bulk_out - 1U] = this;
    if (serusbcfg.int_in > 0U) {
        serusbcfg.usbp->in_params[serusbcfg.int_in - 1U]  = nullptr;
    }

    chSysUnlock();

    // Activate the USB driver and then the USB bus pull-up on D+.
    // Note, a delay is inserted in order to not have to disconnect the cable
    // after a reset.
    board_usb_lld_disconnect_bus();   //usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbconfig);
    board_usb_lld_connect_bus();      //usbConnectBus(serusbcfg.usbp);

    // Block USB device until ready
    while (serusbcfg.usbp->state != USB_ACTIVE) {
        chThdSleepMilliseconds(10);
    }
}

void Receiver::start(tprio_t priority) {
    chDbgAssert(priority < STM32_USB_OTG_THREAD_PRIO,
            "Receiver thread priority must be lower than STM32_USB_OTG_THREAD_PRIO");
    chDbgAssert(m_thread == nullptr, "Receiver cannot start if already running");

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
