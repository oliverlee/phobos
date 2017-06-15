#include "transmitter.h"
#include "hal.h"
#include "packet/serialize.h"
#include "usbconfig.h"

// This define can be useful when sizing message mailbox and memory pools
#define ASSERT_MESSAGE_MEMORY_LIMIT FALSE

namespace message {
Transmitter::Transmitter() :
m_thread(nullptr),
m_bytes_written(0) {
    chMBObjectInit(&m_message_mailbox, m_message_mailbox_buffer, MAILBOX_SIZE);
    chPoolObjectInit(&m_pose_message_pool, sizeof(BicyclePoseMessage), nullptr);
    chPoolLoadArray(&m_pose_message_pool, static_cast<void*>(m_pose_message_buffer), POSE_MESSAGE_POOL_SIZE);
    chPoolObjectInit(&m_simulation_message_pool, sizeof(SimulationMessage), nullptr);
    chPoolLoadArray(&m_simulation_message_pool, static_cast<void*>(m_simulation_message_buffer), SIMULATION_MESSAGE_POOL_SIZE);

    // Initialize a serial-over-USB CDC driver.
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    // Activate the USB driver and then the USB bus pull-up on D+.
    // Note, a delay is inserted in order to not have to disconnect the cable
    // after a reset.
    board_usb_lld_disconnect_bus();   //usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    board_usb_lld_connect_bus();      //usbConnectBus(serusbcfg.usbp);

    // Block USB device until ready
    while (!((SDU1.config->usbp->state == USB_ACTIVE) && (SDU1.state == SDU_READY))) {
        chThdSleepMilliseconds(10);
    }
}

void Transmitter::start(tprio_t priority) {
    m_thread = chThdCreateStatic(m_wa_transmitter_thread,
            sizeof(m_wa_transmitter_thread), priority,
            transmitter_thread_function, this);
}

BicyclePoseMessage* Transmitter::alloc_pose_message() {
    void* msg = chPoolAlloc(&m_pose_message_pool);
#if ASSERT_MESSAGE_MEMORY_LIMIT
    chDbgAssert(msg != nullptr, "Increase transmitter POSE_MESSAGE_POOL_SIZE");
#endif
    return static_cast<BicyclePoseMessage*>(msg);
}

void Transmitter::free_pose_message(BicyclePoseMessage* msg) {
    chPoolFree(&m_pose_message_pool, static_cast<void*>(msg));
}

msg_t Transmitter::transmit_async(BicyclePoseMessage* msg) {
    msg_t status = chMBPost(&m_message_mailbox, reinterpret_cast<msg_t>(msg), TIME_IMMEDIATE);
#if ASSERT_MESSAGE_MEMORY_LIMIT
    chDbgAssert(status == MSG_OK, "Increase transmitter MAILBOX_SIZE");
#endif
    return status;
}

SimulationMessage* Transmitter::alloc_simulation_message() {
    void* msg = chPoolAlloc(&m_simulation_message_pool);
#if ASSERT_MESSAGE_MEMORY_LIMIT
    chDbgAssert(msg != nullptr, "Increase transmitter SIMULATION_MESSAGE_POOL_SIZE");
#endif
    return static_cast<SimulationMessage*>(msg);
}

void Transmitter::free_simulation_message(SimulationMessage* msg) {
    chPoolFree(&m_simulation_message_pool, static_cast<void*>(msg));
}

msg_t Transmitter::transmit_async(SimulationMessage* msg) {
    msg_t status = chMBPost(&m_message_mailbox, reinterpret_cast<msg_t>(msg), TIME_IMMEDIATE);
#if ASSERT_MESSAGE_MEMORY_LIMIT
    chDbgAssert(status == MSG_OK, "Increase transmitter MAILBOX_SIZE");
#endif
    return status;
}

void Transmitter::transmit(SimulationMessage* msg) {
    if (m_thread != nullptr) {
        chSysHalt("Synchronization is not implemented and this function is not thread safe");
    }

    encode_message(reinterpret_cast<msg_t>(msg));
    transmit_packet();
    free_simulation_message(msg);
}

void Transmitter::encode_message(msg_t msg) {
    // For now, we send a simulation message but this needs to be fixed later on.
    // TODO: Presend protobuf tag. See https://github.com/oliverlee/phobos/issues/181#issuecomment-301825244
    m_bytes_written = 0;
    if ((msg % sizeof(stkalign_t)) == 0) {
        if ((reinterpret_cast<BicyclePoseMessage*>(msg) >= m_pose_message_buffer) &&
            (reinterpret_cast<BicyclePoseMessage*>(msg) <= &m_pose_message_buffer[POSE_MESSAGE_POOL_SIZE])) {
            SimulationMessage sim_msg = SimulationMessage_init_zero;
            sim_msg.timestamp = 0;
            sim_msg.pose = *reinterpret_cast<BicyclePoseMessage*>(msg);
            sim_msg.has_pose = true;
            free_pose_message(reinterpret_cast<BicyclePoseMessage*>(msg));
            m_bytes_written = encode_packet(sim_msg);
        } else if (
                (reinterpret_cast<SimulationMessage*>(msg) >= m_simulation_message_buffer) &&
                (reinterpret_cast<SimulationMessage*>(msg) <= &m_simulation_message_buffer[SIMULATION_MESSAGE_POOL_SIZE])) {
            m_bytes_written = encode_packet(*reinterpret_cast<SimulationMessage*>(msg));
            free_simulation_message(reinterpret_cast<SimulationMessage*>(msg));
        } else {
            chDbgAssert(false, "msg pointer does not originate from a valid memory pool");
        }
    } else {
        // memory pool objects are always stack aligned
        chDbgAssert(false, "msg pointer is not stack aligned");
    }
}

void Transmitter::transmit_packet() const {
    // TODO: Change usbTransmit to usbStartTransmitI and encode during USB transmission?
    //       This may result in a single USB transmission buffer containing more than one packet.
    usbTransmit(SDU1.config->usbp, SDU1.config->bulk_in, m_packet_buffer.data(), m_bytes_written);
}

size_t Transmitter::encode_packet(const SimulationMessage& m) {
   const size_t serialize_buffer_len = packet::serialize::encode_delimited(
       m, m_serialize_buffer.data(), m_serialize_buffer.size());

   const cobs::EncodeResult encode_result = cobs::encode(
       m_serialize_buffer.data(), serialize_buffer_len, m_packet_buffer.data(), m_packet_buffer.size());

   // Encoding only fails when the destination buffer is too small, this
   // should not happen as long as we only encode SimulationMessage objects
   // because we allocated m_packet_buffer based on its size.
   chDbgAssert(encode_result.status == cobs::EncodeResult::Status::OK, "Expected encoding to succeed.");

   return encode_result.produced;
}

void Transmitter::transmitter_thread_function(void* p) {
    auto self = static_cast<Transmitter*>(p);

    chRegSetThreadName("transmitter");
    while (!chThdShouldTerminateX()) {
        msg_t msg = reinterpret_cast<msg_t>(nullptr);
        chMBFetch(&self->m_message_mailbox, &msg, TIME_INFINITE);
        self->encode_message(msg);
        self->transmit_packet();
    }
}
} // namespace message
