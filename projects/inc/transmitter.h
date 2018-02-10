#pragma once
#include <array>
#include "cobs.h"
#include "ch.h"
#include "simulation.pb.h"

namespace message {
class Transmitter {
    public:
        Transmitter();
        void start(tprio_t priority);

        BicyclePoseMessage* alloc_pose_message();
        void free_message(BicyclePoseMessage* msg);
        msg_t transmit_async(BicyclePoseMessage* msg); // frees msg on MSG_OK

        SimulationMessage* alloc_simulation_message();
        void free_message(SimulationMessage* msg);
        msg_t transmit_async(SimulationMessage* msg); // frees msg on MSG_OK

        // TODO: REMOVE after config message is defined
        void transmit(SimulationMessage* msg); // frees msg

    private:
        static constexpr size_t POSE_MESSAGE_POOL_SIZE = 2;
        static constexpr size_t SIMULATION_MESSAGE_POOL_SIZE = 2;
        static constexpr size_t MAILBOX_SIZE = POSE_MESSAGE_POOL_SIZE + SIMULATION_MESSAGE_POOL_SIZE;

        struct alignas(sizeof(stkalign_t)) BicyclePoseMessage_stkalign { BicyclePoseMessage m; };
        struct alignas(sizeof(stkalign_t)) SimulationMessage_stkalign { SimulationMessage m; };

        mailbox_t m_message_mailbox;
        MEMORYPOOL_DECL(m_pose_message_pool, sizeof(BicyclePoseMessage_stkalign), nullptr);
        MEMORYPOOL_DECL(m_simulation_message_pool, sizeof(SimulationMessage_stkalign), nullptr);

        msg_t m_message_mailbox_buffer[MAILBOX_SIZE];
        BicyclePoseMessage_stkalign m_pose_message_buffer[POSE_MESSAGE_POOL_SIZE];
        SimulationMessage_stkalign m_simulation_message_buffer[SIMULATION_MESSAGE_POOL_SIZE];

        static constexpr size_t VARINT_MAX_SIZE = 10;
        std::array<uint8_t, sizeof(SimulationMessage) + VARINT_MAX_SIZE> m_serialize_buffer;
        std::array<uint8_t, cobs::max_encoded_length(sizeof(SimulationMessage) + VARINT_MAX_SIZE)> m_packet_buffer;
        THD_WORKING_AREA(m_wa_transmitter_thread, 1280);
        thread_t* m_thread;
        size_t m_bytes_written;

        void encode_message(const BicyclePoseMessage* const msg);
        void encode_message(const SimulationMessage* const msg);
        void transmit_packet() const;
        size_t encode_packet(const SimulationMessage& m);
        static void transmitter_thread_function(void* p);

        bool is_within_pose_message_memory(msg_t msg);
        bool is_within_simulation_message_memory(msg_t msg);
};
} // namespace message
