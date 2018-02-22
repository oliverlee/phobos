#pragma once
#include "ch.h"
#include "cobs.h"
#include "packet/serialize.h"
#include "txrx.pb.h"
#include <array>
#include <type_traits>
#include <algorithm>

namespace message {

class Transmitter {
    public:
        Transmitter();
        void start(tprio_t priority);

        pbSimulation* alloc_simulation_message();
        pbTxSmallPackage* alloc_smallpackage_message();

        msg_t transmit(pbSimulation* msg);
        msg_t transmit(pbTxSmallPackage* msg);

        void free(pbSimulation* msg);
        void free(pbTxSmallPackage* msg);

    private:
        struct alignas(sizeof(stkalign_t)) pbSimulation_stkalign { pbSimulation m; };
        struct alignas(sizeof(stkalign_t)) pbTxSmallPackage_stkalign { pbTxSmallPackage m; };
        static constexpr size_t A = 2;
        static constexpr size_t B = 2;
        std::array<pbSimulation_stkalign, A> m_simulation_pool_buffer;
        std::array<pbTxSmallPackage_stkalign, B> m_smallpackage_pool_buffer;

        mailbox_t m_mailbox;
        MEMORYPOOL_DECL(m_simulation_pool, sizeof(pbSimulation_stkalign), nullptr);
        MEMORYPOOL_DECL(m_smallpackage_pool, sizeof(pbTxSmallPackage_stkalign), nullptr);

        std::array<msg_t, A + B> m_mailbox_buffer;

        // Messages are delimited and prepended with a protobuf varint
        static constexpr size_t N = packet::serialize::VARINT_MAX_SIZE +
            std::max(pbSimulation_size, pbTxSmallPackage_size);
        static constexpr size_t M = cobs::max_encoded_length(N);
        std::array<uint8_t, N> m_serialize_buffer;
        std::array<uint8_t, M> m_packet_buffer;

        THD_WORKING_AREA(m_wa_transmitter_thread, 1028);
        thread_t* m_thread;

        // use uint8_t if possible
        using index_t = std::conditional<(N < 256) && (M < 256), uint8_t, size_t>::type;
        index_t m_serialized_size; // byte size of serialized object
        index_t m_encoded_size; // byte size of encoded packet

        void serialize_message(msg_t msg);
        void encode_packet();
        static void transmitter_thread_function(void* p);

        pbTxPackage m_package;
};

} // namespace message
