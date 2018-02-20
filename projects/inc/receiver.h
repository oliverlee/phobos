#pragma once
#include "hal.h"
#include "ch.h"
#include "cobs.h"
#include "packet/serialize.h"
#include "utility.h"
#include "txrx.pb.h"
#include "txrx_common.h"
#include <array>
#include <type_traits>

namespace message {

class Receiver {
    public:
        struct Result {
            /**
             MSG_OK         if a message has been correctly fetched.
             MSG_RESET      if the receiver has been reset.
             MSG_TIMEOUT    if the operation has timed out.
             */
            const msg_t status;

            /**
             If status is MSG_OK, msg points a valid pbRxMaster object.
             Otherwise msg is set to 0.

             The user must free the object after using it.
             */
            const pbRxMaster* const msg;
        };

        Receiver();
        void start(tprio_t priority);

        const Result get(systime_t time);

        void free(const pbRxMaster* msg);

        static void data_received_callback(USBDriver* usbp, usbep_t ep);

    private:
        static void receiver_thread_function(void* p);
        static constexpr eventmask_t EVENT_DATA_AVAILABLE = 1;

        struct alignas(sizeof(stkalign_t)) pbRxMaster_stkalign { pbRxMaster m; };
        static constexpr size_t A = 10;
        std::array<pbRxMaster_stkalign, A> m_receive_pool_buffer;

        mailbox_t m_mailbox;
        MEMORYPOOL_DECL(m_receive_pool, sizeof(pbRxMaster_stkalign), nullptr);

        std::array<msg_t, A> m_mailbox_buffer;

        // Messages are delimited and prepended with a protobuf varint
        static constexpr size_t N = pbRxMaster_size + packet::serialize::VARINT_MAX_SIZE;
        std::array<uint8_t, N> m_deserialize_buffer;

        static constexpr size_t M = static_cast<size_t>(USB_ENDPOINT_MAX_PACKET_SIZE) *
            util::integer_ceil(
                    2*cobs::max_encoded_length(N),
                    static_cast<size_t>(USB_ENDPOINT_MAX_PACKET_SIZE));
        std::array<uint8_t, M> m_packet_buffer;

        THD_WORKING_AREA(m_wa_receiver_thread, 256);
        thread_t* m_thread;

        // use uint8_t if possible
        using index_t = std::conditional<(N < 256) && (M < 256), uint8_t, size_t>::type;
        index_t m_packet_buffer_offset; // bytes available to decode

        void deserialize_message(index_t message_size);
        void decode_packet();
};

} // namespace message
