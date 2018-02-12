#pragma once

namespace message {

template <typename MessageType, typename AlignedMessageType, size_t N>
bool from_pool(const MessageType* msg, const std::array<AlignedMessageType, N>& pool_buffer) {
    const size_t offset = static_cast<size_t>(
            (reinterpret_cast<AlignedMessageType*>(const_cast<MessageType*>(msg)) - &pool_buffer[0]));

    return (offset % sizeof(AlignedMessageType) == 0) && (offset / sizeof(AlignedMessageType) < N);
}

template <typename AlignedMessageType, size_t N>
bool from_pool(msg_t msg, const std::array<AlignedMessageType, N>& pool_buffer) {
    const size_t offset = static_cast<size_t>(
            (reinterpret_cast<AlignedMessageType*>(msg) - &pool_buffer[0]));

    return (offset % sizeof(AlignedMessageType) == 0) && (offset / sizeof(AlignedMessageType) < N);
}

}
