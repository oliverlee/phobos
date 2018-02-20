#pragma once
#include "hal.h"
#include <array>

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


/**
 Define our own USBConfig object as we want to interact with the USB device
 directly for sending and receiving data instead of using the serial class
 input and output queues.

 While we still need to link with serial_usb.c, we want the linker to remove
 unused sdu functions.
 */
extern USBConfig usbcfg;
extern USBEndpointConfig usbep1cfg;

constexpr uint16_t USB_ENDPOINT_MAX_PACKET_SIZE = 0x0040;

void usbStart();
} // namespace message
