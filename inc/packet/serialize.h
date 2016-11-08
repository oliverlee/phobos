#pragma once
#include "pb_encode.h"
#include "pb_decode.h"
#include "osal.h"

namespace packet {
namespace serialize {

/*
 * This template struct maps a nanopb message to the corresponding field struct.
 * See packet/serialize.cc for template specializations.
 */
template <typename T>
struct message_field {
    static const pb_field_t* type;
};

template <typename T>
const pb_field_t* message_field<T>::type = nullptr;

template <typename T>
uint8_t encode(const T& t, uint8_t* buffer, uint8_t buffer_size) {
    osalDbgCheck(message_field<T>::type != nullptr);
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    bool status = pb_encode(&stream, message_field<T>::type, &t);
    osalDbgCheck(status);
    return stream.bytes_written;
}

template <typename T>
bool decode(const uint8_t* buffer, T* t, uint8_t buffer_size) {
    osalDbgCheck(message_field<T>::type != nullptr);
    pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_size);
    bool status = pb_decode(&stream, message_field<T>::type, t);
    osalDbgCheck(status);
    return status;
}

} // namespace serialize
} // namespace packet
