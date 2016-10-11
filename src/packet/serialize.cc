#include "packet/serialize.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "osal.h"

namespace packet {
namespace serialize {

uint8_t encode_bicycle_pose(const BicyclePose& bp, uint8_t* buffer, uint8_t buffer_size) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    bool status = pb_encode(&stream, BicyclePose_fields, &bp);
    osalDbgCheck(status);
    return stream.bytes_written;
}

bool decode_bicycle_pose(const uint8_t* buffer, BicyclePose* bp, uint8_t buffer_size) {
    pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_size);
    bool status = pb_decode(&stream, BicyclePose_fields, bp);
    osalDbgCheck(status);
    return status;
}

} // namespace serialize
} // namespace packet
