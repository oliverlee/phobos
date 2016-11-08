#pragma once
#include <cstdint>

namespace packet {
namespace framing {

constexpr uint8_t COBS_MAX_SIZE_DATA_SET = 254;
constexpr uint8_t COBS_FRAME_STUFF_OVERHEAD = 1;
constexpr auto FRAME_STUFF_OVERHEAD = COBS_FRAME_STUFF_OVERHEAD;

void stuff(const uint8_t* source, uint8_t* dest, uint8_t source_byte_size);
void unstuff(const uint8_t* source, uint8_t* dest, uint8_t source_byte_size);

} // namespace framing
} // namespace packet
