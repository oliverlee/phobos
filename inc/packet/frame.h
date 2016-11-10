#pragma once
#include <cstdint>

/*
 * Packet framing uses the Consistent Overhead Byte Stuffing algorithm for
 * encoding data bytes. The maximum length for each packet is limited to 254 bytes.
 * Refer to: https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
 */

namespace packet {
namespace frame {

constexpr uint8_t COBS_MAX_SIZE_DATA_SET = 254;
constexpr uint8_t COBS_PACKET_FRAME_OVERHEAD = 1;
constexpr uint8_t COBS_PACKET_DELIMITER_SIZE = 1;
constexpr uint8_t COBS_PACKET_OVERHEAD = COBS_PACKET_FRAME_OVERHEAD + COBS_PACKET_DELIMITER_SIZE;
constexpr uint8_t COBS_PACKET_DELIMITER = 0;

constexpr auto PACKET_FRAME_OVERHEAD = COBS_PACKET_FRAME_OVERHEAD;
constexpr auto PACKET_DELIMITER_SIZE = COBS_PACKET_DELIMITER_SIZE;
constexpr auto PACKET_OVERHEAD = PACKET_FRAME_OVERHEAD + PACKET_DELIMITER_SIZE;
constexpr auto PACKET_DELIMITER = COBS_PACKET_DELIMITER;

/*
 * Returns number of bytes in dest buffer after stuffing,
 * including packet frame overhead and delimiter.
 */
uint8_t stuff(const void* source, void* dest, uint8_t source_byte_size);

/*
 * Returns number of bytes in dest buffer after unstuffing.
 * Requires a priori knowledge of number of bytes in source.
 */
uint8_t unstuff(const void* source, void* dest, uint8_t source_byte_size);

/*
 * Returns number of bytes in dest buffer after unstuffing.
 * Assumes source is valid: number of bytes <= COBS_MAX_SIZE_DATA_SET + COBS_PACKET_OVERHEAD.
 */
uint8_t unstuff(const void* source, void* dest);

} // namespace frame
} // namespace packet
