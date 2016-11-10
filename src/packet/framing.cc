#include "packet/framing.h"
#include "osal.h"

/*
 * Packet framing uses the Consistent Overhead Byte Stuffing algorithm for
 * encoding data bytes. The maximum length for each packet is limited to 254 bytes.
 * Refer to: https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
 */

namespace packet {
namespace framing {

uint8_t stuff(const void* source, void* dest, uint8_t source_byte_size) {
    osalDbgCheck(source != nullptr);
    osalDbgCheck(dest != nullptr);
    osalDbgAssert(source_byte_size <= COBS_MAX_SIZE_DATA_SET,
            "source data exceeds allowed frame algorithm size");

    const char* source_ = static_cast<const char*>(source);
    char* dest_ = static_cast<char*>(dest);

    const char* end = source_ + source_byte_size;
    char* code_p = dest_++; /* pointer to write code */
    uint8_t code = 0x01;

    auto write_code = [&]() {
        *code_p = code;
        code_p = dest_++;
        code = 0x01;
    };

    while (source_ < end) {
        if (*source_ == 0) { /* handle zero byte in data */
            write_code();
        } else { /* handle non-zero byte */
            *dest_++ = *source_;
            if (++code == 0xff) { /* reached max COBS data size */
                write_code();
            }
        }
        ++source_;
    }
    write_code();
    *dest_ = COBS_PACKET_DELIMITER;
    return source_byte_size + PACKET_OVERHEAD;
}

uint8_t unstuff(const void* source, void* dest, uint8_t source_byte_size) {
    osalDbgCheck(source != nullptr);
    osalDbgCheck(dest != nullptr);
    // osalDbgAssert(source_byte_size <= COBS_MAX_SIZE_DATA_SET + 1,
    // "invalid data size"); always true due to data type

    const char* source_ = static_cast<const char*>(source);
    char* dest_ = static_cast<char*>(dest);

    const char* end = source_ + source_byte_size;
    while (source_ < end) {
        uint8_t code = *source_++;

        if (code == 0) {
            osalDbgAssert(dest_ != static_cast<char*>(dest), "zero byte input");
            break;
        }
        osalDbgAssert((code - 1) <= (end - source_), "input code too small or source buffer too small");
        for (uint8_t i = 1; i < code; ++i) {
            *dest_++ = *source_++;
        }
        if (code < 0xff) { /* don't write the end-of-packet zero if data length = 0xff */
            *dest_++ = 0;
        }
    }
    return source_byte_size - PACKET_OVERHEAD;
}

uint8_t unstuff(const void* source, void* dest) {
    osalDbgCheck(source != nullptr);
    osalDbgCheck(dest != nullptr);

    const char* source_ = static_cast<const char*>(source);
    char* dest_ = static_cast<char*>(dest);

    while (true) {
        uint8_t code = *source_++;

        if (code == 0) {
            osalDbgAssert(dest_ != static_cast<char*>(dest), "zero byte input");
            break;
        }
        osalDbgAssert((code - 1) <=
                (COBS_MAX_SIZE_DATA_SET - (dest_ - static_cast<char*>(dest) - 1)),
                "input code too small or source buffer too small");
        for (uint8_t i = 1; i < code; ++i) {
            *dest_++ = *source_++;
        }
        if (code < 0xff) { /* don't write the end-of-packet zero if data length = 0xff */
            *dest_++ = 0;
        }
    }
    return dest_ - static_cast<char*>(dest) - 1;
}

} // namespace framing
} // namespace packet
