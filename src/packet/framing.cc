#include "packet/framing.h"
#include "osal.h"

/*
 * Packet framing uses the Consistent Overhead Byte Stuffing algorithm for
 * encoding data bytes. The maximum length for each packet is limited to 254 bytes.
 * Refer to: https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
 */

namespace packet {
namespace framing {

uint8_t stuff(const uint8_t* source, uint8_t* dest, uint8_t source_byte_size) {
    osalDbgCheck(source != nullptr);
    osalDbgCheck(dest != nullptr);
    osalDbgAssert(source_byte_size <= COBS_MAX_SIZE_DATA_SET,
            "source data exceeds allowed frame algorithm size");

    const uint8_t* end = source + source_byte_size;
    uint8_t* code_p = dest++; /* pointer to write code */
    uint8_t code = 0x01;

    auto write_code = [&]() {
        *code_p = code;
        code_p = dest++;
        code = 0x01;
    };

    while (source < end) {
        if (*source == 0) { /* handle zero byte in data */
            write_code();
        } else { /* handle non-zero byte */
            *dest++ = *source;
            if (++code == 0xff) { /* reached max COBS data size */
                write_code();
            }
        }
        ++source;
    }
    write_code();
    *dest = COBS_PACKET_DELIMITER;
    return source_byte_size + PACKET_OVERHEAD;
}

uint8_t unstuff(const uint8_t* source, uint8_t* dest, uint8_t source_byte_size) {
    osalDbgCheck(source != nullptr);
    osalDbgCheck(dest != nullptr);
    // osalDbgAssert(source_byte_size <= COBS_MAX_SIZE_DATA_SET + 1,
    // "invalid data size"); always true due to data type

    const uint8_t* end = source + source_byte_size;
    const uint8_t* start = dest;
    while (source < end) {
        uint8_t code = *source++;

        osalDbgAssert(code != 0, "zero byte input");
        osalDbgAssert((code - 1) <= (end - source), "input code too small or source buffer too small");
        for (uint8_t i = 1; i < code; ++i) {
            *dest++ = *source++;
        }
        if (code < 0xff) { /* don't write the end-of-packet zero if data length = 0xff */
            *dest++ = 0;
        }
    }
    return dest - start;
}

uint8_t unstuff(const uint8_t* source, uint8_t* dest) {
    osalDbgCheck(source != nullptr);
    osalDbgCheck(dest != nullptr);

    const uint8_t* start = dest;
    while (true) {
        uint8_t code = *source++;

        if (code == 0) {
            osalDbgAssert(start == dest, "zero byte input");
            break;
        }
        osalDbgAssert((code - 1) <= (COBS_MAX_SIZE_DATA_SET - (dest - start)),
                "input code too small or source buffer too small");
        for (uint8_t i = 1; i < code; ++i) {
            *dest++ = *source++;
        }
        if (code < 0xff) { /* don't write the end-of-packet zero if data length = 0xff */
            *dest++ = 0;
        }
    }
    return dest - start;
}

} // namespace framing
} // namespace packet
