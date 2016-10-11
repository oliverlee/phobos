#include "packet/framing.h"
#include "osal.h"

/*
 * Packet framing uses the Consistent Overhead Byte Stuffing algorithm for
 * encoding data bytes. The maximum length for each packet is limited to 254 bytes.
 * Refer to: https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
 */

namespace {
    constexpr uint8_t COBS_MAX_SIZE_DATA_SET = 254;
} // namespace

namespace packet {
namespace framing {

void stuff(const uint8_t* source, uint8_t* dest, uint8_t length) {
    osalDbgCheck(source != nullptr);
    osalDbgCheck(dest != nullptr);
    osalDbgAssert(length <= COBS_MAX_SIZE_DATA_SET, "invalid data length");

    const uint8_t* end = source + length;
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
            if (++code == 0xff) { /* reached max data length */
                write_code();
            }
        }
        ++source;
    }
    write_code();
}

void unstuff(const uint8_t* source, uint8_t* dest, uint8_t length) {
    osalDbgCheck(source != nullptr);
    osalDbgCheck(dest != nullptr);
    osalDbgAssert(length <= COBS_MAX_SIZE_DATA_SET + 1, "invalid data length");

    const uint8_t* end = source + length;
    while (source < end) {
        uint8_t code = *source++;

        osalDbgAssert(code != 0, "zero byte input");
        osalDbgAssert(code <= (end - source), "input too short");
        for (uint8_t i = 1; i < code; ++i) {
            *dest++ = *source++;
        }
        if (code < 0xff) { /* don't write the last zero for max data length */
            *dest++ = 0;
        }
    }
}

} // namespace framing
} // namespace packet
