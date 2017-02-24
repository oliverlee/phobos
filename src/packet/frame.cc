#include "debug.h"
#include "packet/frame.h"

/*
 * Packet framing uses the Consistent Overhead Byte Stuffing algorithm for
 * encoding data bytes. The maximum length for each packet is limited to 254 bytes.
 * Refer to: https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
 */

namespace impl {

size_t stuff(const void* source, void* dest, size_t source_byte_size) {
    debug_check(source != nullptr);
    debug_check(dest != nullptr);
    debug_assert(
        source_byte_size <= packet::frame::COBS_MAX_SIZE_PAYLOAD,
        "source data exceeds allowed frame algorithm size"
    );

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
    *code_p = packet::frame::COBS_PACKET_DELIMITER;
    return source_byte_size + packet::frame::PACKET_OVERHEAD;
}

size_t unstuff(const void* source, void* dest, size_t source_byte_size) {
    debug_check(source != nullptr);
    debug_check(dest != nullptr);
    debug_assert(
        source_byte_size <= packet::frame::COBS_MAX_SIZE_PAYLOAD + packet::frame::COBS_PACKET_OVERHEAD,
        "invalid data size"
    );

    const char* source_ = static_cast<const char*>(source);
    char* dest_ = static_cast<char*>(dest);

    const char* end = source_ + source_byte_size;

    while (source_ < end) {
        uint8_t code = *source_++;

        if (code == 0) {
            debug_assert(dest_ != static_cast<char*>(dest), "zero byte input");
            break;
        }
        debug_assert(
            (code - 1) <= (end - source_),
            "input code too small or source buffer too small"
        );
        for (uint8_t i = 1; i < code; ++i) {
            *dest_++ = *source_++;
        }
        if (code < 0xff) { /* don't write the end-of-packet zero if data length = 0xff */
            *dest_++ = 0;
        }
    }
    return source_byte_size - packet::frame::PACKET_OVERHEAD;
}
} // namespace impl

namespace packet {
namespace frame {

size_t stuff(const void* source, void* dest, size_t source_byte_size) {
    size_t sections = (source_byte_size + COBS_MAX_SIZE_PAYLOAD - 1)/COBS_MAX_SIZE_PAYLOAD;
    size_t source_offset = 0;
    size_t dest_offset = 0;
    for (size_t i = 0; i < sections; ++i) {
        size_t payload_size = (i == (sections - 1)) ? source_byte_size % COBS_MAX_SIZE_PAYLOAD : COBS_MAX_SIZE_PAYLOAD;
        size_t stuffed_size = impl::stuff(
                static_cast<const char*>(source) + source_offset,
                static_cast<char*>(dest) + dest_offset, payload_size);
        source_offset += payload_size;
        dest_offset += stuffed_size;
    }
    return dest_offset;
}

size_t unstuff(const void* source, void* dest, size_t source_byte_size) {
    constexpr size_t COBS_MAX_SIZE_SECTION = COBS_MAX_SIZE_PAYLOAD + COBS_PACKET_OVERHEAD;
    size_t sections = (source_byte_size + COBS_MAX_SIZE_SECTION - 1)/COBS_MAX_SIZE_SECTION;
    size_t source_offset = 0;
    size_t dest_offset = 0;
    for (size_t i = 0; i < sections; ++i) {
        size_t section_size = COBS_MAX_SIZE_SECTION;
        if (i == (sections - 1)) {
            section_size = source_byte_size - (sections - 1)*COBS_MAX_SIZE_SECTION;
        }
        size_t unstuffed_size = impl::unstuff(
                static_cast<const char*>(source) + source_offset,
                static_cast<char*>(dest) + dest_offset, section_size);
        source_offset += section_size;
        dest_offset += unstuffed_size;
    }
    return dest_offset;
}

size_t unstuff(const void* source, void* dest) {
    debug_check(source != nullptr);
    debug_check(dest != nullptr);

    const char* source_ = static_cast<const char*>(source);
    char* dest_ = static_cast<char*>(dest);

    while (true) {
        uint8_t code = *source_++;

        if (code == 0) {
            debug_assert(dest_ != static_cast<char*>(dest), "zero byte input");
            break;
        }
        debug_assert(
            (code - 1) <= (COBS_MAX_SIZE_PAYLOAD - (dest_ - static_cast<char*>(dest) - 1)),
            "input code too small or source buffer too small"
        );
        for (uint8_t i = 1; i < code; ++i) {
            *dest_++ = *source_++;
        }
        if (code < 0xff) { /* don't write the end-of-packet zero if data length = 0xff */
            *dest_++ = 0;
        }
    }
    return dest_ - static_cast<char*>(dest) - 1;
}

} // namespace frame
} // namespace packet
