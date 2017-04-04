#pragma once

#include <cstdint>
#include <cstddef>

namespace cobs {
    /**
    Given the length of a decoded byte array, compute the maximum length
    the encoded byte array can attain.

    The maximum length is attained when the decoded byte array contains
    no zeros. The maximum length consists of the decoded byte array
    length, the offset overhead and the frame marker. It is computed as
    decoded_length + floor(decoded_length/254) + 2.

    This function does not handle decoded_length < 0.
    */
    constexpr size_t max_encoded_length(size_t decoded_length) {
        return decoded_length + decoded_length/0xfe + 2;
    }

    /**
    Given the length of a COBS encoded byte array, compute the maximum
    length the decoded byte array can attain.

    A case where the minimum overhead is achieved is when the decoded
    byte array consists only of marker bytes. This means that the
    maximum decoded length can be as big as the encoded length minus
    the packet overhead, consisting of the first offset and the marker
    byte. The maximum decoded length is thus given by encoded_length -
    2.

    This function does not handle encoded_length < 2.
    */
    constexpr size_t max_decoded_length(size_t encoded_length) {
        return encoded_length - 2;
    }

    struct EncodeResult {
        enum class Status {
            OK,
            WRITE_OVERFLOW
        };
        const Status status;

        /**
        If the status is OK, the number of bytes that were written to dst_start.
        Otherwise it is set to 0.
        */
        const size_t produced;
    };

    EncodeResult encode(const uint8_t * const src_start, size_t src_len, uint8_t * const dst_start, size_t dst_len);

    struct DecodeResult {
        enum class Status {
            OK,
            WRITE_OVERFLOW,
            READ_OVERFLOW,
            UNEXPECTED_ZERO
        };
        const Status status;

        /**
        If the status is OK or UNEXPECTED_ZERO, the number of bytes that were
        read from src_start. Otherwise it is set to 0.
        */
        const size_t consumed;

        /**
        If the status is OK, the number of bytes that were written to dst_start.
        Otherwise it is set to 0.
        */
        const size_t produced;
    };

    DecodeResult decode(const uint8_t * const src_start, size_t src_len, uint8_t * const dst_start, size_t dst_len);

}
