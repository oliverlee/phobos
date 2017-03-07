#pragma once

#include <cstdint>
#include <cstddef>

namespace cobs {

    size_t max_encoded_length(size_t decoded_length);

    size_t max_decoded_length(size_t encoded_length);

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
        If the status is OK, the number of bytes that were written to dst_start.
        Otherwise it is set to 0.
        */
        const size_t consumed;

        /**
        If the status is OK or UNEXPECTED_ZERO, the number of bytes that were
        read from src_start. Otherwise it is set to 0.
        */
        const size_t produced;
    };

    DecodeResult decode(const uint8_t * const src_start, size_t src_len, uint8_t * const dst_start, size_t dst_len);

}
