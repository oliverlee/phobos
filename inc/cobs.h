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
        const size_t written;
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
        const size_t read;
        const size_t written;
    };

    DecodeResult decode(const uint8_t * const src_start, size_t src_len, uint8_t * const dst_start, size_t dst_len);

}
