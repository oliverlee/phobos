#pragma once

#include <cstdint>
#include <cstddef>

namespace cobs {

    size_t max_encoded_length(size_t decoded_length);

    size_t max_decoded_length(size_t encoded_length);

    size_t encode(const uint8_t * const src, size_t src_len, uint8_t * const dst, size_t dst_len);

    size_t decode(const uint8_t * const src, size_t src_len, uint8_t * const dst, size_t dst_len);

}
