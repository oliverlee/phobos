#include "cobs.h"
#include "gtest/gtest.h"
#include <vector>
#include <cstring>

namespace {

// non-zero payload
const uint8_t X = 0xdd;

/**
Fill a byte buffer with a given value.
*/
void fill(uint8_t * const buffer, size_t count, uint8_t value) {
    std::memset(buffer, value, count);
}

/**
A structure that represents a sequence of bytes.
*/
struct Rep {
    size_t count;
    uint8_t value;
};

/**
Fill a byte buffer with given repetitions.
*/
size_t fill_repetitions(uint8_t * const buffer, std::vector<Rep> repetitions) {
    size_t offset = 0;

    for (auto const& rep: repetitions) {
        fill(buffer + offset, rep.count, rep.value);
        offset += rep.count;
    }

    return offset;
}

void test_equal_buffers(uint8_t * expected,  size_t expected_len, uint8_t * actual, size_t actual_len) {
    EXPECT_EQ(expected_len, actual_len);

    const uint8_t * expected_end = expected + expected_len;
    while (expected < expected_end) {
        EXPECT_EQ(*(expected++), *(actual++));
    }
}

/**
Ensure that cobs::encode(decoded) == encoded and cobs::decode(encode) ==
decoded.
*/
void test_encode_decode(std::vector<Rep> decoded, std::vector<Rep> encoded) {
    // Define a filler byte value.
    const uint8_t F = 0xdd;

    // Allocate buffers.
    uint8_t dec_exp[1000]; fill(dec_exp, sizeof(dec_exp), F);
    uint8_t dec_act[1000]; fill(dec_act, sizeof(dec_act), F);
    uint8_t enc_exp[1000]; fill(enc_exp, sizeof(enc_exp), F);
    uint8_t enc_act[1000]; fill(enc_act, sizeof(enc_act), F);

    // Write expected outcomes.
    const size_t enc_exp_len = fill_repetitions(enc_exp, encoded);
    const size_t dec_exp_len = fill_repetitions(dec_exp, decoded);

    // Encode.
    const cobs::EncodeResult enc_act_res = cobs::encode(dec_exp, dec_exp_len, enc_act, sizeof(enc_act));
    EXPECT_EQ(enc_act_res.status, cobs::EncodeResult::Status::OK);
    EXPECT_EQ(enc_act_res.written, enc_exp_len);
    test_equal_buffers(enc_exp, sizeof(enc_exp), enc_act, sizeof(enc_act));

    // Decode.
    const cobs::DecodeResult dec_act_res = cobs::decode(enc_exp, enc_exp_len, dec_act, sizeof(dec_act));
    EXPECT_EQ(dec_act_res.status, cobs::DecodeResult::Status::OK);
    EXPECT_EQ(dec_act_res.read, enc_exp_len);
    EXPECT_EQ(dec_act_res.written, dec_exp_len);
    test_equal_buffers(dec_exp, sizeof(dec_exp), dec_act, sizeof(dec_act));
}

} // namespace

TEST(cobs, max_encode_length) {
    EXPECT_EQ(cobs::max_encoded_length(  0),   2U);
    EXPECT_EQ(cobs::max_encoded_length(  1),   3U);
    EXPECT_EQ(cobs::max_encoded_length(  2),   4U);

    EXPECT_EQ(cobs::max_encoded_length(253), 255U);
    EXPECT_EQ(cobs::max_encoded_length(254), 257U);
    EXPECT_EQ(cobs::max_encoded_length(255), 258U);

    EXPECT_EQ(cobs::max_encoded_length(507), 510U);
    EXPECT_EQ(cobs::max_encoded_length(508), 512U);
    EXPECT_EQ(cobs::max_encoded_length(509), 513U);
}

TEST(cobs, max_decode_length) {
    EXPECT_EQ(cobs::max_decoded_length(  2),   0U);
    EXPECT_EQ(cobs::max_decoded_length(255), 253U);
    EXPECT_EQ(cobs::max_decoded_length(256), 254U);
    EXPECT_EQ(cobs::max_decoded_length(257), 255U);
    EXPECT_EQ(cobs::max_decoded_length(258), 256U);
}

TEST(cobs, encode_decode_empty_packet) {
    // Encode the empty packet.
    // [] -> [ 1, 0 ]
    test_encode_decode(
        std::vector<Rep> {},
        std::vector<Rep> {
            { 1, 1 },
            { 1, 0 }
        }
    );
}

TEST(cobs, encode_decode_1_byte_packet) {
    // Encode a packet of 1 non-zero byte.
    // [ x ] -> [ 2, x, 0 ]
    test_encode_decode(
        std::vector<Rep> {
            { 1, X }
        },
        std::vector<Rep> {
            { 1, 2 },
            { 1, X },
            { 1, 0 }
        }
    );
}

TEST(cobs, encode_decode_1_zero_byte_packet) {
    // Encode a packet of 1 zero byte.
    // [ 0 ] -> [ 1, 1, 0 ]
    test_encode_decode(
        std::vector<Rep> {
            { 1, 0 }
        },
        std::vector<Rep> {
            { 1, 1 },
            { 1, 1 },
            { 1, 0 }
        }
    );
}

TEST(cobs, encode_decode_253_byte_packet) {
    // Encode a packet of 253 non-zero bytes.
    // [ 253x ] -> [ 254, 253x, 0 ]
    test_encode_decode(
        std::vector<Rep> {
            { 253,   X }
        },
        std::vector<Rep> {
            {   1, 254 },
            { 253,   X },
            {   1,   0 }
        }
    );
}

TEST(cobs, encode_decode_254_byte_packet) {
    // Encode a packet of 254 non-zero bytes.
    // [ 254x ] -> [ 255, 254x, 1, 0 ]
    test_encode_decode(
        std::vector<Rep> {
            { 254,   X }
        },
        std::vector<Rep> {
            {   1, 255 },
            { 254,   X },
            {   1,   1 },
            {   1,   0 }
        }
    );

}

TEST(cobs, encode_decode_255_byte_packet) {
    // Encode a packet of 255 non-zero bytes. Smallest possible packet
    // that causes insertion of an extra offset marker.
    // [ 255x ] -> [ 255, 254x, 2, x, 0 ]
    test_encode_decode(
        std::vector<Rep> {
            { 255,   X }
        },
        std::vector<Rep> {
            {   1, 255 },
            { 254,   X },
            {   1,   2 },
            {   1,   X },
            {   1,   0 }
        }
    );
}

TEST(cobs, encode_decode_non_maximal_offset_packet) {
    // Encode a packet with non-maximal offsets.
    // [ 20x, 0, 4x, 0, 0, 300x ] -> [ 21, 20x, 5, 4x, 1, 255, 254x, 47, 46x, 0 ]
    test_encode_decode(
        std::vector<Rep> {
            {  20,   X },
            {   1,   0 },
            {   4,   X },
            {   2,   0 },
            { 300,   X }
        },
        std::vector<Rep> {
            {   1,  21 },
            {  20,   X },
            {   1,   5 },
            {   4,   X },
            {   1,   1 },
            {   1, 255 },
            { 254,   X },
            {   1,  47 },
            {  46,   X },
            {   1,   0 }
        }
    );
}
