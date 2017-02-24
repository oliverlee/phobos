#include "cobs.h"
#include <cstring>

/*
Throw exceptions so we can catch them with gdb (catch catch).
*/
void throw_assert(bool condition, const char * const remark) {
    if (condition == false) {
        throw remark;
    }
}

/*
Fill a byte buffer with a given value.
*/
void fill(uint8_t * const buffer, size_t length, uint8_t value) {
    std::memset(buffer, value, length);
}

/*
Check that a byte buffer contains only a given value.
*/
void assert_mem_is(uint8_t * buffer, size_t length, uint8_t value) {
    uint8_t * buffer_end = buffer + length;
    while (buffer < buffer_end) {
        throw_assert(*(buffer++) == value, "");
    }
}

void test_max_encoded_length() {
    throw_assert(cobs::max_encoded_length(  0) ==   2, "");
    throw_assert(cobs::max_encoded_length(  1) ==   3, "");
    throw_assert(cobs::max_encoded_length(  2) ==   4, "");

    throw_assert(cobs::max_encoded_length(253) == 255, "");
    throw_assert(cobs::max_encoded_length(254) == 257, "");
    throw_assert(cobs::max_encoded_length(255) == 258, "");

    throw_assert(cobs::max_encoded_length(507) == 510, "");
    throw_assert(cobs::max_encoded_length(508) == 512, "");
    throw_assert(cobs::max_encoded_length(509) == 513, "");
}

void test_max_decoded_length() {
    throw_assert(cobs::max_decoded_length(  2) ==   0, "");
    throw_assert(cobs::max_decoded_length(255) == 253, "");
    throw_assert(cobs::max_decoded_length(256) == 254, "");
    throw_assert(cobs::max_decoded_length(257) == 255, "");
    throw_assert(cobs::max_decoded_length(258) == 256, "");
}

void test_encode_decode() {
    const uint8_t X = 0xdd; // non-zero payload
    const uint8_t F = 0xcc; // fill

    // Encode the empty packet.
    // [] -> [ 1, 0 ]
    {
        uint8_t b1[1000]; fill(b1, sizeof(b1), X);
        uint8_t b2[1000]; fill(b2, sizeof(b2), F);
        uint8_t b3[1000]; fill(b3, sizeof(b3), F);
        const size_t b1_len = 0;
        const size_t b2_len = cobs::encode(b1, b1_len, b2, sizeof(b2));
        assert_mem_is(&b2[0],                   1, 1);
        assert_mem_is(&b2[1],                   1, 0);
        assert_mem_is(&b2[2], sizeof(b2) - b2_len, F);
        throw_assert(b2_len == 2, "");
        const size_t b3_len = cobs::decode(b2, b2_len, b3, sizeof(b3));
        assert_mem_is(&b3[     0],              b1_len, X);
        assert_mem_is(&b3[b1_len], sizeof(b3) - b1_len, F);
        throw_assert(b3_len == b1_len, "");
    }

    // Encode a packet of 1 non-zero byte.
    // [ x ] -> [ 2, x, 0 ]
    {
        uint8_t b1[1000]; fill(b1, sizeof(b1), X);
        uint8_t b2[1000]; fill(b2, sizeof(b2), F);
        uint8_t b3[1000]; fill(b3, sizeof(b3), F);
        const size_t b1_len = 1;
        const size_t b2_len = cobs::encode(b1, b1_len, b2, sizeof(b2));
        assert_mem_is(&b2[0],                   1, 2);
        assert_mem_is(&b2[1],                   1, X);
        assert_mem_is(&b2[2],                   1, 0);
        assert_mem_is(&b2[3], sizeof(b2) - b2_len, F);
        throw_assert(b2_len == 3, "");
        const size_t b3_len = cobs::decode(b2, b2_len, b3, sizeof(b3));
        assert_mem_is(&b3[     0],              b1_len, X);
        assert_mem_is(&b3[b1_len], sizeof(b3) - b1_len, F);
        throw_assert(b3_len == b1_len, "");
    }

    // Encode a packet of 1 zero byte.
    // [ 0 ] -> [ 1, 1, 0 ]
    {
        uint8_t b1[1000]; fill(b1, sizeof(b1), X);
        uint8_t b2[1000]; fill(b2, sizeof(b2), F);
        uint8_t b3[1000]; fill(b3, sizeof(b3), F);
        b1[0] = 0;
        const size_t b1_len = 1;
        const size_t b2_len = cobs::encode(b1, b1_len, b2, sizeof(b2));
        assert_mem_is(&b2[0],                   1, 1);
        assert_mem_is(&b2[1],                   1, 1);
        assert_mem_is(&b2[2],                   1, 0);
        assert_mem_is(&b2[3], sizeof(b2) - b2_len, F);
        throw_assert(b2_len == 3, "");
        const size_t b3_len = cobs::decode(b2, b2_len, b3, sizeof(b3));
        assert_mem_is(&b3[     0],              b1_len, 0);
        assert_mem_is(&b3[b1_len], sizeof(b3) - b1_len, F);
        throw_assert(b3_len == b1_len, "");
    }

    // Encode a packet of 253 non-zero bytes.
    // [ 253x ] -> [ 254, 253x, 0 ]
    {
        uint8_t b1[1000]; fill(b1, sizeof(b1), X);
        uint8_t b2[1000]; fill(b2, sizeof(b2), F);
        uint8_t b3[1000]; fill(b3, sizeof(b3), F);
        const size_t b1_len = 253;
        const size_t b2_len = cobs::encode(b1, b1_len, b2, sizeof(b2));
        assert_mem_is(&b2[  0],                   1, 254);
        assert_mem_is(&b2[  1],                 253,   X);
        assert_mem_is(&b2[254],                   1,   0);
        assert_mem_is(&b2[255], sizeof(b2) - b2_len,   F);
        throw_assert(b2_len == 255, "");
        const size_t b3_len = cobs::decode(b2, b2_len, b3, sizeof(b3));
        assert_mem_is(&b3[     0],              b1_len, X);
        assert_mem_is(&b3[b1_len], sizeof(b3) - b1_len, F);
        throw_assert(b3_len == b1_len, "");
    }

    // Encode a packet of 254 non-zero bytes.
    // [ 254x ] -> [ 255, 254x, 1, 0 ]
    {
        uint8_t b1[1000]; fill(b1, sizeof(b1), X);
        uint8_t b2[1000]; fill(b2, sizeof(b2), F);
        uint8_t b3[1000]; fill(b3, sizeof(b3), F);
        const size_t b1_len = 254;
        const size_t b2_len = cobs::encode(b1, b1_len, b2, sizeof(b2));
        assert_mem_is(&b2[  0],                   1, 255);
        assert_mem_is(&b2[  1],                 254,   X);
        assert_mem_is(&b2[255],                   1,   1);
        assert_mem_is(&b2[256],                   1,   0);
        assert_mem_is(&b2[257], sizeof(b2) - b2_len,   F);
        throw_assert(b2_len == 257, "");
        const size_t b3_len = cobs::decode(b2, b2_len, b3, sizeof(b3));
        assert_mem_is(&b3[     0],              b1_len, X);
        assert_mem_is(&b3[b1_len], sizeof(b3) - b1_len, F);
        throw_assert(b3_len == b1_len, "");
    }

    // Encode a packet of 255 non-zero bytes. Smallest possible packet
    // that causes insertion of an extra offset marker.
    // [ 255x ] -> [ 255, 254x, 2, x, 0 ]
    {
        uint8_t b1[1000]; fill(b1, sizeof(b1), X);
        uint8_t b2[1000]; fill(b2, sizeof(b2), F);
        uint8_t b3[1000]; fill(b3, sizeof(b3), F);
        const size_t b1_len = 255;
        const size_t b2_len = cobs::encode(b1, b1_len, b2, sizeof(b2));
        assert_mem_is(&b2[  0],                   1, 255);
        assert_mem_is(&b2[  1],                 254,   X);
        assert_mem_is(&b2[255],                   1,   2);
        assert_mem_is(&b2[256],                   1,   X);
        assert_mem_is(&b2[257],                   1,   0);
        assert_mem_is(&b2[258], sizeof(b2) - b2_len,   F);
        throw_assert(b2_len == 258, "");
        const size_t b3_len = cobs::decode(b2, b2_len, b3, sizeof(b3));
        assert_mem_is(&b3[     0],              b1_len, X);
        assert_mem_is(&b3[b1_len], sizeof(b3) - b1_len, F);
        throw_assert(b3_len == b1_len, "");
    }

    // Encode a packet with non-maximal offsets.
    // [ 20x, 0, 4x, 0, 0, 300x ] -> [ 21, 20x, 5, 4x, 1, 255, 254x, 47, 46x, 0 ]
    {
        uint8_t b1[1000]; fill(b1, sizeof(b1), X);
        uint8_t b2[1000]; fill(b2, sizeof(b2), F);
        uint8_t b3[1000]; fill(b3, sizeof(b3), F);
        b1[20] = 0;
        b1[25] = 0;
        b1[26] = 0;
        const size_t b1_len = 327;
        const size_t b2_len = cobs::encode(b1, b1_len, b2, sizeof(b2));
        assert_mem_is(&b2[  0],                   1,  21);
        assert_mem_is(&b2[  1],                  20,   X);
        assert_mem_is(&b2[ 21],                   1,   5);
        assert_mem_is(&b2[ 22],                   4,   X);
        assert_mem_is(&b2[ 26],                   1,   1);
        assert_mem_is(&b2[ 27],                   1, 255);
        assert_mem_is(&b2[ 28],                 254,   X);
        assert_mem_is(&b2[282],                   1,  47);
        assert_mem_is(&b2[283],                  46,   X);
        assert_mem_is(&b2[329],                   1,   0);
        assert_mem_is(&b2[330], sizeof(b2) - b2_len,   F);
        throw_assert(b2_len == 330, "");
        const size_t b3_len = cobs::decode(b2, b2_len, b3, sizeof(b3));
        assert_mem_is(&b3[  0],                  20, X);
        assert_mem_is(&b3[ 20],                   1, 0);
        assert_mem_is(&b3[ 21],                   4, X);
        assert_mem_is(&b3[ 25],                   2, 0);
        assert_mem_is(&b3[ 27],                 300, X);
        assert_mem_is(&b3[327], sizeof(b3) - b1_len, F);
        throw_assert(b3_len == b1_len, "");
    }
}

int main() {
    test_max_encoded_length();
    test_max_decoded_length();
    test_encode_decode();
    return 0;
}
