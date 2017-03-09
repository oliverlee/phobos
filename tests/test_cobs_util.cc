#include "test_cobs_util.h"
#include "gtest/gtest.h"

void test_equal_buffers(uint8_t * expected,  size_t expected_len, uint8_t * actual, size_t actual_len) {
    EXPECT_EQ(expected_len, actual_len);

    const uint8_t * expected_end = expected + expected_len;
    while (expected < expected_end) {
        EXPECT_EQ(*(expected++), *(actual++));
    }
}
