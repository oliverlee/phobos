#include "cobs.h"
#include "test_cobs_util.h"
#include "gtest/gtest.h"
#include <random>
#include <vector>

class CobsRandomDataTest: public ::testing::TestWithParam<size_t> {
    public:
        using value_type = uint8_t;
        std::vector<value_type> b1;
        std::vector<value_type> b2;
        std::vector<value_type> b3;

        void test_encode_decode();

        virtual void SetUp();

    protected:
        std::random_device m_rd; // used to seed rng
        std::mt19937 m_gen;
        std::uniform_int_distribution<value_type> m_dist;
};

void CobsRandomDataTest::SetUp() {
    m_gen = std::mt19937(m_rd()); // seed random number generator

    // Compute buffer sizes.
    const size_t b1_size = GetParam();
    const size_t b2_size = cobs::max_encoded_length(b1_size);
    const size_t b3_size = cobs::max_decoded_length(b2_size);

    // Fill b1 with random data.
    b1.reserve(b1_size);
    for (size_t i = 0; i < b1.capacity(); ++i) {
        b1.push_back(m_dist(m_gen));
    }

    // Resize vectors to the maximum size, will be trimmed later with resize.
    // Shouldn't use reserve because that will cause the resize call to
    // overwrite data.
    b2.resize(b2_size);
    b3.resize(b3_size);
}

void CobsRandomDataTest::test_encode_decode() {
    // encode b1 to b2
    const cobs::EncodeResult enc_res = cobs::encode(b1.data(), b1.size(), b2.data(), b2.capacity());
    ASSERT_EQ(enc_res.status, cobs::EncodeResult::Status::OK);
    b2.resize(enc_res.produced);

    // decode b2 to b3
    const cobs::DecodeResult dec_res = cobs::decode(b2.data(), b2.size(), b3.data(), b3.capacity());
    ASSERT_EQ(dec_res.status, cobs::DecodeResult::Status::OK);
    ASSERT_EQ(dec_res.consumed, b2.size());
    b3.resize(dec_res.produced);

    // test b1 == b3
    test_equal_buffers(b1.data(), b1.size(), b3.data(), b3.size());
}

TEST_P(CobsRandomDataTest, random_input) {
    test_encode_decode();
}

INSTANTIATE_TEST_CASE_P(
        random_input,
        CobsRandomDataTest,
        ::testing::Values(10, 100, 200, 254, 255, 256, 300, 508, 509, 510));
