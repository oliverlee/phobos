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

    const size_t size = GetParam();
    const size_t max_encoded_size = cobs::max_encoded_length(size);
    const size_t max_decoded_size = cobs::max_decoded_length(max_encoded_size);
    b1.reserve(size);
    b2.reserve(max_encoded_size);
    b3.reserve(max_decoded_size);

    // generate random data for b1
    for (size_t i = 0; i < b1.capacity(); ++i) {
        b1.push_back(m_dist(m_gen));
    }
}

void CobsRandomDataTest::test_encode_decode() {
    // encode b1 to b2
    const cobs::EncodeResult enc_res = cobs::encode(b1.data(), b1.size(), b2.data(), b2.capacity());
    ASSERT_EQ(enc_res.status, cobs::EncodeResult::Status::OK);
    b2.resize(enc_res.written);

    // decode b2 to b3
    const cobs::DecodeResult dec_res = cobs::decode(b2.data(), b2.size(), b3.data(), b3.capacity());
    ASSERT_EQ(dec_res.status, cobs::DecodeResult::Status::OK);
    ASSERT_EQ(dec_res.read, b2.size());
    b3.resize(dec_res.written);

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
