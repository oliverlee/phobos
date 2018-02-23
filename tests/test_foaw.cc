#include "foaw.h"
#include "gtest/gtest.h"
#include <array>

namespace {
// buffer with test data
std::array<int, 10> data {
    {0, 1, 2, 3, 3, 2, 1, 0, 0, 0}
};
}

TEST(foaw, large_error) {
    const double dt = 1.0f;
    const double error = 100.0f;
    Foaw<double, data.size()> f(dt, error);

    EXPECT_EQ(f.estimate_velocity(), 0.0f);

    for (auto i: data) {
        f.add_position(i);
        EXPECT_EQ(static_cast<double>(i)/(dt*(data.size() - 1)),
                  f.estimate_velocity());
    }
}

TEST(foaw, small_error) {
    const double dt = 1.0f;
    const double error = 1.0f;
    Foaw<double, data.size()> f(dt, error);

    EXPECT_EQ(f.estimate_velocity(), 0.0f);

    std::array<double, 10> velocity {
        {0.0, 1.0/9, 2.0/4, 3.0/4, 3.0/6, 0.0, -2.0/3, -3.0/4, -3.0/5, -3.0/6}
    };

    for (size_t i = 0; i < data.size(); ++i) {
        f.add_position(data[i]);

        EXPECT_EQ(velocity[i], f.estimate_velocity()) << " at index " << i;
    }
}
