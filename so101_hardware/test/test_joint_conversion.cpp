#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "so101_hardware/joint_conversion.hpp"

namespace conv = so101_hardware::conversion;

namespace
{
constexpr int kSigns[] = {1, -1};
constexpr std::int32_t kOffsets[] = {0, 37, -120};
constexpr double kHalfCount = 0.5 / conv::kCountsPerRad;
constexpr double kTol = 1e-12;
} // namespace

TEST(JointConversion, HandWorkedValues)
{
  EXPECT_NEAR(conv::counts_to_rad(1, 0, 2048), 0.0, kTol);
  EXPECT_NEAR(conv::counts_to_rad(1, 0, 3072), conv::kPi / 2.0, kTol);
  EXPECT_NEAR(conv::counts_to_rad(1, 0, 1024), -conv::kPi / 2.0, kTol);
  EXPECT_NEAR(conv::counts_to_rad(-1, 0, 3072), -conv::kPi / 2.0, kTol);
  EXPECT_NEAR(conv::counts_to_rad(1, 10, 2058), 0.0, kTol);
  EXPECT_EQ(conv::rad_to_counts(1, 10, 0.0), 2058);
}

TEST(JointConversion, CountsRoundTripIsExact)
{
  for (const int s : kSigns) {
    for (const std::int32_t o : kOffsets) {
      for (std::int32_t c = 0; c <= 4095; ++c) {
        ASSERT_EQ(conv::rad_to_counts(s, o, conv::counts_to_rad(s, o, c)), c)
          << "sign " << s << ", offset " << o;
      }
    }
  }
}

TEST(JointConversion, RadRoundTripWithinHalfCount)
{
  for (const int s : kSigns) {
    for (const std::int32_t o : kOffsets) {
      for (int i = -3141; i <= 3141; ++i) {
        const double rad = i * 1e-3;
        const double back = conv::counts_to_rad(s, o, conv::rad_to_counts(s, o, rad));
        ASSERT_LE(std::abs(back - rad), kHalfCount + kTol)
          << "sign " << s << ", offset " << o << ", rad " << rad;
      }
    }
  }
}
