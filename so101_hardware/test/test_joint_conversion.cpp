#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <array>
#include <cstddef>
#include <limits>

#include "so101_hardware/joint_conversion.hpp"

namespace conv = so101_hardware::conversion;

namespace
{
constexpr int kSigns[] = {1, -1};
constexpr std::int32_t kOffsets[] = {0, 37, -120};
constexpr double kHalfCount = 0.5 / conv::kCountsPerRad;
constexpr double kTol = 1e-12;
constexpr std::size_t kN = 6;
// The URDF limits (rad), shoulder_pan .. gripper.
const double kMin[kN] = {-1.9190, -1.9067, -1.8453, -1.7717, -2.96, -0.2070};
const double kMax[kN] = {1.9036, 1.8622, 1.5692, 1.6935, -0.32, 1.9236};

std::array<double, kN> midpoints()
{
  std::array<double, kN> p{};
  for (std::size_t i = 0; i < kN; ++i) {
    p[i] = 0.5 * (kMin[i] + kMax[i]);
  }
  return p;
}
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

TEST(JointConversion, VelocityAppliesSignNotOffset)
{
  EXPECT_NEAR(conv::counts_per_s_to_rad_per_s(1, 1024), conv::kPi / 2.0, kTol);
  EXPECT_NEAR(conv::counts_per_s_to_rad_per_s(-1, 1024), -conv::kPi / 2.0, kTol);
  EXPECT_NEAR(conv::counts_per_s_to_rad_per_s(1, 0), 0.0, kTol);
}

TEST(OutsideLimits, AllInsideIsZero)
{
  const auto p = midpoints();
  bool out[kN];
  EXPECT_EQ(conv::mark_outside_limits(p.data(), kMin, kMax, kN, out), 0u);
  for (std::size_t i = 0; i < kN; ++i) {
    EXPECT_FALSE(out[i]) << "joint " << i;
  }
}

TEST(OutsideLimits, OnTheLimitIsInside)
{
  bool out[kN];
  EXPECT_EQ(conv::mark_outside_limits(kMin, kMin, kMax, kN, out), 0u);
  EXPECT_EQ(conv::mark_outside_limits(kMax, kMin, kMax, kN, out), 0u);
}

// The safety property: no joint is skipped, below or above.
TEST(OutsideLimits, EveryJointIsChecked)
{
  for (std::size_t j = 0; j < kN; ++j) {
    for (const double p_j : {kMin[j] - 1e-3, kMax[j] + 1e-3}) {
      auto p = midpoints();
      p[j] = p_j;
      bool out[kN];
      ASSERT_EQ(conv::mark_outside_limits(p.data(), kMin, kMax, kN, out), 1u)
        << "joint " << j << " at " << p_j;
      for (std::size_t k = 0; k < kN; ++k) {
        EXPECT_EQ(out[k], k == j) << "joint " << j << ", flag " << k;
      }
    }
  }
}

TEST(OutsideLimits, NaNIsOutside)
{
  auto p = midpoints();
  p[2] = std::numeric_limits<double>::quiet_NaN();
  bool out[kN];
  EXPECT_EQ(conv::mark_outside_limits(p.data(), kMin, kMax, kN, out), 1u);
  EXPECT_TRUE(out[2]);
}

TEST(OutsideLimits, CountsEveryViolation)
{
  auto p = midpoints();
  p[0] = 3.0;
  p[2] = 1.62;  // elbow_flex resting on its stop
  bool out[kN];
  EXPECT_EQ(conv::mark_outside_limits(p.data(), kMin, kMax, kN, out), 2u);
  EXPECT_TRUE(out[0]);
  EXPECT_TRUE(out[2]);
}

TEST(OutsideLimits, WristRollPastTheWrapIsOutside)
{
  // Past the encoder wrap, wrist_roll reads near +pi instead of below -2.96.
  auto p = midpoints();
  p[4] = 3.10;
  bool out[kN];
  EXPECT_EQ(conv::mark_outside_limits(p.data(), kMin, kMax, kN, out), 1u);
  EXPECT_TRUE(out[4]);
}

TEST(LimitCommand, SmallStepInsideLimitsPassesThrough)
{
  EXPECT_DOUBLE_EQ(conv::limit_command(0.105, 0.100, -1.0, 1.0, 0.01), 0.105);
}

TEST(LimitCommand, StepIsLimitedInBothDirections)
{
  EXPECT_DOUBLE_EQ(conv::limit_command(0.5, 0.0, -1.0, 1.0, 0.01), 0.01);
  EXPECT_DOUBLE_EQ(conv::limit_command(-0.5, 0.0, -1.0, 1.0, 0.01), -0.01);
}

TEST(LimitCommand, TargetBeyondLimitIsClamped)
{
  EXPECT_DOUBLE_EQ(conv::limit_command(5.0, 0.995, -1.0, 1.0, 0.01), 1.0);
  EXPECT_DOUBLE_EQ(conv::limit_command(-5.0, -0.995, -1.0, 1.0, 0.01), -1.0);
  EXPECT_DOUBLE_EQ(
    conv::limit_command(std::numeric_limits<double>::infinity(), 0.995, -1.0, 1.0, 0.01), 1.0);
}

TEST(LimitCommand, NaNTargetHoldsPrevious)
{
  EXPECT_DOUBLE_EQ(
    conv::limit_command(std::numeric_limits<double>::quiet_NaN(), 0.3, -1.0, 1.0, 0.01), 0.3);
}

TEST(LimitCommand, ZeroStepHolds)
{
  EXPECT_DOUBLE_EQ(conv::limit_command(0.5, 0.3, -1.0, 1.0, 0.0), 0.3);
}

// Safety property: starting inside the limits, no target can produce a goal outside them
// or a step larger than max_step.
TEST(LimitCommand, OutputStaysInsideLimitsAndStep)
{
  const double lo = -1.8453;
  const double hi = 1.5692;
  const double step = 0.01;
  for (double prev = lo; prev <= hi; prev += 0.05) {
    for (double target = -10.0; target <= 10.0; target += 0.137) {
      const double out = conv::limit_command(target, prev, lo, hi, step);
      ASSERT_GE(out, lo) << "prev " << prev << " target " << target;
      ASSERT_LE(out, hi) << "prev " << prev << " target " << target;
      ASSERT_LE(std::abs(out - prev), step + 1e-12) << "prev " << prev << " target " << target;
    }
  }
}

// The activation band, 50 counts as in SO101System.cpp.
constexpr double kBand = 50.0 / conv::kCountsPerRad;

TEST(OutsideBand, ZeroBandMatchesTheLimits)
{
  auto p = midpoints();
  p[2] = kMax[2] + 1e-3;
  bool a[kN];
  bool b[kN];
  EXPECT_EQ(conv::mark_outside_band(p.data(), kMin, kMax, 0.0, kN, a),
    conv::mark_outside_limits(p.data(), kMin, kMax, kN, b));
  for (std::size_t i = 0; i < kN; ++i) {
    EXPECT_EQ(a[i], b[i]) << "joint " << i;
  }
}

TEST(OutsideBand, ElbowRestingOnItsStopIsInsideTheBand)
{
  // Measured rest: up to 3109 counts, upper limit 3071 counts (38 counts above).
  auto p = midpoints();
  p[2] = kMax[2] + 38.0 / conv::kCountsPerRad;
  bool out[kN];
  EXPECT_EQ(conv::mark_outside_band(p.data(), kMin, kMax, kBand, kN, out), 0u);
  EXPECT_EQ(conv::mark_outside_limits(p.data(), kMin, kMax, kN, out), 1u);
  EXPECT_TRUE(out[2]);
}

TEST(OutsideBand, TheBandEdgeIsInside)
{
  bool out[kN];
  for (std::size_t j = 0; j < kN; ++j) {
    auto p = midpoints();
    p[j] = kMax[j] + kBand;
    EXPECT_EQ(conv::mark_outside_band(p.data(), kMin, kMax, kBand, kN, out), 0u) << "joint " << j;
    p[j] = kMin[j] - kBand;
    EXPECT_EQ(conv::mark_outside_band(p.data(), kMin, kMax, kBand, kN, out), 0u) << "joint " << j;
  }
}

// The safety property again: past the band, no joint is skipped, below or above.
TEST(OutsideBand, EveryJointPastTheBandIsChecked)
{
  for (std::size_t j = 0; j < kN; ++j) {
    for (const double p_j : {kMin[j] - kBand - 1e-3, kMax[j] + kBand + 1e-3}) {
      auto p = midpoints();
      p[j] = p_j;
      bool out[kN];
      ASSERT_EQ(conv::mark_outside_band(p.data(), kMin, kMax, kBand, kN, out), 1u)
        << "joint " << j << " at " << p_j;
      for (std::size_t k = 0; k < kN; ++k) {
        EXPECT_EQ(out[k], k == j) << "joint " << j << ", flag " << k;
      }
    }
  }
}

TEST(OutsideBand, NaNAndTheWrapStayOutside)
{
  auto p = midpoints();
  p[2] = std::numeric_limits<double>::quiet_NaN();
  p[4] = 3.10;  // wrist_roll past the encoder wrap
  bool out[kN];
  EXPECT_EQ(conv::mark_outside_band(p.data(), kMin, kMax, kBand, kN, out), 2u);
  EXPECT_TRUE(out[2]);
  EXPECT_TRUE(out[4]);
}

constexpr std::uint32_t kDebounce = 10;
using Verdict = conv::ServoErrorVerdict;

TEST(ServoErrorDebounce, CleanReadIsOkAndResetsTheRun)
{
  std::uint32_t run = 7;
  EXPECT_EQ(conv::check_servo_error(0x00, run, kDebounce), Verdict::kOk);
  EXPECT_EQ(run, 0u);
}

TEST(ServoErrorDebounce, NineVoltageFlagsInARowAreTolerated)
{
  std::uint32_t run = 0;
  for (std::uint32_t k = 1; k < kDebounce; ++k) {
    EXPECT_EQ(conv::check_servo_error(conv::kErrorVoltage, run, kDebounce),
      Verdict::kVoltageTransient) << "flag " << k;
  }
  EXPECT_EQ(run, kDebounce - 1);
}

TEST(ServoErrorDebounce, TheTenthVoltageFlagInARowFaults)
{
  std::uint32_t run = 0;
  for (std::uint32_t k = 1; k < kDebounce; ++k) {
    conv::check_servo_error(conv::kErrorVoltage, run, kDebounce);
  }
  EXPECT_EQ(conv::check_servo_error(conv::kErrorVoltage, run, kDebounce), Verdict::kFault);
  // Stays faulted (and the counter does not grow past max_run) if read() is called again.
  EXPECT_EQ(conv::check_servo_error(conv::kErrorVoltage, run, kDebounce), Verdict::kFault);
  EXPECT_EQ(run, kDebounce);
}

TEST(ServoErrorDebounce, ACleanReadRestartsTheCount)
{
  std::uint32_t run = 0;
  for (int cycle = 0; cycle < 3; ++cycle) {
    for (std::uint32_t k = 1; k < kDebounce; ++k) {
      ASSERT_EQ(conv::check_servo_error(conv::kErrorVoltage, run, kDebounce),
        Verdict::kVoltageTransient);
    }
    ASSERT_EQ(conv::check_servo_error(0x00, run, kDebounce), Verdict::kOk);
  }
}

TEST(ServoErrorDebounce, MeasuredGlitchPatternNeverFaults)
{
  // 2026-11-17, torque off, 941 s: every voltage flag was a single read between clean reads.
  std::uint32_t run = 0;
  for (int k = 0; k < 10000; ++k) {
    const std::uint8_t error = (k % 7 == 0) ? conv::kErrorVoltage : 0x00;
    ASSERT_NE(conv::check_servo_error(error, run, kDebounce), Verdict::kFault) << "read " << k;
  }
}

TEST(ServoErrorDebounce, OtherBitsFaultImmediately)
{
  // 0x02 angle, 0x04 overheat, 0x08 overcurrent, 0x20 overload; 0x03 and 0x21 carry the
  // voltage bit too and must not be treated as a voltage transient.
  for (const std::uint8_t error : {0x02, 0x04, 0x08, 0x20, 0x03, 0x21, 0xFF}) {
    std::uint32_t run = 0;
    EXPECT_EQ(conv::check_servo_error(error, run, kDebounce), Verdict::kFault)
      << "error 0x" << std::hex << static_cast<int>(error);
  }
}

TEST(ServoErrorDebounce, MaxRunOneFaultsOnTheFirstFlag)
{
  std::uint32_t run = 0;
  EXPECT_EQ(conv::check_servo_error(conv::kErrorVoltage, run, 1), Verdict::kFault);
}
