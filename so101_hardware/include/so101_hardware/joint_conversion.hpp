#ifndef SO101_HARDWARE__JOINT_CONVERSION_HPP_
#define SO101_HARDWARE__JOINT_CONVERSION_HPP_

#include <cstdint> // for std::int32_t
#include <cstddef> // for std::size_t

namespace so101_hardware
{
namespace conversion
{
// Servo count at URDF zero (LeRobot half-turn homing). 4096 counts per revolution
constexpr std::int32_t kMidCounts = 2048;
constexpr double kPi = 3.14159265358979323846;
constexpr double kCountsPerRad = 4096.0 / (2.0 * kPi);

// URDF angle (rad) -> servo counts, rounded to the nearest count.
// sign is +1 or -1 (servo direction vs URDF axis), offset is in counts.
std::int32_t rad_to_counts(int sign, std::int32_t offset, double rad);

// Servo counts -> URDF angle (rad). Exact inverse of rad_to_counts for every integer count
// counts must already be sign-magnitude decoded (Present_Position, sign bit 15).
double counts_to_rad(int sign, std::int32_t offset, std::int32_t counts);

// Present_Velocity (counts/s, measured 2026-11-08) -> rad/s.
// Sign applies, offset does not: a constant position shift has no rate.
double counts_per_s_to_rad_per_s(int sign, std::int32_t counts_per_s);

// The next goal for one joint: target clamped to [min, max], then moved at most max_step
// (>= 0) from previous, the goal sent last cycle. A NaN target holds previous.
double limit_command(double target, double previous, double min, double max, double max_step);

// Checks EVERY joint: outside[i] is true when positions[i] is below mins[i], above
// maxs[i], or NaN. Returns how many joints are outside. Each array holds count elements.
std::size_t mark_outside_limits(
  const double * positions, const double * mins, const double * maxs, std::size_t count,
  bool * outside);

// Same check with a band: a joint counts as outside only when it is more than margin (>= 0)
// below mins[i] or above maxs[i], or NaN. margin = 0 is exactly mark_outside_limits.
std::size_t mark_outside_band(
  const double * positions, const double * mins, const double * maxs, double margin,
  std::size_t count, bool * outside);

// Servo status error byte, bit 0 (SCServo SDK ERRBIT_VOLTAGE): input voltage outside the
// servo's Min/Max_Voltage_Limit.
constexpr std::uint8_t kErrorVoltage = 0x01;

// What read() should do with one servo's status error byte.
enum class ServoErrorVerdict
{
  kOk,                // no flag
  kVoltageTransient,  // voltage flag alone, fewer than max_run reads in a row so far: keep going
  kFault,             // any other bit, or the voltage flag alone max_run reads in a row
};

// Debounce for one servo. run counts consecutive reads whose error byte is exactly
// kErrorVoltage; a clean read resets it. max_run (>= 1) is the run length that faults;
// max_run = 1 faults on the first flag. Any other bit faults at once, alone or with bit 0.
ServoErrorVerdict check_servo_error(std::uint8_t error, std::uint32_t & run, std::uint32_t max_run);
} // namespace conversion
} // namespace so101_hardware

#endif // SO101_HARDWARE__JOINT_CONVERSION_HPP_
