#ifndef SO101_HARDWARE__JOINT_CONVERSION_HPP_
#define SO101_HARDWARE__JOINT_CONVERSION_HPP_

#include <cstdint> // for std::int32_t

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

} // namespace conversion
} // namespace so101_hardware

#endif // SO101_HARDWARE__JOINT_CONVERSION_HPP_
