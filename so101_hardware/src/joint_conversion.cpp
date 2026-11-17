#include "so101_hardware/joint_conversion.hpp"

#include <algorithm>
#include <cmath>

namespace so101_hardware
{
namespace conversion
{
std::int32_t rad_to_counts(int sign, std::int32_t offset, double rad)
{
  return kMidCounts + offset + static_cast<std::int32_t>(std::lround(sign * rad * kCountsPerRad));
}

double counts_to_rad(int sign, std::int32_t offset, std::int32_t counts)
{
  return sign * static_cast<double>(counts - kMidCounts - offset) / kCountsPerRad;
}

double counts_per_s_to_rad_per_s(int sign, std::int32_t counts_per_s)
{
  return sign * static_cast<double>(counts_per_s) / kCountsPerRad;
}

double limit_command(double target, double previous, double min, double max, double max_step)
{
  if (std::isnan(target)) {
    return previous;
  }
  const double inside = std::clamp(target, min, max);
  return std::clamp(inside, previous - max_step, previous + max_step);
}

std::size_t mark_outside_limits(
  const double * positions, const double * mins, const double * maxs, std::size_t count,
  bool * outside)
{
  return mark_outside_band(positions, mins, maxs, 0.0, count, outside);
}

std::size_t mark_outside_band(
  const double * positions, const double * mins, const double * maxs, double margin,
  std::size_t count, bool * outside)
{
  std::size_t n = 0;
  for (std::size_t i = 0; i < count; ++i) {
    // "Not inside" rather than "below or above": NaN fails every comparison, so it lands here.
    outside[i] = !(positions[i] >= mins[i] - margin && positions[i] <= maxs[i] + margin);
    if (outside[i]) {
      ++n;
    }
  }
  return n;
}

ServoErrorVerdict check_servo_error(std::uint8_t error, std::uint32_t & run, std::uint32_t max_run)
{
  if (error == 0) {
    run = 0;
    return ServoErrorVerdict::kOk;
  }
  if (error != kErrorVoltage) {
    return ServoErrorVerdict::kFault;
  }
  if(run < max_run) {
    ++run;
  }
  return run >= max_run ? ServoErrorVerdict::kFault : ServoErrorVerdict::kVoltageTransient;
}
} // namespace conversion
} // namespace so101_hardware
