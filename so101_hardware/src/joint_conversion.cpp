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
  std::size_t n = 0;
  for (std::size_t i = 0; i < count; ++i) {
    // "Not inside" rather than "below or above": NaN fails every comparison, so it lands here.
    outside[i] = !(positions[i] >= mins[i] && positions[i] <= maxs[i]);
    if (outside[i]) {
      ++n;
    }
  }
  return n;
}

} // namespace conversion
} // namespace so101_hardware
