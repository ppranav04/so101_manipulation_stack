#include "so101_hardware/joint_conversion.hpp"

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

} // namespace conversion
} // namespace so101_hardware
