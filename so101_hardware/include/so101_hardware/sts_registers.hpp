#ifndef SO101_HARDWARE__STS_REGISTERS_HPP_
#define SO101_HARDWARE__STS_REGISTERS_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace so101_hardware
{
namespace sts
{
namespace reg
{
constexpr std::uint8_t kId = 5;
constexpr std::uint8_t kReturnDelay = 7;
constexpr std::uint8_t kMinPositionLimit = 9;
constexpr std::uint8_t kMaxPositionLimit = 11;
constexpr std::uint8_t kHomingOffset = 31;
constexpr std::uint8_t kOperatingMode = 33;
constexpr std::uint8_t kTorqueEnable = 40;

constexpr unsigned kHomingOffsetSignBit = 11;
constexpr std::uint8_t kModePosition = 0;

            // One READ covers every field above: addresses [kConfigBlockStart, kTorqueEnable].
constexpr std::uint8_t kConfigBlockStart = kId;
constexpr std::uint8_t kConfigBlockLength = static_cast<std::uint8_t>(kTorqueEnable -
  kConfigBlockStart + 1);                                                                                                 // 36
}         // namespace reg
          // What the servo's EEPROM says
struct ServoConfig
{
  std::uint8_t id{0};
  std::uint8_t return_delay{0};
  std::uint16_t min_position_limit{0};
  std::uint16_t max_position_limit{0};
  std::int32_t homing_offset{0};
  std::uint8_t operating_mode{0xFF};           // invalid by default
  bool torque_enabled{false};
};

        // What the URDF says it must be
struct ServoExpectation
{
  std::uint8_t id{0};
  std::int32_t homing_offset{0};
  std::int32_t min_counts{0};
  std::int32_t max_counts{0};
};

        // block holds the bytes of addresses [kConfigBlockStart, kConfigBlockStart + size)
bool decode_config_block(const std::uint8_t *block, std::size_t size, ServoConfig & out);

        // Returns every mismatch joined by "; ", or an empty string if the servo is acceptable
std::string check_servo_config(const ServoConfig & actual, const ServoExpectation & expected);
}     // namespace sts
} // namespace so101_hardware

#endif // SO101_HARDWARE__STS_REGISTERS_HPP_
