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

// RAM control registers written by the driver (2-byte values little-endian).
constexpr std::uint8_t kGoalPosition = 42;
constexpr unsigned kGoalPositionSignBit = 15;
// Lock = 0 allows EEPROM writes. The driver never writes it.
constexpr std::uint8_t kLock = 55;

// Live state, read every control cycle (2 bytes each, little-endian).
constexpr std::uint8_t kPresentPosition = 56;
constexpr std::uint8_t kPresentVelocity = 58;
constexpr std::uint8_t kPresentLoad = 60;
constexpr std::uint8_t kPresentVoltage = 62;      // 1 byte, 0.1 V units
constexpr std::uint8_t kPresentTemperature = 63;  // 1 byte, deg C

constexpr unsigned kHomingOffsetSignBit = 11;
constexpr unsigned kPresentPositionSignBit = 15;
constexpr unsigned kPresentVelocitySignBit = 15;
constexpr std::uint8_t kModePosition = 0;

// One READ covers every field above: addresses [kConfigBlockStart, kTorqueEnable].
constexpr std::uint8_t kConfigBlockStart = kId;
constexpr std::uint8_t kConfigBlockLength = static_cast<std::uint8_t>(kTorqueEnable -
  kConfigBlockStart + 1); // 36

// One SYNC READ covers position, velocity, load, voltage and temperature: addresses [56, 63].
constexpr std::uint8_t kPresentBlockStart = kPresentPosition;
constexpr std::uint8_t kPresentBlockLength = 8;
} // namespace reg

// The driver may write RAM registers only: addresses [Torque_Enable 40, Lock 55).
// Below 40 is EEPROM, and writing Lock could unlock EEPROM, so both are refused.
constexpr bool driver_may_write(std::uint8_t address, std::uint8_t length)
{
  return length > 0 && address >= reg::kTorqueEnable &&
         static_cast<unsigned>(address) + length <= reg::kLock;
}

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

// Live state of one servo, decoded from the block at kPresentBlockStart.
struct PresentState
{
  std::int32_t position{0};   // counts, sign bit 15 applied
  std::int32_t velocity{0};   // register units (not yet verified), sign bit 15 applied
  std::uint16_t load_raw{0};  // undecoded: sign bit and unit not yet verified
  std::uint8_t voltage_dv{0};     // supply voltage at the servo, 0.1 V units
  std::uint8_t temperature_c{0};  // deg C
};

// block holds the bytes of addresses [kPresentBlockStart, kPresentBlockStart + size)
bool decode_present_block(const std::uint8_t * block, std::size_t size, PresentState & out);

// block holds the bytes of addresses [kConfigBlockStart, kConfigBlockStart + size)
bool decode_config_block(const std::uint8_t *block, std::size_t size, ServoConfig & out);

// Returns every mismatch joined by "; ", or an empty string if the servo is acceptable
std::string check_servo_config(const ServoConfig & actual, const ServoExpectation & expected);
}     // namespace sts
} // namespace so101_hardware

#endif // SO101_HARDWARE__STS_REGISTERS_HPP_
