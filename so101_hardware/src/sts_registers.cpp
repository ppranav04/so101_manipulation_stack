#include "so101_hardware/sts_registers.hpp"

#include "so101_hardware/sts_protocol.hpp"

namespace so101_hardware
{
namespace sts
{
namespace
{
std::uint8_t byte_at(const std::uint8_t * block, std::uint8_t address)
{
  return block[address - reg::kConfigBlockStart];
}

std::uint16_t u16_at(const std::uint8_t * block, std::uint8_t address)
{
  return make_u16(
                    byte_at(block, address), byte_at(block, static_cast<std::uint8_t>(address + 1))
  );
}

void append(std::string & out, const std::string & problem)
{
  if (!out.empty()) {
    out += "; ";
  }
  out += problem;
}
}         // namespace

bool decode_config_block(const std::uint8_t * block, std::size_t size, ServoConfig & out)
{
  if (block == nullptr || size < reg::kConfigBlockLength) {
    return false;
  }

  out.id = byte_at(block, reg::kId);
  out.return_delay = byte_at(block, reg::kReturnDelay);
  out.min_position_limit = u16_at(block, reg::kMinPositionLimit);
  out.max_position_limit = u16_at(block, reg::kMaxPositionLimit);
  out.homing_offset = decode_sign_magnitude(u16_at(block, reg::kHomingOffset),
        reg::kHomingOffsetSignBit);
  out.operating_mode = byte_at(block, reg::kOperatingMode);
  out.torque_enabled = byte_at(block, reg::kTorqueEnable) != 0;
  return true;
}

std::string check_servo_config(const ServoConfig & actual, const ServoExpectation & expected)
{
  std::string problems;
  if (actual.id != expected.id) {
    append(problems,
          "EEPROM ID " + std::to_string(actual.id) + " != expected " + std::to_string(expected.id));
  }
  if (actual.operating_mode != reg::kModePosition) {
    append(problems,
          "operating mode " + std::to_string(actual.operating_mode) + " is not position mode (0)");
  }
  if (actual.homing_offset != expected.homing_offset) {
    append(problems,
          "homing offset " + std::to_string(actual.homing_offset) +
          " != URDF eeprom_homing_offset " + std::to_string(expected.homing_offset) +
          " (recalibrated without updating the URDF?)");
  }
  if (expected.min_counts < actual.min_position_limit) {
    append(problems,
          "URDF lower limit " + std::to_string(expected.min_counts) +
          " counts is below EEPROM min " + std::to_string(actual.min_position_limit));
  }
  if (expected.max_counts > actual.max_position_limit) {
    append(problems,
          "URDF upper limit " + std::to_string(expected.max_counts) +
          " counts is above EEPROM max " + std::to_string(actual.max_position_limit));
  }
  return problems;
}
}     // namespace sts
} // namesapce so101_hardware
