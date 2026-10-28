#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>

#include "so101_hardware/sts_registers.hpp"

namespace sts = so101_hardware::sts;
namespace reg = so101_hardware::sts::reg;


namespace
{
using Block = std::array<std::uint8_t, reg::kConfigBlockLength>;

void put_u8(Block & b, std::uint8_t address, std::uint8_t v)
{
  b[address - reg::kConfigBlockStart] = v;
}

void put_u16(Block & b, std::uint8_t address, std::uint16_t v)
{
  put_u8(b, address, static_cast<std::uint8_t>(v & 0xFFu));
  put_u8(b, static_cast<std::uint8_t>(address + 1), static_cast<std::uint8_t>(v >> 8));
}
Block make_block(std::uint8_t id, std::uint16_t homing_raw)
{
  Block b{};
  put_u8(b, reg::kId, id);
  put_u8(b, reg::kReturnDelay, 0);
  put_u16(b, reg::kMinPositionLimit, 500);
  put_u16(b, reg::kMaxPositionLimit, 3500);
  put_u16(b, reg::kHomingOffset, homing_raw);
  put_u8(b, reg::kOperatingMode, reg::kModePosition);
  put_u8(b, reg::kTorqueEnable, 1);
  return b;
}

sts::ServoConfig decoded(const Block & b)
{
  sts::ServoConfig c;
  EXPECT_TRUE(sts::decode_config_block(b.data(), b.size(), c));
  return c;
}

sts::ServoExpectation matching(std::uint8_t id, std::int32_t homing)
{
  sts::ServoExpectation e;
  e.id = id;
  e.homing_offset = homing;
  e.min_counts = 1000;
  e.max_counts = 3000;
  return e;
}

bool contains(const std::string & s, const char * needle)
{
  return s.find(needle) != std::string::npos;
}
} // namespace

TEST(DecodeConfigBlock, DecodesAllFields) {
        const sts::ServoConfig c = decoded(make_block(3, 100));
        EXPECT_EQ(c.id, 3);
        EXPECT_EQ(c.return_delay, 0);
        EXPECT_EQ(c.min_position_limit, 500);
        EXPECT_EQ(c.max_position_limit, 3500);
        EXPECT_EQ(c.homing_offset, 100);
        EXPECT_EQ(c.operating_mode, reg::kModePosition);
        EXPECT_TRUE(c.torque_enabled);
}

TEST(DecodeConfigBlock, NegativeHomingOffsetUsesSignBit11) {
        // Two's complement would read 0x0864 as +2148; sign-magnitude reads it as -100.
        EXPECT_EQ(decoded(make_block(1, 0x0800 | 100)).homing_offset, -100);
}

TEST(DecodeConfigBlock, ZeroHomingOffset) {
        EXPECT_EQ(decoded(make_block(1, 0)).homing_offset, 0);
}

TEST(DecodeConfigBlock, RejectsShortBlockAndNull) {
        const Block b = make_block(1, 0);
        sts::ServoConfig c;
        EXPECT_FALSE(sts::decode_config_block(b.data(), b.size() - 1, c));
        EXPECT_FALSE(sts::decode_config_block(nullptr, b.size(), c));
}

TEST(CheckServoConfig, MatchingConfigHasNoProblems) {
        EXPECT_TRUE(sts::check_servo_config(decoded(make_block(2, 50)), matching(2, 50)).empty());
}

TEST(CheckServoConfig, ReportsNonPositionMode) {
        Block b = make_block(2, 50);
        put_u8(b, reg::kOperatingMode, 1);
        EXPECT_TRUE(contains(sts::check_servo_config(decoded(b), matching(2, 50)),
      "operating mode"));
}

TEST(CheckServoConfig, ReportsHomingOffsetMismatch) {
        EXPECT_TRUE(contains(sts::check_servo_config(decoded(make_block(2, 50)), matching(2, 51)),
      "homing offset"));
}

TEST(CheckServoConfig, ReportsUrdfRangeOutsideEepromLimits) {
        sts::ServoExpectation e = matching(2, 50);
        e.min_counts = 400; // EEPROM min is 500
        e.max_counts = 3600; // EEPROM max is 3500
        const std::string p = sts::check_servo_config(decoded(make_block(2, 50)), e);
        EXPECT_TRUE(contains(p, "lower limit")) << "problems were: " << p;
        EXPECT_TRUE(contains(p, "upper limit")) << "problems were: " << p;
}

TEST(CheckServoConfig, ReportsEveryProblemNotJustTheFirst) {
        Block b = make_block(2, 50);
        put_u8(b, reg::kOperatingMode, 1);
        const std::string p = sts::check_servo_config(decoded(b), matching(2, 99));
        EXPECT_TRUE(contains(p, "operating mode"));
        EXPECT_TRUE(contains(p, "homing offset"));
}
