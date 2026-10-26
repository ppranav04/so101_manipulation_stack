#ifndef SO101_HARDWARE__STS_PROTOCOL_HPP_
#define SO101_HARDWARE__STS_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>

namespace so101_hardware
{
namespace sts
{
// Instruction codes
constexpr std::uint8_t kInstPing = 0x01;
constexpr std::uint8_t kInstRead = 0x02;
constexpr std::uint8_t kInstWrite = 0x03;
constexpr std::uint8_t kInstSyncRead = 0x82;
constexpr std::uint8_t kInstSyncWrite = 0x83;

constexpr std::uint8_t kHeader = 0xFF;

// Bytes in every packet except params
constexpr std::uint8_t kPacketOverhead = 6;

// LEN is one byte and equals params + 2, so at most 253 params
constexpr std::size_t kMaxParams = 253;

// Low byte of the bitwise NOT of the sum of data[0..size).
std::uint8_t checksum(const std::uint8_t * data, std::size_t size);

// Writes an instruction packet into out (caller owned, no allocation).
// Returns the number of bytes written or 0 if the packets does not fit or the inputs are invalid

std::size_t encode_instruction(
  std::uint8_t id, std::uint8_t instruction,
  const std::uint8_t * params, std::size_t param_count, std::uint8_t * out, std::size_t out_size);

enum class ParseStatus
{
  kOk,
  kTruncated,
  kBadHeader,
  kBadLength,
  kBadChecksum,
  kWrongId,
};

struct StatusPacket
{
  ParseStatus status{ParseStatus::kTruncated};
  std::uint8_t servo_error{0};
  std::size_t params_offset{0};
  std::size_t param_count{0};
};

// Params one status packet starting at data[0]. Bytes after the packets are ignored.
StatusPacket parse_status(const std::uint8_t * data, std::size_t size, std::uint8_t expected_id);

// STS regisers are a little-endian: low byte first.
constexpr std::uint16_t make_u16(std::uint8_t low, std::uint8_t high)
{
  return static_cast<std::uint16_t>(low | (high << 8));
}

constexpr std::uint8_t low_byte(std::uint16_t value)
{
  return static_cast<std::uint8_t>(value & 0xFFu);
}

constexpr std::uint8_t high_byte(std::uint16_t value)
{
  return static_cast<std::uint8_t>(value >> 8);
}

// Sign-magnitude decode
constexpr std::int32_t decode_sign_magnitude(std::uint16_t raw, unsigned sign_bit)
{
  const unsigned mask = 1u << sign_bit;
  const auto magnitude = static_cast<std::int32_t>(raw & (mask - 1u));
  return (raw & mask) != 0u ? -magnitude : magnitude;
}

} // namespace sts

} //namsepace so101_hardware

#endif // SO101_HARDWARE__STS_PROTOCOL_HPP_
