#include "so101_hardware/sts_protocol.hpp"

#include <cstddef>
#include <cstdint>

namespace so101_hardware
{
namespace sts
{
std::uint8_t checksum(const std::uint8_t * data, std::size_t size)
{
  unsigned int sum = 0;
  for (std::size_t i = 0; i < size; ++i) {
    sum += data[i];
  }
  return static_cast<std::uint8_t>(~sum & 0xFFu);
}

std::size_t encode_instruction(
  std::uint8_t id, std::uint8_t instruction,
  const std::uint8_t * params, std::size_t param_count, std::uint8_t * out, std::size_t out_size)
{
  if (out == nullptr || param_count > kMaxParams || (param_count > 0 && params == nullptr )) {
    return 0;
  }

  const std::size_t total = param_count + kPacketOverhead;
  if (out_size < total) {
    return 0;
  }

  out[0] = kHeader;
  out[1] = kHeader;
  out[2] = id;
  out[3] = static_cast<std::uint8_t>(param_count + 2);
  out[4] = instruction;
  for (std::size_t i = 0; i < param_count; ++i) {
    out[5 + i] = params[i];
  }
            // Checksum covers ID, LEN, INST and parallel: bytes [2, total -1)
  out[total - 1] = checksum(out + 2, total - 3);
  return total;
}

StatusPacket parse_status(const std::uint8_t * data, std::size_t size, std::uint8_t expected_id)
{
  StatusPacket result;
  if(data == nullptr || size < kPacketOverhead) {
    result.status = ParseStatus::kTruncated;
    return result;
  }

  if(data[0] != kHeader || data[1] != kHeader) {
    result.status = ParseStatus::kBadHeader;
    return result;
  }
  const std::size_t length = data[3];
  if (length < 2) {
    result.status = ParseStatus::kBadLength;
    return result;
  }
  const std::size_t total = length + 4;
  if (size < total) {
    result.status = ParseStatus::kTruncated;
    return result;
  }
            // Check integrity before trusting any field
  if (checksum(data + 2, total - 3) != data[total - 1]) {
    result.status = ParseStatus::kBadChecksum;
    return result;
  }
  if (data[2] != expected_id) {
    result.status = ParseStatus::kWrongId;
    return result;
  }

  result.status = ParseStatus::kOk;
  result.servo_error = data[4];
  result.params_offset = 5;
  result.param_count = length - 2;
  return result;

}
} // namespace
} // namespace so101_hardware
