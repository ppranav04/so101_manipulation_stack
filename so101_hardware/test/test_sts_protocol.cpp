#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "so101_hardware/sts_protocol.hpp"

namespace sts = so101_hardware::sts;

namespace
{
using Bytes = std::vector<std::uint8_t>;

    // Encodes into a roomy buffer and returns exactly the bytes written.
Bytes encode(std::uint8_t id, std::uint8_t instruction, const Bytes & params)
{
  std::array<std::uint8_t, 64> out{};
  const std::size_t n = sts::encode_instruction(id, instruction, params.data(), params.size(),
      out.data(), out.size());
  return Bytes(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(n));
}

sts::StatusPacket parse(const Bytes & bytes, std::uint8_t expected_id)
{
  return sts::parse_status(bytes.data(), bytes.size(), expected_id);
}
} // namespace

TEST(StsEncode, PingId1) {
    EXPECT_EQ(encode(1, sts::kInstPing, {}), (Bytes{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}));
}

TEST(StsEncode, ReadPresentPosition) {
    // Read 2 bytes starting at register 0x38 (present position)
    EXPECT_EQ(encode(1, sts::kInstRead, {0x38, 0x02}),
    (Bytes{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE}));
}

TEST(StsEncode, BufferTooSmallReturnsZero) {
    std::array<std::uint8_t, 3> out{};
    EXPECT_EQ(sts::encode_instruction(1, sts::kInstPing, nullptr, 0, out.data(), out.size()), 0u);
}

// --Parsing--

TEST(StsParse, PingReplyOk) {
    const auto r = parse({0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC}, 1);
    EXPECT_EQ(r.status, sts::ParseStatus::kOk);
    EXPECT_EQ(r.servo_error, 0);
    EXPECT_EQ(r.param_count, 0u);
}

TEST(StsParams, ReadReplyExposesParams) {
    // Present position 2048 = 0x0800, sent low byte first.
    const Bytes b{0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2};
    const auto r = parse(b, 1);
    ASSERT_EQ(r.status, sts::ParseStatus::kOk);
    ASSERT_EQ(r.param_count, 2u);
    EXPECT_EQ(sts::make_u16(b[r.params_offset], b[r.params_offset + 1]), 2048);
}

TEST(StsParse, BadChecksumRejected) {
    EXPECT_EQ(parse({0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFD}, 1).status,
    sts::ParseStatus::kBadChecksum);
}

TEST(StsParse, WrongIdRejected) {
    // Valid packet from ID 2 while expecting ID 1.
    EXPECT_EQ(parse({0xFF, 0xFF, 0x02, 0x02, 0x00, 0xFB}, 1).status, sts::ParseStatus::kWrongId);
}

TEST(StsParse, TooShortIsTruncated) {
    EXPECT_EQ(parse({0xFF, 0xFF, 0x01, 0x02}, 1).status, sts::ParseStatus::kTruncated);
}

TEST(StsParse, ShorterThanLengthFieldIsTruncated) {
    // LEN says 4 moree bytes follow, but only 2 arrived
    EXPECT_EQ(parse({0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00}, 1).status, sts::ParseStatus::kTruncated);
}

TEST(StsParse, BadHeaderRejected) {
    EXPECT_EQ(parse({0xFE, 0xFF, 0x01, 0x02, 0x00, 0xFC}, 1).status, sts::ParseStatus::kBadHeader);
}

TEST(StsParse, ServoErrorByteExposed) {
    const auto r = parse({0xFF, 0xFF, 0x01, 0x02, 0x20, 0xDC}, 1);
    EXPECT_EQ(r.status, sts::ParseStatus::kOk);
    EXPECT_EQ(r.servo_error, 0x20);
}

// --- Byte Helpers --

TEST(StsBytes, LittleEndianRoundTrip) {
    EXPECT_EQ(sts::make_u16(0x00, 0x08), 2048);
    EXPECT_EQ(sts::low_byte(2048), 0x00);
    EXPECT_EQ(sts::high_byte(2048), 0x08);
}

TEST(StsBytes, SignMagnitude) {
    EXPECT_EQ(sts::decode_sign_magnitude(0x0064, 15), 100);
    EXPECT_EQ(sts::decode_sign_magnitude(0x8064, 15), -100);
}
