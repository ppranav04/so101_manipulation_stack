#include "so101_hardware/sts_bus.hpp"

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <algorithm>

namespace so101_hardware
{
namespace
{
using Clock = std::chrono::steady_clock;

constexpr std::size_t kStatusOverhead = 6;

std::int64_t micros_since(Clock::time_point start)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start).count();
}
}  // namespace

bool StsBus::open(const std::string & path, int baud)
{
  return port_.open(path, baud);
}

void StsBus::close()
{
  port_.close();
}

bool StsBus::is_open() const
{
  return port_.is_open();
}

TransactResult StsBus::transact(
  std::uint8_t id, std::uint8_t instruction,
  const std::uint8_t * params, std::size_t param_count, std::size_t expected_param_count,
  std::chrono::microseconds timeout)
{
  const auto start = Clock::now();
  const auto deadline = start + timeout;
  TransactResult result;

  auto done = [&](BusStatus status){
      result.status = status;
      result.elapsed_us = micros_since(start);
      return result;
    };

  if (!port_.is_open()) {
    result.error_number = EBADF;
    return done(BusStatus::io_error);
  }


  const std::size_t reply_len = kStatusOverhead + expected_param_count;
  if (reply_len > rx_.size()) {
    result.error_number = EINVAL;
    return done(BusStatus::io_error);
  }

  BusStatus status = send(id, instruction, params, param_count, result.error_number);
  if (status != BusStatus::ok) {
    return done(status);
  }

  status = read_exact(reply_len, deadline, result.error_number);
  if (status != BusStatus::ok) {
    return done(status);
  }

  // Integrity first (checksum, header, length), then ID: parse status owns that order
  result.packet = sts::parse_status(rx_.data(), reply_len, id);
  result.parse_status = result.packet.status;
  if (result.parse_status != sts::ParseStatus::kOk) {
    return done(BusStatus::parse_error);
  }
  if (result.packet.param_count != expected_param_count) {
    result.parse_status = sts::ParseStatus::kBadLength;
    return done(BusStatus::parse_error);
  }

  return done(BusStatus::ok);
}

TransactResult StsBus::ping(std::uint8_t id, std::chrono::microseconds timeout)
{
  return transact(id, sts::kInstPing, nullptr, 0, 0, timeout);
}

BusStatus StsBus::send(
  std::uint8_t id, std::uint8_t instruction,
  const std::uint8_t * params, std::size_t param_count, int & error_number)
{
  const std::size_t tx_len = sts::encode_instruction(id, instruction, params, param_count,
      tx_.data(), tx_.size());
  if (tx_len == 0) {
    error_number = EINVAL;
    return BusStatus::io_error;
  }

  // Drop stale input e.g. a late reply to an earlier transaction that timed out.
  // Without this that late reply would be parsed as the answer to this request
  if (::tcflush(port_.fd(), TCIFLUSH) != 0) {
    error_number = errno;
    return BusStatus::io_error;
  }

  return write_all(tx_len, error_number);
}

BusStatus StsBus::write_all(std::size_t count, int & error_number)
{
  std::size_t sent = 0;
  while (sent < count) {
    const ssize_t n = ::write(port_.fd(), tx_.data() + sent, count - sent);
    if (n > 0) {
      sent += static_cast<std::size_t>(n);
      continue;
    }

    if (n < 0 && errno == EINTR) {
      continue;
    }
    error_number = (n < 0) ? errno : EIO;
    return BusStatus::io_error;
  }

  return BusStatus::ok;
}

BusStatus StsBus::read_exact(
  std::size_t count, std::chrono::steady_clock::time_point deadline, int & error_number)
{
  std::size_t got = 0;
  while (got < count) {
    const auto now = Clock::now();
    if (now >= deadline) {
      return BusStatus::timeout;
    }

    const auto remaining_ms = std::chrono::ceil<std::chrono::milliseconds>(deadline - now).count();

    pollfd pfd{};
    pfd.fd = port_.fd();
    pfd.events = POLLIN;
    const int ready = ::poll(&pfd, 1, static_cast<int>(remaining_ms));
    if (ready < 0) {
      if(errno == EINTR) {
        continue;
      }
      error_number = errno;
      return BusStatus::io_error;
    }
    if (ready == 0) {
      continue;
    }

    if (pfd.revents & POLLIN) {
      const ssize_t n = ::read(port_.fd(), rx_.data() + got, count - got);
      if (n > 0) {
        got += static_cast<std::size_t>(n);
        continue;
      }
      if (n < 0 && (errno == EINTR || errno == EAGAIN)) {
        continue;
      }
      if (n == 0 && !(pfd.revents & (POLLHUP | POLLERR))) {
        continue;
      }
      error_number = (n < 0) ? errno : EIO;
      return BusStatus::io_error;
    }
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      error_number = EIO;
      return BusStatus::io_error;
    }
  }
  return BusStatus::ok;
}

TransactResult StsBus::read_registers(
  std::uint8_t id, std::uint8_t address, std::uint8_t length,
  std::uint8_t * out, std::size_t out_size, std::chrono::microseconds timeout)
{
  if (out == nullptr || length == 0 || out_size < length) {
    TransactResult result;
    result.error_number = EINVAL;
    return result;
  }


  const std::uint8_t params[2] = {address, length};
  TransactResult result = transact(id, sts::kInstRead, params, 2, length, timeout);
  if (result.status == BusStatus::ok) {
    std::memcpy(out, rx_.data() + result.packet.params_offset, length);
  }
  return result;
}

TransactResult StsBus::write_registers(
  std::uint8_t id, std::uint8_t address, const std::uint8_t * data, std::uint8_t length,
  std::chrono::microseconds timeout)
{
  if (data == nullptr || length == 0 || length > kMaxDataPerServo || id == sts::kBroadcastId) {
    TransactResult result;
    result.error_number = EINVAL;
    return result;
  }

  std::array<std::uint8_t, 1 + kMaxDataPerServo> params{};
  params[0] = address;
  std::memcpy(params.data() + 1, data, length);
  return transact(id, sts::kInstWrite, params.data(), 1u + length, 0, timeout);
}

BusStatus StsBus::sync_write(
  const std::uint8_t * ids, std::size_t id_count, std::uint8_t address, std::uint8_t length,
  const std::uint8_t * data, int & error_number)
{
  if (!port_.is_open()) {
    error_number = EBADF;
    return BusStatus::io_error;
  }
  if (ids == nullptr || data == nullptr || id_count == 0 || id_count > kMaxSyncServos ||
    length == 0 || length > kMaxDataPerServo)
  {
    error_number = EINVAL;
    return BusStatus::io_error;
  }

  // Params: ADDR LEN, then for each servo its ID followed by its length data bytes.
  std::array<std::uint8_t, kSyncWriteMaxParams> params{};
  params[0] = address;
  params[1] = length;
  std::size_t n = 2;
  for (std::size_t k = 0; k < id_count; ++k) {
    params[n++] = ids[k];
    std::memcpy(params.data() + n, data + k * length, length);
    n += length;
  }
  return send(sts::kBroadcastId, sts::kInstSyncWrite, params.data(), n, error_number);
}

SyncReadResult StsBus::sync_read(
  const std::uint8_t * ids, std::size_t id_count, std::uint8_t address, std::uint8_t length,
  std::uint8_t * out, std::size_t out_size, std::chrono::microseconds timeout)
{
  const auto start = Clock::now();
  const auto deadline = start + timeout;
  SyncReadResult result;

  auto done = [&](BusStatus status){
      result.status = status;
      result.elapsed_us = micros_since(start);
      return result;
    };

  if(!port_.is_open()) {
    result.error_number = EBADF;
    return done(BusStatus::io_error);
  }
  if (ids == nullptr || id_count == 0 || id_count > kMaxSyncServos ||
    length == 0 || length > kMaxDataPerServo ||
    out == nullptr || out_size < id_count * length)
  {
    result.error_number = EINVAL;
    return done(BusStatus::io_error);
  }

    // Params: start address, data length, then the IDs in the order the servos reply.
  std::array<std::uint8_t, 2 + kMaxSyncServos> params{};
  params[0] = address;
  params[1] = length;
  std::copy(ids, ids + id_count, params.begin() + 2);

  BusStatus status = send(sts::kBroadcastId, sts::kInstSyncRead, params.data(), 2 + id_count,
        result.error_number);
  if (status != BusStatus::ok) {
    return done(status);
  }


    // Each servo answers with its own status packet, list order: FF FF ID LEN ERR data CHK
  const std::size_t reply_len = kStatusOverhead + length;
  for (std::size_t k = 0; k < id_count; ++k) {
    result.failed_index = k;
    status = read_exact(reply_len, deadline, result.error_number);
    if (status != BusStatus::ok) {
      return done(status);
    }
    const sts::StatusPacket packet = sts::parse_status(rx_.data(), reply_len, ids[k]);
    result.parse_status = packet.status;
    if (packet.status != sts::ParseStatus::kOk) {
      return done(BusStatus::parse_error);
    }
    if (packet.param_count != length) {
      result.parse_status = sts::ParseStatus::kBadLength;
      return done(BusStatus::parse_error);
    }

    result.servo_errors[k] = packet.servo_error;
    std::memcpy(out + k * length, rx_.data() + packet.params_offset, length);
  }

  return done(BusStatus::ok);
}


} // namespace so101_hardware
