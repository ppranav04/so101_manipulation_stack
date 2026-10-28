#include "so101_hardware/sts_bus.hpp"

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>

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

  const std::size_t tx_len = sts::encode_instruction(id, instruction, params, param_count,
      tx_.data(), tx_.size());
  if (tx_len == 0) {
    result.error_number = EINVAL;
    return done(BusStatus::io_error);
  }

    // Drop stale input e.g. a late reply to an earlier transaction that timed out.
    // Wihtout thism that late reply would be parsed as the answer to this request
  if(::tcflush(port_.fd(), TCIFLUSH) != 0) {
    result.error_number = errno;
    return done(BusStatus::io_error);
  }

  BusStatus status = write_all(tx_len, result.error_number);
  if (status != BusStatus::ok) {
    return done (status);
  }
  status = read_exact(reply_len, deadline, result.error_number);
  if (status != BusStatus::ok) {
    return done (status);
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


} // namespace so101_hardware
