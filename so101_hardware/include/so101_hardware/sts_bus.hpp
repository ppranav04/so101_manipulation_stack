#ifndef SO101_HARDWARE__STS_BUS_HPP_
#define SO101_HARDWARE__STS_BUS_HPP_

#include "so101_hardware/sts_protocol.hpp"
#include "so101_hardware/serial_port.hpp"

#include <array>
#include <cstdint>
#include <chrono>
#include <cstddef>
#include <string>

namespace so101_hardware
{
enum class BusStatus
{
  ok,       // the reply arrived and was valud
  timeout,       // the servo didn't answer in time
  io_error,       // Linux itself failed for example the USB cable was pulled
  parse_error       // bytes arrived but were wrong such as a bad checksum or the wrong ID
};

struct TransactResult
{
  BusStatus status{BusStatus::io_error};
  sts::ParseStatus parse_status{sts::ParseStatus::kOk};
  sts::StatusPacket packet;
  int error_number{0};
  std::int64_t elapsed_us;
};

class StsBus{
public:
  StsBus() = default;
  StsBus(const StsBus &) = delete;
  StsBus & operator=(const StsBus &) = delete;

  [[nodiscard]] bool open(const std::string & path, int baud);
  void close();
  bool is_open() const;

  TransactResult transact(
    std::uint8_t id, std::uint8_t instruction,
    const std::uint8_t * params, std::size_t param_count,
    std::size_t expected_param_count, std::chrono::microseconds timeout
  );

  TransactResult ping(std::uint8_t count, std::chrono::microseconds timeout);

private:
  BusStatus write_all(std::size_t count, int & error_number);
  BusStatus read_exact(
    std::size_t count, std::chrono::steady_clock::time_point deadline,
    int & error_number);

        // Buffer sizing. Packet = fFF FF ID LEN ... CHK

  static constexpr std::size_t kNumServos = 6;
  static constexpr std::size_t kMaxDataPerServo = 6;
  static constexpr std::size_t kTxBufferSize = 7 + kNumServos * (1 + kMaxDataPerServo);
  static constexpr std::size_t kRxBufferSize = kNumServos * (6 + kMaxDataPerServo);

  SerialPort port_;
  std::array<std::uint8_t, kTxBufferSize> tx_{};
  std::array<std::uint8_t, kRxBufferSize> rx_{};
};

} // namespace so101_hardware

#endif // SO101_HARDWARE__STS_BUS_HPP_
