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
  ok,             // the reply arrived and was valid
  timeout,        // the servo didn't answer in time
  io_error,       // Linux itself failed for example the USB cable was pulled
  parse_error     // bytes arrived but were wrong such as a bad checksum or the wrong ID
};

struct TransactResult
{
  BusStatus status{BusStatus::io_error};
  sts::ParseStatus parse_status{sts::ParseStatus::kOk};
  sts::StatusPacket packet;
  int error_number{0};
  std::int64_t elapsed_us{0};
};

// Most servos one SYNC READ can address: size the result and the bus buffers.
constexpr std::size_t kMaxSyncServos = 6;
// failed_index value when no servo's reply is to blame (bad arguments, flush or write error).
constexpr std::size_t kNoServo = kMaxSyncServos;

struct SyncReadResult
{
  BusStatus status{BusStatus::io_error};
  sts::ParseStatus parse_status{sts::ParseStatus::kOk};
  std::size_t failed_index{kNoServo}; // position in ids[] whose reply failed; valid only if !ok
  std::array<std::uint8_t, kMaxSyncServos> servo_errors{};
  int error_number{0};
  std::int64_t elapsed_us{0};
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

  TransactResult ping(std::uint8_t id, std::chrono::microseconds timeout);

  TransactResult read_registers(
    std::uint8_t id, std::uint8_t address, std::uint8_t length,
    std::uint8_t * out, std::size_t out_size, std::chrono::microseconds timeout);

  // Writes length bytes from data to one servo, starting at address, and waits for its
  // status reply (a WRITE to the broadcast ID gets no reply, so it is rejected).
  TransactResult write_registers(
    std::uint8_t id, std::uint8_t address, const std::uint8_t * data, std::uint8_t length,
    std::chrono::microseconds timeout);

  // Reads the same register block from several servos with one request.
  // out receives id_count * length bytes, servo k's block at out + k * length.
  // On any status other than ok, out may be partly written and must not be used.
  SyncReadResult sync_read(
    const std::uint8_t * ids, std::size_t id_count, std::uint8_t address, std::uint8_t length,
    std::uint8_t * out, std::size_t out_size, std::chrono::microseconds timeout);

  // Writes length bytes at address to several servos in one broadcast packet. Servos never
  // reply to SYNC WRITE, so ok means "sent", not "received". data holds id_count * length
  // bytes, servo k's block at data + k * length.
  BusStatus sync_write(
    const std::uint8_t * ids, std::size_t id_count, std::uint8_t address, std::uint8_t length,
    const std::uint8_t * data, int & error_number);

private:
  BusStatus send(
    std::uint8_t id, std::uint8_t instruction,
    const std::uint8_t * params, std::size_t param_count, int & error_number);

  BusStatus write_all(std::size_t count, int & error_number);
  BusStatus read_exact(
    std::size_t count, std::chrono::steady_clock::time_point deadline,
    int & error_number);

  // Buffer sizing. Packet = FF FF ID LEN ... CHK

  static constexpr std::size_t kMaxDataPerServo = 6;
  // SYNC WRITE parameters: ADDR LEN, then per servo its ID and data bytes.
  static constexpr std::size_t kSyncWriteMaxParams = 2 + kMaxSyncServos * (1 + kMaxDataPerServo);
  static constexpr std::size_t kTxBufferSize = 8 + kMaxSyncServos * (1 + kMaxDataPerServo);
  static constexpr std::size_t kRxBufferSize = kMaxSyncServos * (6 + kMaxDataPerServo);

  SerialPort port_;
  std::array<std::uint8_t, kTxBufferSize> tx_{};
  std::array<std::uint8_t, kRxBufferSize> rx_{};
};

} // namespace so101_hardware

#endif // SO101_HARDWARE__STS_BUS_HPP_
