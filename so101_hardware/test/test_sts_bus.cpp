#include <gtest/gtest.h>

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "so101_hardware/sts_bus.hpp"
#include "so101_hardware/sts_protocol.hpp"
#include "so101_hardware/sts_registers.hpp"

using so101_hardware::BusStatus;
using so101_hardware::StsBus;
using namespace std::chrono_literals;

namespace
{
// Known-answer packets (checksum = ~(ID + LEN + payload) & 0xFF).
const std::vector<std::uint8_t> kPingId1 = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
const std::vector<std::uint8_t> kReplyId1 = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
const std::vector<std::uint8_t> kReplyId2 = {0xFF, 0xFF, 0x02, 0x02, 0x00, 0xFB};

// Read up to n bytes from fd, giving up at the timeout. Returns bytes actually read.
std::size_t read_n(int fd, std::uint8_t * buf, std::size_t n, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::size_t got = 0;
  while (got < n) {
    const auto left = std::chrono::ceil<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (left <= 0) {
      break;
    }
    pollfd pfd{};
    pfd.fd = fd;
    pfd.events = POLLIN;
    if (::poll(&pfd, 1, static_cast<int>(left)) <= 0) {
      continue;
    }
    const ssize_t r = ::read(fd, buf + got, n - got);
    if (r > 0) {
      got += static_cast<std::size_t>(r);
    } else if (r == 0 || (errno != EINTR && errno != EAGAIN)) {
      break;
    }
  }
  return got;
}

void write_bytes(int fd, const std::uint8_t * data, std::size_t n)
{
  std::size_t sent = 0;
  while (sent < n) {
    const ssize_t w = ::write(fd, data + sent, n - sent);
    if (w > 0) {
      sent += static_cast<std::size_t>(w);
    } else if (w < 0 && errno != EINTR) {
      return;
    }
  }
}

// SYNC READ of 6 bytes from address 56 (0x38) for IDs 1 and 2:
// FF FF FE LEN=N+4 INST=82 ADDR LEN_DATA IDs... CHK
const std::vector<std::uint8_t> kSyncReadId1Id2 =
{0xFF, 0xFF, 0xFE, 0x06, 0x82, 0x38, 0x06, 0x01, 0x02, 0x38};

// One servo's reply: FF FF ID LEN ERR data CHK, checksum over ID..data.
std::vector<std::uint8_t> status_packet(
  std::uint8_t id, std::uint8_t error,
  const std::vector<std::uint8_t> & data)
{
  std::vector<std::uint8_t> p = {0xFF, 0xFF, id, static_cast<std::uint8_t>(data.size() + 2),
    error};
  p.insert(p.end(), data.begin(), data.end());
  p.push_back(so101_hardware::sts::checksum(p.data() + 2, p.size() - 2));
  return p;
}

const std::vector<std::uint8_t> kData1 = {0x00, 0x08, 0x10, 0x00, 0x20, 0x00};
const std::vector<std::uint8_t> kData2 = {0x01, 0x0C, 0x00, 0x80, 0x05, 0x04};
const std::uint8_t kIds[] = {1, 2};

void write_bytes(int fd, const std::vector<std::uint8_t> & bytes)
{
  write_bytes(fd, bytes.data(), bytes.size());
}
}  // namespace

// The bus opens the pty's slave end as if it were /dev/ttyACM0. A thread on the
// master end plays the servo: it waits for the full request, then runs a reply script.
class StsBusPty : public ::testing::Test
{
protected:
  void SetUp() override
  {
    master_ = ::posix_openpt(O_RDWR | O_NOCTTY);
    ASSERT_GE(master_, 0);
    ASSERT_EQ(::grantpt(master_), 0);
    ASSERT_EQ(::unlockpt(master_), 0);
    char name[128] = {};
    ASSERT_EQ(::ptsname_r(master_, name, sizeof(name)), 0);
    ASSERT_TRUE(bus_.open(name, 1000000));
  }

  void TearDown() override
  {
    if (servo_.joinable()) {
      servo_.join();
    }
    bus_.close();
    if (master_ >= 0) {
      ::close(master_);
    }
  }

  // Start the fake servo. It only replies after reading the whole request, so the
  // bus's input flush always happens before any reply bytes exist.
  void start_servo(std::size_t request_len, std::function<void(int)> reply)
  {
    servo_ = std::thread(
      [this, request_len, reply]() {
        request_.assign(request_len, 0);
        request_.resize(read_n(master_, request_.data(), request_len, 500ms));
        if (request_.size() == request_len) {
          reply(master_);
        }
      });
  }

  int master_{-1};
  StsBus bus_;
  std::thread servo_;
  std::vector<std::uint8_t> request_;
};

TEST_F(StsBusPty, PingOk)
{
  start_servo(kPingId1.size(), [](int fd) {write_bytes(fd, kReplyId1);});

  const auto r = bus_.ping(1, 50ms);
  servo_.join();

  EXPECT_EQ(request_, kPingId1);  // exact bytes on the wire
  EXPECT_EQ(r.status, BusStatus::ok);
  EXPECT_EQ(r.packet.servo_error, 0);
  EXPECT_GT(r.elapsed_us, 0);
}

TEST_F(StsBusPty, SilentServoTimesOutWithinDeadline)
{
  // No servo thread: the request goes out and nothing ever answers.
  const auto r = bus_.ping(1, 20ms);

  EXPECT_EQ(r.status, BusStatus::timeout);
  EXPECT_GE(r.elapsed_us, 20000);
  EXPECT_LT(r.elapsed_us, 40000);  // generous upper bound for a loaded CI runner
}

TEST_F(StsBusPty, SplitReplyIsReassembled)
{
  start_servo(
    kPingId1.size(), [](int fd) {
      write_bytes(fd, kReplyId1.data(), 3);
      std::this_thread::sleep_for(5ms);
      write_bytes(fd, kReplyId1.data() + 3, 3);
    });

  const auto r = bus_.ping(1, 50ms);
  EXPECT_EQ(r.status, BusStatus::ok);
}

TEST_F(StsBusPty, PartialReplyTimesOut)
{
  start_servo(kPingId1.size(), [](int fd) {write_bytes(fd, kReplyId1.data(), 4);});

  const auto r = bus_.ping(1, 20ms);
  EXPECT_EQ(r.status, BusStatus::timeout);
}

TEST_F(StsBusPty, WrongIdIsParseError)
{
  start_servo(kPingId1.size(), [](int fd) {write_bytes(fd, kReplyId2);});

  const auto r = bus_.ping(1, 50ms);
  EXPECT_EQ(r.status, BusStatus::parse_error);
  EXPECT_EQ(r.parse_status, so101_hardware::sts::ParseStatus::kWrongId);
}

TEST_F(StsBusPty, StaleInputIsFlushedBeforeRequest)
{
  // A late reply from an earlier, timed-out transaction is already waiting.
  write_bytes(master_, kReplyId2);
  std::this_thread::sleep_for(5ms);  // let it reach the slave's input queue

  start_servo(kPingId1.size(), [](int fd) {write_bytes(fd, kReplyId1);});

  const auto r = bus_.ping(1, 50ms);
  EXPECT_EQ(r.status, BusStatus::ok);  // without the flush this would be kWrongId
}

TEST_F(StsBusPty, SyncReadTwoServosOk)
{
  start_servo(
    kSyncReadId1Id2.size(), [](int fd) {
      write_bytes(fd, status_packet(1, 0x00, kData1));
      write_bytes(fd, status_packet(2, 0x00, kData2));
    });

  std::uint8_t out[12] = {};
  const auto r = bus_.sync_read(kIds, 2, 0x38, 6, out, sizeof(out), 50ms);
  servo_.join();

  EXPECT_EQ(request_, kSyncReadId1Id2);
  ASSERT_EQ(r.status, BusStatus::ok);
  EXPECT_EQ(std::vector<std::uint8_t>(out, out + 6), kData1);
  EXPECT_EQ(std::vector<std::uint8_t>(out + 6, out + 12), kData2);
  EXPECT_EQ(r.servo_errors[0], 0);
  EXPECT_EQ(r.servo_errors[1], 0);
}

TEST_F(StsBusPty, SyncReadReportsServoErrorFlags)
{
  start_servo(
    kSyncReadId1Id2.size(), [](int fd) {
      write_bytes(fd, status_packet(1, 0x00, kData1));
      write_bytes(fd, status_packet(2, 0x20, kData2));
    });

  std::uint8_t out[12] = {};
  const auto r = bus_.sync_read(kIds, 2, 0x38, 6, out, sizeof(out), 50ms);

  EXPECT_EQ(r.status, BusStatus::ok);  // the bus worked; the servo reports a fault
  EXPECT_EQ(r.servo_errors[0], 0x00);
  EXPECT_EQ(r.servo_errors[1], 0x20);
}

TEST_F(StsBusPty, SyncReadSilentSecondServoTimesOut)
{
  start_servo(
    kSyncReadId1Id2.size(), [](int fd) {
      write_bytes(fd, status_packet(1, 0x00, kData1));
    });

  std::uint8_t out[12] = {};
  const auto r = bus_.sync_read(kIds, 2, 0x38, 6, out, sizeof(out), 20ms);

  EXPECT_EQ(r.status, BusStatus::timeout);
  EXPECT_EQ(r.failed_index, 1u);
}

TEST_F(StsBusPty, SyncReadOutOfOrderIsParseError)
{
  start_servo(
    kSyncReadId1Id2.size(), [](int fd) {
      write_bytes(fd, status_packet(2, 0x00, kData2));
      write_bytes(fd, status_packet(1, 0x00, kData1));
    });

  std::uint8_t out[12] = {};
  const auto r = bus_.sync_read(kIds, 2, 0x38, 6, out, sizeof(out), 50ms);

  EXPECT_EQ(r.status, BusStatus::parse_error);
  EXPECT_EQ(r.parse_status, so101_hardware::sts::ParseStatus::kWrongId);
  EXPECT_EQ(r.failed_index, 0u);
}

TEST_F(StsBusPty, SyncReadRejectsTooSmallOutput)
{
  std::uint8_t out[11] = {};  // needs 2 * 6 = 12
  const auto r = bus_.sync_read(kIds, 2, 0x38, 6, out, sizeof(out), 50ms);

  EXPECT_EQ(r.status, BusStatus::io_error);
  EXPECT_EQ(r.error_number, EINVAL);
  EXPECT_EQ(r.failed_index, so101_hardware::kNoServo);  // no servo is to blame
}

// WRITE Torque_Enable (40 = 0x28) = 1 to ID 1: FF FF 01 LEN=4 03 28 01 CHK
const std::vector<std::uint8_t> kWriteTorqueOnId1 =
{0xFF, 0xFF, 0x01, 0x04, 0x03, 0x28, 0x01, 0xCE};

TEST_F(StsBusPty, WriteRegistersOk)
{
  start_servo(kWriteTorqueOnId1.size(), [](int fd) {write_bytes(fd, status_packet(1, 0x00, {}));});

  const std::uint8_t on = 1;
  const auto r = bus_.write_registers(1, 40, &on, 1, 50ms);
  servo_.join();

  EXPECT_EQ(request_, kWriteTorqueOnId1);
  EXPECT_EQ(r.status, BusStatus::ok);
  EXPECT_EQ(r.packet.servo_error, 0);
}

TEST_F(StsBusPty, WriteRegistersSilentServoTimesOut)
{
  start_servo(kWriteTorqueOnId1.size(), [](int) {});

  const std::uint8_t on = 1;
  const auto r = bus_.write_registers(1, 40, &on, 1, 20ms);
  servo_.join();

  EXPECT_EQ(r.status, BusStatus::timeout);
}

TEST_F(StsBusPty, WriteRegistersRejectsBadArguments)
{
  const std::uint8_t data[7] = {};
  EXPECT_EQ(bus_.write_registers(1, 40, nullptr, 1, 50ms).error_number, EINVAL);
  EXPECT_EQ(bus_.write_registers(1, 40, data, 0, 50ms).error_number, EINVAL);
  EXPECT_EQ(bus_.write_registers(1, 40, data, 7, 50ms).error_number, EINVAL);
  EXPECT_EQ(bus_.write_registers(0xFE, 40, data, 1, 50ms).error_number, EINVAL);
}

// SYNC WRITE Goal_Position (42 = 0x2A, 2 bytes): ID 1 -> 2048, ID 2 -> 3000.
// FF FF FE LEN=0A 83 2A 02 | 01 00 08 | 02 B8 0B | CHK
const std::vector<std::uint8_t> kSyncWriteGoalId1Id2 =
{0xFF, 0xFF, 0xFE, 0x0A, 0x83, 0x2A, 0x02, 0x01, 0x00, 0x08, 0x02, 0xB8, 0x0B, 0x7A};

TEST_F(StsBusPty, SyncWriteSendsOnePacket)
{
  start_servo(kSyncWriteGoalId1Id2.size(), [](int) {});  // servos never reply to SYNC WRITE

  const std::uint8_t goals[4] = {0x00, 0x08, 0xB8, 0x0B};
  int err = 0;
  const BusStatus s = bus_.sync_write(kIds, 2, 42, 2, goals, err);
  servo_.join();

  EXPECT_EQ(s, BusStatus::ok);
  EXPECT_EQ(request_, kSyncWriteGoalId1Id2);  // exact bytes on the wire
}

TEST_F(StsBusPty, SyncWriteRejectsBadArguments)
{
  const std::uint8_t ids[7] = {1, 2, 3, 4, 5, 6, 7};
  const std::uint8_t data[42] = {};
  int err = 0;
  EXPECT_EQ(bus_.sync_write(nullptr, 2, 42, 2, data, err), BusStatus::io_error);
  EXPECT_EQ(err, EINVAL);
  EXPECT_EQ(bus_.sync_write(ids, 2, 42, 2, nullptr, err), BusStatus::io_error);
  EXPECT_EQ(bus_.sync_write(ids, 0, 42, 2, data, err), BusStatus::io_error);
  EXPECT_EQ(bus_.sync_write(ids, 7, 42, 2, data, err), BusStatus::io_error);
  EXPECT_EQ(bus_.sync_write(ids, 2, 42, 0, data, err), BusStatus::io_error);
  EXPECT_EQ(bus_.sync_write(ids, 2, 42, 7, data, err), BusStatus::io_error);
}

// Hardware-in-the-loop: one SYNC READ of position, velocity and load (56..61) from all six
// servos, timed. Skipped unless SO101_PORT is set. Read-only: no register is written.
TEST(StsBusHardware, SyncReadAllServosLatency)
{
  const char * port = std::getenv("SO101_PORT");
  if (port == nullptr) {
    GTEST_SKIP() << "set SO101_PORT to the by-id path to run against the real arm";
  }

  StsBus bus;
  ASSERT_TRUE(bus.open(port, 1000000));

  constexpr int kRounds = 2000;
  const std::uint8_t ids[] = {1, 2, 3, 4, 5, 6};
  std::uint8_t out[6 * 6] = {};
  std::vector<std::int64_t> samples;
  samples.reserve(kRounds);
  int failures = 0;

  for (int i = 0; i < kRounds; ++i) {
    const auto r = bus.sync_read(ids, 6, 56, 6, out, sizeof(out), 10ms);
    if (r.status == BusStatus::ok) {
      samples.push_back(r.elapsed_us);
    } else {
      ++failures;
      std::printf(
        "round %d failed: status %d at index %zu, errno %d\n",
        i, static_cast<int>(r.status), r.failed_index, r.error_number);
    }
  }

  ASSERT_FALSE(samples.empty());
  std::sort(samples.begin(), samples.end());
  const auto pct = [&](double p) {
      return samples[static_cast<std::size_t>(p * static_cast<double>(samples.size() - 1))];
    };
  std::printf(
    "sync read latency over %zu ok / %d failed: p50 %lld us, p99 %lld us, max %lld us\n",
    samples.size(), failures,
    static_cast<long long>(pct(0.50)), static_cast<long long>(pct(0.99)),
    static_cast<long long>(samples.back()));

  EXPECT_EQ(failures, 0);
}

TEST(StsBus, ClosedBusIsIoError)
{
  StsBus bus;
  const auto r = bus.ping(1, 10ms);
  EXPECT_EQ(r.status, BusStatus::io_error);
  EXPECT_EQ(r.error_number, EBADF);
}

// Hardware-in-the-loop. Skipped unless SO101_PORT names the real adapter.
// Nothing else (e.g. a ROS launch) may have the port open.
TEST(StsBusHardware, PingAllServosLatency)
{
  const char * port = std::getenv("SO101_PORT");
  if (port == nullptr) {
    GTEST_SKIP() << "set SO101_PORT to the by-id path to run against the real arm";
  }

  StsBus bus;
  ASSERT_TRUE(bus.open(port, 1000000));

  constexpr int kRounds = 500;
  constexpr std::uint8_t kFirstId = 1;
  constexpr std::uint8_t kLastId = 6;
  std::vector<std::int64_t> samples;
  samples.reserve(kRounds * (kLastId - kFirstId + 1));
  int failures = 0;

  for (std::uint8_t id = kFirstId; id <= kLastId; ++id) {
    for (int i = 0; i < kRounds; ++i) {
      const auto r = bus.ping(id, 10ms);
      if (r.status == BusStatus::ok) {
        samples.push_back(r.elapsed_us);
      } else {
        ++failures;
        std::printf(
          "id %u round %d failed: status %d errno %d\n",
          id, i, static_cast<int>(r.status), r.error_number);
      }
    }
  }

  ASSERT_FALSE(samples.empty());
  std::sort(samples.begin(), samples.end());
  const auto pct = [&](double p) {
      return samples[static_cast<std::size_t>(p * static_cast<double>(samples.size() - 1))];
    };
  std::printf(
    "ping latency over %zu ok / %d failed: p50 %lld us, p99 %lld us, max %lld us\n",
    samples.size(), failures,
    static_cast<long long>(pct(0.50)), static_cast<long long>(pct(0.99)),
    static_cast<long long>(samples.back()));

  EXPECT_EQ(failures, 0);
}

// Hardware-in-the-loop, read-only: what unit is Present_Velocity in?
// Torque must be off. During the 10 s run, move shoulder_pan (ID 1) back and forth by hand,
// slow and fast, both directions. Fits v_reg = a * v_fd, where v_fd is the central finite
// difference of Present_Position in counts/s. a = 1 means the register is in counts/s.
TEST(StsBusHardware, VelocityUnitAgainstFiniteDifference)
{
  namespace sts = so101_hardware::sts;
  namespace reg = so101_hardware::sts::reg;
  using Clock = std::chrono::steady_clock;

  const char * port = std::getenv("SO101_PORT");
  if (port == nullptr) {
    GTEST_SKIP() << "set SO101_PORT to the by-id path to run against the real arm";
  }

  StsBus bus;
  ASSERT_TRUE(bus.open(port, 1000000));

  const std::uint8_t ids[] = {1};
  std::uint8_t torque = 1;
  const auto tr = bus.read_registers(ids[0], reg::kTorqueEnable, 1, &torque, 1, 10ms);
  ASSERT_EQ(tr.status, BusStatus::ok);
  if (torque != 0) {
    GTEST_SKIP() << "torque is on for ID 1: turn it off before moving the joint by hand";
  }

  constexpr int kSamples = 1000;  // 10 s at 100 Hz
  std::vector<double> t_s;
  std::vector<double> pos;
  std::vector<double> vel;
  t_s.reserve(kSamples);
  pos.reserve(kSamples);
  vel.reserve(kSamples);
  int failures = 0;

  std::printf("move shoulder_pan by hand now, for 10 s\n");
  const auto t0 = Clock::now();
  auto next = t0;
  for (int i = 0; i < kSamples; ++i) {
    next += 10ms;
    std::uint8_t raw[reg::kPresentBlockLength] = {};
    const auto r = bus.sync_read(
      ids, 1, reg::kPresentBlockStart, reg::kPresentBlockLength, raw, sizeof(raw), 10ms);
    const auto now = Clock::now();
    sts::PresentState s;
    if (r.status == BusStatus::ok && sts::decode_present_block(raw, sizeof(raw), s)) {
      t_s.push_back(std::chrono::duration<double>(now - t0).count());
      pos.push_back(s.position);
      vel.push_back(s.velocity);
    } else {
      ++failures;
    }
    std::this_thread::sleep_until(next);
  }

  // Least squares through the origin over samples where the joint is clearly moving.
  constexpr double kMinCountsPerS = 50.0;  // ~4.4 deg/s; below this the difference is noise
  double sxy = 0.0;
  double sxx = 0.0;
  double syy = 0.0;
  int used = 0;
  int sign_agree = 0;
  for (std::size_t k = 1; k + 1 < pos.size(); ++k) {
    const double v_fd = (pos[k + 1] - pos[k - 1]) / (t_s[k + 1] - t_s[k - 1]);
    if (std::abs(v_fd) < kMinCountsPerS) {
      continue;
    }
    sxy += vel[k] * v_fd;
    sxx += v_fd * v_fd;
    syy += vel[k] * vel[k];
    ++used;
    if ((vel[k] > 0.0) == (v_fd > 0.0) && vel[k] != 0.0) {
      ++sign_agree;
    }
  }

  ASSERT_GT(used, 100) << "not enough motion: move the joint during the run";
  const double a = sxy / sxx;
  const double r2 = (sxy * sxy) / (sxx * syy);
  std::printf(
    "velocity fit over %d moving samples: v_reg = %.3f * v_fd (counts/s), R^2 %.3f, "
    "sign agreement %d/%d, read failures %d\n",
    used, a, r2, sign_agree, used, failures);

  EXPECT_EQ(failures, 0);
}
