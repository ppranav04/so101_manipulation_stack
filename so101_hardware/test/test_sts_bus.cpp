#include <gtest/gtest.h>

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
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
