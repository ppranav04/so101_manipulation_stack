#include <gtest/gtest.h>

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <cstdlib>
#include <string>

#include "so101_hardware/serial_port.hpp"

using so101_hardware::SerialPort;

// No device needed

TEST(SerialPort, FreshPortIsClosed) {
    SerialPort port;
    EXPECT_FALSE(port.is_open());
    EXPECT_EQ(port.fd(), -1);
}

TEST(SerialPort, CloseTwiceIsSafe) {
    SerialPort port;
    port.close();
    port.close();
    EXPECT_FALSE(port.is_open());
}

TEST(SerialPort, MissingPathFailsWithPathInMessage) {
    SerialPort port;
    const std::string path = "/dev/so101_does_not_exist";
    EXPECT_FALSE(port.open(path, 1000000));
    EXPECT_FALSE(port.is_open());
    EXPECT_NE(port.last_error().find(path), std::string::npos) << port.last_error();
}

TEST(SerialPort, UnsupportedBaudFailsWithRateInMessage) {
    SerialPort port;
    EXPECT_FALSE(port.open("/dev/null", 250000));
    EXPECT_NE(port.last_error().find("250000"), std::string::npos) << port.last_error();
}

TEST(SerialPort, NonTtvIsRejected) {
    // /dev/null opens fine but it not a terminal, so TIOCEXCL / termios must fail.
    SerialPort port;
    EXPECT_FALSE(port.open("/dev/null", 1000000));
    EXPECT_FALSE(port.is_open());
}

// Pseudo terminal

class PtyTest : public ::testing::Test{
protected:
  void SetUp() override
  {
    master_ = ::posix_openpt(O_RDWR | O_NOCTTY);
    ASSERT_GE(master_, 0);
    ASSERT_EQ(::grantpt(master_), 0);
    ASSERT_EQ(::unlockpt(master_), 0);
    std::array<char, 128> name{};
    ASSERT_EQ(::ptsname_r(master_, name.data(), name.size()), 0);
    slave_path_ = name.data();
  }

  void TearDown() override
  {
    if (master_ >= 0) {
      ::close(master_);
    }
  }

  int master_{-1};
  std::string slave_path_;

};

TEST_F (PtyTest, OpensAndConfiguresATty) {
    SerialPort port;
    ASSERT_TRUE(port.open(slave_path_, 1000000)) << port.last_error();
    EXPECT_TRUE(port.is_open());
    EXPECT_GE(port.fd(), 0);
}

TEST_F(PtyTest, SecondOpenRefusedWhileHeld) {
    if (::geteuid() == 0) {
    GTEST_SKIP() << "root bypasses TIOCEXCL (CI containers often run as root)";
    }
    SerialPort first;
    ASSERT_TRUE(first.open(slave_path_, 1000000)) << first.last_error();
    SerialPort second;
    EXPECT_FALSE(second.open(slave_path_, 1000000));
}

TEST_F(PtyTest, DestructorClosesTheFd)
{
  int saved_fd = -1;
  {
    SerialPort first;
    ASSERT_TRUE(first.open(slave_path_, 1000000)) << first.last_error();
    saved_fd = first.fd();
    ASSERT_GE(saved_fd, 0);
  }  // first is destroyed here; its destructor must close saved_fd

  // Ask the kernel whether that fd number still exists. A closed fd gives -1 / EBADF.
  const int result = ::fcntl(saved_fd, F_GETFD);
  const int err = errno;  // copy straight away, before anything else can change it
  EXPECT_EQ(result, -1);
  EXPECT_EQ(err, EBADF);
}
