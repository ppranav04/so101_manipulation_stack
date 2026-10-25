#include "so101_hardware/serial_port.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>

namespace so101_hardware
{
namespace
{
bool to_speed(int baud, speed_t & out)
{
  switch(baud) {
    case 1000000:
      out = B1000000;
      return true;

    case 500000:
      out = B500000;
      return true;
    case 115200:
      out = B115200;
      return true;
    case 57600:
      out = B57600;
      return true;
    case 38400:
      out = B38400;
      return true;
    default:
      return false;
  }
}
}     // namespace

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & path, int baud)
{
  close();
  last_error_.clear();

  speed_t speed = 0;
  if (!to_speed(baud, speed)) {
    last_error_ = "baud rate " + std::to_string(baud) + " is not supported by this driver";
    return false;
  }

  // O_NOCTTY: never become this process's controlling terminal
  // O_CLOEXEC: don't leak the fd into child processes.
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (fd_ < 0) {
    return fail("open", path);
  }

  // Refuse any second opener (e.g. a leftover teleop script) while we hold the bus
  if(::ioctl(fd_, TIOCEXCL) != 0) {
    return fail("ioctl(TIOCEXCL)", path);
  }

  termios tty{};
  if(::tcgetattr(fd_, &tty) != 0) {
    return fail("tcgetattr", path);
  }

  ::cfmakeraw(&tty);
  tty.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::cfsetispeed(&tty, speed) != 0 || ::cfsetospeed(&tty, speed) != 0) {
    return fail("cfsetspeed", path);
  }
  if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
    return fail("tcsetattr", path);
  }

  // tcsetattr reports success if ANY change applied, so read back and check the speed
  termios check{};

  if (::tcgetattr(fd_, &check) != 0) {
    return fail("tcgetattr (verify)", path);
  }

  if (::cfgetispeed(&check) != speed || ::cfgetospeed(&check) != speed) {
    last_error_ = "Driver did not accept baud " + std::to_string(baud) + " on '" + path + "'";
    close();
    return false;
  }

  // Drop stale bytes in both directions so old data is never parsed as a reply.
  if (::tcflush(fd_, TCIOFLUSH) != 0) {
    return fail("tcflush", path);
  }

  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const
{
  return fd_ >= 0;
}

int SerialPort::fd() const
{
  return fd_;
}

const std::string & SerialPort::last_error() const
{
  return last_error_;
}

bool SerialPort::fail(const std::string & step, const std::string & path)
{
  const int err = errno;
  last_error_ = step + " failed on '" + path + "': " + std::strerror(err);
  close();
  return false;
}
} // namespace so101_hardware
