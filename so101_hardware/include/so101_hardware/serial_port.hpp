#ifndef SO101_HARDWARE__SERIAL_PORT_HPP_
#define SO101_HARDWARE__SERIAL_PORT_HPP_

#include <string>
#include <cerrno>

namespace so101_hardware
{

class SerialPort{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Opens and configures port and returns false and last_error() on failure
  [[nodiscard]] bool open(const std::string & path, int baud);

  // Closes the port if open. Safe to call more than once.
  void close();

  bool is_open() const;
  int fd() const;
  const std::string & last_error() const;

private:
  // Records "<step> failed on '<path>': <errno text>", closes the port, returns false
  bool fail(const std::string & step, const std::string & path);

  int fd_{-1};
  std::string last_error_;
};
} // namespace so101_hardware

#endif // SO101_HARDWARE__SERIAL_PORT_HPP_
