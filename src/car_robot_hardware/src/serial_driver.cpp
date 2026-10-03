#include "car_robot_hardware/serial_driver.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <format>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace car_base_hardware
{

namespace
{

constexpr std::size_t kMaxRxBuffer = 1024;

rclcpp::Logger logger() {return rclcpp::get_logger("CarBaseSerialDriver");}

std::optional<speed_t> toSpeed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return std::nullopt;
  }
}

double finiteOrZero(double v) {return std::isfinite(v) ? v : 0.0;}

}  // namespace

SerialDriver::~SerialDriver() {close();}

bool SerialDriver::open(
  const std::string & port, int baud_rate, std::chrono::milliseconds startup_delay)
{
  close();

  const auto speed = toSpeed(baud_rate);
  if (!speed) {
    RCLCPP_ERROR(logger(), "Unsupported baud rate %d", baud_rate);
    return false;
  }

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(
      logger(), "Cannot open %s: %s (is your user in the 'dialout' group?)",
      port.c_str(), std::strerror(errno));
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(logger(), "tcgetattr failed: %s", std::strerror(errno));
    close();
    return false;
  }

  cfmakeraw(&tty);  // 8N1, no echo, no canonical mode, no software flow control
  cfsetispeed(&tty, *speed);
  cfsetospeed(&tty, *speed);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(CSTOPB | CRTSCTS | PARENB);
  tty.c_cc[VMIN] = 0;   // fully non-blocking reads
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger(), "tcsetattr failed: %s", std::strerror(errno));
    close();
    return false;
  }

  ioctl(fd_, TIOCEXCL);  // exclusive access, so nobody else steals the port

  std::this_thread::sleep_for(startup_delay);
  tcflush(fd_, TCIOFLUSH);
  rx_buffer_.clear();
  wheels_ = {};

  RCLCPP_INFO(logger(), "Serial port %s opened @ %d baud", port.c_str(), baud_rate);
  return true;
}

void SerialDriver::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialDriver::writeLine(std::string_view line)
{
  if (!isOpen()) {
    return false;
  }
  while (!line.empty()) {
    const ssize_t n = ::write(fd_, line.data(), line.size());
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return true;  // TX buffer full: drop it, the next cycle sends a fresh command
      }
      RCLCPP_ERROR(logger(), "Serial write failed: %s", std::strerror(errno));
      return false;
    }
    line.remove_prefix(static_cast<std::size_t>(n));
  }
  return true;
}

bool SerialDriver::sendVelocity(double left_rad_s, double right_rad_s)
{
  return writeLine(
    std::format("V {:.4f} {:.4f}\n", finiteOrZero(left_rad_s), finiteOrZero(right_rad_s)));
}

bool SerialDriver::sendPid(double kp, double ki, double kd)
{
  return writeLine(std::format("P {:.5f} {:.5f} {:.5f}\n", kp, ki, kd));
}

bool SerialDriver::resetEncoders()
{
  return writeLine("R\n");
}

std::optional<int> SerialDriver::poll()
{
  if (!isOpen()) {
    return std::nullopt;
  }

  std::array<char, 256> buf{};
  while (true) {
    const ssize_t n = ::read(fd_, buf.data(), buf.size());
    if (n > 0) {
      rx_buffer_.append(buf.data(), static_cast<std::size_t>(n));
      continue;
    }
    if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    }
    if (errno == EINTR) {
      continue;
    }
    RCLCPP_ERROR(logger(), "Serial read failed: %s", std::strerror(errno));
    return std::nullopt;
  }

  int states = 0;
  std::size_t start = 0;
  for (std::size_t end; (end = rx_buffer_.find('\n', start)) != std::string::npos; start = end + 1) {
    if (handleLine(std::string_view{rx_buffer_}.substr(start, end - start))) {
      ++states;
    }
  }
  rx_buffer_.erase(0, start);

  if (rx_buffer_.size() > kMaxRxBuffer) {  // garbage without newline, resync
    rx_buffer_.clear();
  }
  return states;
}

bool SerialDriver::handleLine(std::string_view line)
{
  while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) {
    line.remove_suffix(1);
  }
  if (line.empty()) {
    return false;
  }

  if (!line.starts_with("S ")) {
    RCLCPP_DEBUG(logger(), "ESP32: %.*s", static_cast<int>(line.size()), line.data());
    return false;
  }

  // sscanf needs a NUL-terminated string; state lines are short.
  const std::string text{line};
  std::array<double, 4> v{};
  const bool ok =
    std::sscanf(text.c_str(), "S %lf %lf %lf %lf", &v[0], &v[1], &v[2], &v[3]) == 4 &&
    std::ranges::all_of(v, [](double x) {return std::isfinite(x);});
  if (!ok) {
    RCLCPP_DEBUG(logger(), "Malformed state line: '%s'", text.c_str());
    return false;
  }

  wheels_[kLeft] = {v[0], v[2]};
  wheels_[kRight] = {v[1], v[3]};
  return true;
}

}  // namespace car_base_hardware
