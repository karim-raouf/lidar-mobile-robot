#pragma once

#include <array>
#include <chrono>
#include <optional>
#include <string>
#include <string_view>

namespace car_base_hardware
{

/// Latest measured state of one wheel (as reported by the ESP32).
struct WheelState
{
  double position{0.0};  // [rad], accumulated since the last encoder reset
  double velocity{0.0};  // [rad/s], filtered on the ESP32
};

/**
 * Plain POSIX serial driver for the ESP32 motor controller.
 *
 * Wire protocol (ASCII, '\n' terminated, 8N1)
 *   Host -> ESP32
 *     V <left_rad_s> <right_rad_s>   wheel velocity setpoints (also feeds the ESP32 watchdog)
 *     P <kp> <ki> <kd>               set velocity PID gains
 *     R                              reset encoder counters
 *   ESP32 -> Host
 *     S <pos_l> <pos_r> <vel_l> <vel_r>   state, streamed at ~50 Hz (rad, rad, rad/s, rad/s)
 *     anything else                       info / acks, logged at debug level
 *
 * Not thread safe; ros2_control calls read()/write() from a single thread.
 */
class SerialDriver
{
public:
  static constexpr std::size_t kLeft = 0;
  static constexpr std::size_t kRight = 1;

  SerialDriver() = default;
  ~SerialDriver();
  SerialDriver(const SerialDriver &) = delete;
  SerialDriver & operator=(const SerialDriver &) = delete;

  /// Open and configure the port, then wait `startup_delay` for the ESP32 to boot
  /// (opening the port toggles DTR/RTS, which resets most ESP32 dev boards).
  [[nodiscard]] bool open(
    const std::string & port, int baud_rate, std::chrono::milliseconds startup_delay);
  void close();
  [[nodiscard]] bool isOpen() const {return fd_ >= 0;}

  bool sendVelocity(double left_rad_s, double right_rad_s);
  bool sendPid(double kp, double ki, double kd);
  bool resetEncoders();

  /// Non-blocking: drain the serial buffer and parse all complete lines.
  /// @return number of new state messages parsed, or std::nullopt on a fatal I/O error.
  [[nodiscard]] std::optional<int> poll();

  /// Most recent state received, indexed by kLeft / kRight.
  [[nodiscard]] const std::array<WheelState, 2> & wheels() const {return wheels_;}

private:
  bool writeLine(std::string_view line);
  bool handleLine(std::string_view line);  // true if the line was a valid state message

  int fd_{-1};
  std::string rx_buffer_;
  std::array<WheelState, 2> wheels_{};
};

}  // namespace car_base_hardware
