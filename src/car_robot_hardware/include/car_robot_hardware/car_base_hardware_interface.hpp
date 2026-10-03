#pragma once

#include <chrono>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "car_robot_hardware/serial_driver.hpp"

namespace car_base_hardware
{

/**
 * ros2_control SystemInterface for the 2-wheel differential base driven by an ESP32.
 *
 * Joints (in URDF order): [0] = left wheel, [1] = right wheel
 *   command interface : velocity [rad/s]
 *   state interfaces  : position [rad], velocity [rad/s]
 *
 * Hardware parameters (all optional):
 *   serial_port       serial device                      (default /dev/ttyUSB0)
 *   baud_rate         serial speed                       (default 115200)
 *   timeout_ms        max time without state from ESP32  (default 500)
 *   startup_delay_ms  wait after opening the port        (default 2000)
 *   kp, ki, kd        velocity PID gains; sent to the ESP32 on activation
 *                     only if all three are given
 */
class CarBaseHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  SerialDriver driver_;

  // Parameters
  std::string port_{"/dev/ttyUSB0"};
  int baud_rate_{115200};
  std::chrono::milliseconds timeout_{500};
  std::chrono::milliseconds startup_delay_{2000};
  bool send_pid_{false};
  double kp_{0.0};
  double ki_{0.0};
  double kd_{0.0};

  std::string left_joint_;
  std::string right_joint_;

  std::chrono::steady_clock::time_point last_rx_;
};

}  // namespace car_base_hardware
