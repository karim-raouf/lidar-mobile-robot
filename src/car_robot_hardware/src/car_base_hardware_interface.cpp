#include "car_robot_hardware/car_base_hardware_interface.hpp"

#include <algorithm>
#include <optional>
#include <stdexcept>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace car_base_hardware
{

using hardware_interface::CallbackReturn;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::return_type;

namespace
{

std::optional<std::string> findParam(
  const hardware_interface::HardwareInfo & info, const std::string & name)
{
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) {
    return std::nullopt;
  }
  return it->second;
}

bool hasInterface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  return std::ranges::any_of(interfaces, [&](const auto & i) {return i.name == name;});
}

}  // namespace

CallbackReturn CarBaseHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2) {
    RCLCPP_FATAL(
      get_logger(), "Expected exactly 2 joints (left, right), got %zu", info_.joints.size());
    return CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != HW_IF_VELOCITY ||
      !hasInterface(joint.state_interfaces, HW_IF_POSITION) ||
      !hasInterface(joint.state_interfaces, HW_IF_VELOCITY))
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' needs one velocity command interface and position + velocity state "
        "interfaces", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  left_joint_ = info_.joints[SerialDriver::kLeft].name;
  right_joint_ = info_.joints[SerialDriver::kRight].name;

  try {
    port_ = findParam(info_, "serial_port").value_or(port_);
    if (const auto v = findParam(info_, "baud_rate")) {baud_rate_ = std::stoi(*v);}
    if (const auto v = findParam(info_, "timeout_ms")) {
      timeout_ = std::chrono::milliseconds{std::stoi(*v)};
    }
    if (const auto v = findParam(info_, "first_state_timeout_ms")) {
      first_state_timeout_ = std::chrono::milliseconds{std::stoi(*v)};
    }
    if (const auto v = findParam(info_, "startup_delay_ms")) {
      startup_delay_ = std::chrono::milliseconds{std::stoi(*v)};
    }

    const auto kp = findParam(info_, "kp");
    const auto ki = findParam(info_, "ki");
    const auto kd = findParam(info_, "kd");
    send_pid_ = kp && ki && kd;
    if (send_pid_) {
      kp_ = std::stod(*kp);
      ki_ = std::stod(*ki);
      kd_ = std::stod(*kd);
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Invalid hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn CarBaseHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (!driver_.open(port_, baud_rate_, startup_delay_)) {
    return CallbackReturn::ERROR;
  }

  for (const auto & joint : {left_joint_, right_joint_}) {
    set_state(joint + "/" + HW_IF_POSITION, 0.0);
    set_state(joint + "/" + HW_IF_VELOCITY, 0.0);
    set_command(joint + "/" + HW_IF_VELOCITY, 0.0);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CarBaseHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  driver_.close();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CarBaseHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  set_command(left_joint_ + "/" + HW_IF_VELOCITY, 0.0);
  set_command(right_joint_ + "/" + HW_IF_VELOCITY, 0.0);

  if (!driver_.sendVelocity(0.0, 0.0)) {
    return CallbackReturn::ERROR;
  }
  if (send_pid_ && !driver_.sendPid(kp_, ki_, kd_)) {
    return CallbackReturn::ERROR;
  }

  // Don't start the read() timeout until the ESP32 has actually started streaming.
  const auto deadline = std::chrono::steady_clock::now() + first_state_timeout_;
  while (true) {
    const auto received = driver_.poll();
    if (!received) {
      return CallbackReturn::ERROR;
    }
    if (*received > 0) {
      break;
    }
    if (std::chrono::steady_clock::now() > deadline) {
      const auto & last = driver_.lastIgnoredLine();
      RCLCPP_ERROR(
        get_logger(), "No state message ('S ...') from ESP32 within %lld ms on %s. Last line "
        "received: %s", static_cast<long long>(first_state_timeout_.count()), port_.c_str(),
        last.empty() ? "<nothing>" : ("'" + last + "'").c_str());
      return CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }

  last_rx_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(get_logger(), "Connected to ESP32 on %s", port_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn CarBaseHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  driver_.sendVelocity(0.0, 0.0);  // halt the motors before handing back control
  return CallbackReturn::SUCCESS;
}

return_type CarBaseHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto received = driver_.poll();
  if (!received) {
    return return_type::ERROR;
  }

  const auto now = std::chrono::steady_clock::now();
  if (*received > 0) {
    last_rx_ = now;
  } else if (now - last_rx_ > timeout_) {
    const auto & last = driver_.lastIgnoredLine();
    RCLCPP_ERROR(
      get_logger(), "No state from ESP32 for more than %lld ms. Last line received: %s",
      static_cast<long long>(timeout_.count()),
      last.empty() ? "<nothing>" : ("'" + last + "'").c_str());
    return return_type::ERROR;
  }

  const auto & wheels = driver_.wheels();
  set_state(left_joint_ + "/" + HW_IF_POSITION, wheels[SerialDriver::kLeft].position);
  set_state(left_joint_ + "/" + HW_IF_VELOCITY, wheels[SerialDriver::kLeft].velocity);
  set_state(right_joint_ + "/" + HW_IF_POSITION, wheels[SerialDriver::kRight].position);
  set_state(right_joint_ + "/" + HW_IF_VELOCITY, wheels[SerialDriver::kRight].velocity);

  return return_type::OK;
}

return_type CarBaseHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto left = get_command<double>(left_joint_ + "/" + HW_IF_VELOCITY);
  const auto right = get_command<double>(right_joint_ + "/" + HW_IF_VELOCITY);

  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000, "Command [rad/s] L: %.3f R: %.3f", left, right);

  return driver_.sendVelocity(left, right) ? return_type::OK : return_type::ERROR;
}

}  // namespace car_base_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  car_base_hardware::CarBaseHardwareInterface, hardware_interface::SystemInterface)
