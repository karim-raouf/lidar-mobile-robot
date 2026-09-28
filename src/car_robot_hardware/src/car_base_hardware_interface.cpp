#include "car_robot_hardware/car_base_hardware_interface.hpp"
#include <cmath>


namespace car_base_hardware {

hardware_interface::CallbackReturn 
    CarBaseHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
{
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    port_ = info_.hardware_parameters.count("serial_port") ? info_.hardware_parameters.at("serial_port") : "/dev/ttyUSB0";
    baud_rate_ = info_.hardware_parameters.count("baud_rate") ? std::stod(info_.hardware_parameters.at("baud_rate")) : 115200;
    ticks_per_rev_ = info_.hardware_parameters.count("ticks_per_rev") ? std::stod(info_.hardware_parameters.at("ticks_per_rev")) : 20;
    max_rpm_ = info_.hardware_parameters.count("max_rpm") ? std::stod(info_.hardware_parameters.at("max_rpm")) : 200;


    hw_commands_.resize(info_.joints.size(), 0.0);
    hw_velocities_.resize(info_.joints.size(), 0.0);
    hw_positions_.resize(info_.joints.size(), 0.0);
    last_encoder_ticks_.resize(info_.joints.size(), 0.0);


    driver_ = std::make_shared<MotorDriver>();

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> 
    CarBaseHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces_;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces_.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
        state_interfaces_.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    }
    return state_interfaces_;
}


std::vector<hardware_interface::CommandInterface> 
    CarBaseHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces_;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces_.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
    }
    return command_interfaces_;
} 
        


hardware_interface::CallbackReturn 
    CarBaseHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void) previous_state;
    if (!driver_->openPort(port_, get_baud_macro(baud_rate_)))
    {
        RCLCPP_ERROR(rclcpp::get_logger("ESP32HardwareInterface"),
                 "Failed to open serial port: %s", port_.c_str());
        return hardware_interface::CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("ESP32HardwareInterface"), "Connected to ESP32 on %s", port_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn 
    CarBaseHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    (void) previous_state;

    driver_->writeString("m 0 0 0 0\n");
    driver_->closePort();

    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::return_type
    CarBaseHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void) time;
    (void) period;
    if (!driver_->isConnected()) {return hardware_interface::return_type::ERROR;}

    // Drain incoming buffer and get the latest line starting with 'e'
    std::string line;
    std::string latest_data = "";
    while (!(line = driver_->readLine()).empty()) {
        if (line[0] == 'e') latest_data = line;
    }

    if (!latest_data.empty()) {
        long fl = 0, fr = 0, rl = 0, rr = 0;
        if (sscanf(latest_data.c_str(), "e %ld %ld %ld %ld", &fl, &fr, &rl, &rr) == 4) {
        long current_ticks[4] = {fl, fr, rl, rr};
        double dt = period.seconds();

        for (int i = 0; i < 4; ++i) {
            long delta_ticks = current_ticks[i] - last_encoder_ticks_[i];
            last_encoder_ticks_[i] = current_ticks[i];

            // rad = (ticks / ticks_per_rev) * 2 * PI
            double delta_rad = (static_cast<double>(delta_ticks) / ticks_per_rev_) * (2.0 * M_PI);
            hw_positions_[i] += delta_rad;
            if (dt > 0.0) {
            hw_velocities_[i] = delta_rad / dt;
            }
        }
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
    CarBaseHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void) time;
    (void) period;

    if (!driver_->isConnected()) {return hardware_interface::return_type::ERROR;}

    // Map requested angular velocities (rad/s) to PWM duty range (-255 to 255)
    // Max rad/s = (max_rpm / 60) * 2 * PI
    double max_rad_s = (max_rpm_ / 60.0) * (2.0 * M_PI);
    int pwm[4] = {0, 0, 0, 0};

    for (int i = 0; i < 4; ++i) {
        double ratio = hw_commands_[i] / max_rad_s;
        pwm[i] = static_cast<int>(std::clamp(ratio * 255.0, -255.0, 255.0));
    }

    char buffer[64];
    // Format: "m <FL> <FR> <RL> <RR>\n"
    snprintf(buffer, sizeof(buffer), "m %d %d %d %d\n", pwm[0], pwm[1], pwm[2], pwm[3]);
    driver_->writeString(buffer);

    return hardware_interface::return_type::OK;
}


} // namespace car_base_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(car_base_hardware::CarBaseHardwareInterface, hardware_interface::SystemInterface)