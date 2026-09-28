#pragma once

#include <hardware_interface/system_interface.hpp>
#include "car_robot_hardware/motor_driver.hpp"


namespace car_base_hardware {

class CarBaseHardwareInterface: public hardware_interface::SystemInterface
{
    public:
        hardware_interface::CallbackReturn 
            on_activate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn 
            on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        

        // SystemInterface override
        hardware_interface::CallbackReturn 
            on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

        std::vector<hardware_interface::StateInterface> 
            export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> 
            export_command_interfaces() override;

        hardware_interface::return_type
            read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type
            write(const rclcpp::Time &time, const rclcpp::Duration &period) override;


    private:
        std::shared_ptr<MotorDriver> driver_;
        std::string port_;
        int baud_rate_;
        double ticks_per_rev_;
        double max_rpm_;

        // 4 Wheels: 0=FL, 1=FR, 2=RL, 3=RR
        std::vector<double> hw_commands_;
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<long> last_encoder_ticks_;


        speed_t get_baud_macro(int baud_rate) {
            switch (baud_rate) {
                case 9600:   return B9600;
                case 19200:  return B19200;
                case 38400:  return B38400;
                case 57600:  return B57600;
                case 115200: return B115200;
                case 230400: return B230400;
                case 460800: return B460800;
                case 921600: return B921600;
                default:     return B115200; // Provide a sensible default fallback
            }
        }

};

} // namespace car_base_hardware