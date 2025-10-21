#ifndef ARM_HARDWARE_INTERFACE_HPP
#define ARM_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "ICLStepper.h"
#include <modbus/modbus.h>
#include <vector>

namespace robot_hardware {

class ArmHardwareInterface : public hardware_interface::SystemInterface {
public:
    // Lifecycle node override
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    
    // SystemInterface override
    hardware_interface::CallbackReturn 
        on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::return_type 
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    // Expose state/command storage to controller_manager
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
private:
    modbus_t* modbus_ctx_;
    std::string port_;
    int baudrate_;

    std::vector<std::string> joint_names_;
    std::vector<int> motor_ids_;
    std::vector<std::unique_ptr<ICLStepper>> steppers_;
    std::vector<int> pulses_per_revolution_;
    std::vector<int> gear_ratios_;
    std::vector<double> pos_, vel_;
    std::vector<double> cmd_pos_;
    std::vector<int> is_flipped_;
}; // class ArmHardwareInterface

} // namespace robot_hardware

#endif // ARM_HARDWARE_INTERFACE_HPP