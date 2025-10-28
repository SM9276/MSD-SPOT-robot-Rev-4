#include "robot_hardware/arm_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <modbus/modbus.h>
#include <cstdlib>

namespace robot_hardware {

    hardware_interface::CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo & info){
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        info_ = info;

        const size_t n = info_.joints.size();
        joint_names_.resize(n);
        joint_names_ = {
            "Revolute1",
            "Revolute2",
            "Revolute3",
            "Revolute4",
            "Revolute5",
            "Revolute6"
        };
        pos_.resize(n);
        vel_.assign(n, 0.0);
        cmd_pos_.resize(n);
        joint_offsets_.resize(n);
        for (size_t i = 0; i < n; ++i) {
            double initial_position = 0.0;
            for (const auto & state_interface : info_.joints[i].state_interfaces) {
                if (state_interface.name == hardware_interface::HW_IF_POSITION && !state_interface.initial_value.empty()) {
                    char *endptr = nullptr;
                    initial_position = std::strtod(state_interface.initial_value.c_str(), &endptr);
                    if (endptr == state_interface.initial_value.c_str()) {
                        initial_position = 0.0;
                    }
                    break;
                }
            }
            pos_[i] = initial_position;
            cmd_pos_[i] = initial_position;
            joint_offsets_[i] = initial_position;
        }
        motor_ids_ = {1, 2, 3, 4, 5, 10}; // Default motor IDs, 10 means unconfigured
        port_ = "/dev/ttyUSB0";
        baudrate_ = 115200;
        steppers_.resize(n);
        pulses_per_revolution_ = {10000, 10000, 10000, 10000, 10000, 10000};
        // {51.2, 100, 120, 100, 100, 100}
        gear_ratios_ = {51, 100, 120, 100, 100, 100};
        is_flipped_ = {-1, -1, -1, -1, -1, 1}; // Direction multipliers for each joint
        modbus_ctx_ = modbus_new_rtu(port_.c_str(), baudrate_, 'N', 8, 1);
        if (modbus_connect(modbus_ctx_) == -1) {
            modbus_free(modbus_ctx_);
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state){
        (void)previous_state;
        // Initialize Motors
        for (int i=0; i < motor_ids_.size(); i++){
            if (motor_ids_[i] == 10) continue; // Skip unconfigured motors
            steppers_[i] = std::make_unique<ICLStepper>(motor_ids_[i], modbus_ctx_, pulses_per_revolution_[i], gear_ratios_[i]);
            if (steppers_[i]->initialize() != 0){
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state){
        (void)previous_state;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state){
        (void)previous_state;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    ArmHardwareInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> si;
        si.reserve(joint_names_.size() * 3);
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            si.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_[i]);
            si.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_[i]);
        }
        return si;
    }

    std::vector<hardware_interface::CommandInterface>
    ArmHardwareInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> ci;
        ci.reserve(joint_names_.size());
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            ci.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &cmd_pos_[i]);
        }
        return ci;
    }

    hardware_interface::return_type ArmHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period){
        (void)time;
        // Update state from hardware
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            if (motor_ids_[i] == 10) continue; // Skip unconfigured motors
            const double p = steppers_[i]->get_position_radians() * is_flipped_[i];
            pos_[i] = p + joint_offsets_[i];
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ArmHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period){
        // Send target positions to hardware
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            if (motor_ids_[i] == 10) continue; // Skip unconfigured motors
            // Add limits/safety as needed
            const double target = (cmd_pos_[i] - joint_offsets_[i]) * is_flipped_[i];
            steppers_[i]->set_position_radians(target, 0.2);
        }
        return hardware_interface::return_type::OK;
    }

} // namespace robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_hardware::ArmHardwareInterface, hardware_interface::SystemInterface)
