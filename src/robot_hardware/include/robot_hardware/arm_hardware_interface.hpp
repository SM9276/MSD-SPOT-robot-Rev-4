#ifndef ROBOT_HARDWARE__ARM_HARDWARE_INTERFACE_HPP_
#define ROBOT_HARDWARE__ARM_HARDWARE_INTERFACE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp> 
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <modbus/modbus.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "robot_hardware/ICLStepper.h"

namespace robot_hardware
{

class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmHardwareInterface)

  ArmHardwareInterface() = default;

  ~ArmHardwareInterface() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // ------------------------------------------------------
  // Modbus
  // ------------------------------------------------------

  modbus_t * modbus_ctx_{nullptr};

  bool modbus_connected_{false};

  std::string port_{"/dev/ttyUSB0"};

  int baudrate_{115200};


  // ------------------------------------------------------
  // Motors
  // ------------------------------------------------------

  std::vector<std::unique_ptr<ICLStepper>> steppers_;

  std::vector<std::string> joint_names_;

  std::vector<int> motor_ids_;

  std::vector<int> pulses_per_revolution_;

  std::vector<int> gear_ratios_;

  std::vector<int> directions_;

  std::vector<double> joint_offsets_;

  std::vector<double> default_velocities_;

  std::vector<double> min_positions_;

  std::vector<double> max_positions_;


  // ------------------------------------------------------
  // ROS joint state
  // ------------------------------------------------------

  std::vector<double> pos_;

  std::vector<double> vel_;

  std::vector<double> cmd_pos_;

  std::vector<double> last_sent_pos_;


  // ------------------------------------------------------
  // Communication failure tracking
  // ------------------------------------------------------

  std::vector<int> consecutive_read_failures_;

  std::vector<int> consecutive_write_failures_;

  static constexpr int MAX_CONSECUTIVE_READ_FAILURES = 10;

  static constexpr int MAX_CONSECUTIVE_WRITE_FAILURES = 5;


  // Motor ID 10 is currently being used as "not connected".
  //
  // Change this when Revolute6 receives its real Modbus ID.
  static constexpr int DISABLED_MOTOR_ID = 10;


  // Don't resend virtually identical position commands.
  static constexpr double COMMAND_EPSILON = 1e-4;


  // ------------------------------------------------------
  // Helpers
  // ------------------------------------------------------

  double counts_to_ros_radians(
    std::size_t joint_index,
    int32_t counts) const;

  double ros_to_motor_radians(
    std::size_t joint_index,
    double ros_position) const;

  void disable_all_motors();

  void close_modbus();
};

}  // namespace robot_hardware

#endif  // ROBOT_HARDWARE__ARM_HARDWARE_INTERFACE_HPP_
