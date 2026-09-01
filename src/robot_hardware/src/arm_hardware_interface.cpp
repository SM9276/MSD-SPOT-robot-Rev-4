#include "robot_hardware/arm_hardware_interface.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <unordered_map>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace robot_hardware
{

namespace
{

constexpr double TWO_PI = 6.28318530717958647692;

double get_double_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name,
  double default_value)
{
  const auto it = parameters.find(name);

  if (it == parameters.end()) {
    return default_value;
  }

  return std::stod(it->second);
}


int get_int_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name,
  int default_value)
{
  const auto it = parameters.find(name);

  if (it == parameters.end()) {
    return default_value;
  }

  return std::stoi(it->second);
}

}  // namespace


// ========================================================
// Destructor
// ========================================================

ArmHardwareInterface::~ArmHardwareInterface()
{
  steppers_.clear();

  close_modbus();
}


// ========================================================
// on_init
// ========================================================

hardware_interface::CallbackReturn
ArmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ArmHardwareInterface"),
    "Initializing SPOT arm hardware interface.");

  try {
    // ----------------------------------------------------
    // Hardware parameters
    // ----------------------------------------------------

    const auto port_it =
      info_.hardware_parameters.find("port");

    if (port_it != info_.hardware_parameters.end()) {
      port_ = port_it->second;
    }

    const auto baud_it =
      info_.hardware_parameters.find("baudrate");

    if (baud_it != info_.hardware_parameters.end()) {
      baudrate_ = std::stoi(baud_it->second);
    }


    // ----------------------------------------------------
    // Allocate arrays
    // ----------------------------------------------------

    const std::size_t joint_count = info_.joints.size();

    joint_names_.resize(joint_count);

    motor_ids_.resize(joint_count);

    pulses_per_revolution_.resize(joint_count);

    gear_ratios_.resize(joint_count);

    directions_.resize(joint_count);

    joint_offsets_.resize(joint_count);

    default_velocities_.resize(joint_count);

    min_positions_.resize(joint_count);

    max_positions_.resize(joint_count);

    pos_.assign(joint_count, 0.0);

    vel_.assign(joint_count, 0.0);

    cmd_pos_.assign(joint_count, 0.0);

    last_sent_pos_.assign(
      joint_count,
      std::numeric_limits<double>::quiet_NaN());

    consecutive_read_failures_.assign(
      joint_count,
      0);

    consecutive_write_failures_.assign(
      joint_count,
      0);

    steppers_.resize(joint_count);


    // ----------------------------------------------------
    // Read joint parameters from ros2_control Xacro
    // ----------------------------------------------------

    for (std::size_t i = 0; i < joint_count; ++i) {
      const auto & joint = info_.joints[i];

      joint_names_[i] = joint.name;

      motor_ids_[i] =
        get_int_parameter(
        joint.parameters,
        "motor_id",
        DISABLED_MOTOR_ID);

      pulses_per_revolution_[i] =
        get_int_parameter(
        joint.parameters,
        "pulses_per_revolution",
        10000);

      gear_ratios_[i] =
        get_int_parameter(
        joint.parameters,
        "gear_ratio",
        100);

      directions_[i] =
        get_int_parameter(
        joint.parameters,
        "direction",
        1);

      joint_offsets_[i] =
        get_double_parameter(
        joint.parameters,
        "offset",
        0.0);

      default_velocities_[i] =
      	get_double_parameter(
        joint.parameters,
        "default_velocity",
        0.05);

      min_positions_[i] =
        get_double_parameter(
        joint.parameters,
        "min_position",
        -std::numeric_limits<double>::infinity());

      max_positions_[i] =
        get_double_parameter(
        joint.parameters,
        "max_position",
        std::numeric_limits<double>::infinity());


      // Force direction to +1 or -1.
      if (directions_[i] >= 0) {
        directions_[i] = 1;
      } else {
        directions_[i] = -1;
      }


      RCLCPP_INFO(
        rclcpp::get_logger("ArmHardwareInterface"),
        "%s: motor_id=%d ppr=%d gear=%d direction=%d",
        joint_names_[i].c_str(),
        motor_ids_[i],
        pulses_per_revolution_[i],
        gear_ratios_[i],
        directions_[i]);
    }


    RCLCPP_INFO(
      rclcpp::get_logger("ArmHardwareInterface"),
      "Hardware interface initialized for %zu joints.",
      joint_count);

    RCLCPP_INFO(
      rclcpp::get_logger("ArmHardwareInterface"),
      "Modbus device: %s @ %d baud",
      port_.c_str(),
      baudrate_);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ArmHardwareInterface"),
      "Exception during on_init(): %s",
      e.what());

    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


// ========================================================
// State interfaces
// ========================================================

std::vector<hardware_interface::StateInterface>
ArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface>
  state_interfaces;

  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &pos_[i]);

    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_VELOCITY,
      &vel_[i]);
  }

  return state_interfaces;
}


// ========================================================
// Command interfaces
// ========================================================

std::vector<hardware_interface::CommandInterface>
ArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface>
  command_interfaces;

  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &cmd_pos_[i]);
  }

  return command_interfaces;
}


// ========================================================
// Configure
// ========================================================

hardware_interface::CallbackReturn
ArmHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ArmHardwareInterface"),
    "Configuring motors...");


  // ------------------------------------------------------
  // Create Modbus context
  // ------------------------------------------------------

  if (modbus_ctx_ == nullptr) {
    modbus_ctx_ =
      modbus_new_rtu(
      port_.c_str(),
      baudrate_,
      'N',
      8,
      1);

    if (modbus_ctx_ == nullptr) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ArmHardwareInterface"),
        "modbus_new_rtu() failed.");

      return hardware_interface::CallbackReturn::ERROR;
    }
  }


  // ------------------------------------------------------
  // Connect Modbus
  // ------------------------------------------------------

  if (!modbus_connected_) {
    RCLCPP_INFO(
      rclcpp::get_logger("ArmHardwareInterface"),
      "Opening Modbus RTU %s at %d baud.",
      port_.c_str(),
      baudrate_);

    if (modbus_connect(modbus_ctx_) == -1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ArmHardwareInterface"),
        "Unable to connect to %s: %s",
        port_.c_str(),
        modbus_strerror(errno));

      close_modbus();

      return hardware_interface::CallbackReturn::ERROR;
    }

    modbus_connected_ = true;

    RCLCPP_INFO(
      rclcpp::get_logger("ArmHardwareInterface"),
      "Modbus connection established.");
  }


  // ------------------------------------------------------
  // Construct stepper objects
  // ------------------------------------------------------

  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    consecutive_read_failures_[i] = 0;
    consecutive_write_failures_[i] = 0;

    if (motor_ids_[i] == DISABLED_MOTOR_ID) {
      RCLCPP_WARN(
        rclcpp::get_logger("ArmHardwareInterface"),
        "%s has motor_id=%d; physical motor will be skipped.",
        joint_names_[i].c_str(),
        DISABLED_MOTOR_ID);

      steppers_[i].reset();

      continue;
    }


    steppers_[i] =
      std::make_unique<ICLStepper>(
      motor_ids_[i],
      modbus_ctx_,
      pulses_per_revolution_[i],
      gear_ratios_[i]);


    const int result =
      steppers_[i]->initialize();

    if (result != 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ArmHardwareInterface"),
        "Failed to initialize %s (Modbus ID %d).",
        joint_names_[i].c_str(),
        motor_ids_[i]);

      disable_all_motors();

      return hardware_interface::CallbackReturn::ERROR;
    }


    RCLCPP_INFO(
      rclcpp::get_logger("ArmHardwareInterface"),
      "Initialized %s (Modbus ID %d)",
      joint_names_[i].c_str(),
      motor_ids_[i]);
  }


  return hardware_interface::CallbackReturn::SUCCESS;
}


// ========================================================
// Activate
// ========================================================

hardware_interface::CallbackReturn
ArmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ArmHardwareInterface"),
    "Activating hardware and synchronizing ROS command state "
    "to motor positions.");


  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    consecutive_read_failures_[i] = 0;
    consecutive_write_failures_[i] = 0;


    // ----------------------------------------------------
    // Disabled/unconnected joint
    // ----------------------------------------------------

    if (motor_ids_[i] == DISABLED_MOTOR_ID) {
      vel_[i] = 0.0;

      cmd_pos_[i] = pos_[i];

      last_sent_pos_[i] = cmd_pos_[i];

      continue;
    }


    if (!steppers_[i]) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ArmHardwareInterface"),
        "Stepper object does not exist for %s.",
        joint_names_[i].c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }


    // ----------------------------------------------------
    // Read current motor position
    // ----------------------------------------------------

    const int32_t raw_counts =
      steppers_[i]->read_position();

    if (raw_counts == INT32_MIN) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ArmHardwareInterface"),
        "Unable to synchronize initial position for %s.",
        joint_names_[i].c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }


    pos_[i] =
      counts_to_ros_radians(
      i,
      raw_counts);

    vel_[i] = 0.0;


    // ----------------------------------------------------
    // Very important:
    //
    // Synchronize the command buffer to the CURRENT motor
    // position so activating ros2_control does not cause an
    // immediate jump.
    // ----------------------------------------------------

    cmd_pos_[i] = pos_[i];

    last_sent_pos_[i] = pos_[i];


    RCLCPP_INFO(
      rclcpp::get_logger("ArmHardwareInterface"),
      "%s current ROS position: %.6f rad",
      joint_names_[i].c_str(),
      pos_[i]);
  }


  return hardware_interface::CallbackReturn::SUCCESS;
}


// ========================================================
// Deactivate
// ========================================================

hardware_interface::CallbackReturn
ArmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ArmHardwareInterface"),
    "Deactivating arm hardware.");

  disable_all_motors();

  return hardware_interface::CallbackReturn::SUCCESS;
}


// ========================================================
// Error
// ========================================================

hardware_interface::CallbackReturn
ArmHardwareInterface::on_error(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(
    rclcpp::get_logger("ArmHardwareInterface"),
    "Hardware interface entered error handling.");

  disable_all_motors();

  return hardware_interface::CallbackReturn::SUCCESS;
}


// ========================================================
// READ
// ========================================================

hardware_interface::return_type
ArmHardwareInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    // ----------------------------------------------------
    // Revolute6 currently has no physical motor.
    // ----------------------------------------------------

    if (motor_ids_[i] == DISABLED_MOTOR_ID) {
      vel_[i] = 0.0;

      continue;
    }


    if (!steppers_[i]) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ArmHardwareInterface"),
        "Stepper object missing for %s.",
        joint_names_[i].c_str());

      continue;
    }


    const int32_t raw_counts =
      steppers_[i]->read_position();


    // ----------------------------------------------------
    // COMMUNICATION FAILURE
    //
    // Do NOT immediately kill ros2_control because a single
    // Modbus response was missed.
    //
    // Keep the last valid position and retry next cycle.
    // ----------------------------------------------------

    if (raw_counts == INT32_MIN) {
      ++consecutive_read_failures_[i];

      vel_[i] = 0.0;


      // Don't spam the terminal every control cycle.
      if (
        consecutive_read_failures_[i] == 1 ||
        consecutive_read_failures_[i] % 5 == 0)
      {
        RCLCPP_WARN(
          rclcpp::get_logger("ArmHardwareInterface"),
          "Position read failed for %s (%d/%d). "
          "Keeping previous position %.6f rad.",
          joint_names_[i].c_str(),
          consecutive_read_failures_[i],
          MAX_CONSECUTIVE_READ_FAILURES,
          pos_[i]);
      }


      // --------------------------------------------------
      // Only fault the hardware after MANY consecutive
      // failures.
      // --------------------------------------------------

      if (
        consecutive_read_failures_[i] >=
        MAX_CONSECUTIVE_READ_FAILURES)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("ArmHardwareInterface"),
          "%s exceeded maximum consecutive Modbus read "
          "failures.",
          joint_names_[i].c_str());

        return hardware_interface::return_type::ERROR;
      }


      continue;
    }


    // ----------------------------------------------------
    // Successful read
    // ----------------------------------------------------

    if (consecutive_read_failures_[i] > 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("ArmHardwareInterface"),
        "%s Modbus communication recovered after %d "
        "failed read(s).",
        joint_names_[i].c_str(),
        consecutive_read_failures_[i]);
    }


    consecutive_read_failures_[i] = 0;


    pos_[i] =
      counts_to_ros_radians(
      i,
      raw_counts);

    vel_[i] = 0.0;
  }


  return hardware_interface::return_type::OK;
}


// ========================================================
// WRITE
// ========================================================

hardware_interface::return_type
ArmHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    // ----------------------------------------------------
    // Disabled Revolute6
    // ----------------------------------------------------

    if (motor_ids_[i] == DISABLED_MOTOR_ID) {
      continue;
    }


    if (!steppers_[i]) {
      continue;
    }


    // ----------------------------------------------------
    // Ignore NaN / invalid commands
    // ----------------------------------------------------

    if (!std::isfinite(cmd_pos_[i])) {
      continue;
    }


    double target =
      cmd_pos_[i];


    // ----------------------------------------------------
    // Apply optional hardware limits
    // ----------------------------------------------------

    if (std::isfinite(min_positions_[i])) {
      target =
        std::max(
        target,
        min_positions_[i]);
    }

    if (std::isfinite(max_positions_[i])) {
      target =
        std::min(
        target,
        max_positions_[i]);
    }


    // ----------------------------------------------------
    // Don't continuously send the same command.
    // ----------------------------------------------------

    if (
      std::isfinite(last_sent_pos_[i]) &&
      std::abs(target - last_sent_pos_[i]) <
      COMMAND_EPSILON)
    {
      continue;
    }


    const double motor_target =
      ros_to_motor_radians(
      i,
      target);


    // ----------------------------------------------------
    // Send physical command
    // ----------------------------------------------------

    const int result =
      steppers_[i]->set_position_radians(
      motor_target,
      default_velocities_[i]);


    if (result != 0) {
      ++consecutive_write_failures_[i];

      RCLCPP_ERROR(
        rclcpp::get_logger("ArmHardwareInterface"),
        "Position command failed for %s (%d/%d).",
        joint_names_[i].c_str(),
        consecutive_write_failures_[i],
        MAX_CONSECUTIVE_WRITE_FAILURES);


      if (
        consecutive_write_failures_[i] >=
        MAX_CONSECUTIVE_WRITE_FAILURES)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("ArmHardwareInterface"),
          "%s exceeded maximum consecutive write failures.",
          joint_names_[i].c_str());

        return hardware_interface::return_type::ERROR;
      }


      // Don't update last_sent_pos_.
      //
      // This causes the same command to be retried during
      // the next write cycle.
      continue;
    }


    // ----------------------------------------------------
    // Successful command
    // ----------------------------------------------------

    consecutive_write_failures_[i] = 0;

    last_sent_pos_[i] = target;
  }


  return hardware_interface::return_type::OK;
}


// ========================================================
// Raw counts -> ROS radians
// ========================================================

double
ArmHardwareInterface::counts_to_ros_radians(
  std::size_t joint_index,
  int32_t counts) const
{
  const double denominator =
    static_cast<double>(
    pulses_per_revolution_[joint_index]) *
    static_cast<double>(
    gear_ratios_[joint_index]);


  const double motor_radians =
    static_cast<double>(counts) *
    TWO_PI /
    denominator;


  return
    (
    motor_radians *
    static_cast<double>(
      directions_[joint_index])
    ) +
    joint_offsets_[joint_index];
}


// ========================================================
// ROS radians -> physical motor radians
// ========================================================

double
ArmHardwareInterface::ros_to_motor_radians(
  std::size_t joint_index,
  double ros_position) const
{
  // directions_ is always either +1 or -1.
  //
  // The inverse of +/-1 is itself.

  return
    (
    ros_position -
    joint_offsets_[joint_index]
    ) *
    static_cast<double>(
    directions_[joint_index]);
}


// ========================================================
// Disable motors
// ========================================================

void
ArmHardwareInterface::disable_all_motors()
{
  for (std::size_t i = 0; i < steppers_.size(); ++i) {
    if (
      motor_ids_[i] == DISABLED_MOTOR_ID ||
      !steppers_[i])
    {
      continue;
    }


    RCLCPP_INFO(
      rclcpp::get_logger("ArmHardwareInterface"),
      "Disabling %s",
      joint_names_[i].c_str());


    (void)steppers_[i]->disable_motor();
  }
}


// ========================================================
// Close Modbus
// ========================================================

void
ArmHardwareInterface::close_modbus()
{
  if (modbus_ctx_ == nullptr) {
    return;
  }


  if (modbus_connected_) {
    modbus_close(modbus_ctx_);

    modbus_connected_ = false;
  }


  modbus_free(modbus_ctx_);

  modbus_ctx_ = nullptr;
}

}  // namespace robot_hardware


PLUGINLIB_EXPORT_CLASS(
  robot_hardware::ArmHardwareInterface,
  hardware_interface::SystemInterface)
