# robot_servoing

This is a simple ROS 2 package for controlling a robotic arm with a gamepad.  
Compatible with ROS 2 Humble.

NOTE: Gamepad functionality still does not work on arm hardware, only in simulation. We believe this issue has to do with the ros2_control Controller Manager configuration. After countless hours of troubleshooting we were unable to fix the issue before delivery date. 

## Overview

The `robot_servoing` package provides nodes and modules to facilitate servoing and basic manipulation tasks for a robot arm using ROS 2. This package is designed to be easy to use and integrate with other ROS 2 projects.

## Features

- ROS 2 compliant nodes for robot servoing.
- Basic control interfaces for a robotic arm.
- Modular code structure.

## Installation

1. Clone the repository into your ROS 2 workspace (e.g., `~/ros2_ws/src`):

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/trahnt/msd_robot_arm_2.git
   ```

2. Build the workspace:

   ```bash
   cd ~/ros2_ws
   colcon build
   ```

## Usage

1. Source your workspace after building:

   ```bash
   source ~/ros2_ws/install/setup.bash
   source /opt/ros/humble/setup.bash
   source <your_moveit_ws>/install/setup.bash
   ```

2. Launch or run the package's nodes directly. For example:

   ```bash
   ros2 launch robot_servoing servo2.launch.py
   ```

## Package Structure

```
robot_servoing/
├── src/
├── CMakeLists.txt
├── package.xml
└── ... (other files/modules)
```

## Dependencies

- ROS 2 Humble
- [rclcpp](https://docs.ros2.org/latest/api/rclcpp/)
- Other dependencies as specified in `package.xml`

## License

See [LICENSE](../LICENSE) for licensing information.

## Authors

Maintained by 
   - [trahnt](https://github.com/trahnt)
   - [khb120](https://github.com/khb120)

---

For advanced usage and API documentation, see the source code and comments in the respective modules!
