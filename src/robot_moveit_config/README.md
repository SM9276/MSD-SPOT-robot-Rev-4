# robot_moveit_config

This is a ROS 2 MoveIt configuration package for the MSD Robot Arm. It contains all configuration files required to plan, control, and simulate the robot arm using the MoveIt motion planning framework.

## Package Contents

- `config/`: MoveIt configuration files, including kinematics, controllers, and planning groups.
- `launch/`: Example launch files to start MoveIt for your robot.
- `urdf/`: The robot's Unified Robot Description Format files (URDF/XACRO).
- `srdf/`: Semantic Robot Description Format file (`robot.srdf`) describing planning groups and robot semantics.

## Requirements

- ROS 2 Humble
- MoveIt 2
- ros2_control
- The robot's dependencies

## Features

- Motion planning configuration for the MSD Robot Arm
- Controller and planning group definitions
- Ready-to-use simulation support

## Support

For issues or questions, please open an issue in this repository or reach out to the maintainers [trahnt](https://github.com/trahnt) and [khb120](https://github.com/khb120).
