# spotarm_assembly_description

This is a ROS 2 Humble package providing the description files (URDF, meshes, etc.) for the Spot Arm assembly.

## Package Overview

- **Purpose:** Holds robot description files for the Spot Arm, used for simulation, visualization (RViz), and as a basis for motion planning and control.
- **ROS 2 Distribution:** Designed for ROS 2 Humble.
- **Contents:** URDF/Xacro robot description, configuration files, and associated meshes.

## Installation

Clone this repository into your ROS 2 workspace's `src` directory and build:

```bash
cd ~/ros2_ws/src
git clone https://github.com/trahnt/msd_robot_arm_2.git
cd ~/ros2_ws
colcon build
```

## Usage

To launch visualization in RViz:

```bash
ros2 launch spotarm_assembly_description view_robot.launch.py
```

Or to display robot model:

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args --param robot_description:="$(cat src/msd_robot_arm_2/spotarm_assembly_description/urdf/spotarm.xacro)"
```

## Folder Structure

- `urdf/`: Contains robot URDF/Xacro files.
- `meshes/`: 3D models used in URDF.
- `config/`: Configuration files if available.

## Dependencies

- ROS 2 Humble
- [robot_state_publisher](https://github.com/ros/robot_state_publisher)
- [rviz2](https://github.com/ros2/rviz)

## License

See the main repository's LICENSE file for details.

## Maintainer

- [trahnt](https://github.com/trahnt)
- [khb120](https://github.com/khb120)
