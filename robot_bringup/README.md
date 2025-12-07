# robot_bringup

This is the `robot_bringup` package for ROS2 in the [msd_robot_arm_2](https://github.com/trahnt/msd_robot_arm_2) project.  
It provides the launch files and configuration needed to start up and initialize the robot arm system.

## Features

- Robot hardware and simulation launch files
- Sensor and actuator configuration
- Integration with other ROS2 packages in the project

## Installation

Clone the repository and build the workspace:

```bash
git clone https://github.com/trahnt/msd_robot_arm_2.git
cd msd_robot_arm_2
colcon build
```

## Usage

To bring up the robot:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source <your_moveit_ws>/install/setup.bash
ros2 launch robot_bringup robot.launch.py
```

## Requirements

- ROS2 Humble
- ros2_control
- MoveIt
- Other dependencies as listed in `package.xml` and `setup.py`

## Directory Structure

- `launch/` — Launch files to start the robot system
- `config/` — Configuration files for robot description, sensors, and controllers

## License

See [LICENSE](../LICENSE) for license information.

## Maintainer

- [trahnt](https://github.com/trahnt)
- [khb120](https://github.com/khb120)
