# robot_commander

## Overview

`robot_commander` is a ROS 2 package (compatible with the Humble release) designed to provide a set of nodes and tools for commanding and controlling a robot arm. It serves as an interface for sending movement commands, handling robot state feedback, and integrating with other ROS 2 components in a modular robotics system.

## Features

- Publish and subscribe to robot arm commands and status topics.
- Node(s) for interfacing with robot motion planning.
- Framework for easily extending and integrating control logic.
- Example scripts and launch files for getting started quickly.

## Installation

Clone the repository into your ROS 2 workspace (`src` directory):

```bash
cd ~/ros2_ws/src
git clone https://github.com/trahnt/msd_robot_arm_2.git
```

Install dependencies (from your workspace root):

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
colcon build --packages-select robot_commander
```

Source the workspace:

```bash
source install/setup.bash
source /opt/ros/humble/setup.bash
```

## Usage

Launch the robot commander node:

```bash
ros2 launch robot_commander robot_commander.launch.py
```

You can interact with the node(s) using standard ROS 2 topics and services or through the provided sample scripts.

## Package Structure

- `robot_commander/`
  - `src/` &ndash; Source code for nodes and utilities
  - `launch/` &ndash; Launch files for starting nodes
  - `msg/` &ndash; Custom ROS messages (if any)
  - `CMakeLists.txt`, `package.xml` &ndash; ROS 2 build and metadata files

## Contributing

Contributions are welcome! Please fork the repository, make your changes, and submit a pull request. Issues and suggestions for improvements can also be submitted via the GitHub issues page.

## License

This project is licensed under the MIT License. See the [LICENSE](../LICENSE) file for details.

## Maintainers

- [trahnt](https://github.com/trahnt)
- [khb120](https://github.com/khb120)
