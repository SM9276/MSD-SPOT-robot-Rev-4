# robot_hardware

This is a ROS 2 Humble package for hardware interface and control for a robot arm.

## Features

- Interfaces with the robot arm hardware components.
- Provides sensor and actuator integration.
- Basic configuration and launch files for bringing up robot hardware in ROS 2 environments.

## Dependencies

- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
- Standard ROS 2 packages ([rclcpp](https://docs.ros2.org/latest/api/rclcpp/), [sensor_msgs](https://docs.ros2.org/latest/api/sensor_msgs/), etc.)

## Building & Sourcing

```sh
cd ~/ros2_ws
colcon build --packages-select robot_hardware
source /opt/ros/humble/setup.bash
source install/setup.bash
source <your_moveit_ws>/install/setup.bash
```

Modify configuration files in the `config/` and `launch/` folders as needed for your setup.

## License

See [LICENSE](../LICENSE) for details.

## Author

Maintained by 
- [trahnt](https://github.com/trahnt)
- [khb120](https://github.com/khb120)
