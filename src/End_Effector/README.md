# End Effector PWM

This package contains a ROS 2 node that sets one PWM output for the end effector.

## ROS 2 interface

The `pwm_output_node` subscribes to `/end_effector/duty_cycle` using
`std_msgs/msg/Float32`. Send a value from `0.0` to `1.0`: `0.0` turns the output
off and `1.0` is fully on. The PWM frequency is set when the node starts.

## Try it without hardware

From the ROS 2 workspace root, build and source the package:

```bash
colcon build --packages-select end_effector
source install/setup.bash
ros2 run end_effector pwm_output_node --ros-args -p backend:=mock
```

In another sourced terminal, publish a duty cycle:

```bash
ros2 topic pub --once /end_effector/duty_cycle std_msgs/msg/Float32 "{data: 0.125}"
```

## Raspberry Pi output

Install `gpiozero` in the Python environment used by ROS 2:

sudo apt update
sudo apt install python3-gpiozero

```bash
sudo bash -c '
    source /opt/ros/humble/setup.bash
    source /home/pi/MSD-SPOT-robot-Rev-4-main/install/setup.bash
ros2 run end_effector pwm_output_node --ros-args \
  -p backend:=gpiozero -p gpio_pin:=23 -p frequency_hz:=50.0
  '
```

```bash
sudo bash -c '
    source /opt/ros/humble/setup.bash
    source /home/pi/MSD-SPOT-robot-Rev-4-main/install/setup.bash
    ros2 topic pub --once /end_effector/duty_cycle std_msgs/msg/Float32 "{data: 0.125}"
  '
```
use these for the "/usr/bin/env: ‘python3\r’: No such file or directory
[ros2run]: Process exited with failure 127"

sed -i 's/\r$//' install/end_effector/lib/end_effector/pwm_output_node
head -1 install/end_effector/lib/end_effector/pwm_output_node | cat -A

