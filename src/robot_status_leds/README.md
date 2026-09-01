# robot_status_leds

ROS2 Humble package to set Adafruit NeoPixel (WS281x) LED strips to configured colors
based on whether the robot is moving or static.

NOTE: Package was left mostly untested due to time constraints.

## Installation (Raspberry Pi 4)
1. System prerequisites (on Raspberry Pi OS):
   - Update and install python3-pip:
     ```bash
     sudo apt update
     sudo apt install -y python3-pip python3-dev
     ```

2. Install Blinka and NeoPixel Python libraries:
   ```bash
   sudo pip3 install rpi_ws281x adafruit-blinka adafruit-circuitpython-neopixel
   ```

   Notes:
   - Depending on your Pi kernel and Python environment, rpi_ws281x may require additional setup or be installed from source.
   - The NeoPixel data pin should be connected to GPIO18 (physical pin 12) for PWM timing unless you use an alternative supported pin.

3. Add the package to your ROS2 workspace:
   - Copy the `robot_status_leds` package into `src/` of your workspace.
   - Build:
     ```bash
     colcon build --packages-select robot_status_leds
     ```
   - Source:
     ```bash
     source install/setup.bash
     ```

4. Run:
   ```bash
   ros2 run robot_status_leds led_status_node
   ```

   Example with parameters:
   ```bash
   ros2 run robot_status_leds led_status_node --ros-args \
     -p led_backend:=rpi_neopixel \
     -p led_count:=60 \
     -p led_pin:=18 \
     -p brightness:=0.4 \
     -p moving_color:="#FF0000" \
     -p static_color:="#0000FF"
   ```

## Parameters
- led_backend: 'rpi_neopixel' (default) or 'mock'
- led_count: number of LEDs in the strip (default 30)
- led_pin: GPIO pin number (default 18 -> board.D18)
- brightness: float 0.0..1.0 (default 0.5)
- static_color: hex string "#RRGGBB" for static (default "#0000FF")
- moving_color: hex string "#RRGGBB" for moving (default "#00FF00")
- velocity_threshold: absolute joint velocity threshold to consider motion (default 0.01)
- moving_timeout_ms: timeout after last detected motion to consider robot still moving (default 500 ms)
- update_rate_hz: LED update frequency (default 10 Hz)
- joint_states_topic: topic name for JointState subscription (default 'joint_states')

## Notes
- Run the node with appropriate privileges if rpi_ws281x complains; some setups require root.
- Tune velocity_threshold and moving_timeout_ms to avoid flicker or missed transitions.
- If you prefer not to run as root, follow community guidance for setting device permissions for rpi_ws281x.
