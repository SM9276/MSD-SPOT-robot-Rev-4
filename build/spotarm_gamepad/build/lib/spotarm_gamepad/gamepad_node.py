#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped


class SpotArmGamepad(Node):

    def __init__(self):
        super().__init__('spotarm_gamepad')

        # --------------------------------------------------
        # Xbox-style defaults
        # We can change these after seeing your /joy output.
        # --------------------------------------------------

        self.axis_x = 0
        self.axis_y = 1
        self.axis_z = 4

        self.deadman_button = 4       # LB

        self.deadzone = 0.20

        # --------------------------------------------------
        # Latest joystick state
        # --------------------------------------------------

        self.latest_joy = None
        self.last_joy_time = None

        # --------------------------------------------------
        # ROS
        # --------------------------------------------------

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )

        # Always publish at 20 Hz.
        self.timer = self.create_timer(
            0.05,
            self.publish_command
        )

        self.get_logger().info(
            'SPOT ARM GAMEPAD NODE STARTED'
        )

        self.get_logger().info(
            'Publishing to /servo_node/delta_twist_cmds'
        )

        self.get_logger().info(
            'Waiting for /joy...'
        )

    # ------------------------------------------------------
    # JOY CALLBACK
    # ------------------------------------------------------

    def joy_callback(self, msg):

        first_message = self.latest_joy is None

        self.latest_joy = msg
        self.last_joy_time = self.get_clock().now()

        if first_message:
            self.get_logger().info(
                f'Received /joy: '
                f'{len(msg.axes)} axes, '
                f'{len(msg.buttons)} buttons'
            )

    # ------------------------------------------------------
    # HELPERS
    # ------------------------------------------------------

    def get_axis(self, index):

        if self.latest_joy is None:
            return 0.0

        if index >= len(self.latest_joy.axes):
            return 0.0

        value = float(self.latest_joy.axes[index])

        if abs(value) < self.deadzone:
            return 0.0

        return value

    def get_button(self, index):

        if self.latest_joy is None:
            return 0

        if index >= len(self.latest_joy.buttons):
            return 0

        return self.latest_joy.buttons[index]

    # ------------------------------------------------------
    # PUBLISH LOOP
    # ------------------------------------------------------

    def publish_command(self):

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Default is ALWAYS stopped.
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        # --------------------------------------------------
        # No controller data = STOP
        # --------------------------------------------------

        if self.latest_joy is None:
            self.twist_pub.publish(msg)
            return

        # --------------------------------------------------
        # Controller watchdog
        # --------------------------------------------------

        now = self.get_clock().now()

        age = (
            now - self.last_joy_time
        ).nanoseconds / 1e9

        if age > 0.5:
            self.twist_pub.publish(msg)
            return

        # --------------------------------------------------
        # LB deadman
        # --------------------------------------------------

        deadman = self.get_button(
            self.deadman_button
        )

        if deadman == 0:
            self.twist_pub.publish(msg)
            return

        # --------------------------------------------------
        # Read sticks
        # --------------------------------------------------

        x = self.get_axis(self.axis_x)
        y = self.get_axis(self.axis_y)
        z = self.get_axis(self.axis_z)

        # Send UNIT-LESS Servo commands.
        msg.twist.linear.x = x
        msg.twist.linear.y = -y
        msg.twist.linear.z = z

        self.twist_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = SpotArmGamepad()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        # Send zero command before exiting.
        stop = TwistStamped()

        stop.header.stamp = (
            node.get_clock().now().to_msg()
        )

        stop.header.frame_id = 'base_link'

        for _ in range(5):
            node.twist_pub.publish(stop)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
