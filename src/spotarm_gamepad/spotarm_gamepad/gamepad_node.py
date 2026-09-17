#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Joy

from geometry_msgs.msg import Vector3

from std_msgs.msg import Empty
from std_msgs.msg import String


class SpotArmGamepad(Node):

    def __init__(self):

        super().__init__('spotarm_gamepad')


        # ==========================================================
        # Parameters
        # ==========================================================

        self.declare_parameter(
            'deadzone',
            0.15
        )

        self.declare_parameter(
            'normal_step',
            0.002
        )

        self.declare_parameter(
            'fast_step',
            0.005
        )

        self.declare_parameter(
            'publish_rate_hz',
            5.0
        )


        # ----------------------------------------------------------
        # Xbox-style defaults.
        #
        # Verify these with:
        #
        #   ros2 topic echo /joy
        #
        # ----------------------------------------------------------

        self.declare_parameter(
            'axis_x',
            1
        )

        self.declare_parameter(
            'axis_y',
            0
        )

        self.declare_parameter(
            'axis_z',
            4
        )


        self.declare_parameter(
            'axis_x_sign',
            1.0
        )

        self.declare_parameter(
            'axis_y_sign',
            1.0
        )

        self.declare_parameter(
            'axis_z_sign',
            1.0
        )


        self.declare_parameter(
            'button_a',
            0
        )

        self.declare_parameter(
            'button_b',
            1
        )

        self.declare_parameter(
            'button_x',
            2
        )

        self.declare_parameter(
            'button_y',
            3
        )

        self.declare_parameter(
            'button_lb',
            4
        )

        self.declare_parameter(
            'button_rb',
            5
        )


        # ==========================================================
        # Load parameters
        # ==========================================================

        self.deadzone = float(
            self.get_parameter(
                'deadzone'
            ).value
        )

        self.normal_step = float(
            self.get_parameter(
                'normal_step'
            ).value
        )

        self.fast_step = float(
            self.get_parameter(
                'fast_step'
            ).value
        )

        publish_rate = float(
            self.get_parameter(
                'publish_rate_hz'
            ).value
        )


        self.axis_x = int(
            self.get_parameter(
                'axis_x'
            ).value
        )

        self.axis_y = int(
            self.get_parameter(
                'axis_y'
            ).value
        )

        self.axis_z = int(
            self.get_parameter(
                'axis_z'
            ).value
        )


        self.axis_x_sign = float(
            self.get_parameter(
                'axis_x_sign'
            ).value
        )

        self.axis_y_sign = float(
            self.get_parameter(
                'axis_y_sign'
            ).value
        )

        self.axis_z_sign = float(
            self.get_parameter(
                'axis_z_sign'
            ).value
        )


        self.button_a = int(
            self.get_parameter(
                'button_a'
            ).value
        )

        self.button_b = int(
            self.get_parameter(
                'button_b'
            ).value
        )

        self.button_x = int(
            self.get_parameter(
                'button_x'
            ).value
        )

        self.button_y = int(
            self.get_parameter(
                'button_y'
            ).value
        )

        self.button_lb = int(
            self.get_parameter(
                'button_lb'
            ).value
        )

        self.button_rb = int(
            self.get_parameter(
                'button_rb'
            ).value
        )


        # ==========================================================
        # ROS publishers
        # ==========================================================

        self.relative_pub = (
            self.create_publisher(
                Vector3,
                '/spotarm/relative_move',
                10
            )
        )

        self.named_target_pub = (
            self.create_publisher(
                String,
                '/spotarm/named_target',
                10
            )
        )

        self.stop_pub = (
            self.create_publisher(
                Empty,
                '/spotarm/stop',
                10
            )
        )


        # ==========================================================
        # Joystick subscriber
        # ==========================================================

        self.joy_sub = (
            self.create_subscription(
                Joy,
                '/joy',
                self.joy_callback,
                20
            )
        )


        # ==========================================================
        # State
        # ==========================================================

        self.last_joy = None

        self.previous_buttons = []

        self.deadman_was_active = False


        # ==========================================================
        # Motion timer
        # ==========================================================

        if publish_rate <= 0.0:
            publish_rate = 5.0

        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.motion_timer
        )


        self.get_logger().info(
            'SPOT arm gamepad controller ready.'
        )

        self.get_logger().info(
            'LB = deadman'
        )

        self.get_logger().info(
            'RB = fast mode'
        )

        self.get_logger().info(
            'A = UP'
        )

        self.get_logger().info(
            'X = HOME'
        )

        self.get_logger().info(
            'Y = READY'
        )

        self.get_logger().info(
            'B = STOP'
        )


    # ==============================================================
    # Helpers
    # ==============================================================

    def axis(self, index):

        if self.last_joy is None:
            return 0.0

        if index < 0:
            return 0.0

        if index >= len(
            self.last_joy.axes
        ):
            return 0.0

        return float(
            self.last_joy.axes[index]
        )


    def button(self, index):

        if self.last_joy is None:
            return False

        if index < 0:
            return False

        if index >= len(
            self.last_joy.buttons
        ):
            return False

        return bool(
            self.last_joy.buttons[index]
        )


    def previous_button(self, index):

        if index < 0:
            return False

        if index >= len(
            self.previous_buttons
        ):
            return False

        return bool(
            self.previous_buttons[index]
        )


    def rising_edge(self, index):

        return (
            self.button(index)
            and
            not self.previous_button(index)
        )


    # ==============================================================
    # Stick shaping
    # ==============================================================

    def shape_axis(self, value):

        if abs(value) <= self.deadzone:
            return 0.0

        sign = 1.0

        if value < 0.0:
            sign = -1.0

        magnitude = (
            abs(value) - self.deadzone
        ) / (
            1.0 - self.deadzone
        )

        magnitude = max(
            0.0,
            min(
                magnitude,
                1.0
            )
        )

        # Quadratic response:
        #
        # small stick input = very precise
        # full stick        = full step
        magnitude = (
            magnitude *
            magnitude
        )

        return sign * magnitude


    # ==============================================================
    # Stop
    # ==============================================================

    def send_stop(self):

        self.stop_pub.publish(
            Empty()
        )

        self.get_logger().warn(
            'STOP sent.'
        )


    # ==============================================================
    # Named states
    # ==============================================================

    def send_named_target(
        self,
        name
    ):

        msg = String()

        msg.data = name

        self.named_target_pub.publish(
            msg
        )

        self.get_logger().info(
            f'Named target: {name}'
        )


    # ==============================================================
    # Joy callback
    # ==============================================================

    def joy_callback(
        self,
        msg
    ):

        old_buttons = (
            list(
                self.last_joy.buttons
            )
            if self.last_joy is not None
            else []
        )

        self.previous_buttons = (
            old_buttons
        )

        self.last_joy = msg


        deadman = self.button(
            self.button_lb
        )


        # ----------------------------------------------------------
        # B always acts as stop
        # ----------------------------------------------------------

        if self.rising_edge(
            self.button_b
        ):
            self.send_stop()


        # ----------------------------------------------------------
        # Releasing deadman clears pending motion and stops current
        # trajectory.
        # ----------------------------------------------------------

        if (
            self.deadman_was_active
            and
            not deadman
        ):
            self.send_stop()


        # ----------------------------------------------------------
        # Named states require deadman
        # ----------------------------------------------------------

        if deadman:

            if self.rising_edge(
                self.button_a
            ):
                self.send_named_target(
                    'up'
                )

            elif self.rising_edge(
                self.button_x
            ):
                self.send_named_target(
                    'home'
                )

            elif self.rising_edge(
                self.button_y
            ):
                self.send_named_target(
                    'ready'
                )


        self.deadman_was_active = (
            deadman
        )


    # ==============================================================
    # Motion timer
    # ==============================================================

    def motion_timer(self):

        if self.last_joy is None:
            return


        # ----------------------------------------------------------
        # Deadman
        # ----------------------------------------------------------

        if not self.button(
            self.button_lb
        ):
            return


        # ----------------------------------------------------------
        # Choose speed
        # ----------------------------------------------------------

        step = self.normal_step

        if self.button(
            self.button_rb
        ):
            step = self.fast_step


        # ----------------------------------------------------------
        # Read axes
        # ----------------------------------------------------------

        x = self.shape_axis(
            self.axis(
                self.axis_x
            )
        )

        y = self.shape_axis(
            self.axis(
                self.axis_y
            )
        )

        z = self.shape_axis(
            self.axis(
                self.axis_z
            )
        )


        # ----------------------------------------------------------
        # Apply direction
        # ----------------------------------------------------------

        x *= self.axis_x_sign
        y *= self.axis_y_sign
        z *= self.axis_z_sign


        # ----------------------------------------------------------
        # Nothing requested
        # ----------------------------------------------------------

        if (
            abs(x) < 1e-6
            and
            abs(y) < 1e-6
            and
            abs(z) < 1e-6
        ):
            return


        # ----------------------------------------------------------
        # Publish a tiny relative increment.
        #
        # Commander accumulates these while a previous trajectory
        # is executing.
        # ----------------------------------------------------------

        msg = Vector3()

        msg.x = x * step
        msg.y = y * step
        msg.z = z * step

        self.relative_pub.publish(
            msg
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = SpotArmGamepad()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.send_stop()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
