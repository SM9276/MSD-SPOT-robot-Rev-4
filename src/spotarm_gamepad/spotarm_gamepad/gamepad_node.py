#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3


class SpotArmGamepad(Node):

    def __init__(self):
        super().__init__('spotarm_gamepad')

        # ============================================================
        # PARAMETERS
        # ============================================================

        # Common Xbox-style mappings.
        # Override from command line if needed.
        self.declare_parameter('axis_x', 0)
        self.declare_parameter('axis_y', 1)
        self.declare_parameter('axis_z', 4)

        # LB by default
        self.declare_parameter('deadman_button', 4)

        # Ignore small joystick drift.
        self.declare_parameter('deadzone', 0.35)

        # Stick must return closer to center than this before
        # another command becomes available.
        self.declare_parameter('rearm_deadzone', 0.20)

        # Cartesian distance for one stick command.
        #
        # 0.01 = 1 cm
        self.declare_parameter('step_size', 0.01)

        # Do not issue another command too soon.
        self.declare_parameter('command_cooldown', 0.75)

        self.declare_parameter('invert_x', False)
        self.declare_parameter('invert_y', True)
        self.declare_parameter('invert_z', False)

        # ============================================================
        # LOAD PARAMETERS
        # ============================================================

        self.axis_x = int(
            self.get_parameter('axis_x').value
        )

        self.axis_y = int(
            self.get_parameter('axis_y').value
        )

        self.axis_z = int(
            self.get_parameter('axis_z').value
        )

        self.deadman_button = int(
            self.get_parameter('deadman_button').value
        )

        self.deadzone = float(
            self.get_parameter('deadzone').value
        )

        self.rearm_deadzone = float(
            self.get_parameter('rearm_deadzone').value
        )

        self.step_size = float(
            self.get_parameter('step_size').value
        )

        self.command_cooldown = float(
            self.get_parameter('command_cooldown').value
        )

        self.invert_x = bool(
            self.get_parameter('invert_x').value
        )

        self.invert_y = bool(
            self.get_parameter('invert_y').value
        )

        self.invert_z = bool(
            self.get_parameter('invert_z').value
        )

        # ============================================================
        # STATE
        # ============================================================

        # True means one new movement command is allowed.
        self.armed = True

        self.last_command_time = None
        self.deadman_was_pressed = False
        self.received_first_joy = False

        # ============================================================
        # ROS
        # ============================================================

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.relative_pub = self.create_publisher(
            Vector3,
            '/spotarm/relative_move',
            10
        )

        # ============================================================
        # STARTUP INFORMATION
        # ============================================================

        self.get_logger().info(
            '================================================='
        )

        self.get_logger().info(
            'SPOT ARM GAMEPAD - DISCRETE MOVEIT MODE'
        )

        self.get_logger().info(
            '================================================='
        )

        self.get_logger().info(
            'Output topic: /spotarm/relative_move'
        )

        self.get_logger().info(
            f'axis_x          = {self.axis_x}'
        )

        self.get_logger().info(
            f'axis_y          = {self.axis_y}'
        )

        self.get_logger().info(
            f'axis_z          = {self.axis_z}'
        )

        self.get_logger().info(
            f'deadman_button  = {self.deadman_button}'
        )

        self.get_logger().info(
            f'deadzone         = {self.deadzone:.2f}'
        )

        self.get_logger().info(
            f'rearm_deadzone   = {self.rearm_deadzone:.2f}'
        )

        self.get_logger().info(
            f'step_size        = {self.step_size:.3f} m'
        )

        self.get_logger().info(
            f'command_cooldown = {self.command_cooldown:.2f} sec'
        )

        self.get_logger().info(
            ''
        )

        self.get_logger().info(
            'Hold LB and move ONE stick direction.'
        )

        self.get_logger().info(
            'One deflection = one MoveIt command.'
        )

        self.get_logger().info(
            'Return stick to center before next command.'
        )

        self.get_logger().info(
            'Release LB = no new commands.'
        )

        self.get_logger().info(
            '================================================='
        )

    # ================================================================
    # HELPERS
    # ================================================================

    def get_axis(self, msg, index):

        if index < 0:
            return 0.0

        if index >= len(msg.axes):
            return 0.0

        return float(msg.axes[index])

    def get_button(self, msg, index):

        if index < 0:
            return False

        if index >= len(msg.buttons):
            return False

        return bool(msg.buttons[index])

    def sticks_centered(self, x, y, z):

        return (
            abs(x) < self.rearm_deadzone
            and abs(y) < self.rearm_deadzone
            and abs(z) < self.rearm_deadzone
        )

    def command_ready(self):

        if self.last_command_time is None:
            return True

        elapsed = (
            self.get_clock().now()
            - self.last_command_time
        ).nanoseconds / 1e9

        return elapsed >= self.command_cooldown

    # ================================================================
    # JOYSTICK CALLBACK
    # ================================================================

    def joy_callback(self, msg):

        if not self.received_first_joy:

            self.received_first_joy = True

            self.get_logger().info(
                f'Joystick detected: '
                f'{len(msg.axes)} axes, '
                f'{len(msg.buttons)} buttons'
            )

        # ------------------------------------------------------------
        # READ RAW AXES
        # ------------------------------------------------------------

        x = self.get_axis(
            msg,
            self.axis_x
        )

        y = self.get_axis(
            msg,
            self.axis_y
        )

        z = self.get_axis(
            msg,
            self.axis_z
        )

        # ------------------------------------------------------------
        # AXIS DIRECTIONS
        # ------------------------------------------------------------

        if self.invert_x:
            x = -x

        if self.invert_y:
            y = -y

        if self.invert_z:
            z = -z

        # ------------------------------------------------------------
        # RE-ARM WHEN STICK RETURNS TO CENTER
        # ------------------------------------------------------------

        if self.sticks_centered(x, y, z):

            if not self.armed:

                self.get_logger().info(
                    'Sticks centered - ready for next command'
                )

            self.armed = True

        # ------------------------------------------------------------
        # DEADMAN
        # ------------------------------------------------------------

        deadman = self.get_button(
            msg,
            self.deadman_button
        )

        if not deadman:

            if self.deadman_was_pressed:

                self.get_logger().info(
                    'Deadman released'
                )

            self.deadman_was_pressed = False

            # Important:
            # no movement command is published here.
            return

        if not self.deadman_was_pressed:

            self.get_logger().info(
                'Deadman pressed'
            )

        self.deadman_was_pressed = True

        # ------------------------------------------------------------
        # ALREADY USED THIS STICK DEFLECTION
        # ------------------------------------------------------------

        if not self.armed:
            return

        # ------------------------------------------------------------
        # COOLDOWN
        # ------------------------------------------------------------

        if not self.command_ready():
            return

        # ------------------------------------------------------------
        # REQUIRE REAL DEFLECTION
        # ------------------------------------------------------------

        abs_x = abs(x)
        abs_y = abs(y)
        abs_z = abs(z)

        maximum = max(
            abs_x,
            abs_y,
            abs_z
        )

        if maximum < self.deadzone:
            return

        # ------------------------------------------------------------
        # SELECT ONLY ONE AXIS
        #
        # This prevents accidental diagonal XYZ moves.
        # Whichever stick axis is deflected the most wins.
        # ------------------------------------------------------------

        command = Vector3()

        command.x = 0.0
        command.y = 0.0
        command.z = 0.0

        if abs_x >= abs_y and abs_x >= abs_z:

            direction = 1.0 if x > 0.0 else -1.0

            command.x = (
                self.step_size * direction
            )

            axis_name = 'X'

        elif abs_y >= abs_x and abs_y >= abs_z:

            direction = 1.0 if y > 0.0 else -1.0

            command.y = (
                self.step_size * direction
            )

            axis_name = 'Y'

        else:

            direction = 1.0 if z > 0.0 else -1.0

            command.z = (
                self.step_size * direction
            )

            axis_name = 'Z'

        # ------------------------------------------------------------
        # PUBLISH EXACTLY ONE MOVE
        # ------------------------------------------------------------

        self.relative_pub.publish(
            command
        )

        self.last_command_time = (
            self.get_clock().now()
        )

        # Lock out additional commands until the stick is centered.
        self.armed = False

        self.get_logger().info(
            'MOVEIT JOG: '
            f'{axis_name} '
            f'dx={command.x:+.3f} '
            f'dy={command.y:+.3f} '
            f'dz={command.z:+.3f}'
        )


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = SpotArmGamepad()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            'Gamepad controller stopped.'
        )

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
