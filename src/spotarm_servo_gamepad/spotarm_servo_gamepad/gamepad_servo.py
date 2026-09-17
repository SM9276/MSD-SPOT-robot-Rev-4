#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Joy

from geometry_msgs.msg import TwistStamped

from std_srvs.srv import Trigger


class SpotArmServoGamepad(Node):

    def __init__(self):

        super().__init__(
            'spotarm_servo_gamepad'
        )


        # ==========================================================
        # PARAMETERS
        # ==========================================================

        self.declare_parameter(
            'command_frame',
            'base_link'
        )

        self.declare_parameter(
            'deadzone',
            0.15
        )

        self.declare_parameter(
            'normal_gain',
            0.35
        )

        self.declare_parameter(
            'fast_gain',
            1.0
        )

        self.declare_parameter(
            'publish_rate',
            20.0
        )


        # ----------------------------------------------------------
        # Xbox-style mappings
        #
        # VERIFY WITH:
        #
        # ros2 topic echo /joy
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
            -1.0
        )

        self.declare_parameter(
            'axis_z_sign',
            1.0
        )


        self.declare_parameter(
            'deadman_button',
            4
        )

        self.declare_parameter(
            'fast_button',
            5
        )


        # ==========================================================
        # READ PARAMETERS
        # ==========================================================

        self.command_frame = str(
            self.get_parameter(
                'command_frame'
            ).value
        )

        self.deadzone = float(
            self.get_parameter(
                'deadzone'
            ).value
        )

        self.normal_gain = float(
            self.get_parameter(
                'normal_gain'
            ).value
        )

        self.fast_gain = float(
            self.get_parameter(
                'fast_gain'
            ).value
        )

        self.publish_rate = float(
            self.get_parameter(
                'publish_rate'
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


        self.deadman_button = int(
            self.get_parameter(
                'deadman_button'
            ).value
        )

        self.fast_button = int(
            self.get_parameter(
                'fast_button'
            ).value
        )


        # ==========================================================
        # STATE
        # ==========================================================

        self.latest_joy = None

        self.servo_started = False

        self.start_request_pending = False


        # ==========================================================
        # JOYSTICK SUBSCRIBER
        # ==========================================================

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            20
        )


        # ==========================================================
        # SERVO TWIST PUBLISHER
        # ==========================================================

        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            20
        )


        # ==========================================================
        # START SERVO SERVICE
        # ==========================================================

        self.start_servo_client = (
            self.create_client(
                Trigger,
                '/servo_node/start_servo'
            )
        )


        # ==========================================================
        # TIMERS
        # ==========================================================

        self.start_timer = self.create_timer(
            0.5,
            self.try_start_servo
        )


        if self.publish_rate <= 0.0:
            self.publish_rate = 20.0


        self.command_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_command
        )


        # ==========================================================
        # STARTUP OUTPUT
        # ==========================================================

        self.get_logger().info(
            'SPOT MoveIt Servo gamepad controller started.'
        )

        self.get_logger().info(
            f'Command frame: {self.command_frame}'
        )

        self.get_logger().info(
            'LB = deadman'
        )

        self.get_logger().info(
            'RB = fast mode'
        )

        self.get_logger().info(
            'Left stick = X/Y'
        )

        self.get_logger().info(
            'Right stick Y = Z'
        )

        self.get_logger().info(
            'Waiting for MoveIt Servo start service...'
        )


    # ==============================================================
    # JOYSTICK CALLBACK
    # ==============================================================

    def joy_callback(
        self,
        msg
    ):

        self.latest_joy = msg


    # ==============================================================
    # SERVO STARTUP
    # ==============================================================

    def try_start_servo(self):

        if self.servo_started:
            return

        if self.start_request_pending:
            return

        if not self.start_servo_client.service_is_ready():
            return


        self.get_logger().info(
            'Starting MoveIt Servo...'
        )


        request = Trigger.Request()

        future = (
            self.start_servo_client.call_async(
                request
            )
        )

        self.start_request_pending = True

        future.add_done_callback(
            self.start_servo_finished
        )


    def start_servo_finished(
        self,
        future
    ):

        self.start_request_pending = False

        try:

            response = future.result()

        except Exception as exc:

            self.get_logger().error(
                f'Failed to call start_servo: {exc}'
            )

            return


        if response.success:

            self.servo_started = True

            self.get_logger().info(
                'MoveIt Servo is ACTIVE.'
            )

        else:

            self.get_logger().error(
                'MoveIt Servo refused to start: '
                + response.message
            )


    # ==============================================================
    # HELPERS
    # ==============================================================

    def get_axis(
        self,
        index
    ):

        if self.latest_joy is None:
            return 0.0

        if index < 0:
            return 0.0

        if index >= len(
            self.latest_joy.axes
        ):
            return 0.0

        return float(
            self.latest_joy.axes[index]
        )


    def get_button(
        self,
        index
    ):

        if self.latest_joy is None:
            return False

        if index < 0:
            return False

        if index >= len(
            self.latest_joy.buttons
        ):
            return False

        return bool(
            self.latest_joy.buttons[index]
        )


    # ==============================================================
    # AXIS SHAPING
    # ==============================================================

    def shape_axis(
        self,
        value
    ):

        magnitude = abs(
            value
        )


        # Deadzone
        if magnitude <= self.deadzone:
            return 0.0


        # Rescale the remaining range from:
        #
        # deadzone ... 1
        #
        # into:
        #
        # 0 ... 1
        normalized = (
            magnitude - self.deadzone
        ) / (
            1.0 - self.deadzone
        )


        normalized = max(
            0.0,
            min(
                normalized,
                1.0
            )
        )


        # Quadratic joystick response.
        #
        # This gives very fine control near the center.
        normalized = (
            normalized *
            normalized
        )


        return math.copysign(
            normalized,
            value
        )


    # ==============================================================
    # ZERO COMMAND
    # ==============================================================

    def create_zero_command(self):

        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = (
            self.command_frame
        )


        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0


        return msg


    # ==============================================================
    # PUBLISH SERVO COMMAND
    # ==============================================================

    def publish_command(self):

        if not self.servo_started:
            return


        # ----------------------------------------------------------
        # No controller yet:
        #
        # send zero.
        # ----------------------------------------------------------

        if self.latest_joy is None:

            self.twist_pub.publish(
                self.create_zero_command()
            )

            return


        # ----------------------------------------------------------
        # Deadman
        # ----------------------------------------------------------

        deadman = self.get_button(
            self.deadman_button
        )


        if not deadman:

            self.twist_pub.publish(
                self.create_zero_command()
            )

            return


        # ----------------------------------------------------------
        # Speed mode
        # ----------------------------------------------------------

        gain = self.normal_gain


        if self.get_button(
            self.fast_button
        ):

            gain = self.fast_gain


        # ----------------------------------------------------------
        # Read / shape sticks
        # ----------------------------------------------------------

        x = self.shape_axis(
            self.get_axis(
                self.axis_x
            )
        )

        y = self.shape_axis(
            self.get_axis(
                self.axis_y
            )
        )

        z = self.shape_axis(
            self.get_axis(
                self.axis_z
            )
        )


        # ----------------------------------------------------------
        # Direction corrections
        # ----------------------------------------------------------

        x *= self.axis_x_sign

        y *= self.axis_y_sign

        z *= self.axis_z_sign


        # ----------------------------------------------------------
        # Apply precision / fast gain
        # ----------------------------------------------------------

        x *= gain
        y *= gain
        z *= gain


        # ----------------------------------------------------------
        # Build TwistStamped
        # ----------------------------------------------------------

        msg = TwistStamped()


        # Servo requires a current timestamp.
        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )


        # Cartesian commands relative to the base.
        msg.header.frame_id = (
            self.command_frame
        )


        # Translation
        msg.twist.linear.x = x

        msg.twist.linear.y = y

        msg.twist.linear.z = z


        # ----------------------------------------------------------
        # IMPORTANT:
        #
        # Rotational gamepad control intentionally disabled.
        #
        # Revolute6 is currently not physically connected.
        # ----------------------------------------------------------

        msg.twist.angular.x = 0.0

        msg.twist.angular.y = 0.0

        msg.twist.angular.z = 0.0


        self.twist_pub.publish(
            msg
        )


# ==================================================================
# MAIN
# ==================================================================

def main(args=None):

    rclpy.init(
        args=args
    )


    node = SpotArmServoGamepad()


    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass


    finally:

        # Send a final zero command before exiting.
        if node.servo_started:

            for _ in range(4):

                node.twist_pub.publish(
                    node.create_zero_command()
                )


        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()

