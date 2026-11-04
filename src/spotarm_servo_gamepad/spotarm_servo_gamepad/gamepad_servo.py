#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Joy

from geometry_msgs.msg import TwistStamped

from std_srvs.srv import Trigger

from sensor_msgs.msg import JointState

from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration

from control_msgs.action import FollowJointTrajectory

from trajectory_msgs.msg import JointTrajectoryPoint


class SpotArmServoGamepad(Node):

    def __init__(self):

        super().__init__('spotarm_servo_gamepad')


        # ==========================================================
        # PARAMETERS
        # ==========================================================

        self.declare_parameter('command_frame', 'base_link')

        self.declare_parameter('deadzone', 0.15)

        self.declare_parameter('normal_gain', 0.35)

        self.declare_parameter('fast_gain', 1.0)

        self.declare_parameter('publish_rate', 20.0)


        # ----------------------------------------------------------
        # Xbox-style mappings
        #
        # VERIFY WITH:
        #
        # ros2 topic echo /joy
        # ----------------------------------------------------------

        self.declare_parameter('axis_x', 1)

        self.declare_parameter('axis_y', 0)

        self.declare_parameter('axis_z', 4)


        self.declare_parameter('axis_x_sign', 1.0)

        self.declare_parameter('axis_y_sign', -1.0)

        self.declare_parameter('axis_z_sign', 1.0)


        self.declare_parameter('deadman_button', 4)

        self.declare_parameter('fast_button', 5)


        # D-pad (hat) axes: digital -1 / 0 / +1.
        # Up/down drives X, left/right drives Y.
        self.declare_parameter('dpad_x_axis', 7)

        self.declare_parameter('dpad_y_axis', 6)

        self.declare_parameter('dpad_x_sign', 1.0)

        self.declare_parameter('dpad_y_sign', -1.0)


        # Pose buttons (only act while the deadman is held).
        # B = stand up straight, X = go to the startup pose.
        self.declare_parameter('up_button', 1)

        self.declare_parameter('start_button', 2)

        self.declare_parameter(
            'joint_names',
            ['Revolute1', 'Revolute2', 'Revolute3', 'Revolute4', 'Revolute5', 'Revolute6']
        )

        self.declare_parameter('up_pose', [0.4745, -1.5707, 0.0, 0.0, 0.0, 0.0])

        self.declare_parameter('start_pose', [0.4745, -2.0161, -1.5595, 0.8, 1.2, 0.0])

        # Slowest joint speed used to size the move duration (rad/s).
        self.declare_parameter('pose_move_speed', 0.1)

        self.declare_parameter('pose_min_duration', 5.0)

        # Releasing the deadman stops a pose move in progress.
        self.declare_parameter('cancel_pose_on_deadman_release', True)


        # ==========================================================
        # READ PARAMETERS
        # ==========================================================

        self.command_frame = str(self.get_parameter('command_frame').value)

        self.deadzone = float(self.get_parameter('deadzone').value)

        self.normal_gain = float(self.get_parameter('normal_gain').value)

        self.fast_gain = float(self.get_parameter('fast_gain').value)

        self.publish_rate = float(self.get_parameter('publish_rate').value)


        self.axis_x = int(self.get_parameter('axis_x').value)

        self.axis_y = int(self.get_parameter('axis_y').value)

        self.axis_z = int(self.get_parameter('axis_z').value)


        self.axis_x_sign = float(self.get_parameter('axis_x_sign').value)

        self.axis_y_sign = float(self.get_parameter('axis_y_sign').value)

        self.axis_z_sign = float(self.get_parameter('axis_z_sign').value)


        self.deadman_button = int(self.get_parameter('deadman_button').value)

        self.fast_button = int(self.get_parameter('fast_button').value)


        self.dpad_x_axis = int(self.get_parameter('dpad_x_axis').value)

        self.dpad_y_axis = int(self.get_parameter('dpad_y_axis').value)

        self.dpad_x_sign = float(self.get_parameter('dpad_x_sign').value)

        self.dpad_y_sign = float(self.get_parameter('dpad_y_sign').value)


        self.up_button = int(self.get_parameter('up_button').value)

        self.start_button = int(self.get_parameter('start_button').value)

        self.joint_names = list(self.get_parameter('joint_names').value)

        self.up_pose = [float(v) for v in self.get_parameter('up_pose').value]

        self.start_pose = [float(v) for v in self.get_parameter('start_pose').value]

        self.pose_move_speed = float(self.get_parameter('pose_move_speed').value)

        self.pose_min_duration = float(self.get_parameter('pose_min_duration').value)

        self.cancel_pose_on_release = bool(
            self.get_parameter('cancel_pose_on_deadman_release').value
        )


        # ==========================================================
        # STATE
        # ==========================================================

        self.latest_joy = None

        self.servo_started = False

        self.start_request_pending = False

        self.zero_sent = 0

        self.max_zero_msgs = 5

        self.joint_positions = {}

        self.prev_up = False

        self.prev_start = False

        self.pose_moving = False

        self.pose_goal_handle = None

        self.pose_cancel_requested = False


        # ==========================================================
        # JOYSTICK SUBSCRIBER
        # ==========================================================

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 20)


        # ==========================================================
        # POSE MOVES (B = stand up, X = startup pose)
        # ==========================================================

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            20
        )

        self.pose_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )


        # ==========================================================
        # SERVO TWIST PUBLISHER
        # ==========================================================

        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 20)


        # ==========================================================
        # START SERVO SERVICE
        # ==========================================================

        self.start_servo_client = (self.create_client(Trigger, '/servo_node/start_servo'))


        # ==========================================================
        # TIMERS
        # ==========================================================

        self.start_timer = self.create_timer(0.5, self.try_start_servo)


        if self.publish_rate <= 0.0:
            self.publish_rate = 20.0


        self.command_timer = self.create_timer(1.0 / self.publish_rate, self.publish_command)


        # ==========================================================
        # STARTUP OUTPUT
        # ==========================================================

        self.get_logger().info('SPOT MoveIt Servo gamepad controller started.')

        self.get_logger().info(f'Command frame: {self.command_frame}')

        self.get_logger().info('LB = deadman')

        self.get_logger().info('RB = fast mode')

        self.get_logger().info('LB + B = stand up, LB + X = startup pose')

        self.get_logger().info('Left stick = X/Y')

        self.get_logger().info('Right stick Y = Z')

        self.get_logger().info('Waiting for MoveIt Servo start service...')


    # ==============================================================
    # JOYSTICK CALLBACK
    # ==============================================================

    def joy_callback(self, msg):

        self.latest_joy = msg


    def joint_state_callback(self, msg):

        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position


    # ==============================================================
    # POSE MOVES (B = stand up, X = startup pose)
    # ==============================================================

    def handle_pose_buttons(self):

        if self.latest_joy is None:
            return


        deadman = self.get_button(self.deadman_button)

        up = self.get_button(self.up_button)

        start = self.get_button(self.start_button)


        up_pressed = up and not self.prev_up

        start_pressed = start and not self.prev_start

        self.prev_up = up

        self.prev_start = start


        # A move in progress only continues while the deadman is held.
        if self.pose_moving:

            if (not deadman and self.cancel_pose_on_release):
                self.cancel_pose_move()

            return


        if not deadman:
            return


        if up_pressed:

            self.start_pose_move('upright', self.up_pose)

        elif start_pressed:

            self.start_pose_move('startup', self.start_pose)


    def start_pose_move(self, label, target):

        if len(target) != len(self.joint_names):

            self.get_logger().error(
                f'{label} pose has {len(target)} values, '
                f'expected {len(self.joint_names)}.'
            )

            return


        if not self.pose_client.server_is_ready():

            self.get_logger().warn('arm_controller action server not ready; ' 'pose move ignored.')

            return


        # Size the move from the largest joint change.
        largest = 0.0

        known = True

        for name, value in zip(self.joint_names, target):

            if name in self.joint_positions:

                largest = max(largest, abs(value - self.joint_positions[name]))

            else:
                known = False

        if known:

            duration = max(self.pose_min_duration, largest / max(self.pose_move_speed, 0.01))

        else:

            duration = 30.0


        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = list(self.joint_names)

        point = JointTrajectoryPoint()

        point.positions = [float(v) for v in target]

        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration - int(duration)) * 1e9)
        )

        goal.trajectory.points = [point]


        self.pose_moving = True

        self.pose_cancel_requested = False

        self.get_logger().info(
            f'Moving to {label} pose over {duration:.1f} s '
            '(hold LB to keep moving).'
        )

        future = self.pose_client.send_goal_async(goal)

        future.add_done_callback(self.pose_goal_response)


    def pose_goal_response(self, future):

        handle = future.result()

        if not handle.accepted:

            self.pose_moving = False

            self.get_logger().warn('Pose move was rejected by arm_controller.')

            return


        self.pose_goal_handle = handle


        # The deadman was released before the goal was accepted.
        if self.pose_cancel_requested:

            self.cancel_pose_move()

            return


        handle.get_result_async().add_done_callback(lambda f, h=handle: self.pose_result(f, h))


    def pose_result(self, future, handle):

        # Ignore results from a goal that was already cancelled/replaced.
        if handle is not self.pose_goal_handle:
            return

        self.pose_moving = False

        self.pose_goal_handle = None

        self.get_logger().info('Pose move finished.')


    def cancel_pose_move(self):

        self.pose_cancel_requested = True

        if self.pose_goal_handle is not None:

            self.pose_goal_handle.cancel_goal_async()

            self.pose_goal_handle = None

            self.pose_moving = False

            self.pose_cancel_requested = False

            self.get_logger().info('Pose move cancelled (deadman released).')


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


        self.get_logger().info('Starting MoveIt Servo...')


        request = Trigger.Request()

        future = (self.start_servo_client.call_async(request))

        self.start_request_pending = True

        future.add_done_callback(self.start_servo_finished)


    def start_servo_finished(self, future):

        self.start_request_pending = False

        try:

            response = future.result()

        except Exception as exc:

            self.get_logger().error(f'Failed to call start_servo: {exc}')

            return


        if response.success:

            self.servo_started = True

            self.get_logger().info('MoveIt Servo is ACTIVE.')

        else:

            self.get_logger().error('MoveIt Servo refused to start: ' + response.message)


    # ==============================================================
    # HELPERS
    # ==============================================================

    def get_axis(self, index):

        if self.latest_joy is None:
            return 0.0

        if index < 0:
            return 0.0

        if index >= len(self.latest_joy.axes):
            return 0.0

        return float(self.latest_joy.axes[index])


    def get_button(self, index):

        if self.latest_joy is None:
            return False

        if index < 0:
            return False

        if index >= len(self.latest_joy.buttons):
            return False

        return bool(self.latest_joy.buttons[index])


    # ==============================================================
    # AXIS SHAPING
    # ==============================================================

    def shape_axis(self, value):

        magnitude = abs(value)


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
        normalized = (magnitude - self.deadzone) / (1.0 - self.deadzone)


        normalized = max(0.0, min(normalized, 1.0))


        # Quadratic joystick response.
        #
        # This gives very fine control near the center.
        normalized = (normalized * normalized)


        return math.copysign(normalized, value)


    # ==============================================================
    # ZERO COMMAND
    # ==============================================================

    def create_zero_command(self):

        msg = TwistStamped()

        msg.header.stamp = (self.get_clock().now().to_msg())

        msg.header.frame_id = (self.command_frame)


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

    def publish_zero(self):

        # Servo needs a few zero commands to halt cleanly. After that
        # stay silent: a constant stream of zeros keeps Servo
        # publishing "hold" trajectories that cancel pose-move goals.
        if self.zero_sent >= self.max_zero_msgs:
            return

        self.zero_sent += 1

        self.twist_pub.publish(self.create_zero_command())


    def publish_command(self):

        self.handle_pose_buttons()


        if not self.servo_started:
            return


        # While a pose move runs, keep Servo quiet.
        if self.pose_moving:

            self.publish_zero()

            return


        # ----------------------------------------------------------
        # No controller yet:
        #
        # send zero.
        # ----------------------------------------------------------

        if self.latest_joy is None:

            self.publish_zero()

            return


        # ----------------------------------------------------------
        # Deadman
        # ----------------------------------------------------------

        deadman = self.get_button(self.deadman_button)


        if not deadman:

            self.publish_zero()

            return


        # ----------------------------------------------------------
        # Speed mode
        # ----------------------------------------------------------

        gain = self.normal_gain


        if self.get_button(self.fast_button):

            gain = self.fast_gain


        # ----------------------------------------------------------
        # Read / shape sticks
        # ----------------------------------------------------------

        x = self.shape_axis(self.get_axis(self.axis_x))

        y = self.shape_axis(self.get_axis(self.axis_y))

        z = self.shape_axis(self.get_axis(self.axis_z))


        # ----------------------------------------------------------
        # Direction corrections
        # ----------------------------------------------------------

        x *= self.axis_x_sign

        y *= self.axis_y_sign

        z *= self.axis_z_sign


        # ----------------------------------------------------------
        # D-pad override
        #
        # The D-pad is digital, so it moves exactly one axis at a
        # time. When pressed it replaces the stick on that axis.
        # ----------------------------------------------------------

        dpad_x = self.get_axis(self.dpad_x_axis)

        dpad_y = self.get_axis(self.dpad_y_axis)

        if abs(dpad_x) > 0.5:
            x = math.copysign(1.0, dpad_x) * self.dpad_x_sign

        if abs(dpad_y) > 0.5:
            y = math.copysign(1.0, dpad_y) * self.dpad_y_sign


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
        msg.header.stamp = (self.get_clock().now().to_msg())


        # Cartesian commands relative to the base.
        msg.header.frame_id = (self.command_frame)


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


        if x == 0.0 and y == 0.0 and z == 0.0:

            self.publish_zero()

        else:

            self.zero_sent = 0

            self.twist_pub.publish(msg)


# ==================================================================
# MAIN
# ==================================================================

def main(args=None):

    rclpy.init(args=args)


    node = SpotArmServoGamepad()


    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass


    finally:

        # Send a final zero command before exiting.
        if node.servo_started:

            for _ in range(4):

                node.twist_pub.publish(node.create_zero_command())


        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()

