#!/usr/bin/env python3

import math

import rclpy

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from visualization_msgs.msg import InteractiveMarkerFeedback

from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


class SpotArmTargetGamepad(Node):

    def __init__(self):

        super().__init__(
            'spotarm_target_gamepad'
        )

        # ==========================================================
        # FRAMES
        # ==========================================================

        self.declare_parameter(
            'base_frame',
            'base_link'
        )

        self.declare_parameter(
            'ee_frame',
            'fake_gripper'
        )

        # ==========================================================
        # MOVEIT INTERACTIVE MARKER
        # ==========================================================

        self.declare_parameter(
            'feedback_topic',
            (
                '/rviz_moveit_motion_planning_display/'
                'robot_interaction_interactive_marker_topic/'
                'feedback'
            )
        )

        self.declare_parameter(
            'marker_name',
            'EE:goal_fake_gripper'
        )

        self.declare_parameter(
            'control_name',
            'move'
        )

        # ==========================================================
        # RVIZ MOVEIT REMOTE CONTROL
        # ==========================================================

        self.declare_parameter(
            'plan_topic',
            '/rviz/moveit/plan'
        )

        self.declare_parameter(
            'execute_topic',
            '/rviz/moveit/execute'
        )

        self.declare_parameter(
            'stop_topic',
            '/rviz/moveit/stop'
        )

        self.declare_parameter(
            'display_trajectory_topic',
            '/display_planned_path'
        )

        # Amount of time after receiving the planned trajectory
        # before asking RViz to execute it.
        self.declare_parameter(
            'execute_delay',
            0.20
        )

        # If planning never returns a trajectory, cancel the
        # pending execution request.
        self.declare_parameter(
            'planning_timeout',
            10.0
        )

        # ==========================================================
        # CONTROLLER MAPPING
        # ==========================================================

        # Left stick vertical
        self.declare_parameter(
            'axis_x',
            1
        )

        # Left stick horizontal
        self.declare_parameter(
            'axis_y',
            0
        )

        # Right stick vertical
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

        # LB
        self.declare_parameter(
            'deadman_button',
            4
        )

        # RB
        self.declare_parameter(
            'fast_button',
            5
        )

        # A
        self.declare_parameter(
            'execute_button',
            0
        )

        # B
        self.declare_parameter(
            'cancel_button',
            1
        )

        # ==========================================================
        # TARGET EDITING
        # ==========================================================

        self.declare_parameter(
            'deadzone',
            0.18
        )

        # meters / second
        #
        # These only move the RViz goal.
        # They do NOT control physical robot velocity.
        self.declare_parameter(
            'normal_speed',
            0.05
        )

        self.declare_parameter(
            'fast_speed',
            0.15
        )

        self.declare_parameter(
            'update_rate',
            30.0
        )

        # Maximum distance the preview can be moved away from the
        # edit anchor before being clamped.
        self.declare_parameter(
            'max_preview_radius',
            0.40
        )

        # ==========================================================
        # READ PARAMETERS
        # ==========================================================

        self.base_frame = str(
            self.get_parameter(
                'base_frame'
            ).value
        )

        self.ee_frame = str(
            self.get_parameter(
                'ee_frame'
            ).value
        )

        self.feedback_topic = str(
            self.get_parameter(
                'feedback_topic'
            ).value
        )

        self.marker_name = str(
            self.get_parameter(
                'marker_name'
            ).value
        )

        self.control_name = str(
            self.get_parameter(
                'control_name'
            ).value
        )

        self.plan_topic = str(
            self.get_parameter(
                'plan_topic'
            ).value
        )

        self.execute_topic = str(
            self.get_parameter(
                'execute_topic'
            ).value
        )

        self.stop_topic = str(
            self.get_parameter(
                'stop_topic'
            ).value
        )

        self.display_trajectory_topic = str(
            self.get_parameter(
                'display_trajectory_topic'
            ).value
        )

        self.execute_delay = float(
            self.get_parameter(
                'execute_delay'
            ).value
        )

        self.planning_timeout = float(
            self.get_parameter(
                'planning_timeout'
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

        self.execute_button = int(
            self.get_parameter(
                'execute_button'
            ).value
        )

        self.cancel_button = int(
            self.get_parameter(
                'cancel_button'
            ).value
        )

        self.deadzone = float(
            self.get_parameter(
                'deadzone'
            ).value
        )

        self.normal_speed = float(
            self.get_parameter(
                'normal_speed'
            ).value
        )

        self.fast_speed = float(
            self.get_parameter(
                'fast_speed'
            ).value
        )

        self.update_rate = float(
            self.get_parameter(
                'update_rate'
            ).value
        )

        self.max_preview_radius = float(
            self.get_parameter(
                'max_preview_radius'
            ).value
        )

        # ==========================================================
        # ROS INTERFACES
        # ==========================================================

        # ----------------------------------------------------------
        # Joystick
        # ----------------------------------------------------------

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            20
        )

        # ----------------------------------------------------------
        # MoveIt interactive marker feedback
        # ----------------------------------------------------------

        # Listen to mouse-generated feedback so mouse and gamepad
        # remain synchronized.
        self.feedback_sub = self.create_subscription(
            InteractiveMarkerFeedback,
            self.feedback_topic,
            self.feedback_callback,
            20
        )

        # Publishing here moves the real MoveIt blue goal marker.
        self.feedback_pub = self.create_publisher(
            InteractiveMarkerFeedback,
            self.feedback_topic,
            20
        )

        # ----------------------------------------------------------
        # RViz MoveIt PLAN / EXECUTE / STOP
        # ----------------------------------------------------------

        self.plan_pub = self.create_publisher(
            Empty,
            self.plan_topic,
            10
        )

        self.execute_pub = self.create_publisher(
            Empty,
            self.execute_topic,
            10
        )

        self.stop_pub = self.create_publisher(
            Empty,
            self.stop_topic,
            10
        )

        # ----------------------------------------------------------
        # Watch for a successful planned trajectory
        # ----------------------------------------------------------

        self.display_trajectory_sub = (
            self.create_subscription(
                DisplayTrajectory,
                self.display_trajectory_topic,
                self.planned_trajectory_callback,
                10
            )
        )

        # ==========================================================
        # TF
        # ==========================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ==========================================================
        # STATE
        # ==========================================================

        self.latest_joy = None

        self.previous_buttons = []

        self.target_initialized = False

        self.target_dirty = False

        self.editing = False

        self.was_editing = False

        self.client_id = (
            '/spotarm_target_gamepad'
        )

        self.target_position = [
            0.0,
            0.0,
            0.0
        ]

        self.target_orientation = [
            0.0,
            0.0,
            0.0,
            1.0
        ]

        self.anchor_position = [
            0.0,
            0.0,
            0.0
        ]

        self.last_update_time = (
            self.get_clock().now()
        )

        # ==========================================================
        # PLAN / EXECUTE STATE
        # ==========================================================

        self.waiting_for_plan = False

        self.plan_request_time = None

        self.execute_timer = None

        # ==========================================================
        # TIMERS
        # ==========================================================

        self.control_timer = self.create_timer(
            1.0 / self.update_rate,
            self.control_loop
        )

        self.init_timer = self.create_timer(
            0.5,
            self.initialize_target
        )

        # ==========================================================
        # STARTUP LOG
        # ==========================================================

        self.get_logger().info(
            'SPOT MoveIt goal-marker gamepad started.'
        )

        self.get_logger().info(
            'LB + sticks = move BLUE MoveIt goal only.'
        )

        self.get_logger().info(
            'RB = faster preview movement.'
        )

        self.get_logger().info(
            'A = PLAN and EXECUTE blue MoveIt goal.'
        )

        self.get_logger().info(
            'B = reset blue goal to current physical robot pose.'
        )

        self.get_logger().info(
            'Physical robot does NOT move while editing.'
        )

    # ==============================================================
    # JOYSTICK
    # ==============================================================

    def joy_callback(
        self,
        msg
    ):

        self.latest_joy = msg

    def get_axis(
        self,
        index
    ):

        if self.latest_joy is None:
            return 0.0

        if (
            index < 0
            or
            index >= len(
                self.latest_joy.axes
            )
        ):

            return 0.0

        return float(
            self.latest_joy.axes[
                index
            ]
        )

    def get_button(
        self,
        index
    ):

        if self.latest_joy is None:
            return False

        if (
            index < 0
            or
            index >= len(
                self.latest_joy.buttons
            )
        ):

            return False

        return bool(
            self.latest_joy.buttons[
                index
            ]
        )

    def button_pressed(
        self,
        index
    ):

        current = self.get_button(
            index
        )

        previous = False

        if (
            index >= 0
            and
            index < len(
                self.previous_buttons
            )
        ):

            previous = bool(
                self.previous_buttons[
                    index
                ]
            )

        return (
            current
            and
            not previous
        )

    # ==============================================================
    # JOYSTICK SHAPING
    # ==============================================================

    def shape_axis(
        self,
        value
    ):

        magnitude = abs(
            value
        )

        if magnitude <= self.deadzone:

            return 0.0

        normalized = (

            magnitude
            - self.deadzone

        ) / (

            1.0
            - self.deadzone

        )

        normalized = max(
            0.0,
            min(
                normalized,
                1.0
            )
        )

        # Quadratic response gives finer movement close to center.
        normalized *= normalized

        return math.copysign(
            normalized,
            value
        )

    def sticks_centered(
        self
    ):

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

        return (

            abs(x) < 1e-6
            and
            abs(y) < 1e-6
            and
            abs(z) < 1e-6

        )

    # ==============================================================
    # CURRENT PHYSICAL END-EFFECTOR POSE
    # ==============================================================

    def get_current_robot_pose(
        self
    ):

        try:

            transform = (
                self.tf_buffer.lookup_transform(

                    self.base_frame,

                    self.ee_frame,

                    Time(),

                    timeout=Duration(
                        seconds=0.2
                    )
                )
            )

        except TransformException:

            return None

        translation = (
            transform.transform.translation
        )

        rotation = (
            transform.transform.rotation
        )

        position = [

            float(
                translation.x
            ),

            float(
                translation.y
            ),

            float(
                translation.z
            ),
        ]

        orientation = [

            float(
                rotation.x
            ),

            float(
                rotation.y
            ),

            float(
                rotation.z
            ),

            float(
                rotation.w
            ),
        ]

        return (
            position,
            orientation
        )

    # ==============================================================
    # INITIALIZE BLUE MOVEIT GOAL
    # ==============================================================

    def initialize_target(
        self
    ):

        if self.target_initialized:

            return

        current_pose = (
            self.get_current_robot_pose()
        )

        if current_pose is None:

            return

        position, orientation = (
            current_pose
        )

        self.target_position = list(
            position
        )

        self.target_orientation = list(
            orientation
        )

        self.anchor_position = list(
            position
        )

        self.target_initialized = True

        self.target_dirty = False

        self.publish_moveit_marker()

        self.get_logger().info(

            'Blue MoveIt goal initialized: '

            f'x={position[0]:.3f}, '

            f'y={position[1]:.3f}, '

            f'z={position[2]:.3f}'
        )

        self.init_timer.cancel()

    # ==============================================================
    # MOUSE / MOVEIT FEEDBACK
    # ==============================================================

    def feedback_callback(
        self,
        msg
    ):

        # Ignore feedback we generated ourselves.
        if msg.client_id == self.client_id:

            return

        if msg.marker_name != self.marker_name:

            return

        if msg.event_type not in [

            InteractiveMarkerFeedback.POSE_UPDATE,

            InteractiveMarkerFeedback.MOUSE_UP,

        ]:

            return

        self.target_position = [

            float(
                msg.pose.position.x
            ),

            float(
                msg.pose.position.y
            ),

            float(
                msg.pose.position.z
            ),
        ]

        self.target_orientation = [

            float(
                msg.pose.orientation.x
            ),

            float(
                msg.pose.orientation.y
            ),

            float(
                msg.pose.orientation.z
            ),

            float(
                msg.pose.orientation.w
            ),
        ]

        self.target_initialized = True

        self.target_dirty = True

    # ==============================================================
    # MOVE BLUE MOVEIT MARKER
    # ==============================================================

    def publish_moveit_marker(
        self
    ):

        if not self.target_initialized:

            return

        feedback = (
            InteractiveMarkerFeedback()
        )

        feedback.header.stamp = (
            self.get_clock().now().to_msg()
        )

        feedback.header.frame_id = (
            self.base_frame
        )

        feedback.client_id = (
            self.client_id
        )

        feedback.marker_name = (
            self.marker_name
        )

        feedback.control_name = (
            self.control_name
        )

        feedback.event_type = (
            InteractiveMarkerFeedback.POSE_UPDATE
        )

        feedback.pose.position.x = (
            self.target_position[0]
        )

        feedback.pose.position.y = (
            self.target_position[1]
        )

        feedback.pose.position.z = (
            self.target_position[2]
        )

        feedback.pose.orientation.x = (
            self.target_orientation[0]
        )

        feedback.pose.orientation.y = (
            self.target_orientation[1]
        )

        feedback.pose.orientation.z = (
            self.target_orientation[2]
        )

        feedback.pose.orientation.w = (
            self.target_orientation[3]
        )

        feedback.mouse_point_valid = False

        self.feedback_pub.publish(
            feedback
        )

    # ==============================================================
    # RESET BLUE GOAL TO PHYSICAL ROBOT
    # ==============================================================

    def reset_target(
        self
    ):

        # If a plan is waiting, cancel our local request.
        self.waiting_for_plan = False

        self.plan_request_time = None

        # Tell RViz / MoveIt to stop anything it may currently
        # be executing.
        self.stop_pub.publish(
            Empty()
        )

        current_pose = (
            self.get_current_robot_pose()
        )

        if current_pose is None:

            self.get_logger().warn(
                'Cannot reset target: '
                'fake_gripper TF unavailable.'
            )

            return

        position, orientation = (
            current_pose
        )

        self.target_position = list(
            position
        )

        self.target_orientation = list(
            orientation
        )

        self.anchor_position = list(
            position
        )

        self.target_initialized = True

        self.target_dirty = False

        self.publish_moveit_marker()

        self.get_logger().info(
            'Blue goal reset to current physical robot pose.'
        )

    # ==============================================================
    # EDIT BLUE MOVEIT TARGET
    # ==============================================================

    def move_target(
        self,
        dt
    ):

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

        x *= self.axis_x_sign
        y *= self.axis_y_sign
        z *= self.axis_z_sign

        moving = (

            abs(x) > 1e-6
            or
            abs(y) > 1e-6
            or
            abs(z) > 1e-6

        )

        if not moving:

            return

        speed = self.normal_speed

        if self.get_button(
            self.fast_button
        ):

            speed = (
                self.fast_speed
            )

        proposed = [

            self.target_position[0]
            + x * speed * dt,

            self.target_position[1]
            + y * speed * dt,

            self.target_position[2]
            + z * speed * dt,
        ]

        # ----------------------------------------------------------
        # Preview radius limit
        # ----------------------------------------------------------

        if self.max_preview_radius > 0.0:

            dx = (
                proposed[0]
                - self.anchor_position[0]
            )

            dy = (
                proposed[1]
                - self.anchor_position[1]
            )

            dz = (
                proposed[2]
                - self.anchor_position[2]
            )

            distance = math.sqrt(

                dx * dx
                + dy * dy
                + dz * dz

            )

            if (
                distance
                > self.max_preview_radius
            ):

                scale = (

                    self.max_preview_radius
                    / max(
                        distance,
                        1e-9
                    )

                )

                proposed = [

                    self.anchor_position[0]
                    + dx * scale,

                    self.anchor_position[1]
                    + dy * scale,

                    self.anchor_position[2]
                    + dz * scale,
                ]

        self.target_position = (
            proposed
        )

        self.target_dirty = True

        # This changes ONLY the MoveIt preview.
        self.publish_moveit_marker()

    # ==============================================================
    # A BUTTON — PLAN
    # ==============================================================

    def request_plan_and_execute(
        self
    ):

        if not self.target_initialized:

            self.get_logger().warn(
                'A ignored: goal is not initialized.'
            )

            return

        # ----------------------------------------------------------
        # LB must be released.
        # ----------------------------------------------------------

        if self.get_button(
            self.deadman_button
        ):

            self.get_logger().warn(
                'A ignored: release LB first.'
            )

            return

        # ----------------------------------------------------------
        # Sticks must be centered.
        # ----------------------------------------------------------

        if not self.sticks_centered():

            self.get_logger().warn(
                'A ignored: center sticks first.'
            )

            return

        # ----------------------------------------------------------
        # Goal must have changed.
        # ----------------------------------------------------------

        if not self.target_dirty:

            self.get_logger().info(
                'A ignored: blue goal has not changed.'
            )

            return

        # ----------------------------------------------------------
        # Don't stack planning requests.
        # ----------------------------------------------------------

        if self.waiting_for_plan:

            self.get_logger().warn(
                'A ignored: already waiting for MoveIt planning.'
            )

            return

        # Make absolutely sure MoveIt has the latest target pose.
        self.publish_moveit_marker()

        self.waiting_for_plan = True

        self.plan_request_time = (
            self.get_clock().now()
        )

        self.get_logger().warn(

            'A PRESSED — REQUESTING MOVEIT PLAN: '

            f'x={self.target_position[0]:.3f}, '

            f'y={self.target_position[1]:.3f}, '

            f'z={self.target_position[2]:.3f}'
        )

        # Equivalent to clicking PLAN in RViz.
        self.plan_pub.publish(
            Empty()
        )

    # ==============================================================
    # MOVEIT GENERATED A PLAN
    # ==============================================================

    def planned_trajectory_callback(
        self,
        msg
    ):

        # Ignore normal MoveIt display traffic unless A asked for
        # a plan.
        if not self.waiting_for_plan:

            return

        # A real DisplayTrajectory should contain at least one
        # RobotTrajectory.
        if len(
            msg.trajectory
        ) == 0:

            self.get_logger().warn(
                'MoveIt published an empty DisplayTrajectory.'
            )

            return

        self.waiting_for_plan = False

        self.plan_request_time = None

        self.get_logger().info(
            'MoveIt plan received successfully.'
        )

        self.get_logger().info(
            f'Executing in {self.execute_delay:.2f} seconds...'
        )

        if self.execute_timer is not None:

            self.execute_timer.cancel()

            self.execute_timer = None

        # create_timer() is repeating, so execute_planned_path()
        # immediately cancels it the first time it runs.
        self.execute_timer = self.create_timer(
            self.execute_delay,
            self.execute_planned_path
        )

    # ==============================================================
    # EXECUTE THE PLAN
    # ==============================================================

    def execute_planned_path(
        self
    ):

        if self.execute_timer is not None:

            self.execute_timer.cancel()

            self.execute_timer = None

        self.get_logger().warn(
            'EXECUTING MOVEIT PLAN.'
        )

        # Equivalent to clicking EXECUTE in RViz.
        self.execute_pub.publish(
            Empty()
        )

        # Require the target to be moved again before another A
        # command can execute.
        self.target_dirty = False

        self.anchor_position = list(
            self.target_position
        )

    # ==============================================================
    # PLAN TIMEOUT
    # ==============================================================

    def check_plan_timeout(
        self
    ):

        if not self.waiting_for_plan:

            return

        if self.plan_request_time is None:

            return

        elapsed = (

            self.get_clock().now()
            - self.plan_request_time

        ).nanoseconds / 1_000_000_000.0

        if elapsed < self.planning_timeout:

            return

        self.waiting_for_plan = False

        self.plan_request_time = None

        self.get_logger().error(
            'MoveIt planning timed out. '
            'The robot will NOT execute.'
        )

    # ==============================================================
    # MAIN LOOP
    # ==============================================================

    def control_loop(
        self
    ):

        now = (
            self.get_clock().now()
        )

        dt = (

            now
            - self.last_update_time

        ).nanoseconds / 1_000_000_000.0

        self.last_update_time = now

        # Avoid a large preview jump after temporary scheduling
        # delays.
        dt = max(
            0.0,
            min(
                dt,
                0.10
            )
        )

        self.check_plan_timeout()

        if not self.target_initialized:

            return

        if self.latest_joy is None:

            return

        execute_pressed = (
            self.button_pressed(
                self.execute_button
            )
        )

        cancel_pressed = (
            self.button_pressed(
                self.cancel_button
            )
        )

        # ==========================================================
        # B = RESET
        # ==========================================================

        if cancel_pressed:

            self.reset_target()

        # ==========================================================
        # LB = EDIT MOVEIT PREVIEW
        # ==========================================================

        self.editing = (
            self.get_button(
                self.deadman_button
            )
        )

        if self.editing:

            self.move_target(
                dt
            )

        if (
            self.was_editing
            and
            not self.editing
        ):

            self.get_logger().info(
                'Goal editing finished. '
                'Inspect the blue MoveIt goal. '
                'Press A to Plan + Execute, '
                'or B to reset.'
            )

        # ==========================================================
        # A = PLAN + EXECUTE
        # ==========================================================

        if execute_pressed:

            self.request_plan_and_execute()

        self.was_editing = (
            self.editing
        )

        # Must be LAST for button rising-edge detection.
        self.previous_buttons = list(
            self.latest_joy.buttons
        )


def main(
    args=None
):

    rclpy.init(
        args=args
    )

    node = (
        SpotArmTargetGamepad()
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
