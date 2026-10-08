from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    # ============================================================
    # JOYSTICK
    # ============================================================

    joy_node = Node(

        package='joy',

        executable='joy_node',

        name='joy_node',

        output='screen',

        parameters=[{

            'deadzone':
                0.05,

            'autorepeat_rate':
                20.0,

            'coalesce_interval_ms':
                5,

        }],
    )

    # ============================================================
    # TARGET GAMEPAD
    # ============================================================

    target_gamepad_node = Node(

        package='spotarm_target_gamepad',

        executable='target_gamepad',

        name='spotarm_target_gamepad',

        output='screen',

        parameters=[{

            # ----------------------------------------------------
            # Robot
            # ----------------------------------------------------

            'base_frame':
                'base_link',

            'ee_frame':
                'fake_gripper',

            # ----------------------------------------------------
            # MoveIt interactive marker
            # ----------------------------------------------------

            'feedback_topic':
                (
                    '/rviz_moveit_motion_planning_display/'
                    'robot_interaction_interactive_marker_topic/'
                    'feedback'
                ),

            'marker_name':
                'EE:goal_fake_gripper',

            'control_name':
                'move',

            # ----------------------------------------------------
            # RViz remote MoveIt controls
            # ----------------------------------------------------

            'plan_topic':
                '/rviz/moveit/plan',

            'execute_topic':
                '/rviz/moveit/execute',

            'stop_topic':
                '/rviz/moveit/stop',

            'display_trajectory_topic':
                '/display_planned_path',

            'execute_delay':
                0.20,

            'planning_timeout':
                10.0,

            # ----------------------------------------------------
            # 8BitDo / Xbox-style controller
            # ----------------------------------------------------

            # Left stick vertical
            'axis_x':
                1,

            # Left stick horizontal
            'axis_y':
                0,

            # Right stick vertical
            'axis_z':
                4,

            'axis_x_sign':
                1.0,

            'axis_y_sign':
                -1.0,

            'axis_z_sign':
                1.0,

            # LB
            'deadman_button':
                4,

            # RB
            'fast_button':
                5,

            # A
            'execute_button':
                0,

            # B
            'cancel_button':
                1,

            # ----------------------------------------------------
            # Preview editing
            # ----------------------------------------------------

            'deadzone':
                0.18,

            # meters / second
            'normal_speed':
                0.05,

            'fast_speed':
                0.15,

            'update_rate':
                30.0,

            # meters
            'max_preview_radius':
                0.40,

        }],
    )

    # ============================================================
    # LAUNCH
    # ============================================================

    return LaunchDescription([

        joy_node,

        target_gamepad_node,

    ])
