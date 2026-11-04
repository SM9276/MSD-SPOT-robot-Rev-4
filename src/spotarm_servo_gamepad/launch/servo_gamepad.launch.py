import os
import yaml

from launch import LaunchDescription

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory,
)

from moveit_configs_utils import (
    MoveItConfigsBuilder,
)


def load_yaml_file(path):

    with open(
        path,
        'r'
    ) as file:

        return yaml.safe_load(
            file
        )


def generate_launch_description():

    # ============================================================
    # MOVEIT ROBOT CONFIGURATION
    # ============================================================

    moveit_config = (

        MoveItConfigsBuilder(
            'spotarm_assembly',
            package_name='robot_moveit_config'
        )

        .robot_description(
            file_path=(
                'config/'
                'spotarm_assembly.hardware.urdf.xacro'
            )
        )

        .to_moveit_configs()
    )


    # ============================================================
    # PACKAGE PATHS
    # ============================================================

    package_share = (
        get_package_share_directory(
            'spotarm_servo_gamepad'
        )
    )


    servo_config_path = os.path.join(
        package_share,
        'config',
        'servo.yaml'
    )


    servo_yaml = load_yaml_file(
        servo_config_path
    )


    # Humble expects the Servo parameters under the
    # "moveit_servo" namespace.
    servo_params = {

        'moveit_servo':
            servo_yaml

    }


    # ============================================================
    # STARTUP POSE (for the X button)
    #
    # Read from the same file the hardware launch uses for its
    # startup move, so X always returns to that pose.
    # ============================================================

    pose_joint_names = [
        'Revolute1',
        'Revolute2',
        'Revolute3',
        'Revolute4',
        'Revolute5',
        'Revolute6',
    ]

    try:

        initial_positions = load_yaml_file(
            os.path.join(
                get_package_share_directory(
                    'robot_moveit_config'
                ),
                'config',
                'initial_positions.yaml'
            )
        )['initial_positions']

        start_pose = [
            float(initial_positions[name])
            for name in pose_joint_names
        ]

    except Exception:

        # Fall back to the node's built-in default.
        start_pose = [
            0.4745, -2.0161, -1.5595, 0.0, 1.2, 0.0
        ]


    # ============================================================
    # JOYSTICK DRIVER
    # ============================================================

    joy_node = Node(

        package='joy',

        executable='joy_node',

        name='joy_node',

        output='screen',

        parameters=[{

            # Linux joystick driver deadzone.
            #
            # We perform our main shaping in gamepad_servo.py.
            'deadzone': 0.05,

            # Republish the latest joystick state.
            'autorepeat_rate': 10.0,

        }],

    )


    # ============================================================
    # MOVEIT SERVO
    #
    # ROS 2 Humble uses servo_node_main.
    # ============================================================

    servo_node = Node(

        package='moveit_servo',

        executable='servo_node_main',

        output='screen',

        parameters=[

            servo_params,

            moveit_config.robot_description,

            moveit_config.robot_description_semantic,

            moveit_config.robot_description_kinematics,

        ],

    )


    # ============================================================
    # GAMEPAD → MOVEIT SERVO
    # ============================================================

    gamepad_node = Node(

        package='spotarm_servo_gamepad',

        executable='gamepad_servo',

        name='spotarm_servo_gamepad',

        output='screen',

        parameters=[{

            'command_frame':
                'base_link',

            'deadzone':
                0.15,


            # Normal mode uses 50% of Servo's configured
            # maximum velocity.
            'normal_gain':
                0.5,


            # Hold RB for full configured Servo speed.
            'fast_gain':
                1.0,


            'publish_rate':
                20.0,


            # ====================================================
            # Xbox-style mapping
            # ====================================================

            # Left stick vertical
            'axis_x':
                1,

            # Left stick horizontal
            'axis_y':
                0,

            # Right stick vertical
            'axis_z':
                4,


            # Flip these if directions feel backwards.
            'axis_x_sign':
                1.0,

            'axis_y_sign':
                -1.0,

            'axis_z_sign':
                1.0,


            # D-pad (hat) axes. Up/down -> X, left/right -> Y.
            # Flip the signs if a direction feels backwards.
            'dpad_x_axis':
                7,

            'dpad_y_axis':
                6,

            'dpad_x_sign':
                1.0,

            'dpad_y_sign':
                -1.0,


            # B = stand straight up, X = startup pose.
            # Both only work while the deadman (LB) is held, and
            # releasing LB stops the move.
            'up_button':
                1,

            'start_button':
                2,

            'up_pose':
                [0.4745, -1.5707, 0.0, 0.0, 0.0, 0.0],

            'start_pose':
                start_pose,

            'pose_move_speed':
                0.1,


            # LB
            'deadman_button':
                4,

            # RB
            'fast_button':
                5,

        }],

    )


    # ============================================================
    # LAUNCH
    # ============================================================

    return LaunchDescription([

        joy_node,

        servo_node,

        gamepad_node,

    ])
