import os
import yaml

from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess

from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory,
)

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # =========================================================
    # MoveIt configuration
    # =========================================================

    moveit_config = (
        MoveItConfigsBuilder(
            "spotarm_assembly",
            package_name="robot_moveit_config",
        )
        .robot_description(
            file_path=(
                "config/"
                "spotarm_assembly.hardware.urdf.xacro"
            )
        )
        .to_moveit_configs()
    )

    package_share = get_package_share_directory(
        "robot_moveit_config"
    )

    servo_file = os.path.join(
        package_share,
        "config",
        "spotarm_servo.yaml",
    )

    with open(servo_file, "r") as f:
        servo_yaml = yaml.safe_load(f)

    servo_params = {
        "moveit_servo": servo_yaml
    }

    # =========================================================
    # MoveIt Servo
    # =========================================================

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",

        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
    )

    # =========================================================
    # Start Servo
    # =========================================================

    start_servo = TimerAction(
        period=3.0,

        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/servo_node/start_servo",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            servo_node,
            start_servo,
        ]
    )
