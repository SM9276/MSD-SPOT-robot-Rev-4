from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "spotarm_assembly",
            package_name="robot_moveit_config"
        )
        .to_moveit_configs()
    )

    commander = Node(
        package="robot_commander",
        executable="commander",
        name="commander",
        output="screen",
        parameters=[
            moveit_config.to_dict()
        ],
    )

    return LaunchDescription([
        commander
    ])
