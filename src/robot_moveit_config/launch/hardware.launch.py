import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------

    start_arm_controller = LaunchConfiguration(
        "start_arm_controller"
    )


    # --------------------------------------------------
    # MoveIt configuration
    #
    # Use the REAL hardware URDF wrapper.
    # --------------------------------------------------

    moveit_config = (
        MoveItConfigsBuilder(
            "spotarm_assembly",
            package_name="robot_moveit_config"
        )
        .robot_description(
            file_path="config/spotarm_assembly.hardware.urdf.xacro"
        )
        .to_moveit_configs()
    )


    # --------------------------------------------------
    # Package paths
    # --------------------------------------------------

    package_share = get_package_share_directory(
        "robot_moveit_config"
    )

    controllers_file = os.path.join(
        package_share,
        "config",
        "ros2_controllers_hardware.yaml"
    )

    rviz_config = os.path.join(
        package_share,
        "config",
        "moveit.rviz"
    )


    # --------------------------------------------------
    # Static transform
    #
    # world -> base_link
    # --------------------------------------------------

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher0",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "world",
            "base_link",
        ],
    )


    # --------------------------------------------------
    # Robot State Publisher
    # --------------------------------------------------

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description
        ],
    )


    # --------------------------------------------------
    # ros2_control
    #
    # IMPORTANT:
    # Do NOT set:
    #
    #     name="controller_manager"
    #
    # here.
    #
    # ros2_control_node already creates controller_manager.
    # A global name remap can cause loaded controllers to
    # inherit the controller_manager name.
    # --------------------------------------------------

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            controllers_file,
        ],
    )


    # --------------------------------------------------
    # Joint State Broadcaster
    # --------------------------------------------------

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
        ],
    )


    # --------------------------------------------------
    # Arm trajectory controller
    #
    # Disabled by default for real-hardware bring-up.
    # --------------------------------------------------

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_arm_controller",
        output="screen",
        arguments=[
            "arm_controller",
            "-c",
            "/controller_manager",
        ],
        condition=IfCondition(
            start_arm_controller
        ),
    )


    # --------------------------------------------------
    # MoveIt move_group
    #
    # IMPORTANT:
    # Do not explicitly remap its node name either.
    # --------------------------------------------------

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict()
        ],
    )


    # --------------------------------------------------
    # RViz
    # --------------------------------------------------

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config,
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )


    # --------------------------------------------------
    # Launch Description
    # --------------------------------------------------

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_arm_controller",
                default_value="false",
                description=(
                    "Start the real hardware "
                    "trajectory controller."
                ),
            ),

            static_tf_node,
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            move_group_node,
            rviz_node,
        ]
    )
