import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # ============================================================
    # Launch arguments
    # ============================================================

    start_rviz = LaunchConfiguration("start_rviz")
    start_commander = LaunchConfiguration("start_commander")


    # ============================================================
    # MoveIt configuration
    # ============================================================

    moveit_config = (
        MoveItConfigsBuilder(
            "spotarm_assembly",
            package_name="robot_moveit_config",
        )
        .robot_description(
            file_path="config/spotarm_assembly.hardware.urdf.xacro"
        )
        .to_moveit_configs()
    )


    # ============================================================
    # Paths
    # ============================================================

    package_share = get_package_share_directory(
        "robot_moveit_config"
    )

    controllers_file = os.path.join(
        package_share,
        "config",
        "ros2_controllers_hardware.yaml",
    )

    rviz_config = os.path.join(
        package_share,
        "config",
        "moveit.rviz",
    )


    # ============================================================
    # world -> base_link
    # ============================================================

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


    # ============================================================
    # Robot State Publisher
    # ============================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
        ],
    )


    # ============================================================
    # ros2_control
    #
    # IMPORTANT:
    # Do NOT add:
    #
    #     name="controller_manager"
    #
    # We already discovered that explicitly renaming this process
    # causes controller nodes to inherit the same node name.
    # ============================================================

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            controllers_file,
        ],
    )


    # ============================================================
    # Configure real Arm hardware
    #
    # unconfigured -> inactive
    # ============================================================

    configure_arm = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "set_hardware_component_state",
            "Arm",
            "inactive",
        ],
        output="screen",
    )


    # ============================================================
    # Activate real Arm hardware
    #
    # inactive -> active
    # ============================================================

    activate_arm = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "set_hardware_component_state",
            "Arm",
            "active",
        ],
        output="screen",
    )


    # ============================================================
    # Joint State Broadcaster
    # ============================================================

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


    # ============================================================
    # Real trajectory controller
    # ============================================================

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
    )


    # ============================================================
    # MoveIt move_group
    #
    # Do NOT explicitly set name="move_group".
    # ============================================================

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )


    # ============================================================
    # RViz
    # ============================================================

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
        condition=IfCondition(start_rviz),
    )


    # ============================================================
    # Cartesian commander
    #
    # Same parameters as commander.launch.py.
    # ============================================================

    commander_node = Node(
        package="robot_commander",
        executable="commander",
        name="commander",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
        condition=IfCondition(start_commander),
    )


    # ============================================================
    # Startup sequence
    # ============================================================

    # Give controller_manager a moment to start.
    start_configure_arm = TimerAction(
        period=2.0,
        actions=[
            configure_arm,
        ],
    )


    # After Arm has been configured:
    #
    # inactive -> active
    configure_to_activate = RegisterEventHandler(
        OnProcessExit(
            target_action=configure_arm,
            on_exit=[
                activate_arm,
            ],
        )
    )


    # After Arm activation, start joint states.
    activate_to_joint_states = RegisterEventHandler(
        OnProcessExit(
            target_action=activate_arm,
            on_exit=[
                joint_state_broadcaster_spawner,
            ],
        )
    )


    # After joint state broadcaster finishes spawning,
    # start the trajectory controller.
    joint_states_to_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                arm_controller_spawner,
            ],
        )
    )


    # After arm_controller is loaded/configured/activated,
    # start MoveIt and RViz.
    controller_to_moveit = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[
                move_group_node,
                rviz_node,

                # Give move_group several seconds to become ready
                # before starting our custom commander.
                TimerAction(
                    period=4.0,
                    actions=[
                        commander_node,
                    ],
                ),
            ],
        )
    )


    # ============================================================
    # Launch description
    # ============================================================

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Launch RViz.",
            ),

            DeclareLaunchArgument(
                "start_commander",
                default_value="true",
                description=(
                    "Launch the SpotArm Cartesian commander."
                ),
            ),

            # Initial processes
            static_tf_node,
            robot_state_publisher,
            ros2_control_node,

            # Hardware startup sequence
            start_configure_arm,

            # Event-based sequencing
            configure_to_activate,
            activate_to_joint_states,
            joint_states_to_arm_controller,
            controller_to_moveit,
        ]
    )
