from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_description_share_dir = get_package_share_directory('spotarm_assembly_description')
    robot_bringup_share_dir = get_package_share_directory('robot_bringup')
    robot_moveit_share_dir = get_package_share_directory('robot_moveit_config')

    xacro_file = os.path.join(robot_description_share_dir, 'urdf', 'spotarm_assembly.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(robot_bringup_share_dir, 'config', 'robot_moveit.rviz')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(
            robot_bringup_share_dir,
            'config',
            'ros2_controllers.yaml'
        ),
            {'robot_description': robot_urdf}
        ],
        # arguments=['--ros-args', '-p', 'robot_description:="$(xacro ' + xacro_file + ')"'],
    )

    controller_manager_spawner_jsb_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    controller_manager_spawner_arm_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    led_state_node = Node(
        package='robot_status_leds',
        executable='led_state_node',
        name='led_state_node',
        output='screen',
        parameters=[{
            'led_backend': 'rpi_neopixel',
            'led_count': 120,
            'led_pin': 18,
            'brightness': 0.5,
            'static_color': '#00FF00',
            'moving_color': '#FF0000',
            'velocity_threshold': 0.01,
            'moving_timeout_ms': 500,
        }]
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                robot_moveit_share_dir,
                'launch',
                'move_group.launch.py'
            )
        )
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        move_group_launch,
        controller_manager_node,
        controller_manager_spawner_jsb_node,
        controller_manager_spawner_arm_controller_node,
        rviz_node, 
        led_state_node
    ])
