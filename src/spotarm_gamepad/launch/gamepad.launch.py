from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    joy_node = Node(

        package='joy',

        executable='joy_node',

        name='joy_node',

        output='screen',

        parameters=[{

            'deadzone': 0.05,

            'autorepeat_rate': 20.0,

        }],

    )


    gamepad_node = Node(

        package='spotarm_gamepad',

        executable='gamepad',

        name='spotarm_gamepad',

        output='screen',

        parameters=[{

            # Stick deadzone
            'deadzone': 0.15,

            # Precision mode:
            # 2 mm maximum increment per update
            'normal_step': 0.002,

            # RB fast mode:
            # 5 mm maximum increment
            'fast_step': 0.005,

            # Current robot hardware is still relatively slow.
            'publish_rate_hz': 5.0,


            # Xbox-style defaults
            #
            # Change after inspecting /joy if necessary.

            # Left stick vertical -> X
            'axis_x': 1,

            # Left stick horizontal -> Y
            'axis_y': 0,

            # Right stick vertical -> Z
            'axis_z': 4,


            # Flip any of these from 1.0 to -1.0
            # if an axis feels backwards.
            'axis_x_sign': 1.0,
            'axis_y_sign': 1.0,
            'axis_z_sign': 1.0,


            # Xbox-style buttons
            'button_a': 0,
            'button_b': 1,
            'button_x': 2,
            'button_y': 3,
            'button_lb': 4,
            'button_rb': 5,

        }],

    )


    return LaunchDescription([

        joy_node,

        gamepad_node,

    ])
