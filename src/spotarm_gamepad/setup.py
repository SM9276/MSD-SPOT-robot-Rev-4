from setuptools import find_packages, setup


package_name = 'spotarm_gamepad'


setup(
    name=package_name,

    version='0.0.1',

    packages=find_packages(
        exclude=[
            'test'
        ]
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [
                'resource/' + package_name
            ]
        ),

        (
            'share/' + package_name,
            [
                'package.xml'
            ]
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='msd',

    maintainer_email='msd@example.com',

    description=(
        'Gamepad teleoperation node for '
        'the SPOT robot arm using MoveIt Servo'
    ),

    license='Apache-2.0',

    tests_require=[
        'pytest'
    ],

    entry_points={
        'console_scripts': [
            (
                'gamepad = '
                'spotarm_gamepad.gamepad_node:main'
            ),
        ],
    },
)
