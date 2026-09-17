import os

from glob import glob

from setuptools import (
    find_packages,
    setup,
)


package_name = (
    'spotarm_servo_gamepad'
)


setup(

    name=package_name,

    version='0.1.0',

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

        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),

            glob(
                'launch/*.launch.py'
            )
        ),

        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),

            glob(
                'config/*.yaml'
            )
        ),

    ],

    install_requires=[

        'setuptools',

        'PyYAML',

    ],

    zip_safe=True,

    maintainer='msd',

    maintainer_email='msd@example.com',

    description=(
        'MoveIt Servo gamepad controller '
        'for the SPOT robot arm'
    ),

    license='Apache-2.0',

    entry_points={

        'console_scripts': [

            (
                'gamepad_servo = '
                'spotarm_servo_gamepad.'
                'gamepad_servo:main'
            ),

        ],

    },

)
