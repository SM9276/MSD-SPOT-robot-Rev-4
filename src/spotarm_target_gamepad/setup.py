import os

from glob import glob

from setuptools import find_packages
from setuptools import setup


package_name = (
    'spotarm_target_gamepad'
)


setup(

    name=package_name,

    version='0.2.0',

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

    ],

    install_requires=[

        'setuptools',

    ],

    zip_safe=True,

    maintainer='msd',

    maintainer_email='msd@example.com',

    description=(
        'MoveIt RViz goal-marker gamepad controller '
        'for the SPOT Rev-4 arm'
    ),

    license='Apache-2.0',

    entry_points={

        'console_scripts': [

            (
                'target_gamepad = '
                'spotarm_target_gamepad.'
                'target_gamepad:main'
            ),

        ],

    },

)

