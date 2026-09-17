import os

from glob import glob

from setuptools import find_packages
from setuptools import setup


package_name = 'spotarm_gamepad'


setup(

    name=package_name,

    version='0.1.0',

    packages=find_packages(
        exclude=['test']
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
        'setuptools'
    ],

    zip_safe=True,

    maintainer='msd',

    maintainer_email='msd@example.com',

    description=(
        'Gamepad controller for the SPOT robot arm'
    ),

    license='Apache-2.0',

    entry_points={

        'console_scripts': [

            (
                'gamepad = '
                'spotarm_gamepad.gamepad_node:main'
            ),

        ],

    },

)
