from setuptools import find_packages, setup

package_name = 'robot_status_leds'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kyle-berg',
    maintainer_email='khb120@gmail.com',
    description='Status LEDs controller: set NeoPixel LED colors when robot is moving or static.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
entry_points={
        'console_scripts': [
            'led_state_node = robot_status_leds.led_state_node:main',
        ],
    },
)
