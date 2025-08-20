#!/usr/bin/env python3
from setuptools import find_packages, setup

package_name = 'turtlesim_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install package resource files
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch', ['launch/turtle_test.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cyberai',
    maintainer_email='cyberai@todo.todo',
    description='Learning package to control turtlesim with ROS2',
    license='MIT',
    # Define executable commands
    entry_points={
        'console_scripts': [
            'turtle_publisher = turtlesim_test.turtle_publisher:main',
            'turtle_service = turtlesim_test.turtle_service:main',
            'turtle_param = turtlesim_test.turtle_param:main',
            'turtle_action = turtlesim_test.turtle_action:main',
            'turtle_keyboard = turtlesim_test.turtle_keyboard:main',
        ],
    },
)