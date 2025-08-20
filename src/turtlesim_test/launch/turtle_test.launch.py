#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start turtlesim simulator
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        # Start our turtle publisher
        Node(
            package='turtlesim_test',
            executable='turtle_publisher',
            name='turtle_pub'
        )
    ])