#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare argument for joystick driver choice
    joy_driver_arg = DeclareLaunchArgument(
        'driver',
        default_value='joy',  
        description='Choose joystick driver: "joy"'
    )

    driver = LaunchConfiguration('driver')

    confDir = get_package_share_directory('teleop_control')
    confPath = os.path.join(confDir, "launch/joy.yaml")

    return LaunchDescription([
        joy_driver_arg,

        # --- Joy driver ---
        Node(
            package='joy',
            executable='joy_node',
            name='telop_joy_driver',
            output='screen',
            remappings=[
                ('/joy', '/telop_joy'),
                ('/joy/set_feedback', '/telop_joy/set_feedback')
            ]
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[confPath],
            output='screen',
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/joy', '/telop_joy')
            ]
        ),

        # --- Joy Linux driver ---
        #Node(
        #    package='joy_linux',
        #    executable='joy_linux_node',
        #    name='joy_driver_linux',
        #    output='screen',
        #),

        # Teleop joy
        #Node(
        #    package='teleop_control',
        #    executable='teleop_joy.py',
        #    name='teleop_joy',
        #    output='screen',
        #    remappings=[
        #        ('/cmd_vel', '/telop_cmd_vel')
        #    ]
        #),

    ])
