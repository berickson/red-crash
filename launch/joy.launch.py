#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Joy Node - auto-detects joystick devices
        Node(
            package='joy',
            executable='joy_node',
            name='ps3_joy',
            output='screen',
            parameters=[{
                'deadzone': 0.012,
                'autorepeat_rate': 5.0,
            }]
        ),

        # Teleop Twist Joy Node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            parameters=[{
                'enable_button': 0,  # X button (button 0)
                'scale_linear': 0.2,
                'scale_angular': 0.5,
                'enable_turbo_button': 1,  # Circle button (button 1)
                'scale_linear_turbo': 0.5,
                'scale_angular_turbo': 1.57,
            }]
        ),

        # Soundboard Node (will need to be migrated separately)
        # Node(
        #     package='joy_soundboard_ros',
        #     executable='joy_soundboard.py',
        #     name='soundboard_node',
        #     output='screen'
        # ),
    ])