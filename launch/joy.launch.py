#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Joy Node - matches original ROS1 parameters exactly
        Node(
            package='joy',
            executable='joy_node',
            name='ps3_joy',
            output='screen',
            parameters=[{
                'deadzone': 0.0001,  # (Reduced) Match original deadzone
                'autorepeat_rate': 50.0,  # Match original autorepeat_rate
            }]
        ),
        
        # Teleop Twist Joy Node - fixed axis mappings
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            # remappings=[
            #     ('cmd_vel', 'fake_cmd_vel'),
            # ],
            parameters=[{
                'axis_angular.yaw': 0,  # Left stick horizontal
                'axis_linear.x': 1,     # Left stick vertical
                'enable_button': 1,  # B Button
                'scale_linear.x': 0.5,
                'scale_angular.yaw': 0.05,
                'enable_turbo_button': 0,  # A Button
                'scale_linear_turbo.x': 1.5,
                'scale_angular_turbo.yaw': 0.1,
            }]
        ),        # Soundboard Node (will need to be migrated separately)
        # Node(
        #     package='joy_soundboard_ros',
        #     executable='joy_soundboard.py',
        #     name='soundboard_node',
        #     output='screen'
        # ),
    ])