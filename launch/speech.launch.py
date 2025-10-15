from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speech_ros',
            executable='speech',
            name='speech',
            output='screen',
            parameters=[{
                'speaker_volume_percent': 35.0,
                'use_microphone': True
            }]
        ),
        Node(
            package='speech_ros',
            executable='command_interpreter',
            name='command_interpreter',
            output='screen'
        ),
    ])
