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
                'speaker_volume_percent': 65.0,
                'use_microphone': True,
                'enable_push_to_talk': True,
                'enable_wake_word': False,
                'ptt_button_index': 12,
                'pause_threshold': 1.5,
                'phrase_time_limit': 30.0
            }]
        ),
        Node(
            package='speech_ros',
            executable='command_interpreter',
            name='command_interpreter',
            output='screen'
        ),
    ])
