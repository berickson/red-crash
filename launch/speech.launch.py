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
                'speaker_volume_percent': 200.0,
                'use_microphone': True,
                'enable_push_to_talk': True,
                'enable_wake_word': False,
                'ptt_button_index': 12,
                'pause_threshold': 1.5,
                'phrase_time_limit': 30.0,
                'speaker_id': 65,
                'length_scale': 1.0,
                'noise_scale': 0.667,
                'noise_w_scale': 0.8
            }]
        ),
        Node(
            package='speech_ros',
            executable='command_interpreter',
            name='command_interpreter',
            output='screen'
        ),
    ])
