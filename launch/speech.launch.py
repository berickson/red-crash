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
                'speaker_id': 61,
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
        Node(
            package='speech_ros',
            executable='costume_compliment',
            name='costume_compliment',
            output='screen',
            parameters=[{
                'photo_button_index': 13,
                'image_topic': '/oak/rgb/image_raw',
                'save_photos': True,
            }]
        ),
    ])
