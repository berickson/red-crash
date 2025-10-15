#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments with defaults matching original ROS1 setup
        DeclareLaunchArgument(
            'device_name',
            default_value='/dev/roboclaw',
            description='RoboClaw device path'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for RoboClaw communication'
        ),
        DeclareLaunchArgument(
            'device_port',
            default_value='128',
            description='RoboClaw device address'
        ),

        # RoboClaw Driver Node
        Node(
            package='ros2_roboclaw_driver',
            executable='ros2_roboclaw_driver_node',
            name='roboclaw_node',
            output='screen',
            parameters=[{
                # Device connection parameters
                'device_name': LaunchConfiguration('device_name'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'device_port': LaunchConfiguration('device_port'),
                
                # Acceleration control
                'accel_quad_pulses_per_second': 32000,
                
                # PID settings (from original config)
                'm1_p': 5000.0,
                'm1_i': 0.0,
                'm1_d': 0.0,
                'm1_qpps': 2437,
                'm2_p': 5000.0,
                'm2_i': 0.0,
                'm2_d': 0.0,
                'm2_qpps': 2437,
                
                # Current limits
                'm1_max_current': 25.0,
                'm2_max_current': 25.0,
                
                # Velocity limits
                'max_angular_velocity': 2.0,
                'max_linear_velocity': 2.0,
                
                # Safety timeout
                'max_seconds_uncommanded_travel': 0.01,
                
                # Publishing settings
                'publish_joint_states': False,
                'publish_odom': True,
                
                # Robot geometry (from config file)
                'quad_pulses_per_meter': 1426,
                'quad_pulses_per_revolution': 537.0,
                'wheel_radius': 0.06,
                'wheel_separation': 0.414,
                
                # Status publishing
                'roboclaw_status_topic': 'roboclaw_status',
                'sensor_rate_hz': 20.0,
                
                # Serial timeout (seconds, 0.0 disables timeout)
                'serial_timeout': 0.5,
                
                # Debug settings
                'do_debug': False,
                'do_low_level_debug': False,
            }]
        ),
    ])