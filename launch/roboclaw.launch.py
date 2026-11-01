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
            output='screen',
            # Backstop: restart if the driver still dies (e.g. RoboClaw
            # missing at startup). Mid-run USB drops are handled in the driver.
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                # Device connection parameters
                'device_name': LaunchConfiguration('device_name'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'device_port': LaunchConfiguration('device_port'),
                
                # Acceleration control
                'max_linear_acceleration': 2.5,
                
                # PID settings (from original config)
                'm1_pid_p': 3.0,
                'm1_pid_i': 0.3,
                'm1_pid_d': 1.0,
                'm1_pid_qpps_max': 2437,
                'm2_pid_p': 3.0,
                'm2_pid_i': 0.3,
                'm2_pid_d': 1.0,
                'm2_pid_qpps_max': 2437,
                
                # Current limits
                'm1_max_current': 10.0,
                'm2_max_current': 10.0,
                
                # Velocity limits
                'max_angular_velocity': 2.0,
                'max_linear_velocity': 2.0,
                
                # Safety timeout
                'max_seconds_uncommanded_travel': 0.5,
                
                # Publishing settings
                'publish_joint_states': False,
                'publish_odom': True,
                
                # Robot geometry (from config file)
                'meters_per_quad_pulse': 1.0 / 1426,
                'quad_pulses_per_revolution': 537.0,
                'wheel_separation': 0.414,
                
                # Status publishing
                'roboclaw_status_topic': 'roboclaw_status',
                
                # Serial timeout (seconds, 0.0 disables timeout)
                'serial_timeout': 0.5,
                
                # Current protection parameters
                'current_filter_window_seconds': 1.0,
                'recovery_timeout_seconds': 5.0,
                
                # Debug settings
                'do_debug': False,
                'do_low_level_debug': False,
            }]
        ),
    ])