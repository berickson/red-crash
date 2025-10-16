#!/usr/bin/env python3

"""
Launch file for OAK-D camera using official depthai-ros driver.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Path to our configuration file
    config_file = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        'config',
        'oakd_config.yaml'
    )
    
    # Get the depthai_ros_driver package share directory
    depthai_ros_driver_dir = get_package_share_directory('depthai_ros_driver')
    
    # Path to the official camera launch file
    camera_launch_file = os.path.join(
        depthai_ros_driver_dir,
        'launch',
        'camera.launch.py'
    )
    
    return LaunchDescription([
        # Include the official depthai camera launch file with our parameters
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_file),
            launch_arguments={
                'name': 'oak',                          # Node name and topic prefix
                'params_file': config_file,             # Our config file
                'use_rviz': 'false',                    # Don't auto-launch RViz
                'parent_frame': 'base_link',            # TF parent frame
                'cam_pos_x': '0.10',                    # Camera position X (10cm forward)
                'cam_pos_y': '0.0',                     # Camera position Y
                'cam_pos_z': '0.15',                    # Camera position Z (15cm up)
                'cam_roll': '0.0',                      # Camera roll
                'cam_pitch': '0.0',                     # Camera pitch
                'cam_yaw': '0.0',                       # Camera yaw
            }.items()
        ),
    ])
