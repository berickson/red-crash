#!/bin/bash
# Helper script to clean and rebuild joy_soundboard_ros
# This ensures old sound files are removed before installing new ones

cd /root/ros2_ws
rm -rf install/joy_soundboard_ros/share/joy_soundboard_ros/sounds
colcon build --symlink-install --packages-select joy_soundboard_ros
