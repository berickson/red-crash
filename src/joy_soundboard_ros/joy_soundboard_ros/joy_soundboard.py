#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import glob

from sensor_msgs.msg import Joy
from ament_index_python.packages import get_package_share_directory


class JoySoundboardNode(Node):
    def __init__(self):
        super().__init__('joy_soundboard')
        
        # Get the path to the sounds directory
        package_share_directory = get_package_share_directory('joy_soundboard_ros')
        self.sound_folder = os.path.join(package_share_directory, 'sounds')
        
        self.get_logger().info(f'Sound folder: {self.sound_folder}')
        
        # Keep track of previous joy message to detect button presses
        self.previous_joy_msg = None
        
        # Subscribe to joy topic
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
    
    def play_file_for_button(self, button_number):
        """Play the sound file associated with the given button number"""
        matches = glob.glob(os.path.join(self.sound_folder, f"{button_number}-*"))
        if len(matches) == 1:
            sound_path = f'"{matches[0]}"'
            self.get_logger().info(f'Playing {sound_path}')
            os.system(f'play -v 4.0 {sound_path}')
        elif len(matches) == 0:
            self.get_logger().debug(f'No sound file found for button {button_number}')
        else:
            self.get_logger().warning(f'Multiple sound files found for button {button_number}')
    
    def joy_callback(self, joy_msg):
        """Handle joystick messages and play sounds on button press"""
        if self.previous_joy_msg is not None:
            for i in range(len(joy_msg.buttons)):
                # Detect button press (transition from 0 to 1)
                if joy_msg.buttons[i] == 1 and self.previous_joy_msg.buttons[i] == 0:
                    # Check message age to avoid stuttering from old messages
                    msg_time = joy_msg.header.stamp.sec + joy_msg.header.stamp.nanosec * 1e-9
                    now = self.get_clock().now().nanoseconds * 1e-9
                    
                    if msg_time + 1.0 < now:
                        self.get_logger().info('Ignoring old message')
                    else:
                        self.get_logger().info(f'Button {i} pressed')
                        self.play_file_for_button(i)
        
        self.previous_joy_msg = joy_msg


def main(args=None):
    rclpy.init(args=args)
    node = JoySoundboardNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
