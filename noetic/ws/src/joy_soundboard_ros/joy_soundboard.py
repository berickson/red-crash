#!/usr/bin/env python3

import rospy
import os
import glob


from sensor_msgs.msg import Joy
from rospkg import RosPack

previous_joy_msg = None

rospy.init_node("joy_soundboard", anonymous=True)
rp = RosPack()
sound_folder = rp.get_path("joy_soundboard_ros")+"/sounds"

def play_file_for_button(button_number):
    global sound_folder
    matches = glob.glob(sound_folder + "/" + str(button_number)+"-*")
    if len(matches)==1:
        sound_path = "\""+matches[0]+"\""
        print("playing "+sound_path)
        os.system("play "+sound_path)

def joy_callback(joy_msg : Joy):

    global previous_joy_msg
    joy_msg.buttons
    if previous_joy_msg is not None:
        for i in range(len(joy_msg.buttons)):
            # discard messages more than one second old to avoid stuttering
            if joy_msg.buttons[i] == 1 and previous_joy_msg.buttons[i] == 0:
                msg_secs = joy_msg.header.stamp.secs
                now = rospy.get_time()
                if (msg_secs + 1 < now):
                    print('ignoring old message')
                else:
                    print("you pressed button " + str(i))
                    os.system("pwd")
                    print(sound_folder)
                    play_file_for_button(i)
    previous_joy_msg = joy_msg
    


rospy.Subscriber("joy", Joy, joy_callback)

rospy.spin()

print("Done")