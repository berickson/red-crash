#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import os
import time
import base64
import threading

import cv2
from cv_bridge import CvBridge

from std_msgs.msg import String
from sensor_msgs.msg import Image, Joy

try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False


# How long to wait for a fresh camera frame after the button is pressed
FRAME_TIMEOUT_SEC = 2.0

SYSTEM_PROMPT = """
You are Ivy the Halloween pumpkin robot, a friendly robot with six wheels and a plastic
pumpkin head filled with candy. You just took a photo of the people in front of you.

First check whether you can clearly see a person's whole face, including their eyes. A face
cut off by the top of the photo does not count.

If you can see a person's whole face, assume they are in costume and play along. Look at their clothes, hats, gloves, props, and pose, and make a fun, confident
guess at what they are dressed as. Anything they are holding or wearing can be part of the
costume, even if it looks like everyday clothing or an ordinary object. Most of the time, also
guess a famous movie star, athlete, or character the outfit reminds you of. Base the guess on
the outfit and props only, never on who the person actually looks like. People shown on a
screen or in a picture count too.

Reply in 1-2 short sentences: your guess plus one nice thing about the costume. If there are
several people, cover the group briefly. Never comment on anyone's body, age, or looks beyond
the costume.

If you can see part of a person but not their face, say in one short, playful sentence what
you can see, then say that maybe you should back up and try again.

If there is no person in the photo at all, say in one short sentence what you do see.

Your words will be spoken aloud with piper text to speech, so use plain words only:
no emojis, lists, or symbols."""


class CostumeComplimentNode(Node):
    def __init__(self):
        super().__init__('costume_compliment')

        self.declare_parameter('photo_button_index', 13)  # D-pad Left
        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('model', 'gpt-4.1')
        self.declare_parameter('save_photos', False)

        self.photo_button_index = self.get_parameter('photo_button_index').value
        self.image_topic = self.get_parameter('image_topic').value

        self.bridge = CvBridge()
        self.previous_button_state = 0
        self.busy = False

        # Only subscribed to the camera while waiting for a photo, so frames
        # are not streamed to this node the rest of the time
        self.image_subscriber = None
        self.frame_timer = None

        self.say_publisher = self.create_publisher(String, '/speech/say', 10)

        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

        self.openai_client = None
        if OPENAI_AVAILABLE:
            api_key = os.environ.get('OPENAI_API_KEY')
            if api_key:
                self.openai_client = OpenAI(api_key=api_key, timeout=15.0, max_retries=1)
            else:
                self.get_logger().warn("OPENAI_API_KEY not found in environment")
        else:
            self.get_logger().warn("OpenAI not available - install with: pip install openai")

        self.get_logger().info(
            f"Costume compliments on button {self.photo_button_index}, images from {self.image_topic}")

    def say(self, text):
        """Send text to speech node"""
        self.get_logger().info(f"reply: {text}")
        self.say_publisher.publish(String(data=text))

    def joy_callback(self, joy_msg):
        """Take a photo on the rising edge of the photo button"""
        if self.photo_button_index >= len(joy_msg.buttons):
            return
        button_state = joy_msg.buttons[self.photo_button_index]
        pressed = button_state == 1 and self.previous_button_state == 0
        self.previous_button_state = button_state
        if not pressed:
            return

        # Ignore stale messages so a backlog does not trigger a photo
        msg_time = joy_msg.header.stamp.sec + joy_msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if msg_time + 1.0 < now:
            self.get_logger().info('Ignoring old message')
            return

        if self.busy:
            self.get_logger().info("Still looking at the last photo, ignoring press")
            return
        if self.openai_client is None:
            self.say("I can't look at pictures right now.")
            return

        self.busy = True
        self.say("Ooh, let me see your costume!")
        self.image_subscriber = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data)
        self.frame_timer = self.create_timer(FRAME_TIMEOUT_SEC, self.frame_timeout_callback)

    def _stop_waiting_for_frame(self):
        """Drop the camera subscription and timeout timer"""
        if self.image_subscriber is not None:
            self.destroy_subscription(self.image_subscriber)
            self.image_subscriber = None
        if self.frame_timer is not None:
            self.destroy_timer(self.frame_timer)
            self.frame_timer = None

    def image_callback(self, image_msg):
        """Use the first frame after the press, then call the API off the ROS thread"""
        if self.image_subscriber is None:
            return
        self._stop_waiting_for_frame()
        threading.Thread(target=self._compliment_photo, args=(image_msg,), daemon=True).start()

    def frame_timeout_callback(self):
        """No frame arrived in time, so the camera is not publishing"""
        self._stop_waiting_for_frame()
        self.get_logger().warn(f"No image on {self.image_topic} within {FRAME_TIMEOUT_SEC}s")
        self.say("My camera is not working right now.")
        self.busy = False

    def _compliment_photo(self, image_msg):
        """Send the photo to OpenAI and speak the compliment (runs in its own thread)"""
        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                raise RuntimeError("JPEG encoding failed")

            if self.get_parameter('save_photos').value:
                self.save_photo(jpeg)

            image_url = "data:image/jpeg;base64," + base64.b64encode(jpeg.tobytes()).decode('ascii')
            response = self.openai_client.chat.completions.create(
                model=self.get_parameter('model').value,
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": [
                        {"type": "text", "text": "Here is the photo I just took."},
                        {"type": "image_url", "image_url": {"url": image_url}},
                    ]},
                ],
                max_tokens=100,
                temperature=0.8
            )
            reply = (response.choices[0].message.content or "").strip()

            self.say(reply or "Hmm, I'm not sure what I'm looking at. Try again!")
        except Exception as e:
            self.get_logger().error(f"Costume compliment failed: {e}")
            self.say("Oops, my eyes are a little blurry. Try again!")
        finally:
            self.busy = False

    def save_photo(self, jpeg):
        """Save the photo to a file for debugging"""
        now = time.time()
        timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(now)) + f"_{int(now * 1000) % 1000:03d}"
        folder = "/root/ros2_ws/output/photos/"
        os.makedirs(folder, exist_ok=True)
        filename = os.path.join(folder, f"photo_{timestamp}.jpg")
        with open(filename, 'wb') as f:
            f.write(jpeg.tobytes())
        self.get_logger().info(f"Saved photo to {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = CostumeComplimentNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
