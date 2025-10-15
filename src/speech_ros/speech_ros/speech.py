#!/usr/bin/env python3

# NOTE: this example requires PyAudio because it uses the Microphone class

# Suppress ALSA warnings
from ctypes import *
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
try:
    asound = cdll.LoadLibrary('libasound.so.2')
    asound.snd_lib_error_set_handler(c_error_handler)
except:
    pass

import time
import speech_recognition as sr

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import os

from gtts import gTTS

from std_msgs.msg import String


class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech')
        
        # Declare parameters
        self.declare_parameter('speaker_volume_percent', 35.0)
        self.declare_parameter('use_microphone', True)
        
        self.use_microphone = self.get_parameter('use_microphone').value
        
        # Publisher for utterances
        self.speech_publisher = self.create_publisher(String, '/speech/utterances', 1)
        
        # Subscriber for text-to-speech
        self.say_subscriber = self.create_subscription(
            String,
            'speech/say',
            self.say_callback,
            10)
        
        self.start_time = self.get_clock().now()
        self.stop_listening = None
        self.recognizer = None
        self.microphone = None
        
        if self.use_microphone:
            self.setup_microphone()
    
    def setup_microphone(self):
        """Initialize microphone and start background listening"""
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Set microphone to adjust sensitivity automatically as background noise changes
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
        
        self.stop_listening = self.recognizer.listen_in_background(
            self.microphone, 
            self.listen_callback, 
            phrase_time_limit=5.0)
        
        self.get_logger().info("Listening in the background")
    
    def listen_callback(self, recognizer, audio):
        """Called when audio is detected"""
        try:
            # Use Google Speech Recognition API
            utterance = recognizer.recognize_google(audio)
            self.get_logger().info(f'heard: "{utterance}"')
            self.speech_publisher.publish(String(data=utterance))
        except sr.UnknownValueError:
            self.get_logger().info("no words detected")
        except sr.RequestError as e:
            self.get_logger().info(f"Could not request results from Google Speech Recognition service; {e}")
    
    def say(self, text):
        """Convert text to speech and play it"""
        self.get_logger().info(f'saying "{text}"')
        
        tts = gTTS("uh " + text)
        tts.save('out.mp3')
        self.get_logger().info('mp3 done')
        
        speaker_volume_percent = self.get_parameter('speaker_volume_percent').value
        os.system(f"play --no-show-progress --volume {speaker_volume_percent / 100.0} out.mp3 2>/dev/null")
        os.system("rm out.mp3")
    
    def say_callback(self, ros_string):
        """Handle incoming text-to-speech requests"""
        # Ignore messages from before we started
        elapsed = self.get_clock().now() - self.start_time
        if elapsed < Duration(seconds=1.0):
            self.get_logger().info("Ignoring old message")
            return
        
        if self.use_microphone and self.stop_listening:
            # Stop listening while speaking
            self.stop_listening(wait_for_stop=True)
        
        self.say(ros_string.data)
        
        if self.use_microphone:
            # Restart listening after speaking
            self.microphone = sr.Microphone()
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
            self.stop_listening = self.recognizer.listen_in_background(
                self.microphone, 
                self.listen_callback, 
                phrase_time_limit=5.0)
    
    def cleanup(self):
        """Clean up resources"""
        if self.stop_listening:
            self.stop_listening(wait_for_stop=False)


def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
    
    node.get_logger().info("Done")


if __name__ == '__main__':
    main()
