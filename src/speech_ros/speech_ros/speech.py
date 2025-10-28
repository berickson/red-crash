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
    
    def find_respeaker_device(self):
        """Find the ReSpeaker microphone device index"""
        import pyaudio
        p = pyaudio.PyAudio()
        
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            name = info.get('name', '').lower()
            # ReSpeaker can show up as "respeaker", "seeed", "minidsp", or "vocalfusion"
            if any(keyword in name for keyword in ['respeaker', 'seeed', 'minidsp', 'vocalfusion']):
                self.get_logger().info(f"Found ReSpeaker device: {info.get('name')} at index {i}")
                p.terminate()
                return i
        
        p.terminate()
        return None
    
    def setup_microphone(self):
        """Initialize microphone and start background listening"""
        self.recognizer = sr.Recognizer()
        
        # Configure recognizer settings for better sensitivity
        self.recognizer.energy_threshold = 300  # Lower if needed (try 100-200 for quieter environments)
        self.recognizer.dynamic_energy_threshold = True  # Auto-adjust threshold
        self.recognizer.pause_threshold = 0.8  # Seconds of silence to consider end of phrase
        
        # Find ReSpeaker device
        device_index = self.find_respeaker_device()
        if device_index is not None:
            self.get_logger().info(f"Using ReSpeaker device at index {device_index}")
            # Use native 48kHz sample rate for better audio levels
            self.microphone = sr.Microphone(device_index=device_index, sample_rate=48000)
        else:
            self.get_logger().warn("ReSpeaker device not found, using default microphone")
            self.microphone = sr.Microphone(sample_rate=48000)
        
        # Set microphone to adjust sensitivity automatically as background noise changes
        with self.microphone as source:
             self.recognizer.adjust_for_ambient_noise(source, duration=2.0)
        
        # Log settings for debugging
        self.get_logger().info(f"Energy threshold: {self.recognizer.energy_threshold:.1f}")
        self.get_logger().info(f"Dynamic threshold enabled: {self.recognizer.dynamic_energy_threshold}")
        self.get_logger().info(f"Pause threshold: {self.recognizer.pause_threshold}s")
        
        self.stop_listening = self.recognizer.listen_in_background(
            self.microphone, 
            self.listen_callback, 
            phrase_time_limit=5.0)
        
        self.get_logger().info("Listening in the background")

    def play_audio(self, audio_data):
        self.get_logger().info("Playing audio")
        """Play audio data using system command"""
        with open('temp_audio.wav', 'wb') as f:
            f.write(audio_data.get_wav_data())
        
        speaker_volume_percent = self.get_parameter('speaker_volume_percent').value
        os.system(f"play --no-show-progress --volume {speaker_volume_percent / 100.0} temp_audio.wav 2>/dev/null")
        os.remove('temp_audio.wav')

    def save_utterance(self, audio_data):
        """Save the audio data to a file for debugging"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"/root/ros2_ws/output/audio/utterances/utterance_{timestamp}.wav"
        os.makedirs("/root/ros2_ws/output/audio/utterances/", exist_ok=True)
        with open(filename, 'wb') as f:
            f.write(audio_data.get_wav_data())
        self.get_logger().info(f"Saved utterance to {filename}")
    
    def listen_callback(self, recognizer, audio):
        """Called when audio is detected"""
        try:
            self.save_utterance(audio);
            # self.play_audio(audio)
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
            device_index = self.find_respeaker_device()
            if device_index is not None:
                self.microphone = sr.Microphone(device_index=device_index, sample_rate=48000)
            else:
                self.microphone = sr.Microphone(sample_rate=48000)
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
            self.get_logger().info(f"Restarting listening with threshold: {self.recognizer.energy_threshold:.1f}")
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
