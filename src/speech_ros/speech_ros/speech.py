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
from sensor_msgs.msg import Joy


class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech')
        
        # Declare parameters
        self.declare_parameter('speaker_volume_percent', 35.0)
        self.declare_parameter('use_microphone', True)
        self.declare_parameter('enable_push_to_talk', True)
        self.declare_parameter('ptt_button_index', 12)
        self.declare_parameter('pause_threshold', 1.5)
        self.declare_parameter('phrase_time_limit', 30.0)
        
        self.use_microphone = self.get_parameter('use_microphone').value
        self.enable_push_to_talk = self.get_parameter('enable_push_to_talk').value
        self.ptt_button_index = self.get_parameter('ptt_button_index').value
        self.pause_threshold = self.get_parameter('pause_threshold').value
        self.phrase_time_limit = self.get_parameter('phrase_time_limit').value
        
        # Publisher for utterances
        self.speech_publisher = self.create_publisher(String, '/speech/utterances', 1)
        
        # Publisher for PTT utterances (bypass wake word)
        self.ptt_speech_publisher = self.create_publisher(String, '/speech/ptt_utterances', 1)
        
        # Subscriber for text-to-speech
        self.say_subscriber = self.create_subscription(
            String,
            'speech/say',
            self.say_callback,
            10)
        
        # Subscriber for joystick (push-to-talk)
        if self.enable_push_to_talk:
            self.joy_subscriber = self.create_subscription(
                Joy,
                'joy',
                self.joy_callback,
                10)
            self.get_logger().info(f"Push-to-talk enabled on button {self.ptt_button_index}")
        
        self.start_time = self.get_clock().now()
        self.stop_listening = None
        self.recognizer = None
        self.microphone = None
        
        # State machine for PTT
        self.state = 'idle'  # idle, listening, processing
        self.ptt_button_pressed = False
        
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
        self.recognizer.pause_threshold = self.pause_threshold  # Use configured pause threshold
        
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
        self.get_logger().info(f"Phrase time limit: {self.phrase_time_limit}s")
        
        # Only start background listening if PTT is disabled
        if not self.enable_push_to_talk:
            self.stop_listening = self.recognizer.listen_in_background(
                self.microphone, 
                self.listen_callback, 
                phrase_time_limit=self.phrase_time_limit)
            
            self.get_logger().info("Listening in the background (wake word mode)")
        else:
            self.get_logger().info("Push-to-talk mode active - waiting for button press")

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
    
    def play_sound_file(self, filepath):
        """Play a sound file using system command"""
        if os.path.exists(filepath):
            speaker_volume_percent = self.get_parameter('speaker_volume_percent').value
            os.system(f"play --no-show-progress --volume {speaker_volume_percent / 100.0} {filepath} 2>/dev/null")
        else:
            self.get_logger().warn(f"Sound file not found: {filepath}")
    
    def play_start_listening_sound(self):
        """Play sound when starting to listen"""
        self.play_sound_file("/root/ros2_ws/media/520579__divoljud__clickglass.wav")
    
    def play_thinking_sound(self):
        """Play sound when processing speech"""
        self.play_sound_file("/root/ros2_ws/media/826372__charonfaustinus__small-robot-thinking.wav")
    
    def play_error_sound(self):
        """Play sound when nothing was heard"""
        self.play_sound_file("/root/ros2_ws/media/523426__andersmmg__robot-beep.wav")
    
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
    
    def joy_callback(self, joy_msg):
        """Handle joystick button presses for push-to-talk"""
        if self.ptt_button_index < len(joy_msg.buttons):
            button_state = joy_msg.buttons[self.ptt_button_index]
            
            # Button pressed (transition from not pressed to pressed)
            if button_state == 1 and not self.ptt_button_pressed:
                self.ptt_button_pressed = True
                if self.state == 'idle':
                    self.get_logger().info("PTT button pressed - starting to listen")
                    self.state = 'listening'
                    self.play_start_listening_sound()
                    self.start_ptt_listening()
            
            # Button released (transition from pressed to not pressed)
            elif button_state == 0 and self.ptt_button_pressed:
                self.ptt_button_pressed = False
                if self.state == 'listening':
                    self.get_logger().info("PTT button released - stopping listening")
                    self.stop_ptt_listening()
    
    def start_ptt_listening(self):
        """Start listening for a single phrase in PTT mode"""
        if not self.recognizer or not self.microphone:
            self.get_logger().error("Microphone not initialized")
            return
        
        # Use a thread to listen for audio without blocking
        import threading
        def listen_thread():
            try:
                with self.microphone as source:
                    self.get_logger().info("Listening for speech...")
                    # Listen with the configured pause_threshold and phrase_time_limit
                    audio = self.recognizer.listen(source, 
                                                   timeout=self.phrase_time_limit,
                                                   phrase_time_limit=self.phrase_time_limit)
                
                # Process the audio
                if self.state == 'listening':
                    self.state = 'processing'
                    self.play_thinking_sound()
                    
                self.save_utterance(audio)
                
                try:
                    utterance = self.recognizer.recognize_google(audio)
                    self.get_logger().info(f'heard: "{utterance}"')
                    # Publish to PTT topic to bypass wake word check
                    self.ptt_speech_publisher.publish(String(data=utterance))
                except sr.UnknownValueError:
                    self.get_logger().info("no words detected")
                    self.play_error_sound()
                except sr.RequestError as e:
                    self.get_logger().error(f"Recognition error: {e}")
                    self.play_error_sound()
                
            except sr.WaitTimeoutError:
                self.get_logger().info("Listening timeout - no speech detected")
                self.play_error_sound()
            except Exception as e:
                self.get_logger().error(f"Error during listening: {e}")
                self.play_error_sound()
            finally:
                self.state = 'idle'
        
        thread = threading.Thread(target=listen_thread)
        thread.daemon = True
        thread.start()
    
    def stop_ptt_listening(self):
        """Stop listening in PTT mode (currently handled by timeout/silence detection)"""
        # The listening will naturally stop when silence is detected or timeout occurs
        # This method is here for future enhancements if needed
        pass
    
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
        
        # Only restart background listening if PTT is disabled
        if self.use_microphone and not self.enable_push_to_talk:
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
                phrase_time_limit=self.phrase_time_limit)
    
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
