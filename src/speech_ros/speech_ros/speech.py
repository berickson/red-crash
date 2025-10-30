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
import contextlib
import queue
import threading
import wave
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import os

from piper.voice import PiperVoice
from piper.config import SynthesisConfig
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from sensor_msgs.msg import Joy


# Context manager to suppress JACK audio warnings
@contextlib.contextmanager
def suppress_jack_warnings():
    """Suppress JACK audio server warnings by redirecting stderr at file descriptor level"""
    import sys
    stderr_fd = sys.stderr.fileno()
    old_stderr_fd = os.dup(stderr_fd)
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(devnull_fd, stderr_fd)
        yield
    finally:
        os.dup2(old_stderr_fd, stderr_fd)
        os.close(devnull_fd)
        os.close(old_stderr_fd)


class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech')

        # Declare parameters
        self.declare_parameter('speaker_volume_percent', 35.0)
        self.declare_parameter('use_microphone', True)
        self.declare_parameter('enable_push_to_talk', True)
        self.declare_parameter('enable_wake_word', False)
        self.declare_parameter('ptt_button_index', 12)
        self.declare_parameter('pause_threshold', 1.5)
        self.declare_parameter('phrase_time_limit', 30.0)
        self.declare_parameter('speaker_id', 65)
        self.declare_parameter('length_scale', 1.0)
        self.declare_parameter('noise_scale', 0.667)
        self.declare_parameter('noise_w_scale', 0.8)
        
        self.use_microphone = self.get_parameter('use_microphone').value
        self.enable_push_to_talk = self.get_parameter('enable_push_to_talk').value
        self.enable_wake_word = self.get_parameter('enable_wake_word').value
        self.ptt_button_index = self.get_parameter('ptt_button_index').value
        self.pause_threshold = self.get_parameter('pause_threshold').value
        self.phrase_time_limit = self.get_parameter('phrase_time_limit').value
        
        # Load Piper TTS voice model
        package_share_directory = get_package_share_directory('speech_ros')
        voices_dir = os.path.join(package_share_directory, 'voices')
        self.model_path = os.path.join(voices_dir, 'en_US-libritts_r-medium.onnx')
        self.config_path = os.path.join(voices_dir, 'en_US-libritts_r-medium.onnx.json')
        
        self.get_logger().info(f"Loading Piper TTS voice from {self.model_path}")
        try:
            self.voice = PiperVoice.load(self.model_path, self.config_path, use_cuda=False)
            self.get_logger().info("Piper TTS voice loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load Piper TTS voice: {e}")
            raise
        
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
        
        # log 
        
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
        
        # Task queue for worker thread (holds callables)
        # Audio operations run in worker thread to keep ROS2 callbacks responsive
        # This allows parameter queries and other ROS2 services to work during audio operations
        self.task_queue = queue.Queue()
        self.worker_running = True
        self.state_lock = threading.Lock()
        
        # State machine for dual-mode (PTT + wake word)
        # States: background_listening, ptt_listening, processing, speaking
        self.state = 'background_listening' if self.enable_wake_word else 'idle'
        self.ptt_button_pressed = False
        self.ptt_stop_flag = False  # Flag to stop PTT listening
        self.ptt_audio_frames = []  # Buffer for PTT audio
        
        # Start worker thread for audio operations
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()
        
        if self.use_microphone:
            # Queue microphone setup to run asynchronously (keeps node startup fast)
            self.task_queue.put(self._do_setup_microphone)
    
    def _worker_loop(self):
        """Main loop for worker thread - processes tasks
        
        All blocking audio operations run here to keep ROS2 callbacks responsive.
        This ensures parameter queries, service calls, etc. work during audio playback.
        """
        self.get_logger().info("Worker thread started")
        while self.worker_running:
            try:
                # Wait for tasks with a timeout to allow checking worker_running
                task = self.task_queue.get(timeout=0.1)
                
                if task is None:  # Shutdown signal
                    break
                
                # Task is a callable - just call it
                task()
                
                self.task_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error in worker thread: {e}")
        
        self.get_logger().info("Worker thread stopped")
    
    def _get_state(self):
        """Thread-safe state getter"""
        with self.state_lock:
            return self.state
    
    def _set_state(self, new_state):
        """Thread-safe state setter"""
        with self.state_lock:
            old_state = self.state
            self.state = new_state
            if old_state != new_state:
                self.get_logger().info(f"State transition: {old_state} -> {new_state}")

    
    def find_respeaker_device(self):
        """Find the ReSpeaker microphone device index"""
        import pyaudio
        
        # Suppress JACK warnings during PyAudio initialization
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
    
    def _do_setup_microphone(self):
        """Initialize microphone and start background listening (runs in worker thread)"""
        self.get_logger().info("Initializing microphone...")
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
            with suppress_jack_warnings():
                self.microphone = sr.Microphone(device_index=device_index, sample_rate=48000)
        else:
            self.get_logger().warn("ReSpeaker device not found, using default microphone")
            with suppress_jack_warnings():
                self.microphone = sr.Microphone(sample_rate=48000)
        
        # Set microphone to adjust sensitivity automatically as background noise changes
        with suppress_jack_warnings():
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=2.0)
        
        # Log settings for debugging
        self.get_logger().info(f"Energy threshold: {self.recognizer.energy_threshold:.1f}")
        self.get_logger().info(f"Dynamic threshold enabled: {self.recognizer.dynamic_energy_threshold}")
        self.get_logger().info(f"Pause threshold: {self.recognizer.pause_threshold}s")
        self.get_logger().info(f"Phrase time limit: {self.phrase_time_limit}s")
        
        # Start background listening if wake word is enabled
        if self.enable_wake_word:
            self.stop_listening = self.recognizer.listen_in_background(
                self.microphone, 
                self.listen_callback, 
                phrase_time_limit=self.phrase_time_limit)
            
            self._set_state('background_listening')
            self.get_logger().info("Background listening enabled for wake word detection")
        else:
            self._set_state('idle')
        
        if self.enable_push_to_talk:
            self.get_logger().info("Push-to-talk active - waiting for button press")
        
        if not self.enable_wake_word and not self.enable_push_to_talk:
            self.get_logger().warn("Neither wake word nor push-to-talk enabled!")
        
        self.get_logger().info("Microphone initialization complete")

    def _do_play_audio(self, audio_data):
        """Play audio data using system command (runs in worker thread)"""
        self.get_logger().info("Playing audio")
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
    
    def _do_play_sound_file(self, filepath):
        """Play a sound file using system command (runs in worker thread)"""
        if os.path.exists(filepath):
            speaker_volume_percent = self.get_parameter('speaker_volume_percent').value
            self.get_logger().info(f"Playing sound: {filepath} at volume {speaker_volume_percent}%")
            os.system(f"play --no-show-progress --volume {speaker_volume_percent / 100.0} {filepath} 2>/dev/null")
        else:
            self.get_logger().warn(f"Sound file not found: {filepath}")
    
    def play_start_listening_sound(self):
        """Play sound when starting to listen (queued to worker thread to avoid blocking)"""
        filepath = "/root/ros2_ws/media/826372__charonfaustinus__small-robot-thinking.wav"
        self.task_queue.put(lambda: self._do_play_sound_file(filepath))
    
    def play_thinking_sound(self):
        """Play sound when processing speech (queued to worker thread to avoid blocking)"""
        filepath = "/root/ros2_ws/media/826372__charonfaustinus__small-robot-thinking.wav"
        self.task_queue.put(lambda: self._do_play_sound_file(filepath))
    
    def play_error_sound(self):
        """Play sound when nothing was heard (queued to worker thread to avoid blocking)"""
        filepath = "/root/ros2_ws/media/523426__andersmmg__robot-beep.wav"
        self.task_queue.put(lambda: self._do_play_sound_file(filepath))
    
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
        """Handle joystick button presses for push-to-talk (non-blocking)
        
        Queues tasks to worker thread instead of blocking - keeps ROS2 responsive.
        """
        if self.ptt_button_index < len(joy_msg.buttons):
            button_state = joy_msg.buttons[self.ptt_button_index]
            if button_state != self.ptt_button_pressed:
                self.get_logger().info(f"PTT Button state: {button_state}") 
            else:
                return
            
            # Button pressed (transition from not pressed to pressed)
            if button_state == 1 and not self.ptt_button_pressed:
                self.ptt_button_pressed = True
                current_state = self._get_state()
                if current_state in ['idle', 'background_listening']:
                    self.get_logger().info("PTT button pressed - queueing start")
                    self.task_queue.put(self._do_ptt_start)
            
            # Button released (transition from pressed to not pressed)
            elif button_state == 0 and self.ptt_button_pressed:
                self.ptt_button_pressed = False
                current_state = self._get_state()
                if current_state == 'ptt_listening':
                    self.get_logger().info("PTT button released - queueing stop")
                    self.task_queue.put(self._do_ptt_stop)
    
    def _do_ptt_start(self):
        """Start PTT listening (runs in worker thread)"""
        self.get_logger().info("Starting PTT listening")
        
        # Stop background listening if it's running
        if self.enable_wake_word and self.stop_listening:
            self.get_logger().info("Pausing background listening for PTT")
            self.stop_listening(wait_for_stop=True)
            self.stop_listening = None
        
        self._set_state('ptt_listening')
        self._do_play_sound_file("/root/ros2_ws/media/826372__charonfaustinus__small-robot-thinking.wav")
        
        # Now start the PTT listening in a separate thread
        self._start_ptt_listening()
    
    def _do_ptt_stop(self):
        """Stop PTT listening (runs in worker thread)"""
        self.get_logger().info("Stopping PTT listening")
        self.ptt_stop_flag = True
    
    def _start_ptt_listening(self):
        """Start listening for a single phrase in PTT mode (spawns a thread)"""
        if not self.recognizer:
            self.get_logger().error("Recognizer not initialized")
            return
        
        # Reset stop flag and audio buffer
        self.ptt_stop_flag = False
        self.ptt_audio_frames = []
        
        # Use a thread to listen for audio without blocking
        import threading
        def listen_thread():
            try:
                # Create a new microphone instance for PTT to avoid conflicts
                device_index = self.find_respeaker_device()
                if device_index is not None:
                    with suppress_jack_warnings():
                        ptt_mic = sr.Microphone(device_index=device_index, sample_rate=48000)
                else:
                    with suppress_jack_warnings():
                        ptt_mic = sr.Microphone(sample_rate=48000)
                
                with ptt_mic as source:
                    self.get_logger().info("Listening for speech (hold button)...")
                    
                    # Continuously record while button is held
                    while not self.ptt_stop_flag and self._get_state() == 'ptt_listening':
                        # Read audio in small chunks
                        try:
                            audio_chunk = source.stream.read(source.CHUNK)
                            self.ptt_audio_frames.append(audio_chunk)
                        except Exception as e:
                            self.get_logger().error(f"Error reading audio: {e}")
                            break
                    
                    self.get_logger().info(f"Button released (flag={self.ptt_stop_flag}), processing audio...")
                    
                    # Convert the collected frames to AudioData
                    if self.ptt_audio_frames:
                        import io
                        import wave
                        audio_data = b''.join(self.ptt_audio_frames)
                        
                        # Create a WAV file in memory
                        wav_io = io.BytesIO()
                        with wave.open(wav_io, 'wb') as wav_file:
                            wav_file.setnchannels(1)
                            wav_file.setsampwidth(2)  # 16-bit
                            wav_file.setframerate(source.SAMPLE_RATE)
                            wav_file.writeframes(audio_data)
                        
                        wav_io.seek(0)
                        
                        # Create AudioData object
                        audio = sr.AudioData(audio_data, source.SAMPLE_RATE, 2)
                    else:
                        self.get_logger().warn("No audio recorded")
                        audio = None
                
                # Process the audio
                if audio and self._get_state() == 'ptt_listening':
                    self._set_state('processing')
                    self._do_play_sound_file("/root/ros2_ws/media/826372__charonfaustinus__small-robot-thinking.wav")
                    
                    self.save_utterance(audio)
                    
                    try:
                        utterance = self.recognizer.recognize_google(audio)
                        self.get_logger().info(f'heard: "{utterance}"')
                        # Publish to PTT topic to bypass wake word check
                        self.ptt_speech_publisher.publish(String(data=utterance))
                    except sr.UnknownValueError:
                        self.get_logger().info("no words detected")
                        self._do_play_sound_file("/root/ros2_ws/media/523426__andersmmg__robot-beep.wav")
                    except sr.RequestError as e:
                        self.get_logger().error(f"Recognition error: {e}")
                        self._do_play_sound_file("/root/ros2_ws/media/523426__andersmmg__robot-beep.wav")
                elif not audio:
                    self._do_play_sound_file("/root/ros2_ws/media/523426__andersmmg__robot-beep.wav")
                
            except Exception as e:
                self.get_logger().error(f"Error during listening: {e}")
                self._do_play_sound_file("/root/ros2_ws/media/523426__andersmmg__robot-beep.wav")
            finally:
                # Resume background listening if wake word is enabled
                if self.enable_wake_word:
                    # Give the PTT mic a moment to fully close
                    import time
                    time.sleep(0.2)
                    self.get_logger().info("Resuming background listening")
                    self.stop_listening = self.recognizer.listen_in_background(
                        self.microphone, 
                        self.listen_callback, 
                        phrase_time_limit=self.phrase_time_limit)
                    self._set_state('background_listening')
                else:
                    self._set_state('idle')
        
        thread = threading.Thread(target=listen_thread, daemon=True)
        thread.start()
    
    def _do_say(self, text):
        """Convert text to speech and play it (runs in worker thread)"""
        # Stop background listening if it's running
        if self.enable_wake_word and self.stop_listening:
            self.get_logger().info("Pausing background listening for speech")
            self.stop_listening(wait_for_stop=True)
            self.stop_listening = None
        
        self._set_state('speaking')
        
        speaker_volume_percent = self.get_parameter('speaker_volume_percent').value
        speaker_id = self.get_parameter('speaker_id').value
        length_scale = self.get_parameter('length_scale').value
        noise_scale = self.get_parameter('noise_scale').value
        noise_w_scale = self.get_parameter('noise_w_scale').value
        
        self.get_logger().info(f'saying "{text}" at {speaker_volume_percent}% (speaker_id={speaker_id})')
        
        try:
            # Create synthesis config
            syn_config = SynthesisConfig(
                speaker_id=speaker_id,
                length_scale=length_scale,
                noise_scale=noise_scale,
                noise_w_scale=noise_w_scale,
            )
            
            # Synthesize audio to memory
            audio_chunks = []
            for audio_chunk in self.voice.synthesize(text, syn_config):
                audio_chunks.append(audio_chunk.audio_int16_array)
            
            if not audio_chunks:
                self.get_logger().error("No audio generated")
                return
            
            # Concatenate all audio chunks
            full_audio = np.concatenate(audio_chunks)
            
            # Write to temporary WAV file
            wav_path = '/tmp/piper_tts_output.wav'
            with wave.open(wav_path, 'wb') as wav_file:
                wav_file.setnchannels(1)  # Mono
                wav_file.setsampwidth(2)  # 16-bit
                wav_file.setframerate(self.voice.config.sample_rate)
                wav_file.writeframes(full_audio.tobytes())
            
            # Play using sox (reliable method that we know works)
            os.system(f"play --no-show-progress --volume {speaker_volume_percent / 100.0} {wav_path} 2>/dev/null")
            os.remove(wav_path)
            
            self.get_logger().info('Speech synthesis complete')
            
        except Exception as e:
            self.get_logger().error(f"Error during speech synthesis: {e}")
        
        # Restart background listening if wake word is enabled
        if self.enable_wake_word:
            # Restart listening after speaking
            device_index = self.find_respeaker_device()
            if device_index is not None:
                with suppress_jack_warnings():
                    self.microphone = sr.Microphone(device_index=device_index, sample_rate=48000)
            else:
                with suppress_jack_warnings():
                    self.microphone = sr.Microphone(sample_rate=48000)
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
            self.get_logger().info(f"Restarting listening with threshold: {self.recognizer.energy_threshold:.1f}")
            self.stop_listening = self.recognizer.listen_in_background(
                self.microphone, 
                self.listen_callback, 
                phrase_time_limit=self.phrase_time_limit)
            self._set_state('background_listening')
        else:
            self._set_state('idle')
    
    def say_callback(self, ros_string):
        """Handle incoming text-to-speech requests (non-blocking)
        
        Queues TTS to worker thread instead of blocking - keeps ROS2 responsive.
        """
        # Ignore messages from before we started
        elapsed = self.get_clock().now() - self.start_time
        if elapsed < Duration(seconds=1.0):
            self.get_logger().info("Ignoring old message")
            return
        
        # Queue the speech task - pass the string directly
        self.get_logger().info(f"Queueing TTS: {ros_string.data}")
        self.task_queue.put(lambda: self._do_say(ros_string.data))
    
    def cleanup(self):
        """Clean up resources"""
        self.get_logger().info("Cleaning up speech node")
        
        # Stop worker thread
        self.worker_running = False
        self.task_queue.put(None)  # Shutdown signal
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2.0)
        
        # Stop background listening
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
