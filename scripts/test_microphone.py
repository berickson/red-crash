#!/usr/bin/env python3
"""
Microphone testing and calibration script for ReSpeaker.

This script helps diagnose and tune microphone input levels by:
1. Listing all available audio devices
2. Recording test samples with different configurations
3. Analyzing audio levels in real-time
4. Testing with text-to-speech playback while recording
5. Trying speech recognition with different settings

Run this while the speech service is stopped to avoid conflicts.
"""

import os
import sys
import time
import wave
import pyaudio
import numpy as np
import speech_recognition as sr
from gtts import gTTS

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

# Suppress Jack audio warnings
os.environ['JACK_NO_AUDIO_RESERVATION'] = '1'
os.environ['SDL_AUDIODRIVER'] = 'alsa'

# Suppress PyAudio/PortAudio warnings by redirecting stderr temporarily
import sys
import contextlib

@contextlib.contextmanager
def suppress_stderr():
    """Context manager to suppress stderr output"""
    old_stderr = sys.stderr
    sys.stderr = open(os.devnull, 'w')
    try:
        yield
    finally:
        sys.stderr.close()
        sys.stderr = old_stderr

import time

# Configuration
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000  # Note: ReSpeaker native rate is 48000, may need to try both
RECORD_SECONDS = 5
OUTPUT_DIR = "/root/ros2_ws/output/audio/test"

# Ensure output directory exists
os.makedirs(OUTPUT_DIR, exist_ok=True)


def list_devices():
    """List all available audio devices with detailed information."""
    with suppress_stderr():
        p = pyaudio.PyAudio()
    
    print("\n" + "="*70)
    print("AVAILABLE AUDIO DEVICES")
    print("="*70)
    
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        device_name = info['name']
        max_input = info['maxInputChannels']
        max_output = info['maxOutputChannels']
        default_rate = info['defaultSampleRate']
        
        marker = ""
        name_lower = device_name.lower()
        if any(keyword in name_lower for keyword in ['respeaker', 'seeed', 'minidsp', 'vocalfusion']):
            marker = " <-- ReSpeaker"
        
        print(f"\nDevice {i}: {device_name}{marker}")
        print(f"  Input channels: {max_input}")
        print(f"  Output channels: {max_output}")
        print(f"  Default sample rate: {default_rate}")
    
    with suppress_stderr():
        p.terminate()
    print("\n" + "="*70 + "\n")


def find_respeaker_device():
    """Find the ReSpeaker microphone device index."""
    with suppress_stderr():
        p = pyaudio.PyAudio()
    
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        name = info.get('name', '').lower()
        # ReSpeaker can show up as "respeaker", "seeed", "minidsp", or "vocalfusion"
        if any(keyword in name for keyword in ['respeaker', 'seeed', 'minidsp', 'vocalfusion']):
            with suppress_stderr():
                p.terminate()
            return i
    
    with suppress_stderr():
        p.terminate()
    return None


def test_record_raw(device_index=None, record_seconds=5, gain=1.0, test_name="default", sample_rate=None):
    """
    Record audio and analyze levels in real-time.
    
    Args:
        device_index: Audio device index (None for default)
        record_seconds: Duration to record
        gain: Software gain multiplier (1.0 = no change)
        test_name: Name for the output file
        sample_rate: Sample rate to use (None = use global RATE)
    """
    rate = sample_rate if sample_rate is not None else RATE
    
    with suppress_stderr():
        p = pyaudio.PyAudio()
    
    try:
        with suppress_stderr():
            stream = p.open(
                format=FORMAT,
                channels=CHANNELS,
                rate=rate,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=CHUNK
            )
    except Exception as e:
        print(f"Error opening audio stream: {e}")
        p.terminate()
        return None
    
    print(f"\n{'='*70}")
    print(f"RECORDING TEST: {test_name}")
    print(f"Device: {device_index if device_index is not None else 'default'}")
    print(f"Sample rate: {rate} Hz")
    print(f"Gain: {gain}x")
    print(f"Duration: {record_seconds} seconds")
    print(f"{'='*70}")
    print("\nRecording... Speak into the microphone!\n")
    
    frames = []
    max_level = 0
    avg_levels = []
    
    for i in range(0, int(rate / CHUNK * record_seconds)):
        try:
            data = stream.read(CHUNK, exception_on_overflow=False)
        except Exception as e:
            print(f"Error reading audio: {e}")
            break
            
        # Apply gain
        if gain != 1.0:
            audio_data = np.frombuffer(data, dtype=np.int16)
            audio_data = np.clip(audio_data * gain, -32768, 32767).astype(np.int16)
            data = audio_data.tobytes()
        
        frames.append(data)
        
        # Calculate level
        audio_data = np.frombuffer(data, dtype=np.int16)
        level = np.abs(audio_data).mean()
        max_level = max(max_level, level)
        avg_levels.append(level)
        
        # Print progress every ~0.3 seconds
        if i % 10 == 0:
            bar_length = int(level / 100)
            bar = '#' * min(bar_length, 50)
            print(f"Level: {level:6.1f} | Max: {max_level:6.1f} | {bar}")
    
    stream.stop_stream()
    stream.close()
    
    with suppress_stderr():
        p.terminate()
    
    # Save recording
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    output_path = f"{OUTPUT_DIR}/{test_name}_{timestamp}.wav"
    
    wf = wave.open(output_path, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(rate)
    wf.writeframes(b''.join(frames))
    wf.close()
    
    # Print results
    avg_level = np.mean(avg_levels)
    print(f"\n{'='*70}")
    print(f"RESULTS")
    print(f"{'='*70}")
    print(f"Average level: {avg_level:.1f}")
    print(f"Maximum level: {max_level:.1f}")
    print(f"Ideal range: 1000-10000 (for normal speech)")
    
    if max_level < 500:
        print("WARNING: Very low audio level! Check hardware volume or increase gain.")
    elif max_level < 1000:
        print("NOTE: Low audio level. Consider increasing gain or hardware volume.")
    elif max_level > 15000:
        print("NOTE: High audio level. May cause clipping.")
    else:
        print("Audio level looks reasonable.")
    
    print(f"\nSaved to: {output_path}")
    print(f"{'='*70}\n")
    
    return output_path


def test_speech_recognition(device_index=None, energy_threshold=300, dynamic_threshold=True):
    """
    Test speech recognition with specific settings.
    
    Args:
        device_index: Audio device index (None for default)
        energy_threshold: Energy threshold for speech detection
        dynamic_threshold: Enable dynamic threshold adjustment
    """
    print(f"\n{'='*70}")
    print(f"SPEECH RECOGNITION TEST")
    print(f"{'='*70}")
    print(f"Device: {device_index if device_index is not None else 'default'}")
    print(f"Energy threshold: {energy_threshold}")
    print(f"Dynamic threshold: {dynamic_threshold}")
    print(f"{'='*70}\n")
    
    recognizer = sr.Recognizer()
    recognizer.energy_threshold = energy_threshold
    recognizer.dynamic_energy_threshold = dynamic_threshold
    recognizer.pause_threshold = 0.8
    
    if device_index is not None:
        mic = sr.Microphone(device_index=device_index)
    else:
        mic = sr.Microphone()
    
    print("Adjusting for ambient noise... (please be quiet)")
    with mic as source:
        recognizer.adjust_for_ambient_noise(source, duration=2.0)
    
    print(f"Adjusted energy threshold: {recognizer.energy_threshold:.1f}")
    print("\nListening... Speak now!")
    
    try:
        with mic as source:
            audio = recognizer.listen(source, timeout=10, phrase_time_limit=5)
        
        # Save the recording
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        output_path = f"{OUTPUT_DIR}/speech_test_{timestamp}.wav"
        with open(output_path, 'wb') as f:
            f.write(audio.get_wav_data())
        print(f"\nSaved recording to: {output_path}")
        
        # Try to recognize
        print("Recognizing...")
        text = recognizer.recognize_google(audio)
        print(f"\nRecognized: \"{text}\"")
        print("\nSUCCESS!\n")
        return True
        
    except sr.WaitTimeoutError:
        print("\nNo speech detected (timeout)")
        return False
    except sr.UnknownValueError:
        print("\nCould not understand audio")
        return False
    except sr.RequestError as e:
        print(f"\nAPI error: {e}")
        return False


def test_with_playback(device_index=None, speaker_volume=0.35):
    """
    Test recording while playing back text-to-speech.
    This simulates the robot speaking and listening.
    
    Args:
        device_index: Audio device index for recording (None for default)
        speaker_volume: Speaker volume (0.0 to 1.0)
    """
    print(f"\n{'='*70}")
    print(f"PLAYBACK + RECORDING TEST")
    print(f"{'='*70}")
    print("This will play speech and record at the same time.")
    print("Stand near the microphone and speak after the beep.")
    print(f"{'='*70}\n")
    
    # Generate test speech
    test_text = "Testing microphone. Please say something now."
    print(f"Generating speech: \"{test_text}\"")
    tts = gTTS(test_text)
    tts_file = f"{OUTPUT_DIR}/test_tts.mp3"
    tts.save(tts_file)
    
    # Play and then record
    print("Playing announcement...")
    os.system(f"play --no-show-progress --volume {speaker_volume} {tts_file} 2>/dev/null")
    os.remove(tts_file)
    
    print("\nNow recording your response...")
    test_record_raw(device_index=device_index, record_seconds=5, test_name="with_playback", sample_rate=48000)


def run_full_diagnostic():
    """Run a complete diagnostic sequence."""
    print("\n" + "="*70)
    print("MICROPHONE DIAGNOSTIC AND CALIBRATION TOOL")
    print("="*70)
    print("\nThis will help diagnose microphone input level issues.")
    print("Make sure the speech service is stopped before running this.")
    print("\n" + "="*70)
    
    # Step 1: List devices
    list_devices()
    
    # Step 2: Find ReSpeaker
    device_index = find_respeaker_device()
    if device_index is not None:
        print(f"Found ReSpeaker device at index {device_index}")
    else:
        print("ReSpeaker device not found, using default device")
        device_index = None
    
    input("\nPress ENTER to start test 1: Basic recording (5 seconds)...")
    
    # Step 3: Basic recording test
    test_record_raw(device_index=device_index, record_seconds=5, gain=1.0, test_name="test1_basic_16k", sample_rate=16000)
    
    input("\nPress ENTER to start test 1b: Recording at native 48kHz rate...")
    
    # Step 3b: Test at native sample rate
    test_record_raw(device_index=device_index, record_seconds=5, gain=1.0, test_name="test1b_basic_48k", sample_rate=48000)
    
    input("\nPress ENTER to start test 2: Recording with 2x gain...")
    
    # Step 4: Test with gain
    test_record_raw(device_index=device_index, record_seconds=5, gain=2.0, test_name="test2_gain2x", sample_rate=48000)
    
    input("\nPress ENTER to start test 3: Recording with 4x gain...")
    
    # Step 5: Test with higher gain
    test_record_raw(device_index=device_index, record_seconds=5, gain=4.0, test_name="test3_gain4x", sample_rate=48000)
    
    input("\nPress ENTER to start test 4: Speech recognition with default settings...")
    
    # Step 6: Test speech recognition with default
    test_speech_recognition(device_index=device_index, energy_threshold=300, dynamic_threshold=True)
    
    input("\nPress ENTER to start test 5: Speech recognition with low threshold...")
    
    # Step 7: Test speech recognition with lower threshold
    test_speech_recognition(device_index=device_index, energy_threshold=100, dynamic_threshold=True)
    
    input("\nPress ENTER to start test 6: Playback + recording test...")
    
    # Step 8: Test with playback
    test_with_playback(device_index=device_index)
    
    print("\n" + "="*70)
    print("DIAGNOSTIC COMPLETE")
    print("="*70)
    print(f"\nAll test recordings saved to: {OUTPUT_DIR}")
    print("\nReview the results above and the saved audio files.")
    print("If levels are consistently low, check ALSA settings with:")
    print("  amixer scontrols")
    print("  amixer get Capture")
    print("\n" + "="*70 + "\n")


def main():
    """Main entry point with menu."""
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        
        if command == "list":
            list_devices()
        elif command == "full":
            run_full_diagnostic()
        elif command == "record":
            device_index = find_respeaker_device()
            gain = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
            test_record_raw(device_index=device_index, gain=gain, test_name=f"manual_gain{gain}", sample_rate=48000)
        elif command == "recognize":
            device_index = find_respeaker_device()
            threshold = int(sys.argv[2]) if len(sys.argv) > 2 else 300
            test_speech_recognition(device_index=device_index, energy_threshold=threshold)
        else:
            print("Unknown command. Use: list, full, record [gain], recognize [threshold]")
    else:
        # Interactive menu
        print("\n" + "="*70)
        print("MICROPHONE TEST MENU")
        print("="*70)
        print("\n1. List all audio devices")
        print("2. Run full diagnostic suite (recommended)")
        print("3. Quick recording test")
        print("4. Quick speech recognition test")
        print("5. Test after using speaker")
        print("6. Exit")
        print("\n" + "="*70)
        
        choice = input("\nEnter choice (1-6): ").strip()
        
        if choice == "1":
            list_devices()
        elif choice == "2":
            run_full_diagnostic()
        elif choice == "3":
            device_index = find_respeaker_device()
            test_record_raw(device_index=device_index, test_name="quick_test", sample_rate=48000)
        elif choice == "4":
            device_index = find_respeaker_device()
            test_speech_recognition(device_index=device_index)
        elif choice == "5":
            device_index = find_respeaker_device()
            test_with_playback(device_index=device_index)
        elif choice == "6":
            print("Exiting...")
        else:
            print("Invalid choice")


if __name__ == '__main__':
    main()
