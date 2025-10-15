# speech_ros

Speech recognition and text-to-speech package for ROS2.

## Features

- Speech recognition using Google Speech Recognition API
- Text-to-speech using Google Text-to-Speech (gTTS)
- Voice command interpretation with wake word detection
- Diagnostics monitoring and announcement

## Dependencies

### ROS2 Dependencies
- rclpy
- std_msgs
- diagnostic_msgs

### External Python Dependencies
- speech_recognition - Google Speech Recognition API
- gtts - Google Text-to-Speech
- pyaudio - Microphone input
- sox - Audio playback (system package)

## Installation

Install Python dependencies in the docker container:
```bash
docker exec car bash -c "pip3 install speech_recognition gtts pyaudio"
```

Install sox for audio playback (if not already installed):
```bash
docker exec car bash -c "apt-get update && apt-get install -y sox libsox-fmt-mp3"
```

## Usage

Launch both speech recognition and command interpreter:
```bash
ros2 launch launch/speech.launch.py
```

Or run nodes individually:
```bash
ros2 run speech_ros speech
ros2 run speech_ros command_interpreter
```

## Topics

### Published
- `/speech/utterances` (std_msgs/String) - Recognized speech text

### Subscribed
- `/speech/say` (std_msgs/String) - Text to speak
- `/diagnostics` (diagnostic_msgs/DiagnosticArray) - System diagnostics

## Parameters

- `speaker_volume_percent` (default: 35.0) - Speaker volume (0-100)
- `use_microphone` (default: true) - Enable microphone listening

## Voice Commands

Wake words: "hey robot", "he robot", "arabic", "hear about", "hero but", etc.

Supported commands:
- "what's your name" - Robot introduces itself
- "how old are you" / "when were you born" - Age/birth date
- "will you marry me" - Humorous responses
- "set volume to [number]" - Adjust speaker volume
- "how are you" / "status" - Status check

## Notes

- Requires microphone and speaker access
- Uses Google's free Speech Recognition API (no key required for basic usage)
- Optional: Place Google OAuth2.0 credentials in `secrets/secrets.json` for extended API access
