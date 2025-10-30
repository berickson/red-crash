# Piper TTS Integration Plan

## Overview
Replace the current gTTS (Google Text-to-Speech) implementation in `speech.py` with Piper TTS - a fast, local neural text-to-speech system. Piper offers high-quality voices that run entirely offline without API calls.

## Current State
- Current TTS: gTTS (requires internet, slow, MP3 conversion)
- Speech recognition: Google Speech Recognition API
- Audio playback: sox (`play` command)
- Container: ROS2 Jazzy in Docker (`car` container)
- Models already downloaded:
  - `en_US-lessac-medium.onnx` and `.onnx.json`
  - `en_US-libritts_r-medium.onnx` and `.onnx.json`

## Target Implementation
Based on your friend's implementation at [alfiebot_ws](https://github.com/alansrobotlab2/alfiebot_ws/blob/main/src/alfie_tts/alfie_tts/speech.py):
- Use Piper Python library for TTS
- Voice: `en_US-libritts_r-medium.onnx` with speaker_id=65
- Organize models in a `voices/` directory for cleanliness
- Stream audio directly to sounddevice instead of file-based playback

## Benefits
1. **Offline operation** - No internet required
2. **Faster** - No network latency, direct audio generation
3. **Better quality** - Neural TTS sounds more natural than gTTS
4. **Consistent** - Same voice characteristics every time
5. **Lower latency** - Streaming audio chunks instead of generating full file first

## Implementation Steps

### 1. Install Dependencies in Docker Container
Update `/home/pi/red-crash/docker/Dockerfile`:

```dockerfile
# Add Piper TTS Python library
RUN python3 -m pip install --break-system-packages \
  piper-tts==1.2.0
```

### 2. Organize Voice Models
Create a `voices/` directory structure in the speech_ros package:

```
src/speech_ros/
  voices/
    en_US-lessac-medium.onnx
    en_US-lessac-medium.onnx.json
    en_US-libritts_r-medium.onnx
    en_US-libritts_r-medium.onnx.json
```

Update `src/speech_ros/setup.py` to include voice files as package data:
```python
data_files=[
    # ... existing data_files ...
    ('share/' + package_name + '/voices', glob('voices/*')),
],
```

### 3. Update speech.py Implementation

Replace the gTTS implementation with Piper TTS:

**Key changes:**
- Import Piper modules: `from piper.voice import PiperVoice` and `from piper.config import SynthesisConfig`
- Load voice model at initialization using package resource path
- Replace `_do_say()` method to use Piper streaming synthesis
- Use sounddevice for direct audio streaming (already available in container)
- Keep the worker thread architecture for non-blocking audio playback

**Voice configuration (matching your friend's setup):**
```python
self.model_path = os.path.join(voices_dir, 'en_US-libritts_r-medium.onnx')
self.config_path = os.path.join(voices_dir, 'en_US-libritts_r-medium.onnx.json')
self.speaker_id = 65  # Specific speaker from multi-speaker model
self.length_scale = None  # Can tune speed (1.0 = normal)
self.noise_scale = None   # Can tune variability
self.noise_w = None       # Can tune phoneme width
```

**Audio streaming approach:**
```python
# Initialize once in __init__
self.voice = PiperVoice.load(self.model_path, self.config_path, use_cuda=False)

# In _do_say() method
syn_config = SynthesisConfig(
    speaker_id=self.speaker_id,
    length_scale=self.length_scale,
    noise_scale=self.noise_scale,
    noise_w=self.noise_w,
)

with sd.RawOutputStream(
    samplerate=self.voice.config.sample_rate,
    channels=1,
    dtype='int16',
    latency=0.15,
) as output_stream:
    for audio_chunk in self.voice.synthesize(text, syn_config):
        output_stream.write(audio_chunk.audio_int16_bytes)
```

### 4. Add ROS2 Parameters for Voice Configuration
Add parameters to control TTS behavior:
- `speaker_id` (default: 65) - Which voice in the model to use
- `length_scale` (optional) - Speech speed adjustment
- `noise_scale` (optional) - Voice variability
- `noise_w` (optional) - Phoneme width variation

### 5. Remove gTTS Dependencies
From `speech.py`:
- Remove `from gtts import gTTS`
- Remove MP3 file generation code
- Keep sox for sound effects (beeps, etc.), remove for TTS

### 6. Testing Strategy
1. **Build the container**: Rebuild Docker image with new dependencies
2. **Test voice loading**: Verify models load correctly from package directory
3. **Test basic TTS**: Simple utterance like "Hello world"
4. **Test streaming**: Verify audio plays smoothly without gaps
5. **Test volume control**: Ensure `speaker_volume_percent` parameter works
6. **Test state transitions**: Verify background listening pauses during speech
7. **Test long utterances**: Check behavior with multi-sentence text
8. **Compare quality**: Side-by-side with old gTTS implementation

### 7. Migration Notes

**Advantages over current implementation:**
- Eliminates "uh" prefix hack (was: `gTTS("uh " + text)`)
- No temporary files (`out.mp3`) - cleaner
- Faster startup - no network calls
- More natural pauses between sentences
- Better pronunciation control if needed

**Potential issues to watch:**
- ALSA warnings already suppressed - keep that code
- Voice model files are ~50-70MB each (already downloaded)
- CPU usage higher than gTTS but still reasonable for Pi
- Ensure sounddevice backend is properly configured

### 8. Configuration File Updates
Update launch file if needed to expose new parameters:
```python
# In launch/speech.launch.py
parameters=[
    {'speaker_id': 65},
    {'speaker_volume_percent': 35.0},
    # ... other parameters ...
]
```

## Voice Model Information

**en_US-libritts_r-medium** (recommended, used by your friend):
- Multi-speaker model with 65+ speakers
- Medium quality (22.05 kHz sample rate)
- Good balance of quality vs speed
- Speaker 65 has a clear, pleasant voice

**en_US-lessac-medium** (alternative):
- Single speaker (Lessac trained)
- Medium quality
- Slightly faster than libritts_r
- More consistent voice (no speaker selection needed)

## Implementation Order
1. Update Dockerfile and rebuild container
2. Create voices directory and move model files
3. Update setup.py to include voice files
4. Modify speech.py to use Piper TTS
5. Add ROS2 parameters
6. Test in isolated environment
7. Integrate with full system
8. Update documentation

## Rollback Plan
If issues arise:
- Keep gTTS imports commented but available
- Add a `use_piper_tts` parameter to switch between implementations
- Fall back to gTTS if Piper fails to load

## Success Criteria
- [ ] TTS works offline without internet
- [ ] Voice quality is natural and clear
- [ ] Latency is improved vs gTTS
- [ ] No audio glitches or gaps
- [ ] Background listening properly pauses during speech
- [ ] Volume control works correctly
- [ ] System stability maintained

## Future Enhancements
- Add ability to switch voices via parameter
- Implement SSML support for intonation control
- Add emotion/style parameters if supported
- Consider GPU acceleration (use_cuda=True) if needed
- Pre-load multiple voices for variety

## References
- Piper TTS: https://github.com/rhasspy/piper
- Python API: https://github.com/rhasspy/piper/tree/master/src/python_run
- Your friend's implementation: https://github.com/alansrobotlab2/alfiebot_ws/blob/main/src/alfie_tts/alfie_tts/speech.py
- Voice samples: https://rhasspy.github.io/piper-samples/
- Voice downloads: https://huggingface.co/rhasspy/piper-voices/tree/v1.0.0

## Notes
- The Piper project was archived on Oct 6, 2025, but the code and models remain available
- Development continues at https://github.com/OHF-Voice/piper1-gpl
- Current implementation is stable and production-ready
- Models are ONNX format (optimized neural network inference)
