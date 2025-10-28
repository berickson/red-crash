There is an issue with the microphone input.

It is a seeed studio respeaker directional microphone, I think it's version 2.0 or 3.0, but not sure.
It used to work fine, now it is recording audio at a very low level.

The code that reads the microphone is in /home/pi/red-crash/src/speech_ros/speech_ros/speech.py. There were previous attempts to fix it so there is special code for finding the microphone that I also think doesn't work.

The symptom is that the microphone is hearing something, but the input audio is at a very low level, barely above the background level.

I expect it might have something to do with gain.

The recording triggers, which is good, but the audio level is too low.

As part of debugging, I am outputting heard audio streams at /home/pi/red-crash/output/audio.

Thought include

1. look for obvious tweaks in the code
2. write a test program that plays something to the speaker while at the same time recording, maybe using text to speech (will need to stop spech service to test it). The test recording can be run through checks like level checks and trying to decode the speech with our recognizer to find the best tuning.

## Code Analysis Findings

### Issues Identified in speech.py

1. **Missing Recognizer Configuration**
   - No explicit `energy_threshold` setting (controls when speech detection triggers)
   - Using defaults which may not be optimal for ReSpeaker
   - Key parameters: `energy_threshold` (default ~300), `dynamic_energy_threshold`, `pause_threshold`

2. **Ambient Noise Adjustment May Be Too Conservative**
   - Only samples 1 second by default for calibration
   - After speaking, only adjusts for 0.5 seconds
   - If environment is noisy, may set threshold too high

3. **No Gain/Volume Configuration**
   - No microphone gain or volume multiplier set
   - Using PyAudio defaults which may be too low

4. **Device Finding Logic**
   - Searches for device each time but doesn't verify input capability

### Recommended Fixes

**Quick Fix #1: Add explicit recognizer tuning**
After creating the recognizer in `setup_microphone()`, add:
```python
self.recognizer.energy_threshold = 300  # Start with default, can adjust lower (100-200)
self.recognizer.dynamic_energy_threshold = True  # Keep auto-adjustment
self.recognizer.pause_threshold = 0.8
```

**Quick Fix #2: Extend ambient noise calibration**
Change duration to sample more background noise:
```python
self.recognizer.adjust_for_ambient_noise(source, duration=2.0)
```

**Quick Fix #3: Check ALSA capture volume**
Inside the `car` container, run:
```bash
amixer scontrols
amixer get Capture  # or whatever the capture control is named
```
If capture is low (< 70%), increase it:
```bash
amixer set Capture 90%
```

**Quick Fix #4: Add logging for debugging**
Log recognizer settings to see what threshold is being used:
```python
self.get_logger().info(f"Energy threshold: {self.recognizer.energy_threshold}")
self.get_logger().info(f"Dynamic threshold: {self.recognizer.dynamic_energy_threshold}")
```

### Test Program Plan

Create diagnostic script to:
1. List all audio devices and their capabilities
2. Record fixed duration with different settings
3. Calculate and display audio levels in real-time
4. Save recordings for comparison
5. Test with text-to-speech playback while recording

This will isolate whether it's hardware (ALSA/system) or software (PyAudio/speech_recognition) configuration.

## Test Script Created

A comprehensive test script has been created at `/home/pi/red-crash/scripts/test_microphone.py`.

### Usage

**Stop the speech service first to avoid conflicts:**
```bash
# Find and stop the speech service (in the car container)
docker exec car bash -c "screen -ls | grep speech"  # Find the screen session
docker exec car bash -c "screen -S <session> -X quit"  # Stop it
```

**Run the test script:**
```bash
# Inside the car container
docker exec -it car bash
cd /home/pi/red-crash
python3 scripts/test_microphone.py
```

**Available commands:**
- `python3 scripts/test_microphone.py` - Interactive menu
- `python3 scripts/test_microphone.py list` - List all audio devices
- `python3 scripts/test_microphone.py full` - Run full diagnostic suite (recommended)
- `python3 scripts/test_microphone.py record [gain]` - Quick recording test with optional gain
- `python3 scripts/test_microphone.py recognize [threshold]` - Speech recognition test with optional threshold

**What the full diagnostic does:**
1. Lists all audio devices
2. Finds ReSpeaker automatically
3. Records with 1x gain (baseline)
4. Records with 2x gain
5. Records with 4x gain
6. Tests speech recognition with default threshold (300)
7. Tests speech recognition with low threshold (100)
8. Tests recording while playing back speech

All recordings are saved to `/home/pi/red-crash/output/audio/test/` for analysis.

### Interpreting Results

- **Average level 1000-10000**: Good range for normal speech
- **Max level < 500**: Very low, increase ALSA volume or software gain
- **Max level < 1000**: Low, consider increasing gain
- **Max level > 15000**: Too high, may cause clipping

After running tests, you can apply the findings to the production code in `speech.py`.