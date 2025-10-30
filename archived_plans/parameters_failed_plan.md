# Plan: Fix Parameter Retrieval Failures in Speech Node

## Files to Change

**Primary file:**
- `/home/pi/red-crash/src/speech_ros/speech_ros/speech.py` - Main speech node implementation (all blocking operations fixed here)

**No changes needed to:**
- `/home/pi/red-crash/launch/speech.launch.py` - Launch file (no changes required)

## Problem Analysis

Foxglove Bridge fails to retrieve parameters from `/speech` node with error:
```
[ERROR] Failed to retrieve parameters from node '/speech': Failed to retrieve parameter names for node '/speech'
```

**Root Cause:** The speech node blocks the ROS2 message pump (main thread) during audio operations, preventing it from servicing parameter service requests.

### Blocking Operations Identified

1. **`say_callback` (lines 390-409)**
   - `stop_listening(wait_for_stop=True)` - blocks waiting for background thread
   - `adjust_for_ambient_noise(source, duration=1.0)` - blocks for 1 second
   - Microphone context manager operations - potential I/O blocking

2. **`start_ptt_listening` (lines 263-363)**
   - `stop_listening(wait_for_stop=True)` - blocks waiting for background thread
   - Entire PTT listening logic runs in a separate thread but still stops/starts background listening synchronously

3. **`setup_microphone` (lines 140-169)**
   - `adjust_for_ambient_noise(source, duration=2.0)` - blocks for 2 seconds on startup
   - Microphone device enumeration and initialization

4. **`play_audio`, `play_sound_file` (lines 172-188)**
   - `os.system()` calls block until audio playback completes

## Solution Strategy

Make all blocking operations non-blocking by:
1. Never block the main ROS2 spin thread
2. Use threading for all audio operations (listening, speaking, device management)
3. Use thread-safe state management
4. Keep ROS2 callbacks lightweight - just queue work for worker threads

## Implementation Plan

### Phase 1: Core Infrastructure (High Priority)

#### 1.1 Add Threading Infrastructure
- [ ] Add `import queue` and `import threading` 
- [ ] Create a `CommandQueue` using `queue.Queue()` for thread-safe message passing
- [ ] Create dedicated worker thread for audio operations in `__init__`
- [ ] Implement worker thread main loop that processes queue commands
- [ ] Add thread-safe state management with `threading.Lock()`

#### 1.2 Refactor `say_callback`
- [ ] Change to just enqueue a "say" command to the queue
- [ ] Return immediately without blocking
- [ ] Move all audio stopping/starting logic to worker thread
- [ ] Worker thread handles:
  - Stopping background listening
  - Text-to-speech generation and playback
  - Restarting background listening

#### 1.3 Refactor Audio Playback
- [ ] Move `play_audio()` execution to worker thread
- [ ] Move `play_sound_file()` execution to worker thread
- [ ] Ensure all `os.system()` calls run in worker thread, never main thread

### Phase 2: PTT Handling (High Priority)

#### 2.1 Refactor `joy_callback`
- [ ] Keep callback lightweight - just update state flags
- [ ] Enqueue PTT start/stop commands to worker thread
- [ ] Remove direct calls to `start_ptt_listening()`
- [ ] Let worker thread manage PTT state transitions

#### 2.2 Refactor `start_ptt_listening`
- [ ] Already runs in a thread, but stopping/starting background listening should be async
- [ ] Use queue to coordinate with main worker thread
- [ ] Avoid blocking waits for microphone availability

### Phase 3: Initialization (Medium Priority)

#### 3.1 Async Microphone Setup
- [ ] Move `adjust_for_ambient_noise()` calls to worker thread
- [ ] Make node ready immediately, show "initializing" state
- [ ] Background listening starts after async initialization completes
- [ ] Log when ready to accept commands

### Phase 4: State Management (Medium Priority)

#### 4.1 Thread-Safe State
- [ ] Wrap `self.state` access with locks
- [ ] Wrap `self.ptt_button_pressed` access with locks
- [ ] Use atomic flags for cross-thread communication
- [ ] Document state machine and threading model

### Phase 5: Testing & Validation (High Priority)

#### 5.1 Test Cases
- [ ] Verify Foxglove can retrieve parameters continuously during:
  - Text-to-speech playback
  - PTT button press/release cycles
  - Background listening
  - Audio playback operations
- [ ] Verify no missed joy messages during audio operations
- [ ] Verify speech recognition still works correctly
- [ ] Check for race conditions in state transitions

#### 5.2 Performance
- [ ] Ensure ROS2 callbacks return in < 1ms
- [ ] Monitor queue depth under load
- [ ] Check for memory leaks in worker threads

## Detailed Design

### Worker Thread Architecture

```
Main Thread (ROS2 Spin)           Worker Thread
------------------               ------------------
joy_callback()                   while running:
  -> queue.put(cmd)                cmd = queue.get()
  -> return immediately            if cmd == 'say':
                                     stop_listening()
say_callback()                       do_tts()
  -> queue.put(cmd)                  play_audio()
  -> return immediately              restart_listening()
                                   elif cmd == 'ptt_start':
listen_callback()                    handle_ptt_start()
  -> queue.put(cmd)                elif cmd == 'ptt_stop':
  -> return immediately              handle_ptt_stop()
```

### Command Queue Messages

```python
# Command format: (command_type, data_dict)
('say', {'text': 'hello world'})
('ptt_start', {})
('ptt_stop', {})
('utterance', {'audio': audio_data, 'source': 'wake_word'})
('play_sound', {'filepath': '/path/to/sound.wav'})
('shutdown', {})
```

## Benefits

1. **ROS2 callbacks never block** - always responsive to parameter queries
2. **Better separation of concerns** - audio operations isolated from ROS2
3. **More robust** - failures in audio don't crash ROS2 node
4. **Easier to test** - can test audio and ROS2 independently
5. **Better logging** - can see exactly when blocking operations occur

## Risks & Mitigations

**Risk:** Race conditions between threads
- *Mitigation:* Use locks for shared state, document threading model

**Risk:** Queue filling up if audio operations are slow
- *Mitigation:* Monitor queue depth, add max size with overflow handling

**Risk:** Complex to debug multi-threaded issues
- *Mitigation:* Add comprehensive logging with thread IDs

**Risk:** Background listening thread conflicts with worker thread
- *Mitigation:* Use event flags and careful coordination instead of blocking waits

## Success Criteria

1. Foxglove Bridge can retrieve parameters 100% of the time
2. All ROS2 callbacks return in < 1ms
3. No regression in speech recognition functionality
4. No race conditions or deadlocks in normal operation
5. Clean shutdown without hanging threads

## Implementation Notes

- Start with Phase 1 (infrastructure) and Phase 5.1 (validation) 
- Test after each phase before moving to next
- Use daemon threads where appropriate for clean shutdown
- Consider using `concurrent.futures.ThreadPoolExecutor` for cleaner thread management
- Document all threading assumptions and invariants
