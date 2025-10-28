# Speech Experience Improvement Plan

## Current Issues

1. **Wake word misrecognition** - System hears "academia", "arabic", etc. instead of "hey robot"
2. **Premature timeout** - Stops listening before user finishes speaking (0.8s pause threshold)
3. **No push-to-talk option** - Would be useful with available remote control

## User Experience Workflows

### Mode 1: Speech-Activated (Wake Word)

**Standard Interaction:**
1. User says: "Hey robot" → Robot plays start sound (listening active)
2. User speaks command (pauses are OK, silence-based end detection)
3. Robot plays "thinking" sound → processes command
4. Robot speaks response
5. If nothing was heard/recognized, play error beep

**Follow-up Conversation:**
1. If robot asks a question → automatically stays in listening mode (no wake word needed)
2. User answers within timeout window (e.g., 10 seconds)
3. Same beep/done-listening feedback
4. Robot responds or returns to wake-word mode

**Key Features:**
- Wake word detection runs continuously in background
- Audio feedback confirms state changes
- Silence-based cutoff (configurable `pause_threshold`, default 1.5s)
- Max phrase limit as safety fallback (configurable `phrase_time_limit`, 30-60s)
- Follow-up mode after robot asks question (context-aware)

### Mode 2: Push-to-Talk

**Interaction:**
1. User presses and holds button → Robot plays start sound (listening active)
2. User speaks while holding button
3. User releases button OR silence detected → Robot plays "thinking" sound
4. Robot processes command
5. Robot speaks response
Note: There are no speech timeouts or wake words required. Speech is recorded and used while button is pressed. Same long phrase timeout applies

**Key Features:**
- Hold-to-talk mode (button down = listening)
- No wake word needed (button replaces it)
- Same audio feedback as speech-activated mode
- Silence detection still applies (don't need to hold for entire phrase)
- Can combine with follow-up mode

## Audio Feedback Sounds

- **Start listening:** `media/520579__divoljud__clickglass.wav`
- **Thinking/processing:** `media/826372__charonfaustinus__small-robot-thinking.wav`
- **Nothing heard:** `media/523426__andersmmg__robot-beep.wav`

## Configuration Parameters

```yaml
pause_threshold: 1.5          # seconds of silence before cutoff
phrase_time_limit: 30.0       # max phrase duration (safety fallback)
enable_push_to_talk: true     # enable PTT mode
ptt_button_index: 12          # joystick button for PTT
follow_up_timeout: 15.0       # seconds to wait for follow-up response (when implemented)
```

## Open Questions

1. **Wake word engine:** Evaluate options for dedicated wake word detection
   - Current approach: word list matching on speech recognition output
   - Alternative: Dedicated wake word engine (Porcupine, etc.)
   - Need to assess misrecognition rate after logging improvements

2. **Follow-up mode details:**
   - How long should follow-up window stay open? (10-15s?)
   - What triggers it? (robot asking question with "?" or explicit flag?)
   - How does user know they're in follow-up mode? (audio cue?)

## Implementation Plan


### Phase 1: Push-to-Talk
- [ ] Subscribe to joystick button topic
- [ ] Implement hold-to-talk mode (button down = listening)
- [ ] Use same audio feedback as speech-activated mode
- [ ] Add mode selection via ROS parameter (`enable_push_to_talk`)



### Phase 2: Core Feedback & Timing
- [ ] Add audio feedback system with three sounds (start, thinking, error)
- [ ] Implement configurable silence-based cutoff (`pause_threshold`)
- [ ] Add configurable `phrase_time_limit` as safety fallback (30-60s)
- [ ] Add state machine: idle → listening → processing → responding


### Phase 3: Wake Word Tuning
- [ ] Log all wake word attempts for analysis
- [ ] Evaluate dedicated wake word engine (if misrecognition remains high)
- [ ] Add confidence filtering
- [ ] Consider different wake words


### Phase 4: Follow-up Conversation
- [ ] Add follow-up window after robot asks question (configurable timeout)
- [ ] Skip wake word during follow-up period
- [ ] Use AI to determine if response is relevant follow-up or unrelated
- [ ] Timeout returns to normal wake-word mode
- [ ] Add audio cue when entering follow-up mode


## Files to Modify

- `src/speech_ros/speech_ros/speech.py` - State machine, audio feedback, PTT
- `src/speech_ros/speech_ros/command_interpreter.py` - Follow-up mode, wake word logging
- `launch/speech.launch.py` - Parameters
