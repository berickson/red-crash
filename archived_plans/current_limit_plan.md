# Current Limit Protection Enhancement Plan

## Current Implementation Problems

Based on code analysis of the RoboClaw driver, the current implementation has these issues:

1. **Instantaneous triggering**: Current limit check happens immediately on every sensor read (line 23-42 in `roboclaw_cmd_read_motor_currents.h`)
2. **Noisy signal**: Raw current readings can spike due to sensor noise
3. **Non-recoverable**: Once triggered, `roboclaw_.stop()` is called but there's no automatic recovery mechanism
4. **Immediate shutdown**: System stops completely when over-current detected

## Proposed Solution: "Slow Fuse" with Auto-Recovery

### 1. Time-Averaged Current Monitoring (Slow Fuse)

**Goal**: Require sustained high current before triggering, not just a momentary spike

**Implementation**:
- Add a circular buffer or exponential moving average for M1 and M2 currents
- Track average current over a configurable window (e.g., 1 second at ~20Hz = 20 samples)
- Only trigger alarm if average exceeds threshold
- Add configuration parameters:
  - `current_filter_window_seconds` (default: 1.0, range: 0.0 - 10.0)
    - **0.0 = disabled**: Uses instantaneous current reading (legacy behavior)
    - **> 0.0**: Uses averaged current over window
- **Dynamic configuration**: Parameters can be changed at runtime via ROS2 parameter updates
  - Buffer size will be recalculated based on window size and sensor update rate
  - When window size changes, preserve as much history as possible and recalculate average
  - Maximum buffer size capped at 10 seconds * sensor_rate to prevent excessive memory use

### 2. Alarm State Machine

**States**:
- `NORMAL`: No over-current condition
- `OVER_CURRENT_WARNING`: Average exceeds threshold, motors stopped
- `RECOVERY_WAITING`: cmd_vel is zero, waiting for recovery timeout
- `RECOVERING`: Attempting to resume operation

**Transitions**:
- `NORMAL` → `OVER_CURRENT_WARNING`: When averaged current exceeds limit
- `OVER_CURRENT_WARNING` → `RECOVERY_WAITING`: When cmd_vel == 0 detected (if recovery enabled)
- `OVER_CURRENT_WARNING` → stays in warning: If recovery_timeout == 0.0 (recovery disabled)
- `RECOVERY_WAITING` → `NORMAL`: After configured recovery time (e.g., 5 seconds) with continuous cmd_vel == 0
- `RECOVERY_WAITING` → `OVER_CURRENT_WARNING`: If cmd_vel ≠ 0 received (resets recovery timer)
- Any state → `OVER_CURRENT_WARNING`: If averaged current exceeds limit again

### 3. Required Code Changes

#### Add to `roboclaw.h`:
- Current history buffers for M1 and M2
- Recovery state tracking variables
- Configuration parameters for filter window and recovery timeout
- Method to check if recovery is allowed

#### Modify `roboclaw_cmd_read_motor_currents.h`:
- Replace instantaneous check with averaged check
- Implement state machine logic
- Add recovery mechanism

#### Modify `motor_driver.cpp`:
- Track last cmd_vel timestamp and values (both linear and angular)
- Notify RoboClaw on each cmd_vel message (to track if cmd_vel ≠ 0 during recovery)
- Check if cmd_vel has been zero for recovery period
- Communicate with RoboClaw about recovery state

#### Add to configuration (`motor_driver.yaml`):
- `current_filter_window_seconds: 1.0`  # Slow fuse trip time (0.0 = disabled, use instantaneous)
- `recovery_timeout_seconds: 5.0`  # Cool down after tripping current limit (0.0 = disabled, no auto-recovery)

**Note**: All parameters should be declared as runtime-configurable ROS2 parameters

#### ROS2 Parameter Descriptions (for tooltips):
```cpp
this->declare_parameter<float>("current_filter_window_seconds", 1.0, 
    "Time window for averaging motor current readings. 0.0=instantaneous (legacy), 0.1-10.0=averaged");

this->declare_parameter<float>("recovery_timeout_seconds", 5.0,
    "Seconds of zero cmd_vel required before auto-recovery from over-current. 0.0=disabled");
```

### 4. Implementation Steps

1. [ ] Add data structures for current averaging and state tracking
2. [ ] Implement moving average filter for current measurements with dynamic buffer sizing
3. [ ] Add parameter validation (enforce max window size of 10 seconds, allow 0.0 to disable)
4. [ ] Add ROS2 parameter callback for runtime reconfiguration of filter_window and recovery_timeout
5. [ ] Create alarm state machine in RoboClaw class
6. [ ] Add cmd_vel zero detection and recovery timer in MotorDriver
7. [ ] Update current checking logic to use averaged values when filter_window > 0, instantaneous when == 0
8. [ ] Add configuration parameters with appropriate ranges and descriptions
9. [ ] Add logging for state transitions and debugging
10. [ ] Handle buffer resize when filter_window parameter changes at runtime
11. [ ] Handle recovery_timeout == 0.0 case (stay in alarm state, no auto-recovery)

### 5. Safety Considerations

**Included in implementation:**
- **Log all state transitions for debugging**: Track when state changes occur and why
- **Recovery timer resets if cmd_vel ≠ 0**: Ensures full "cooling off" period with no motion commands before recovery. Prevents recovery attempts while problem conditions may still exist.

**Omitted (future enhancements):**
- ~~Keep instantaneous hard limit as emergency backup (e.g., 2x normal limit)~~ - Not needed for initial implementation
- ~~Consider ramped recovery (gradually increase speed limit after recovery)~~ - Future enhancement
- ~~Maximum recovery attempts before requiring manual intervention~~ - Not needed, can disable recovery with timeout=0.0

## Implementation Details

### Current Averaging Algorithm

**Option A: Simple Moving Average**
```
sum = 0
for each sample in buffer:
    sum += sample
average = sum / buffer_size
```

**Option B: Exponential Moving Average (EMA)**
```
alpha = 2 / (N + 1)  // N = window size in samples
EMA = alpha * current_sample + (1 - alpha) * previous_EMA
```

Recommendation: Start with simple moving average for clarity, switch to EMA if performance is an issue.

### State Machine Variables Needed

```cpp
enum OverCurrentState {
    NORMAL,
    OVER_CURRENT_WARNING,
    RECOVERY_WAITING,
    RECOVERING
};

struct CurrentProtection {
    // State
    OverCurrentState state;
    std::chrono::steady_clock::time_point alarm_triggered_time;
    std::chrono::steady_clock::time_point zero_cmd_vel_time;
    
    // Configuration (runtime adjustable)
    float filter_window_seconds;  // Validated: 0.0 - 10.0 (0.0 = disabled)
    float recovery_timeout_seconds;  // Validated: 0.0 - 60.0 (0.0 = disabled)
    float sensor_update_rate;  // From motor_driver, for buffer size calculation
    
    // Current averaging
    std::deque<float> m1_current_history;
    std::deque<float> m2_current_history;
    float m1_current_average;
    float m2_current_average;
    size_t max_buffer_size;  // Calculated from filter_window * sensor_rate
    
    // Method to update buffer size when parameters change
    void updateBufferSize() {
        max_buffer_size = std::min(
            (size_t)(filter_window_seconds * sensor_update_rate),
            (size_t)(10.0 * sensor_update_rate)  // Hard cap at 10 seconds
        );
        // Trim buffers if they exceed new size
        while (m1_current_history.size() > max_buffer_size) {
            m1_current_history.pop_front();
        }
        while (m2_current_history.size() > max_buffer_size) {
            m2_current_history.pop_front();
        }
    }
};
```

### Recovery Logic Pseudocode

```
On each current reading:
    1. If filter_window_seconds > 0:
        a. Add current to history buffer
        b. If buffer size exceeds max_buffer_size, remove oldest entry
        c. Calculate averaged current from buffer
    2. Else (filter_window_seconds == 0):
        a. Use instantaneous current reading (legacy behavior)
    3. Check state:
        
        if state == NORMAL:
            if averaged_current > threshold:
                state = OVER_CURRENT_WARNING
                stop motors
                record alarm_triggered_time
        
        elif state == OVER_CURRENT_WARNING:
            if recovery_timeout_seconds > 0 and cmd_vel == 0:
                state = RECOVERY_WAITING
                record zero_cmd_vel_time
            // else: stay in OVER_CURRENT_WARNING (recovery disabled)
        
        elif state == RECOVERY_WAITING:
            if cmd_vel != 0:
                state = OVER_CURRENT_WARNING  // Reset timer, user still commanding
            elif averaged_current > threshold:
                state = OVER_CURRENT_WARNING  // Still too high
            elif time_since(zero_cmd_vel_time) > recovery_timeout:
                state = NORMAL
                clear alarm flags
        
        elif state == RECOVERING:
            // Future: implement gradual speed increase
            state = NORMAL

On parameter update:
    1. Validate new parameters (0.0 <= window <= 10.0, 0.0 <= recovery <= 60.0)
    2. Update configuration values
    3. If filter_window_seconds > 0:
        a. Call updateBufferSize() to resize history buffers
        b. Recalculate current averages with new buffer contents
    4. Else (filter_window_seconds == 0):
        a. Clear history buffers (not needed)
        b. Switch to instantaneous checking mode
```

## Testing Plan

1. [ ] Test with normal operation - verify no false alarms
2. [ ] Test with simulated current spike - verify not triggered
3. [ ] Test with sustained over-current - verify alarm triggers
4. [ ] Test recovery with cmd_vel=0 for 5 seconds
5. [ ] Test that movement resumes after recovery
6. [ ] Test multiple recovery cycles
7. [ ] Test with filter_window=0.0 (legacy instantaneous mode)
8. [ ] Test with recovery_timeout=0.0 (no auto-recovery)
9. [ ] Test runtime parameter changes (adjust window and timeout while running)

## Configuration Tuning Guide

### Current Filter Window
- **0.0 seconds**: Disabled - uses instantaneous current (legacy behavior)
- **Too short (0.1-0.5)**: Still sensitive to noise spikes
- **Too long (> 3.0)**: Slow to respond to real over-current
- **Recommended**: 1.0 - 2.0 seconds for most applications
- **Hard limit**: 10.0 seconds maximum to prevent excessive memory usage
- **Can be adjusted at runtime** via ROS2 parameter update

### Recovery Timeout
- **0.0 seconds**: Disabled - no auto-recovery, requires manual intervention
- **Too short (1-3)**: May not give system time to cool down
- **Too long (> 15)**: User frustration waiting for recovery
- **Recommended**: 5 - 10 seconds for most applications
- **Can be adjusted at runtime** via ROS2 parameter update

### Current Limits (m1_max_current, m2_max_current)
- Adjust these directly in configuration to set the actual threshold
- Use averaged current check to avoid false alarms from noise
- These existing parameters remain unchanged
