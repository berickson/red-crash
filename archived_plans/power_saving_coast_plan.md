# Power-Saving Coast Plan

## Problem Statement
After driving the robot, even when cmd_vel is set to zero or no cmd_vel is sent, there is still non-trivial motor current. This is due to the RoboClaw PID controller actively maintaining position/velocity at zero, which continuously draws current to hold the motors in place.

## Goal
Automatically switch motors to coast mode (zero duty cycle) after a configurable period of zero velocity commands, reducing idle power consumption while maintaining responsive control during active operation.

## Current Behavior
- When `cmd_vel` with zero linear and angular velocity is received, `RoboClaw::stop()` is called
- `stop()` uses `CmdDoBufferedM1M2DriveSpeedAccelDistance` with all zeros
- This keeps the PID loop active, maintaining holding torque and drawing current

## Proposed Solution

### Usage Pattern
- **Active driving**: Joystick sends cmd_vel messages, motors respond to commands
- **Stop driving**: User releases enable button, cmd_vel messages stop arriving
- **Auto-coast**: After timeout with no cmd_vel and zero velocity, coast motors to save power
- **Manual wheel movement OK**: User may manually turn wheels while coasted
- **Resume driving**: New cmd_vel messages immediately take control again

### 1. Add Coast Command
Create a new RoboClaw command to set duty cycle to zero (coast mode):
- **File**: `src/ros2_roboclaw_driver/include/roboclaw_cmd_set_duty_cycle.h` (new)
- **Command**: Uses RoboClaw command `kMIXEDDUTY` (command 6)
- **Parameters**: m1_duty=0, m2_duty=0
- **Effect**: Releases motors to coast freely, minimal current draw

### 2. Add Coast Method to RoboClaw
- **File**: `src/ros2_roboclaw_driver/include/roboclaw.h`
- Add public method: `void coast();`
- **File**: `src/ros2_roboclaw_driver/src/roboclaw.cpp`
- Implement `coast()` to execute `CmdSetDutyCycle(0, 0)`

### 3. Add Timeout Logic Based on cmd_vel Reception
Track time since last cmd_vel message (not just non-zero velocity) and automatically coast after timeout.

#### New Parameter
- **Name**: `coast_timeout_seconds`
- **Type**: float
- **Range**: 0.0 to 60.0 seconds
- **Default**: 0.5 seconds (suggested - quick response when enable released)
- **Description**: "Time without any cmd_vel message before automatically coasting motors to save power. Set to 0.0 to disable."
- **Runtime changeable**: Yes

#### Implementation Details
- Add member variable: `float coast_timeout_seconds_`
- Add member variable: `rclcpp::Time last_cmd_vel_time_`
- Add member variable: `bool is_coasting_` (track coast state, **initialize to `true`** so motors start coasted)

#### Logic Flow
1. In `cmdVelCallback()`:
   - **Always** update `last_cmd_vel_time_` to now (regardless of velocity value)
   - Set `is_coasting_ = false` (any cmd_vel takes control back)
   - If velocity is non-zero: send velocity commands as normal
   - If velocity is zero: call `stop()` (active braking with PID)

2. In `mainLoopThread()` (renamed from `publisherThread()`):
   - After reading sensors, check if `coast_timeout_seconds_ > 0.0`
   - If not already coasting:
     - Calculate time since `last_cmd_vel_time_`
     - If time exceeds `coast_timeout_seconds_`: call `coast()` and set `is_coasting_ = true`
   - Once coasting, continue until next cmd_vel arrives

3. When any cmd_vel arrives (zero or non-zero):
   - Reset `is_coasting_ = false`
   - Motor control re-engages (either stop for zero, or velocity command for non-zero)

### Key Insight
The timeout is based on **cmd_vel message reception**, not the velocity value. This means:
- While joystick enable is pressed: cmd_vel arrives regularly → motors under active control
- When enable released: no cmd_vel → timeout expires → coast mode saves power
- User can manually turn wheels while coasted (no resistance)
- When enable pressed again: cmd_vel arrives → instantly back under control

### 4. Files to Modify

#### New Files
- `src/ros2_roboclaw_driver/include/roboclaw_cmd_set_duty_cycle.h`

#### Modified Files
- `src/ros2_roboclaw_driver/include/roboclaw.h` - add coast() method
- `src/ros2_roboclaw_driver/src/roboclaw.cpp` - implement coast()
- `src/ros2_roboclaw_driver/include/motor_driver.h` - add member variables, rename thread
- `src/ros2_roboclaw_driver/src/motor_driver.cpp` - rename publisherThread() to mainLoopThread(), add parameter, timeout logic

## Trade-offs

### Benefits
- Significantly reduced idle power consumption when joystick not in use
- Reduced motor/driver heating during stationary periods
- Extended battery life
- User-configurable timeout for different use cases
- Allows manual wheel movement without motor resistance
- Instant response when joystick re-engaged (any cmd_vel message takes control)

### Considerations
- Robot may drift on inclines after coasting (acceptable per user - not driving anymore)
- User may manually turn wheels while coasted (acceptable per user - may be desired)
- Timeout should be short (0.5s suggested) since it's based on message absence, not velocity
- Setting timeout to 0.0 disables feature (maintains current behavior)
- Works naturally with joystick deadman/enable button behavior

## Testing Plan

1. **Basic functionality**:
   - Send cmd_vel with non-zero velocity, verify motors respond
   - Send cmd_vel with zero velocity, verify motors stop with active braking
   - Stop sending cmd_vel, wait for timeout, verify motors coast (current drops to near-zero)
   - Manually turn wheels, verify they spin freely
   - Send any cmd_vel again, verify motors immediately take control

2. **Joystick integration**:
   - Drive with joystick enable pressed (continuous cmd_vel)
   - Release enable button (cmd_vel stops)
   - Verify motors coast after timeout
   - Press enable again, verify immediate control

3. **Parameter tuning**:
   - Test different timeout values (0.3s, 0.5s, 1.0s, 2.0s)
   - Measure current draw in each state: active, stopped, coasted
   - Verify runtime parameter changes work

4. **Edge cases**:
   - Timeout = 0.0 (feature disabled, always use stop/PID)
   - Very short timeout (0.1s)
   - Robot on incline (verify coasting behavior acceptable)
   - Rapid cmd_vel start/stop cycles

5. **Integration**:
   - Verify interaction with current protection system
   - Verify interaction with watchdog timeout (`max_seconds_uncommanded_travel`)
   - Verify behavior during recovery states
   - Verify coasting doesn't interfere with fault detection

## Implementation Checklist

- [x] Rename `publisherThread()` to `mainLoopThread()` in motor_driver.h and motor_driver.cpp
- [x] Update comments to clarify mainLoopThread responsibilities (sensor reading, monitoring, publishing)
- [x] Create `roboclaw_cmd_set_duty_cycle.h`
- [x] Add `coast()` method declaration to `roboclaw.h`
- [x] Implement `coast()` in `roboclaw.cpp`
- [x] Add `coast_timeout_seconds` parameter to `motor_driver.cpp`
- [x] Add tracking variables to `motor_driver.h` (last_cmd_vel_time_, is_coasting_)
- [x] Implement timeout logic in `cmdVelCallback()`
- [x] Implement coast trigger in `mainLoopThread()`
- [x] Add parameter change handling for runtime updates
- [x] Add explicit coast() call on startup (when coast_timeout_seconds > 0.0)
- [x] Test basic functionality
- [x] Test joystick integration
- [x] Test parameter tuning
- [x] Document in README.md

## Configuration Example

```yaml
# config/roboclaw_driver.yaml
coast_timeout_seconds: 0.5  # Coast after 0.5 seconds without cmd_vel (default - good for joystick)
# coast_timeout_seconds: 0.0  # Disable coasting (always hold position with PID)
# coast_timeout_seconds: 2.0  # Wait longer before coasting (for autonomous navigation with intermittent commands)
```

## Notes
- Coast mode disables PID holding torque completely
- This is different from braking, which actively resists motion
- Timeout based on **message reception**, not velocity value
- Perfect for joystick use: enable button controls whether motors are active
- Manual wheel turning is possible and expected while coasted
- Any cmd_vel message (even zero velocity) immediately re-engages motor control
- Interaction with `max_seconds_uncommanded_travel`: the watchdog is independent and should trigger first (if enabled) to stop motors before coasting kicks in
- Idle current on power supply was about .82 amps before this change
- `is_coasting_` starts as `true` so motors begin in low-power coast state until first cmd_vel arrives
- `mainLoopThread()` (formerly `publisherThread()`) is the main periodic loop that:
  - Reads sensors at `sensor_update_rate` (default 20 Hz)
  - Checks safety conditions (watchdog timeout, coast timeout)
  - Publishes status/odometry/joint states
  - Similar concept to Arduino's `loop()` function
