# Current Behavior

When the robot was unattended for a while, I got this error:

os2_roboclaw_driver_node-1] [ERROR] [1760645117.794697208] []: [RoboClaw::getUlongCommandResult2] Expected CRC of: 0xB74A, but got: 0x00
[ros2_roboclaw_driver_node-1] [ERROR] [1760645117.794942084] []: [RoboClaw::CmdReadMotorCurrents] Uncaught exception in send() !!!
[ros2_roboclaw_driver_node-1] [ERROR] [1760645117.795042919] []: [RoboClaw::Cmd::execute] Exception: [RoboClaw::getUlongCommandResult2] INVALID CRC, retry number: 0
[ros2_roboclaw_driver_node-1] [ERROR] [1760645117.795561115] []: [RoboClaw::getUlongCommandResult2] Expected CRC of: 0x6EF2, but got: 0x00
[ros2_roboclaw_driver_node-1] [ERROR] [1760645117.795752638] []: [RoboClaw::CmdReadMotorCurrents] Uncaught exception in send() !!!
[ros2_roboclaw_driver_node-1] [ERROR] [1760645117.795899585] []: [RoboClaw::Cmd::execute] Exception: [RoboClaw::getUlongCommandResult2] INVALID CRC, retry number: 1
[ros2_roboclaw_driver_node-1] [ERROR] [1760645117.796402633] []: [RoboClaw::getUlongCommandResult2] Expected CRC of: 0x6EF2, but got: 0x00
[ros2_roboclaw_driver_node-1] [ERROR] [1760645117.796535506] []: [RoboClaw::CmdReadMotorCurrents] Uncaught exception in send() !!!
[ros2_roboclaw_driver_node-1] [ERROR] [1760645117.796623379] []: [RoboClaw::Cmd::execute] Exception: [RoboClaw::getUlongCommandResult2] INVALID CRC, retry number: 2
[ros2_roboclaw_driver_node-1] [ERROR] [1760645117.796680880] []: [RoboClaw::Cmd::execute] RETRY COUNT EXCEEDED

The roboclaw ddriver then exited. When I relaunched the service everythi

# Desired behavior
The system should log a warning to the ros logs that it has had an error sending data the roboclaw device
The system should enter a "disconnected state"
The /roboclaw_status/error_state should be set to disconnected
The system should keep trying to run
When the system is able to send data again, the status should return to normal, and normal operations can resume

# How to verify
[ ] Run the system, manually unplug the serial and verify that the status changes, replug, and then verify that the robot becomes controllable again

# Plan

## Root Cause Analysis
The issue occurs when CRC errors during serial communication cause exceptions in `roboclaw_cmd.h::Cmd::execute()`. After 3 retries, a `TRoboClawException` with "RETRY COUNT EXCEEDED" is thrown. This exception is not caught in the `publisherThread()` loop in `motor_driver.cpp`, causing the thread to terminate and the driver to die.

## Implementation Plan

### 1. Add Connection State Management to RoboClaw class
**File: `src/ros2_roboclaw_driver/include/roboclaw.h`**
- Add enum for connection state:
  - `CONNECTED` - normal operation
  - `DISCONNECTED` - serial communication failed
- Add member variables:
  - `ConnectionState connection_state_`
  - `uint32_t consecutive_errors_` - counter for consecutive failures
  - `uint32_t error_threshold_` - how many errors before marking disconnected (default: 3)
  - `std::chrono::steady_clock::time_point last_successful_communication_`
- Add public methods:
  - `ConnectionState getConnectionState()`
  - `void setConnectionState(ConnectionState state, const char* reason)`
  - `void recordSuccessfulCommunication()`
  - `void recordFailedCommunication()`

### 2. Modify Command Execution to Handle Connection State
**File: `src/ros2_roboclaw_driver/include/roboclaw_cmd.h`**
- Modify `Cmd::execute()` to:
  - Catch `TRoboClawException` after max retries
  - Call `roboclaw_.recordFailedCommunication()` on exception
  - Call `roboclaw_.recordSuccessfulCommunication()` on success
  - If disconnected state detected, log warning but don't throw exception (allow graceful degradation)

### 3. Update RoboClaw Implementation
**File: `src/ros2_roboclaw_driver/src/roboclaw.cpp`**
- Initialize connection state variables in constructor
- Implement connection state management methods:
  - `recordSuccessfulCommunication()`: reset error counter, set CONNECTED if was disconnected
  - `recordFailedCommunication()`: increment counter, set DISCONNECTED if threshold exceeded
  - `setConnectionState()`: log state transitions with timestamps
- Modify `readSensorGroup()` to:
  - Wrap sensor reading commands in try-catch
  - On exception, transition to DISCONNECTED state
  - Continue running without crashing
  - Periodically retry connection when disconnected
- Update `getErrorString()` to append connection state to error string when disconnected

### 4. Add Connection State to Status Message
**File: `src/ros2_roboclaw_driver/msg/RoboClawStatus.msg`**
- Add field: `string connection_state`

### 5. Publish Connection State
**File: `src/ros2_roboclaw_driver/src/motor_driver_node.cpp`**
- In main loop, add:
  - `roboClawStatus.connection_state = <connection state as string>`
  - Ensure status continues publishing even when disconnected

### 6. Wrap Sensor Reading in Try-Catch
**File: `src/ros2_roboclaw_driver/src/motor_driver.cpp`**
- In `publisherThread()`, wrap `readSensorGroup()` call in try-catch:
  - Log warning on exception
  - Don't let exception terminate the thread
  - Continue loop to allow recovery

### 7. Handle Disconnected State in Command Velocity
**File: `src/ros2_roboclaw_driver/src/motor_driver.cpp`**
- In `cmdVelCallback()`:
  - Check connection state before sending commands
  - If DISCONNECTED, log warning and return early
  - Don't attempt to send motor commands when disconnected

### 8. Add Auto-Recovery Logic
**File: `src/ros2_roboclaw_driver/src/roboclaw.cpp`**
- In `readSensorGroup()` or `Cmd::execute()`:
  - When in DISCONNECTED state, periodically attempt simple commands (e.g., getVersion)
  - If successful, transition back to CONNECTED
  - Log recovery event



## Testing Plan
1. Build and deploy changes to robot
2. Start RoboClaw driver normally
3. Verify status shows CONNECTED
4. Physically disconnect serial cable
5. Verify:
   - Status transitions to DISCONNECTED within ~3 failed reads
   - Driver continues running (no crash)
   - Warning logs are generated
   - Status message continues publishing
6. Reconnect serial cable
7. Verify:
   - Status transitions back to CONNECTED
   - Robot becomes controllable again
   - Recovery is logged

## Files to Modify
1. `src/ros2_roboclaw_driver/include/roboclaw.h` - Add connection state enum and methods
2. `src/ros2_roboclaw_driver/src/roboclaw.cpp` - Implement connection state logic
3. `src/ros2_roboclaw_driver/include/roboclaw_cmd.h` - Update execute() error handling
4. `src/ros2_roboclaw_driver/src/motor_driver.cpp` - Add try-catch in publisherThread
5. `src/ros2_roboclaw_driver/msg/RoboClawStatus.msg` - Add connection_state field
6. `src/ros2_roboclaw_driver/src/motor_driver_node.cpp` - Publish connection state

## Implementation Complete
All changes have been implemented. The driver will now:
- Detect serial communication failures and transition to DISCONNECTED state
- Continue running without crashing when serial errors occur
- Automatically recover when communication is restored
- Publish connection state in the roboclaw_status topic
- Log all state transitions for debugging