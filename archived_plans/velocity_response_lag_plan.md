Status: Solved
Reason: PID P term was so high that it was causing problems




# Velocity Response Lag Issue Plan

## Description

The robot exhibits stepped/plateau velocity response instead of smooth ramping when receiving cmd_vel commands. When a step input (square wave) is sent to cmd_vel, the robot's actual velocity (from odometry) shows:
1. Initial ramp to approximately 50% of target velocity
2. Plateau/hover at this intermediate velocity for ~0.5-1 second
3. Then continues ramping to the full target velocity, maybe with additional stepping

# Analysis of Message Flow (Paper Trace)

## Message Path from cmd_vel to RoboClaw:

1. **cmd_vel callback** (motor_driver.cpp:368-419)
   - Receives geometry_msgs::Twist from /cmd_vel topic
   - Clips velocities to max_linear_velocity and max_angular_velocity
   - Checks connection state (returns early if DISCONNECTED)
   - Checks current protection state (stops motors if OVER_CURRENT_WARNING or RECOVERY_WAITING)
   - Calculates differential drive velocities for m1/m2
   - Converts m/s to quad pulses per second
   - Calls RoboClaw::doMixedSpeedAccel() immediately

2. **doMixedSpeedAccel** (roboclaw.cpp:115-120)
   - Creates CmdDoM1M2DriveSpeedAccel command object
   - Calls command.execute()

3. **Cmd::execute** (roboclaw_cmd.h:8-58)
   - Acquires mutex lock (buffered_command_mutex_)
   - Retries up to 3 times on exception
   - Calls send() while holding mutex
   - Releases mutex, then calls process()
   - Records success/failure for connection tracking

4. **CmdDoM1M2DriveSpeedAccel::send** (roboclaw_cmd_do_m1m2_drive_speed_accel.h:17-28)
   - Calls writeN2(true, 14, address, MIXEDSPEEDACCEL, accel, m1_speed, m2_speed)
   - MIXEDSPEEDACCEL = command 40 (0x28)
   - Sends: [address][cmd][accel_4bytes][m1_speed_4bytes][m2_speed_4bytes][crc_2bytes]
   - Expects ACK (0xFF) response from RoboClaw

5. **writeN2** (roboclaw.cpp:781-815)
   - Writes bytes via writeByte2()
   - Reads ACK byte via readByteWithTimeout2()
   - Timeout is 11ms per byte read
   - Throws exception if ACK != 0xFF

## Potential Stall Points:

### 1. Serial Communication Timing
- **readByteWithTimeout2()** uses poll() with 11ms timeout (roboclaw.cpp:468)
- Each failed read waits full 11ms before throwing exception
- With retries (up to 3 attempts), a failed command could take 33-50ms
- At 20Hz cmd_vel rate, commands arrive every 50ms
- Serial baud is 115200 (configured), actual byte time is ~87 microseconds

### 2. Mutex Contention
- **buffered_command_mutex_** serializes all RoboClaw commands
- During readSensorGroup() (called at 20Hz), 10+ commands are sent sequentially:
  - CmdReadMotorVelocityPIDQ (M1 and M2)
  - CmdReadLogicBatteryVoltage
  - CmdReadMainBatteryVoltage
  - CmdReadEncoder (M1 and M2)
  - CmdReadMotorCurrents
  - CmdReadEncoderSpeed (M1 and M2)
  - CmdReadTemperature
  - CmdReadStatus
- Each sensor read holds the mutex during send() phase
- If cmd_vel arrives during sensor reading, it must wait for mutex

### 3. RoboClaw Internal Processing
- **MIXEDSPEEDACCEL (cmd 40)** is NOT buffered (unlike MIXEDSPEEDACCELDIST cmd 46)
- Commands are processed immediately by RoboClaw firmware
- Each new command REPLACES the previous acceleration ramp
- If RoboClaw receives a command while still accelerating, it starts a NEW ramp from current velocity
- This could explain plateaus: robot reaches intermediate velocity, new command arrives, starts new ramp from that point

### 4. Command Rate vs Sensor Rate Interaction
- Sensor polling: 20Hz (every 50ms)
- cmd_vel publishing: varies by input source (joy typically 10-30Hz)
- If sensor read blocks cmd_vel for 10-20ms, timing jitter could cause:
  - Old cmd_vel processed after robot already reached intermediate velocity
  - Next cmd_vel delayed, causing apparent plateau

### 5. PID Loop Timing in RoboClaw
- RoboClaw runs internal PID loop at unknown rate (likely 100-500Hz)
- Current PID: P=10.0, I=0.0, D=0.0
- Low P value might cause slow velocity tracking
- Zero I means no integral wind-up, but also no steady-state error correction
- Each MIXEDSPEEDACCEL command provides new setpoint and acceleration rate

## Specific Hypotheses:

### Hypothesis A: Command Replacement During Ramp
**Mechanism:** Every 50ms (or at cmd_vel rate), a new MIXEDSPEEDACCEL command is sent. If the robot hasn't reached target velocity yet, the RoboClaw aborts the current ramp and starts fresh.

**Evidence for:**
- Line 413 comment: "Use immediate speed+accel command (no distance buffering)"
- MIXEDSPEEDACCEL is an "immediate" command (vs buffered commands)
- Explains why acceleration appears "stepped" - each command restarts the ramp

**Test:** Reduce cmd_vel publish rate to 5Hz (200ms between commands) to let ramps complete

### Hypothesis B: Sensor Read Blocking cmd_vel
**Mechanism:** During readSensorGroup() (takes ~20-50ms for 10 commands), incoming cmd_vel must wait. By the time cmd_vel processes, the robot's actual velocity has changed, but an old target is applied.

**Evidence for:**
- Mutex serialization of all commands
- Both sensor reads and cmd_vel use same mutex
- 20Hz sensor rate matches ~50ms timing

**Test:** Log timestamps of cmd_vel arrival vs actual command send time

### Hypothesis C: Serial Timeout Delays
**Mechanism:** Occasional CRC errors or timeouts cause retries, delaying subsequent commands. This creates irregular timing that disrupts smooth acceleration.

**Evidence for:**
- 3 retry attempts per command
- 11ms timeout per byte read
- Connection tracking shows consecutive_errors counter

**Test:** Enable do_debug to log all communication errors and timing

### Hypothesis D: Acceleration Parameter Too Low
**Mechanism:** accel_quad_pulses_per_second=5000 might be too slow, combined with frequent command restarts, creates apparent plateaus.

**Evidence for:**
- Config shows 5000 qpps², translates to ~3.5 m/s²
- Not particularly low, but if commands restart ramps frequently...

**Test:** Increase to 50000 (10x) to make ramps nearly instantaneous

## Recommended Investigation Steps:

1. **Run the velocity response test script (Robot on blocks!):**
   ```bash
   # From host or inside container
   cd /home/pi/red-crash  # or /root/ros2_ws on container
   python3 scripts/test_velocity_response.py
   ```
   This interactive test script:
   - Sends step inputs to cmd_vel
   - Monitors RoboClawStatus in real-time
   - Displays SP vs PV with timestamps
   - Shows velocity error (SP - PV)
   - Provides multiple test patterns (step, ramp, rotation)
   
   Watch for:
   - Initial ramp behavior
   - Plateaus where PV stops tracking SP
   - Time to reach target velocity
   - Velocity error trends

2. **Monitor SP vs PV in real-time using RoboClawStatus:**
   - New fields added to `/roboclaw_status` topic:
     - `m1_commanded_speed` (SP - setpoint in m/s)
     - `m1_current_speed` (PV - process value in m/s)
     - `m2_commanded_speed` (SP - setpoint in m/s)
     - `m2_current_speed` (PV - process value in m/s)
     - `m1_current_speed_qpps` (PV in quadrature pulses per second)
     - `m2_current_speed_qpps` (PV in quadrature pulses per second)
   - Use rqt_plot or Foxglove to visualize:
     ```bash
     ros2 topic echo /roboclaw_status
     # Or use rqt_plot:
     rqt_plot /roboclaw_status/m1_commanded_speed /roboclaw_status/m1_current_speed
     ```
   - This will show exactly when plateaus occur and how SP vs PV evolve

2. **Add timing logs to cmd_vel callback:**
   - Log when callback is entered
   - Log when doMixedSpeedAccel returns
   - Compare against actual velocity changes
   
3. **Add timing logs to readSensorGroup:**
   - Log start/end times
   - Measure actual duration
   - Check for mutex blocking patterns
   
4. **Enable protocol debugging:**
   - Set do_debug: true
   - Observe command/response patterns
   
5. **Test cmd_vel rate variation:**
   - Try 5Hz (slow enough for ramps to complete)
   - Try 50Hz (fast enough to approximate continuous control)
   
6. **Research RoboClaw MIXEDSPEED command:**
   - MIXEDSPEED (cmd 37) sends velocity without acceleration parameter
   - Might avoid ramp restarts
   - Need to understand if it respects acceleration limits set elsewhere

## Key Code Locations:

- cmdVelCallback: motor_driver.cpp:368-419
- doMixedSpeedAccel: roboclaw.cpp:115-120
- Cmd::execute: roboclaw_cmd.h:8-58
- readSensorGroup: roboclaw.cpp:506-596
- buffered_command_mutex_: roboclaw.cpp:38 (static mutex)

# Root Cause Identified (from test data):

**PRIMARY ISSUE: Zero integral gain in PID controller**

Test results show:
- Steady-state error proportional to commanded velocity (not ramp-up plateaus)
- At SP=1.3 m/s: PV ~0.9 m/s (30% error)
- At SP=1.0 m/s: PV ~0.8 m/s (20% error)  
- At SP=0.5 m/s: PV ~0.47 m/s (6% error)
- At SP=0.3 m/s: PV ~0.29 m/s (3% error)

Current PID: P=10.0, **I=0.0**, D=0.0

With I=0, the P-only controller cannot eliminate steady-state error under load (friction, back-EMF).

# Possible causes hypothesized earlier (RESOLVED):
1. ~~MIXEDSPEEDACCEL commands restart acceleration ramps~~ - Not the issue (no plateaus during ramp)
2. ~~Sensor reads block cmd_vel processing~~ - Timing is adequate
3. ~~Serial communication delays/retries~~ - Not causing velocity tracking issues
4. ~~Low acceleration parameter~~ - Acceleration is fine, tracking at steady-state is the problem


## Expected Behavior

- Motor velocity (PV) should track commanded velocity (SP) with minimal steady-state error (<5%)
- Smooth acceleration ramp matching acceleration parameter
- Stable tracking without high-frequency oscillation

## PID Tuning Plan

### Current State
- **P=10.0, I=0.0, D=0.0** (velocity PID)
- Config file: `/home/pi/red-crash/src/ros2_roboclaw_driver/config/motor_driver.yaml`
- Parameters: `velocity_pid_p_qpps`, `velocity_pid_i_qpps`, `velocity_pid_d_qpps`, `velocity_pid_qpps`

### Phase 1: Add Integral Term
The integral term accumulates error over time and drives it to zero at steady state.

1. **Start conservative**: Set I=1.0 (10% of P)
   ```yaml
   velocity_pid_p_qpps: 10.0
   velocity_pid_i_qpps: 1.0   # NEW - start conservative
   velocity_pid_d_qpps: 0.0
   velocity_pid_qpps: 11000
   ```

2. **Test with medium velocity** (0.5-1.0 m/s):
   ```bash
   docker exec -it car bash -c "cd /root/ros2_ws && python3 scripts/test_velocity_response.py"
   # Run test option 2 or 3 (medium step)
   ```

3. **Monitor for**:
   - Reduced steady-state error (should approach zero)
   - Potential overshoot or oscillation (if I is too high)
   - Integral windup during acceleration

### Phase 2: Increase I if Needed
If steady-state error is still >5%:

1. **Double I term**: I=2.0
2. **Retest**: Monitor tracking and overshoot
3. **Iterate**: Increase I until error <5% OR oscillation appears

### Phase 3: Add Derivative Term (if oscillation occurs)
If increasing I causes oscillation:

1. **Add D term**: D=0.5 (5% of P)
   - D term provides damping
   - Reduces overshoot and oscillation
   
2. **Balance I and D**: Classic rule-of-thumb: D ≈ I/4

### Phase 4: Fine-Tune P
Once I and D are working:

1. **Increase P** if response is too slow
2. **Decrease P** if there's overshoot or oscillation

### Expected Results After Tuning
- Steady-state error <5% at all velocities
- Smooth tracking without oscillation
- Rise time <0.5s for typical velocity changes

## Actions Taken So Far

1. **Fixed differential drive kinematics**: Removed incorrect division by `wheel_radius_` in velocity calculations
2. **Tried PID tuning**: Change values of P only tuning from 1 to 10,000.
3. **Inlined recovery logic**: Moved `notifyCmdVel()` functionality directly into `cmdVelCallback()`
4. **Fixed recovery abort**: Added missing `transitionState()` call when cmd_vel interrupts recovery
5. **Built and tested**: Confirmed issue persists after kinematic fix


## Debugging Plan

### Phase 1: Add Integral Gain (PRIMARY FIX)
```bash
# Edit config file to add I term
docker exec -it car bash -c "nano /root/ros2_ws/src/ros2_roboclaw_driver/config/motor_driver.yaml"

# Change:
#   velocity_pid_i_qpps: 0.0
# To:
#   velocity_pid_i_qpps: 1.0

# Rebuild and restart
docker exec -it car bash -c "cd /root/ros2_ws && colcon build --symlink-install --packages-select ros2_roboclaw_driver"
docker exec -it car bash -c "pkill -f ros2_roboclaw_driver_node"
# Wait for screen session to restart node automatically

# Test
docker exec -it car bash -c "cd /root/ros2_ws && python3 scripts/test_velocity_response.py"
```

### Phase 2: Iterative PID Tuning
Use Ziegler-Nichols or manual tuning process (see PID Tuning Plan above)

### Phase 3: Analyze High-Frequency Oscillation (if present after PID tuning)
If PV still shows high-frequency oscillation after tuning:

1. **Add encoder filtering**: Low-pass filter on velocity measurement
2. **Check encoder resolution**: Verify quadrature counting
3. **Adjust D term**: May help dampen oscillation



## Success Criteria

- Smooth velocity ramp with no intermediate plateaus
- Actual velocity closely follows expected acceleration curve
- Response time matches calculated acceleration parameters
- No velocity oscillations or overshoots

## Files to Monitor

- `/home/pi/red-crash/src/ros2_roboclaw_driver/src/motor_driver.cpp`
- `/home/pi/red-crash/src/ros2_roboclaw_driver/config/motor_driver.yaml`
- Joy controller output rate (`ros2 topic hz /cmd_vel`)
- Odometry output (`ros2 topic echo /odom/twist/twist/linear/x`)

## Next Steps

1. Implement Phase 1 logging
2. Collect data with step input cmd_vel commands
3. Analyze message timing and velocity change patterns
4. Adjust acceleration parameters or command strategy based on findings