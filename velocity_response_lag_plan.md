# Velocity Response Lag Issue Plan

## Description

The robot exhibits stepped/plateau velocity response instead of smooth ramping when receiving cmd_vel commands. When a step input (square wave) is sent to cmd_vel, the robot's actual velocity (from odometry) shows:
1. Initial ramp to approximately 50% of target velocity
2. Plateau/hover at this intermediate velocity for ~0.5-1 second
3. Then continues ramping to the full target velocity, maybe with additional stepping

# Possible causes hypothesized so far:
Messages could be clogged or rejected
Acceleration or PID is being processed slowly
Old data is being sent


## Expected Behavior

- Smooth acceleration ramp from 0 to target velocity
- Ramp duration should match acceleration parameters (currently ~3.5 m/s²)
- No intermediate plateaus or stepping behavior

## Actions Taken So Far

1. **Fixed differential drive kinematics**: Removed incorrect division by `wheel_radius_` in velocity calculations
2. **Tried PID tuning**: Change values of P only tuning from 1 to 10,000.
3. **Inlined recovery logic**: Moved `notifyCmdVel()` functionality directly into `cmdVelCallback()`
4. **Fixed recovery abort**: Added missing `transitionState()` call when cmd_vel interrupts recovery
5. **Built and tested**: Confirmed issue persists after kinematic fix


## Debugging Plan
1. **Log commanded velocities**
Create

2. **Compare commanded vs actual velocities**
   ```cpp
   // Track last commanded velocity for comparison
   static int32_t last_cmd_m1 = 0, last_cmd_m2 = 0;
   int32_t velocity_error_m1 = m1_velocity_qpps - last_cmd_m1;
   RCUTILS_LOG_INFO("[Publisher] Velocity errors: M1_err=%d, M2_err=%d", velocity_error_m1, velocity_error_m2);
   ```

### Phase 3: Acceleration Parameter Testing
1. **Test with very high acceleration**: Set `accel_quad_pulses_per_second: 50000` to make ramp nearly instantaneous
2. **Test with velocity change detection**: Only send MIXEDSPEEDACCEL when velocity changes significantly
3. **Test cmd_vel rate limiting**: Reduce joy controller publish rate to 5 Hz

### Phase 4: Alternative Command Testing
1. **Research RoboClaw MIXEDSPEED command**: Test if non-accelerated speed command eliminates stepping
2. **Test position-hold mode**: Use MIXEDSPEEDACCELDIST with large distance to prevent ramp restarts
3. **Compare with direct serial commands**: Bypass ROS and test with raw RoboClaw commands

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