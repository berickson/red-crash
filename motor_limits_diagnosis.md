# Motor Speed Limit Diagnosis

**Date:** October 22, 2025  
**Issue:** High PID integral gain (I=100) still showing 15-30% velocity error

## Problem

With I=100 (very high integral gain), the system should eliminate steady-state error **IF** the motors can physically reach the commanded speed. The fact that error persists suggests we're hitting physical motor limits.

## Current Configuration

From `motor_driver.yaml`:
```yaml
m1_pid_p: 10.0
m1_pid_i: 100.0    # Very high - should eliminate SS error if possible
m1_pid_d: 0.0
m1_pid_qpps_max: 2437  # <-- This may be INCORRECT

# Robot geometry
quad_pulses_per_meter: 1426
wheel_radius: 0.06
```

## Test Results with I=100

From ramp test (0.1-1.4 m/s):
- Maximum achieved: ~1.02 m/s (M1), ~1.04 m/s (M2)
- At SP=0.9 m/s: PV ~0.72-0.74 m/s (18.5% error)
- At SP=1.0 m/s: PV ~0.78-0.83 m/s (19.6% error)
- Error increases with commanded speed (motor saturation)

## Root Cause Analysis

### Hypothesis: Incorrect `m1_pid_qpps_max` Setting

The `m1_pid_qpps_max` parameter tells RoboClaw the maximum speed your motors can achieve. If set too high:
- RoboClaw thinks motors can go faster than they actually can
- PID commands are scaled incorrectly
- Motor hits voltage limit before reaching setpoint
- Even high integral gain can't compensate (physically impossible)

### How m1_pid_qpps_max Works

RoboClaw scales velocity commands:
```
motor_command = (desired_speed_qpps / m1_pid_qpps_max) * max_duty_cycle
```

If `m1_pid_qpps_max = 2437` but motors max out at 1700 qpps:
- Commanding 2000 qpps → RoboClaw sends 82% duty cycle
- Motor actually produces 1700 qpps (100% duty at that speed)
- Result: Steady-state error of ~15%

## Diagnostic Steps

### Step 1: Measure Actual Maximum Speed

Run a full-throttle test to find real motor limits:

```bash
# Inside car container
docker exec -it car bash -c "cd /root/ros2_ws && python3 scripts/test_velocity_response.py"

# Run test 1 (step to 1.0 m/s)
# Watch the maximum PV values achieved
# Note the RoboClawStatus m1_current_speed_qpps and m2_current_speed_qpps
```

Monitor `/roboclaw_status` topic:
```bash
ros2 topic echo /roboclaw_status | grep current_speed_qpps
```

Look for:
- `m1_current_speed_qpps`: Actual motor speed in QPPS
- What's the maximum value when commanding high speeds?

### Step 2: Calculate Correct m1_pid_qpps_max

If maximum observed speed is:
- PV = 0.85 m/s at full throttle
- qpps = PV * quad_pulses_per_meter = 0.85 * 1426 = 1212 qpps

Then set:
```yaml
m1_pid_qpps_max: 1212  # Use measured maximum
m2_qpps: 1212  # Or measure separately
```

### Step 3: Alternatively - Use Auto-Tuning

BasicMicro Motion Studio can auto-tune to find correct QPPS:
1. Connect RoboClaw via USB to computer
2. Open Motion Studio
3. Run auto-tuning wizard
4. It will measure max speed and calculate QPPS

Reference: https://resources.basicmicro.com/auto-tuning-with-motion-studio/

## Quick Test Commands

```bash
# Check current motor speeds while commanding 1.0 m/s
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}}" &
ros2 topic echo /roboclaw_status --field m1_current_speed_qpps,m2_current_speed_qpps

# Kill the publisher
pkill -f "ros2 topic pub"
```

## Expected Fix

Once `m1_pid_qpps_max` is set to actual motor maximum:
- RoboClaw will scale commands correctly
- 100% duty cycle = m1_pid_qpps_max
- Motors can reach commanded speeds below m1_pid_qpps_max
- Integral term can eliminate steady-state error

Then reduce I term back to reasonable value (3-10) since error will be eliminated by correct scaling.

## Alternative: Check Launch File

I noticed `launch/roboclaw.launch.py` has different PID values:
```python
'm1_pid_p': 5000.0,
'm1_pid_i': 100.0,
```

But config file has:
```yaml
m1_pid_p: 10.0
m1_pid_i: 100.0   # (after your changes)
```

**Which one is actually running?** The launch file parameters override the config file!

Check if the launch file is being used and has different/conflicting settings.

## Next Steps

1. **Verify which config is active** (launch file vs yaml file)
2. **Measure actual max motor speed** from RoboClawStatus
3. **Calculate correct m1_pid_qpps_max** based on measurements
4. **Update config** with correct QPPS values
5. **Reduce I term** back to 3-5 once scaling is correct
