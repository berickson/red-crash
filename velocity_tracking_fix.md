# Velocity Tracking Issue - Root Cause & Fix

**Date:** October 22, 2025  
**Issue:** Robot wheels fail to reach commanded velocity setpoints

## Root Cause

**TWO issues identified:**

1. **Zero integral gain (I=0) in velocity PID controller** - FIXED
2. **Launch file overriding config file with incorrect PID values** - FIXED

### Critical Discovery

The `launch/roboclaw.launch.py` file was **hardcoding PID parameters**:
- Launch file: `m1_pid_p: 5000.0`, `m1_pid_i: 100.0`
- Config file edits were being **completely ignored**!

This meant your config file changes had **no effect** - the system was always using P=5000 (extremely high, causing instability).

### Evidence from Test Data

Running `test_velocity_response.py` showed consistent steady-state error proportional to commanded velocity:

| Commanded (SP) | Actual (PV) | Error | Error % |
|---------------|-------------|-------|---------|
| 1.3 m/s | ~0.9 m/s | ~0.4 m/s | 30% |
| 1.0 m/s | ~0.8 m/s | ~0.2 m/s | 20% |
| 0.5 m/s | ~0.47 m/s | ~0.03 m/s | 6% |
| 0.3 m/s | ~0.29 m/s | ~0.01 m/s | 3% |

### Why This Happens

**P-only control (I=0) cannot eliminate steady-state error under load:**
- Motor friction, back-EMF, and load create a constant disturbance
- Proportional control needs error to generate output
- Without integral term, there's always residual error proportional to load

**Current config** (`motor_driver.yaml`):
```yaml
m1_pid_p: 10.0
m1_pid_i: 0.0   # <-- PROBLEM: No integral action
m1_pid_d: 0.0
m1_pid_qpps_max: 2437
```

## Recommended Fix

### CRITICAL: Fix Launch File (REQUIRED FIRST)

The launch file must be updated to use the config file instead of hardcoded values.

**This has been fixed in** `launch/roboclaw.launch.py` - it now properly loads from config file.

### Step 1: Set Conservative PID Values ✓ DONE

Config file now set to:
```yaml
m1_pid_p: 10.0
m1_pid_i: 3.0    # Conservative integral gain
m1_pid_d: 0.0
m1_pid_qpps_max: 2437

m2_pid_p: 10.0
m2_pid_i: 3.0    # Conservative integral gain
m2_pid_d: 0.0
m2_pid_qpps: 2437
```

### Step 2: Rebuild and Restart

```bash
# Rebuild the package
docker exec -it car bash -c "cd /root/ros2_ws && colcon build --symlink-install --packages-select ros2_roboclaw_driver"

# Restart the driver node (screen will auto-restart)
docker exec -it car bash -c "pkill -f ros2_roboclaw_driver_node"

# Wait ~5 seconds for restart, then test
sleep 5
docker exec -it car bash -c "cd /root/ros2_ws && python3 scripts/test_velocity_response.py"
# Select test 4 (ramp test 0-0.9 m/s)
```

### Step 3: Evaluate and Measure Motor Limits

**Looking for:**
- What's the maximum PV velocity achieved?
- Is error <5% at low speeds (0.1-0.5 m/s)?
- Does error increase proportionally with speed?

**If maximum velocity is significantly less than 0.9 m/s:**
- Note the max velocity achieved (e.g., 0.65 m/s)
- Calculate actual QPPS: `max_vel * 1426` (quad_pulses_per_meter)
- Update `m1_pid_qpps_max` in config to this measured value

**If tracking is good below 0.5 m/s but poor above:**
- Likely hitting motor voltage/current limits
- Update `m1_pid_qpps_max` to measured maximum
- Reduce `max_linear_velocity` to realistic value

**If oscillation appears:**
- Add D term: `m1_pid_d: 0.5` to dampen

**Looking for:**
- Steady-state error reduced to <5%
- No excessive overshoot or oscillation
- Stable tracking at all velocities

**If error still >5%:** Increase I to 2.0, then 3.0, etc.  
**If oscillation appears:** Add D term (start with D=0.25)

## Tuning Guidelines

### Ziegler-Nichols Method (simplified)
1. Start: P=10.0, I=1.0, D=0.0
2. Increase I until error eliminated or oscillation starts
3. If oscillation: Add D ≈ I/4
4. Fine-tune P for response speed

### Expected Final Values (ballpark)
- P: 10-20 (adjust for response speed)
- I: 1-5 (eliminate steady-state error)
- D: 0-2 (damping if needed)

## Success Criteria

- [ ] Steady-state error <5% at all velocities (0.3-1.5 m/s)
- [ ] No sustained oscillation in velocity tracking
- [ ] Smooth acceleration ramps
- [ ] Stable tracking during turning maneuvers

## Additional Notes

### High-Frequency Oscillation (±0.05 m/s at 20 Hz)
Observed in test data - may be:
- Encoder quantization noise
- P-only control hunting
- Sensor timing jitter

**If persists after PID tuning:**
- Consider low-pass filter on velocity measurement
- Check encoder resolution and counting
- Adjust D term for damping

### Related Files
- Config: `/home/pi/red-crash/src/ros2_roboclaw_driver/config/motor_driver.yaml`
- Test script: `/home/pi/red-crash/scripts/test_velocity_response.py`
- Status message: Includes SP vs PV fields for monitoring
- Planning doc: `/home/pi/red-crash/velocity_response_lag_plan.md`
