# RoboClaw Driver Parameter Architecture Planning

## Current Problems

### 1. Parameter Declaration Split Across Multiple Files
**Current State**:
- Some parameters declared in `MotorDriver::declareParameters()` (motor_driver.cpp)
- Other parameters declared in `motor_driver_node.cpp` main()
- Current protection parameters declared in both places
- No single source of truth

**Impact**:
- Difficult to maintain
- Easy to miss parameters when updating
- Inconsistent parameter handling
- Documentation scattered

### 2. Incomplete Parameter Metadata
**Issues**:
- Many parameters lack descriptions
- No constraints/ranges on most parameters
- No read-only flags where appropriate
- No parameter grouping or organization

## Proposed Solution

### Phase 1: Centralize Parameter Declarations

#### Create Parameter Declaration Structure
**File**: `src/ros2_roboclaw_driver/src/motor_driver_parameters.cpp`

```cpp
void declareMotorDriverParameters(rclcpp::Node::SharedPtr node) {
    // Device connection parameters
    declareParameter(node, "device_name", "/dev/roboclaw",
        "RoboClaw device path", 
        ParameterType::STRING,
        ReadOnly::NO);
    
    declareParameter(node, "baud_rate", 115200,
        "Serial baud rate for RoboClaw communication",
        ParameterType::INT,
        ReadOnly::YES,  // Requires restart
        IntRange{9600, 460800});
    
    // ... all other parameters
}
```

**Benefits**:
- Single location for all parameters
- Consistent format
- Easy to review and update
- Can generate documentation from this

### Phase 2: Add Comprehensive Parameter Metadata

#### Complete Parameter Reference

### 1. Device Connection

| Parameter | Type | Default | Range/Options | Runtime | Description | How to Measure |
|-----------|------|---------|---------------|---------|-------------|----------------|
| `device_name` | string | `/dev/roboclaw` | Valid device path | No | Serial device path for RoboClaw | Check `/dev/` for USB serial device |
| `baud_rate` | int | 115200 | 9600, 19200, 38400, 57600, 115200, 230400, 460800 | No | Serial baud rate (must match RoboClaw config) | Set in RoboClaw Motion Studio |
| `device_port` | int | 128 | 128-135 | No | RoboClaw device address on serial bus | Set in RoboClaw Motion Studio (default 128) |
| `serial_timeout` | float | 0.5 | 0.0-5.0 s | No | Timeout for serial communication (0.0=disabled) | Start at 0.5s, increase if timeouts occur |

### 2. Robot Geometry

| Parameter | Type | Default | Range | Runtime | Description | How to Measure |
|-----------|------|---------|-------|---------|-------------|----------------|
| `wheel_separation` | float | 0.345 | 0.1-2.0 m | No | Distance between left and right wheel contact points | Measure center-to-center of wheel contact patches |
| `meters_per_quad_pulse` | float | 0.000701 | 0.00001-0.01 | No | Linear distance traveled per encoder count | Drive robot known distance, divide by encoder counts |

**Note**: The old parameters `wheel_radius`, `quad_pulses_per_meter`, and `quad_pulses_per_revolution` will be **deprecated**. They are redundant since `meters_per_quad_pulse` directly expresses the calibration. Users measure this by:
1. Mark starting position on floor
2. Command robot to drive straight (e.g., 1 meter)
3. Measure actual distance traveled
4. Read encoder counts from both wheels, average them
5. `meters_per_quad_pulse = actual_distance / average_encoder_counts`

### 3. Motor Control - PID Tuning

| Parameter | Type | Default | Range | Runtime | Description |
|-----------|------|---------|-------|---------|-------------|
| `m1_p` | float | 5000.0 | 0.0-100000.0 | Yes | Motor 1 (left) PID proportional gain |
| `m1_i` | float | 0.0 | 0.0-100000.0 | Yes | Motor 1 (left) PID integral gain |
| `m1_d` | float | 0.0 | 0.0-100000.0 | Yes | Motor 1 (left) PID derivative gain |
| `m1_qpps` | int | 2437 | 0-1000000 | Yes | Motor 1 max speed in quad pulses/sec (for PID scaling, not a limit) |
| `m2_p` | float | 5000.0 | 0.0-100000.0 | Yes | Motor 2 (right) PID proportional gain |
| `m2_i` | float | 0.0 | 0.0-100000.0 | Yes | Motor 2 (right) PID integral gain |
| `m2_d` | float | 0.0 | 0.0-100000.0 | Yes | Motor 2 (right) PID derivative gain |
| `m2_qpps` | int | 2437 | 0-1000000 | Yes | Motor 2 max speed in quad pulses/sec (for PID scaling, not a limit) |

**Important**: `qpps` (quadrature pulses per second) is **NOT** a velocity limit - it's a PID tuning parameter that tells the RoboClaw's internal PID controller what the expected maximum speed is for proper gain scaling. Use `max_linear_velocity` to actually limit robot speed.

**How to Measure**:
- Run motors at full throttle and read encoder speed from RoboClaw
- Or use RoboClaw auto-tuning in Motion Studio to get this value

**PID Tuning Notes**:
- Start PID with P gain only, increase until oscillation, then back off 50%
- Add D gain to reduce oscillation
- Add I gain only if there's steady-state error

### 4. Motion Limits

| Parameter | Type | Default | Range | Runtime | Description |
|-----------|------|---------|-------|---------|-------------|
| `max_linear_velocity` | float | 2.0 | 0.0-10.0 m/s | Yes | Maximum forward/backward velocity |
| `max_angular_velocity` | float | 2.0 | 0.0-10.0 rad/s | Yes | Maximum rotation velocity |
| `max_linear_acceleration` | float | 2.0 | 0.1-20.0 m/s² | Yes | Maximum linear acceleration |
| `m1_max_current` | float | 10.0 | 0.0-50.0 A | Yes | Motor 1 current limit (trip threshold) |
| `m2_max_current` | float | 10.0 | 0.0-50.0 A | Yes | Motor 2 current limit (trip threshold) |

**Migration Note**: `accel_quad_pulses_per_second` will be **deprecated** in favor of `max_linear_acceleration` in m/s². The conversion:
```
max_linear_acceleration = accel_quad_pulses_per_second * meters_per_quad_pulse
```

### 5. Current Protection

| Parameter | Type | Default | Range | Runtime | Description |
|-----------|------|---------|-------|---------|-------------|
| `current_filter_window_seconds` | float | 1.0 | 0.0-10.0 s | Yes | Time window for averaging current readings. 0.0=instantaneous, >0.0=filtered |
| `recovery_timeout_seconds` | float | 5.0 | 0.0-60.0 s | Yes | Seconds of zero cmd_vel before auto-recovery from over-current. 0.0=no auto-recovery |

**Current Protection Behavior**:
- When motor current exceeds `m1_max_current` or `m2_max_current`, motors stop
- If `recovery_timeout_seconds > 0.0`, automatically recovers after timeout with zero velocity commands
- Filtering prevents nuisance trips from current spikes

### 6. Safety

| Parameter | Type | Default | Range | Runtime | Description |
|-----------|------|---------|-------|---------|-------------|
| `max_seconds_uncommanded_travel` | float | 0.01 | 0.0-5.0 s | Yes | Watchdog timeout - stops motors if no cmd_vel received |

**Safety Notes**:
- Very low values (0.01s) require high-rate cmd_vel publishers
- Typical values: 0.5-1.0s for teleop, 0.1s for autonomous
- Set to 0.0 to disable watchdog (not recommended)

### 7. Publishing & Topics

| Parameter | Type | Default | Options | Runtime | Description |
|-----------|------|---------|---------|---------|-------------|
| `publish_joint_states` | bool | false | true/false | No | Publish wheel joint states for robot_state_publisher |
| `publish_odom` | bool | true | true/false | No | Publish odometry on /odom topic |
| `roboclaw_status_topic` | string | `roboclaw_status` | Any valid topic | No | Topic name for RoboClaw status messages |
| `sensor_update_rate` | float | 20.0 | 1.0-100.0 Hz | No | Rate for reading encoders and publishing status |

### 8. Debug & Logging

**Note**: ROS2 has built-in logger level control that should be preferred over custom parameters.

**Current Custom Parameters** (consider deprecating):

| Parameter | Type | Default | Options | Runtime | Description |
|-----------|------|---------|---------|---------|-------------|
| `do_debug` | bool | false | true/false | Yes | Enable protocol-level debug logging (command packets) |
| `do_low_level_debug` | bool | false | true/false | Yes | Enable byte-level serial debug logging (every read/write) |

**Warning**: `do_low_level_debug=true` produces massive log output (every byte) and may impact performance.

**Current Logging Behavior**:
- `do_debug=false` → Only errors and important info (normal operation)
- `do_debug=true` → Logs command-level protocol transactions (read/write buffers per command)
- `do_low_level_debug=true` → Logs every individual byte sent/received on serial port

**Mapping to ROS2 Logger Levels**:

| Current Setting | ROS2 Equivalent | What Gets Logged |
|----------------|-----------------|------------------|
| `do_debug=false, do_low_level_debug=false` | `INFO` level (default) | Errors, warnings, startup parameters, important events |
| `do_debug=true, do_low_level_debug=false` | `DEBUG` level | + Command packets, protocol transactions, buffered hex data |
| `do_low_level_debug=true` | Custom or `DEBUG` with namespace | + Every serial byte (massive output, special case) |

**Recommended ROS2 Approach**:

```bash
# Set logger level at runtime (no restart needed)
ros2 run rqt_logger_level rqt_logger_level

# Or via command line
ros2 service call /roboclaw_node/set_logger_level rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{name: 'roboclaw_node', level: 10}]}"  # DEBUG=10, INFO=20, WARN=30, ERROR=40

# Or in launch file
ros2 run ros2_roboclaw_driver ros2_roboclaw_driver_node --ros-args --log-level debug
```

**Migration Plan**:
1. Replace command-level logging:
   ```cpp
   // OLD:
   if (do_debug_) {
       appendToWriteLog("ReadEncoder: encoder: %d, WROTE: ", encoder);
   }
   
   // NEW:
   RCLCPP_DEBUG(get_logger(), "ReadEncoder: encoder: %d, WROTE: [hex data]", encoder);
   ```

2. Replace byte-level logging with special logger:
   ```cpp
   // For extremely verbose byte-level logging, use a sub-logger
   auto serial_logger = get_logger().get_child("serial");
   RCLCPP_DEBUG(serial_logger, "Write: %02X", byte);
   // Users can control with: ros2 logger set roboclaw_node.serial debug
   ```

3. Benefits:
   - Standard ROS2 tooling works (rqt_logger_level, CLI)
   - No need for custom parameters
   - Can change at runtime without custom callback
   - Follows ROS2 conventions
   - Can control different log levels independently (e.g., protocol vs serial bytes)
   - Integration with launch files and rqt tools

#### Parameter Metadata Template
Each parameter should have:

```cpp
struct ParameterSpec {
    std::string name;
    std::string description;  // Tooltip/help text
    VariantType default_value;
    bool runtime_changeable;  // vs requires restart
    std::optional<Range> range;  // min/max/step
    std::optional<std::vector<std::string>> allowed_values;  // for enums
    std::optional<std::string> unit;  // "m", "rad/s", "A", etc.
    std::string category;  // For grouping
};
```

### Phase 3: Implement Parameter Validation and Callbacks

#### Runtime Parameter Updates
**Current Issues**:
- Only 2 parameters have callbacks (current protection)
- Other parameters can't be changed at runtime even if they should be
- No validation on parameter changes

**Solution**:
```cpp
rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
    
    for (const auto &param : parameters) {
        // Validate parameter
        if (!validateParameter(param)) {
            return failure("Parameter out of range");
        }
        
        // Apply parameter based on category
        if (isRuntimeChangeable(param.get_name())) {
            applyParameterChange(param);
        } else {
            return failure("Parameter requires node restart");
        }
    }
    
    return success();
}
```

#### Parameters That Should Be Runtime Changeable
- All PID values (for tuning)
- All limits (max current, velocity, acceleration)
- Current protection settings
- Safety timeouts
- Debug flags

#### Parameters That Require Restart
- Device connection (device_name, baud_rate, device_port, serial_timeout)
- Robot geometry (wheel_separation, meters_per_quad_pulse)
- Publishing configuration (publish_joint_states, publish_odom, roboclaw_status_topic, sensor_update_rate)

### Phase 4: Documentation and User Experience

#### Auto-Generated Parameter Documentation
Create script to generate markdown table from parameter definitions:

```markdown
## Parameters

### Device Connection

| Parameter | Type | Default | Range | Runtime | Description |
|-----------|------|---------|-------|---------|-------------|
| device_name | string | /dev/roboclaw | - | No | RoboClaw device path |
| baud_rate | int | 115200 | 9600-460800 | No | Serial baud rate |
| device_port | int | 128 | 128-135 | No | RoboClaw device address |

### Motor Control - Limits

| Parameter | Type | Default | Range | Runtime | Description |
|-----------|------|---------|-------|---------|-------------|
| m1_max_current | float | 25.0 | 0.0-50.0 A | Yes | Max allowed M1 current |
| m2_max_current | float | 25.0 | 0.0-50.0 A | Yes | Max allowed M2 current |
...
```

#### Enhanced Launch File
- Group parameters by category with comments
- Show ranges and units in comments
- Indicate which require restart

```python
# Device Connection (Requires Restart)
'device_name': '/dev/roboclaw',  # Device path
'baud_rate': 115200,             # 9600-460800 baud
'device_port': 128,              # 128-135, RoboClaw address

# Motor Control - Limits (Runtime Changeable)
'm1_max_current': 25.0,          # 0.0-50.0 A, Max M1 current
'm2_max_current': 25.0,          # 0.0-50.0 A, Max M2 current
```

#### Parameter Inspection Tools
Make it easy for users to see current values:
```bash
# List all parameters with descriptions
ros2 param describe roboclaw_node --all

# Get parameter with metadata
ros2 param describe roboclaw_node m1_max_current

# Parameter name: m1_max_current
#   Type: double
#   Description: Maximum allowed current for motor 1 (Amps)
#   Constraints:
#     Min value: 0.0
#     Max value: 50.0
#   Runtime changeable: Yes
#   Current value: 3.0
```

## Implementation Plan

### Step 1: Add Parameter Metadata (High Priority)
- [ ] Add descriptions to all 28 parameters
- [ ] Add constraints (ranges) to all numeric parameters
- [ ] Add proper ParameterDescriptor for each parameter with:
  - [ ] Description text
  - [ ] Read-only flag (for 10 restart-required params)
  - [ ] Floating point range constraints (IntegerRange/FloatingPointRange)
- [ ] Mark read-only parameters appropriately
- [ ] Test parameter descriptions with `ros2 param describe`

### Step 2: Implement SI Unit Conversions (High Priority)
- [ ] Add `meters_per_quad_pulse` parameter (replaces wheel_radius, quad_pulses_per_meter, quad_pulses_per_revolution)
- [ ] Add `max_linear_acceleration` parameter (replaces accel_quad_pulses_per_second)
- [ ] Update velocity/odometry calculations to use meters_per_quad_pulse
- [ ] Update acceleration commands to use max_linear_acceleration
- [ ] Keep m1_qpps/m2_qpps in encoder units (PID tuning parameter, not user-facing limit)
- [ ] Add deprecation warnings for old parameters

### Step 3: Implement Runtime Parameter Updates (High Priority)
- [ ] Create parameter callback that handles all 18 runtime-changeable parameters:
  - [ ] PID parameters (m1_p, m1_i, m1_d, m1_qpps, m2_p, m2_i, m2_d, m2_qpps)
  - [ ] Motion limits (max_linear_velocity, max_angular_velocity, max_linear_acceleration)
  - [ ] Current limits (m1_max_current, m2_max_current)
  - [ ] Current protection (current_filter_window_seconds, recovery_timeout_seconds)
  - [ ] Safety (max_seconds_uncommanded_travel)
  - [ ] Debug flags (do_debug, do_low_level_debug)
- [ ] Add validation for all parameter changes
- [ ] Apply changes to RoboClaw when PID parameters update
- [ ] Add logging for parameter changes
- [ ] Reject runtime changes to 10 restart-required parameters with clear error
- [ ] Test runtime parameter changes work correctly

### Step 4: Migrate to ROS2 Logger Levels (Optional/Future)
- [ ] Replace `if (do_debug_)` with `RCLCPP_DEBUG()` macros
- [ ] Replace `if (do_low_level_debug_)` with sub-logger pattern
- [ ] Consider deprecating do_debug and do_low_level_debug parameters
- [ ] Update documentation to recommend ROS2 logger level control

### Step 5: Organize and Document (Medium Priority)
- [ ] Update launch file with 8 parameter categories:
  1. Device Connection (4 params)
  2. Robot Geometry (2 params)
  3. Motor Control - PID Tuning (8 params)
  4. Motion Limits (5 params)
  5. Current Protection (2 params)
  6. Safety (1 param)
  7. Publishing & Topics (4 params)
  8. Debug & Logging (2 params)
- [ ] Add comments with ranges and units (SI units where applicable)
- [ ] Indicate which require restart vs runtime changeable
- [ ] Create parameter reference documentation from tables in this document
- [ ] Update README with parameter information and migration guide

### Step 6: Testing and Validation (Before Release)
- [ ] Test with Foxglove Bridge (no errors, all parameters visible)
- [ ] Test with `ros2 param describe` (all descriptions present)
- [ ] Test with `ros2 param list` (all 28 parameters listed)
- [ ] Test runtime parameter changes for all 18 runtime-changeable params
- [ ] Test parameter constraints (reject invalid values with clear messages)
- [ ] Test that 10 read-only parameters reject runtime changes with error
- [ ] Verify SI unit conversions are correct (meters_per_quad_pulse, max_linear_acceleration)
- [ ] Test that deprecated parameters still work but log warnings
- [ ] Verify all parameters show up in tools (rqt_reconfigure, Foxglove)

## Success Criteria

- [ ] All parameters have descriptions
- [ ] All numeric parameters have constraints
- [ ] Runtime-changeable parameters update correctly
- [ ] Read-only parameters reject runtime changes with clear error
- [ ] Parameter changes are logged
- [ ] Documentation is complete and accurate

## Migration Notes

### For Users
- New parameter descriptions and constraints available
- Some parameters can now be changed at runtime without restart

### Breaking Changes
**Parameters Deprecated** (old names still work with warnings):
- `wheel_radius` → Use `meters_per_quad_pulse` instead
- `quad_pulses_per_meter` → Use `meters_per_quad_pulse` instead  
- `quad_pulses_per_revolution` → Use `meters_per_quad_pulse` instead
- `accel_quad_pulses_per_second` → Use `max_linear_acceleration` instead

**No Breaking Changes**:
- All existing parameter names still work
- All existing parameter defaults unchanged
- Deprecated parameters will log warnings but continue to function

## Future Enhancements

### Parameter Presets
Allow loading parameter sets:
```bash
ros2 param load roboclaw_node aggressive_tuning.yaml
ros2 param load roboclaw_node conservative_tuning.yaml
```

### Parameter Validation Service
Create service to validate parameters before applying:
```bash
ros2 service call /roboclaw_node/validate_parameters roboclaw_msgs/srv/ValidateParameters
```

### Auto-Tuning Support
Parameters to enable auto-tuning modes:
- `auto_tune_pid: bool` - Enable PID auto-tuning
- `auto_tune_current_limits: bool` - Learn safe current limits

### Telemetry
Publish parameter changes as events:
- Topic: `/roboclaw_node/parameter_events`
- Useful for logging, debugging, and monitoring
