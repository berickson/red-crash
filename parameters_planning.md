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

#### Parameter Categories
Group parameters logically:

1. **Device Connection** (Read-Only, require restart)
   - device_name
   - baud_rate  
   - device_port
   - serial_timeout

2. **Motor Control - PID** (Runtime changeable)
   - m1_p, m1_i, m1_d, m1_qpps
   - m2_p, m2_i, m2_d, m2_qpps

3. **Motor Control - Limits** (Runtime changeable)
   - m1_max_current (0.0 - 50.0 A)
   - m2_max_current (0.0 - 50.0 A)
   - max_angular_velocity (0.0 - 10.0 rad/s)
   - max_linear_velocity (0.0 - 5.0 m/s)
   - accel_quad_pulses_per_second (100 - 100000)

4. **Robot Geometry** (Read-Only, require restart)
   - wheel_radius (0.01 - 0.5 m)
   - wheel_separation (0.1 - 2.0 m)
   - quad_pulses_per_meter (100 - 10000)
   - quad_pulses_per_revolution (100 - 10000)

5. **Current Protection** (Runtime changeable)
   - current_filter_window_seconds (0.0 - 10.0 s)
   - recovery_timeout_seconds (0.0 - 60.0 s)

6. **Safety** (Runtime changeable)
   - max_seconds_uncommanded_travel (0.0 - 5.0 s)

7. **Publishing** (Read-Only, require restart)
   - publish_joint_states (bool)
   - publish_odom (bool)
   - roboclaw_status_topic (string)
   - sensor_update_rate (1.0 - 100.0 Hz)

8. **Debug** (Runtime changeable)
   - do_debug (bool)
   - do_low_level_debug (bool)

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
- Device connection (device_name, baud_rate, device_port)
- Robot geometry (wheel_radius, wheel_separation, pulses per meter/rev)
- Publishing configuration (topics, rates, enable/disable)
- Serial timeout (affects low-level communication)

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
- [ ] Add descriptions to all parameters
- [ ] Add constraints (ranges) to all numeric parameters
- [ ] Add proper ParameterDescriptor for each parameter
- [ ] Mark read-only parameters appropriately
- [ ] Test parameter descriptions with `ros2 param describe`

### Step 2: Implement Runtime Parameter Updates (Medium Priority)
- [ ] Create parameter callback that handles all runtime-changeable parameters
- [ ] Add validation for all parameter changes
- [ ] Apply changes to RoboClaw when parameters update
- [ ] Add logging for parameter changes
- [ ] Test runtime parameter changes work correctly

### Step 3: Organize and Document (Medium Priority)
- [ ] Group parameters by category in launch file
- [ ] Add comments with ranges and units
- [ ] Create parameter reference documentation
- [ ] Update README with parameter information

### Step 4: Testing and Validation (Before Release)
- [ ] Test with Foxglove Bridge (no errors)
- [ ] Test with `ros2 param` commands
- [ ] Test runtime parameter changes
- [ ] Test parameter constraints (reject invalid values)
- [ ] Test that read-only parameters can't be changed at runtime
- [ ] Verify all parameters show up in tools

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
- None expected - parameter names and defaults unchanged

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
