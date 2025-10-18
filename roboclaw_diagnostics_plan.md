# RoboClaw Diagnostics Plan

## Overview
Implement standard ROS2 diagnostics system to monitor the health of the RoboClaw motor controller device.

## Goals
- Monitor RoboClaw device health in real-time
- Detect communication errors and hardware issues
- Provide visibility into device status through standard ROS2 diagnostics tools
- Enable automated monitoring and alerting
- Compatible with Foxglove, R

## ROS2 Diagnostics System

### Components
- Use `diagnostic_updater` package
- Publish to `/diagnostics` topic (type: `diagnostic_msgs/DiagnosticArray`)
- Integration with standard ROS2 diagnostic tools (rqt_robot_monitor, etc.)

## Built-in RoboClaw Diagnostics (Already Available)

The RoboClaw device provides comprehensive built-in diagnostics via the **GETERROR** command (command 90), which returns a 32-bit status word. The driver already reads these in `readSensorGroup()`:

### Status Bits Decoded by Device (Bits 0-31)
**DO NOT RE-CALCULATE** - these are monitored by the RoboClaw hardware:
- Bit 0: M1 OverCurrent Warning (hardware monitored)
- Bit 1: M2 OverCurrent Warning (hardware monitored)
- Bit 2: E-Stop (hardware detected)
- Bit 3: Temperature Error (hardware monitored)
- Bit 4: Temperature2 Error (hardware monitored)
- Bit 5: Main Battery High Error (hardware monitored)
- Bit 6: Logic Battery High Error (hardware monitored)
- Bit 7: Logic Battery Low Error (hardware monitored)
- Bit 8: M2 Driver Fault (hardware detected)
- Bit 9: M1 Driver Fault (hardware detected)
- Bit 10: Main Battery High Warning (hardware monitored)
- Bit 11: Main Battery Low Warning (hardware monitored)
- Bit 12: Temperature Warning (hardware monitored)
- Bit 13: Temperature2 Warning (hardware monitored)
- Bit 14: M1 Home (hardware sensor)
- Bit 15: M2 Home (hardware sensor)
- Bits 16-31: Extended status flags

### Device Readings Already Available
**DO NOT RE-CALCULATE** - use these existing values from `readSensorGroup()`:
- Main battery voltage (`CmdReadMainBatteryVoltage`)
- Logic battery voltage (`CmdReadLogicBatteryVoltage`)
- Motor currents M1 & M2 (`CmdReadMotorCurrents`)
- Temperature (`CmdReadTemperature`)
- Encoder values & status (`CmdReadEncoder`)
- Encoder speeds (`CmdReadEncoderSpeed`)
- Motor PID settings (`CmdReadMotorVelocityPIDQ`)
- Firmware version (`CmdReadFirmwareVersion`)
- Serial timeout setting (`CmdReadSerialTimeout`)

## Metrics to Monitor (Driver-Level Only)

### Communication Health (Software-Level Monitoring)
- [ ] Serial communication error count (driver level)
- [ ] Command timeout rate (driver tracked)
- [ ] Failed command count (consecutive errors tracked)
- [ ] Last successful communication timestamp (already tracked)
- [ ] Connection state (CONNECTED/DISCONNECTED - already implemented)

### Software Protection State
- [ ] Current protection state machine (NORMAL/OVER_CURRENT_WARNING/RECOVERY_WAITING/RECOVERING)
- [ ] Filtered current averages (m1_current_average_, m2_current_average_)
- [ ] Time since last non-zero cmd_vel (last_nonzero_cmd_vel_time_)
- [ ] Recovery timing information

### Driver Performance
- [ ] Sensor update rate (actual vs configured)
- [ ] Time since last sensor update (last_sensor_read_time_)

## Diagnostic Levels

### OK (Green)
- All communications successful
- Device responding normally (connection_state_ == CONNECTED)
- Device error_status == 0 ("normal")
- Current protection state == NORMAL
- All device readings within normal ranges

### WARN (Yellow)
- Device warning flags set (bits 10-13: battery warnings, temperature warnings)
- Current protection state == OVER_CURRENT_WARNING or RECOVERY_WAITING
- Occasional communication errors (consecutive_errors_ < error_threshold_)
- Minor performance issues (slow sensor updates)

### ERROR (Red)
- Device error flags set (bits 2-9: E-stop, driver faults, battery errors, temp errors)
- Connection state == DISCONNECTED
- Current protection state persists too long
- Persistent communication failures (consecutive_errors_ >= error_threshold_)
- Hardware faults reported by device

### STALE (Gray)
- No recent sensor updates (last_sensor_read_time_ too old)
- Lost connection to device (connection_state_ == DISCONNECTED for extended time)

## Implementation Tasks

### Phase 1: Basic Diagnostics
- [x] Add diagnostic_updater dependency to package.xml
- [x] Create DiagnosticTask class for RoboClaw
- [x] Implement basic communication health monitoring
- [x] Publish initial diagnostics
- [x] **1a. Remove connection_state from RoboClawStatus message** - Connection state belongs in diagnostics, not sensor data. Status messages should stop publishing when disconnected. Clients detect disconnection via message timestamp monitoring or diagnostics subscription.
  - Removed `connection_state` field from `msg/RoboClawStatus.msg`
  - Updated `motor_driver_node.cpp` to only publish status when CONNECTED
  - Created `CONNECTION_STATE_DESIGN.md` documenting the design rationale

### Phase 2: Device Metrics
- [ ] Add battery voltage monitoring
- [ ] Add motor current monitoring
- [ ] Add temperature monitoring
- [ ] Add error flag decoding

### Phase 3: Advanced Monitoring
- [ ] Implement trend analysis
- [ ] Add configurable thresholds via parameters
- [ ] Implement detailed error reporting
- [ ] Add performance metrics

### Phase 4: Integration
- [ ] Test by unplugging and replugging. Should see error in state in diagnostics, and it should recover after re-plugged and resume normal operation
- [ ] Test with Foxglove Studio
- [ ] Test with rqt_robot_monitor
- [ ] Document diagnostic keys and meanings in README.md
- [ ] Add launch file configuration
- [ ] Update README.md

## Configuration Parameters

```yaml
roboclaw_diagnostics:
  update_rate: 1.0  # Hz
  thresholds:
    min_battery_voltage: 11.0
    max_motor_current: 10.0
    max_temperature: 70.0
    max_comm_error_rate: 0.1
    comm_timeout: 1.0
```

## Diagnostic Keys

- `roboclaw`: Top-level device status (aggregates all sub-diagnostics)
  - Reports: firmware version, connection state, overall health
  
- `roboclaw: Communication`: Serial communication health
  - Reports: consecutive_errors_, last_successful_communication_, connection_state_
  - Status from: Driver tracking, not device
  
- `roboclaw: Hardware Status`: Device hardware status
  - Reports: error_status bits decoded, error_string from device
  - Status from: Device GETERROR command (bits 0-31)
  
- `roboclaw: Battery`: Battery voltage status
  - Reports: main_battery_level, logic_battery_level
  - Status from: Device CmdReadMainBatteryVoltage/CmdReadLogicBatteryVoltage
  - Check against: warning/error bits 5,6,7,10,11 from device status
  
- `roboclaw: Motor 1`: Motor 1 status
  - Reports: m1_current_average_, encoder status, velocity, over-current alarm
  - Status from: Device CmdReadMotorCurrents + driver filtering + encoder commands
  - Check against: device bit 0 (M1 OverCurrent), bit 9 (M1 Driver Fault)
  
- `roboclaw: Motor 2`: Motor 2 status
  - Reports: m2_current_average_, encoder status, velocity, over-current alarm
  - Status from: Device CmdReadMotorCurrents + driver filtering + encoder commands
  - Check against: device bit 1 (M2 OverCurrent), bit 8 (M2 Driver Fault)
  
- `roboclaw: Temperature`: Device temperature
  - Reports: temperature value
  - Status from: Device CmdReadTemperature
  - Check against: device bits 3,4,12,13 (temp errors and warnings)
  
- `roboclaw: Current Protection`: Software current protection state
  - Reports: current_protection_state_, recovery status, cmd_vel tracking
  - Status from: Driver software state machine (not device)

## References
- ROS2 diagnostic_updater: https://github.com/ros/diagnostics
- diagnostic_msgs: https://github.com/ros2/common_interfaces
- Current implementation: src/ros2_roboclaw_driver/

## Key Design Principles

### Don't Duplicate Device Monitoring
- **DO NOT** re-calculate currents, voltages, or temperatures - use device readings
- **DO NOT** re-implement over-current detection - device already monitors this in hardware
- **DO NOT** re-check battery levels - device has built-in thresholds
- **DO** use the 32-bit error_status from GETERROR command as source of truth for hardware state

### Driver-Level Diagnostics Only
- **DO** monitor serial communication health (driver responsibility)
- **DO** track connection state (CONNECTED/DISCONNECTED)
- **DO** report software current protection state machine
- **DO** calculate derived metrics (e.g., speed error = commanded - actual)
- **DO** monitor sensor update timing

### Use Existing Infrastructure
- All device readings are already cached in `g_sensor_value_group_`
- `readSensorGroup()` already runs periodically and updates all values
- `getErrorStatus()` and `getErrorString()` already decode the 32-bit status
- Connection tracking (`consecutive_errors_`, `connection_state_`) already implemented

## Notes
- Diagnostics should not trigger additional RoboClaw commands (use cached values)
- Diagnostics should not interfere with normal operation
- Keep diagnostic overhead minimal
- Provide actionable information in diagnostic messages
- Clearly distinguish device-reported vs driver-calculated metrics in messages
