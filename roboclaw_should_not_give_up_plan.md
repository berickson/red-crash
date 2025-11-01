# RoboClaw Driver Resilience Plan

## Problem Statement

The RoboClaw driver currently terminates the entire node when communication failures exceed retry limits. This causes the robot to lose motor control completely when the USB cable comes loose or has transient connection issues, even after the cable is reconnected.

**Error pattern observed:**
```
[ERROR] [RoboClaw::get2ByteCommandResult2] invalid CRC expected: 0x10EB, got: 0x00
[ERROR] [ReadLogicBatteryVoltage] Communication error (final attempt): CRC mismatch
[ERROR] [RoboClaw::Cmd::execute] RETRY COUNT EXCEEDED for ReadLogicBatteryVoltage
terminate called after throwing an instance of 'RoboClaw::TRoboClawException'
[ERROR] [ros2_roboclaw_driver_node-1]: process has died [pid 171, exit code -6]
```

**Root cause:** When `Cmd::execute()` exhausts retries, it throws an exception that propagates up to `main()` and terminates the process.

---

## Design Goals

1. **Never terminate the node** - Communication failures should degrade gracefully, not crash
2. **Automatic recovery** - Detect and recover from USB disconnection/reconnection
3. **Fail-safe motor control** - Stop motors safely when communication fails
4. **Clear status reporting** - Diagnostics should show connection state and error details
5. **Configurable retry behavior** - Allow tuning of retry counts and timeout periods

---

## Current Architecture Analysis

### Exception Flow

1. Low-level I/O (`readByteWithTimeout2`, `writeByte2`) throws `TRoboClawException` variants:
   - `TimeoutException` - no data received within timeout
   - `CrcException` - CRC validation failed
   - `CommunicationException` - generic comm errors
   - `DeviceNotRespondingException` - device not available

2. Command wrappers (`Cmd::execute()`) catch exceptions and retry 3 times
   - Retries 1-2: Log WARN, call `recordError()`, continue
   - Retry 3: Log ERROR, call `recordFailedCommunication()`, **throw exception**

3. Upper layers (`readSensorGroup()`, `motor_driver.cpp`) call `cmd.execute()`:
   - If exception propagates, entire node crashes
   - No recovery mechanism in place

### Connection State Tracking

Already implemented (good foundation):
- `ConnectionState` enum: CONNECTED, DISCONNECTED
- `setConnectionState()` tracks transitions
- `recordSuccessfulCommunication()` resets error counters
- `recordFailedCommunication()` increments error counters
- After 3 consecutive errors, state → DISCONNECTED

### Current Reconnection Logic

In `readSensorGroup()`:
```cpp
if (connection_state_ == DISCONNECTED) {
    // Try reconnect every 5 seconds
    openPort();  // throws on failure
    // If successful, fall through to read sensors
}
```

**Problem:** If `openPort()` throws, exception propagates and crashes node.

---

## Solution Architecture

### 1. Never Let Exceptions Escape `readSensorGroup()`

**Change:** Wrap entire `readSensorGroup()` in try-catch that suppresses exceptions.

**Implementation:**
```cpp
void RoboClaw::readSensorGroup() {
    try {
        // All sensor reading code...
    } catch (const TRoboClawException& e) {
        RCUTILS_LOG_WARN("[RoboClaw::readSensorGroup] Communication failure: %s", e.what());
        recordFailedCommunication(e);
        // Don't re-throw - let caller continue
        return;  // Exit gracefully with stale sensor data
    } catch (const std::exception& e) {
        RCUTILS_LOG_WARN("[RoboClaw::readSensorGroup] Unexpected exception: %s", e.what());
        setConnectionState(DISCONNECTED, "Unexpected exception");
        return;
    }
}
```

**Rationale:**
- Sensor reads happen in background timer (20Hz)
- If one sensor read fails, try again next cycle
- Node stays alive to accept future `cmd_vel` and attempt recovery

---

### 2. Never Let Exceptions Escape `cmdVelCallback()`

**Change:** Wrap motor commands in try-catch.

**Implementation in motor_driver.cpp:**
```cpp
void MotorDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    try {
        // Calculate velocities, check over-current protection...
        roboclaw->doMixedSpeedAccel(...);  // This can throw
    } catch (const RoboClaw::TRoboClawException& e) {
        RCUTILS_LOG_ERROR("[MotorDriver::cmdVelCallback] Failed to send cmd: %s", e.what());
        roboclaw->recordFailedCommunication(e);
        // Motor command failed but node stays alive
        return;
    } catch (const std::exception& e) {
        RCUTILS_LOG_ERROR("[MotorDriver::cmdVelCallback] Unexpected error: %s", e.what());
        return;
    }
}
```

---

### 3. Enhanced Reconnection Logic

**Problem:** Current reconnection only happens in `readSensorGroup()` and throws on failure.

**Solution:** Separate reconnection attempt into non-throwing function.

**Implementation:**
```cpp
bool RoboClaw::attemptReconnection() {
    // Called periodically when DISCONNECTED
    if (device_port_ >= 0) {
        close(device_port_);
        device_port_ = -1;
    }
    
    try {
        openPort();
        // Port opened successfully
        setConnectionState(CONNECTED, "Reconnection successful");
        RCUTILS_LOG_INFO("[RoboClaw] Successfully reconnected to %s", device_name_.c_str());
        return true;
    } catch (const TRoboClawException& e) {
        // Still can't connect - stay DISCONNECTED
        RCUTILS_LOG_WARN("[RoboClaw] Reconnection attempt failed: %s", e.what());
        return false;
    }
}
```

**Usage in readSensorGroup():**
```cpp
void RoboClaw::readSensorGroup() {
    if (connection_state_ == DISCONNECTED) {
        static auto last_reconnect = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_reconnect).count();
        
        if (elapsed >= 5) {
            last_reconnect = now;
            if (!attemptReconnection()) {
                return;  // Failed, don't try to read sensors
            }
            // Success, fall through to read sensors
        } else {
            return;  // Not time to retry yet
        }
    }
    
    try {
        // All sensor reading commands...
    } catch (...) {
        // Handle as above
    }
}
```

---

### 4. Fail-Safe Motor Behavior When DISCONNECTED

**Problem:** If disconnected, we can't send motor commands, but we should ensure safe state.

**Solution:** Check connection state before sending motor commands.

**Implementation in motor_driver.cpp:**
```cpp
void MotorDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    auto* roboclaw = RoboClaw::singleton();
    if (roboclaw->getConnectionState() == RoboClaw::DISCONNECTED) {
        static auto last_warning = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_warning).count();
        
        if (elapsed_ms >= 1000) {  // Warn once per second
            RCUTILS_LOG_WARN("[MotorDriver] Cannot send cmd_vel: RoboClaw disconnected");
            last_warning = now;
        }
        return;  // Silently drop commands when disconnected
    }
    
    try {
        // Normal command processing...
    } catch (...) {
        // Handle as above
    }
}
```

---

### 5. Improve Diagnostics

**Add to diagnostic updater:**

```cpp
void diagnosticOverall(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    RoboClaw* roboclaw = RoboClaw::singleton();
    RoboClaw::ConnectionState conn_state = roboclaw->getConnectionState();
    
    if (conn_state == RoboClaw::DISCONNECTED) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, 
                     "DISCONNECTED - attempting reconnection every 5 seconds");
        stat.add("Connection State", "DISCONNECTED");
        stat.add("Reconnection Status", "Retrying...");
        
        // Show error statistics
        const auto& stats = roboclaw->getErrorStats();
        stat.add("Total Communication Errors", stats.total_errors);
        stat.add("Last Error", stats.last_error_message);
        return;  // Don't try to read device when disconnected
    }
    
    // Normal diagnostics when connected...
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Connected");
    stat.add("Connection State", "CONNECTED");
    // ... rest of normal diagnostics
}
```

---

### 6. Configuration Parameters

**Add new parameters for resilience tuning:**

```yaml
# In roboclaw config YAML
roboclaw_resilience:
  max_command_retries: 3              # Retries per command before giving up
  reconnection_interval_seconds: 5    # Time between reconnect attempts
  error_threshold_for_disconnect: 3   # Consecutive errors before marking DISCONNECTED
  failed_command_warning_interval_ms: 1000  # Rate-limit warnings
```

**Implementation in motor_driver.cpp:**
```cpp
void MotorDriver::declareParameters() {
    // ... existing parameters ...
    
    rcl_interfaces::msg::ParameterDescriptor max_retries_desc;
    max_retries_desc.description = "Maximum command retry attempts before marking DISCONNECTED";
    max_retries_desc.integer_range.resize(1);
    max_retries_desc.integer_range[0].from_value = 1;
    max_retries_desc.integer_range[0].to_value = 10;
    max_retries_desc.integer_range[0].step = 1;
    node_->declare_parameter<int>("max_command_retries", 3, max_retries_desc);
    
    rcl_interfaces::msg::ParameterDescriptor reconnect_interval_desc;
    reconnect_interval_desc.description = "Seconds between reconnection attempts";
    reconnect_interval_desc.floating_point_range.resize(1);
    reconnect_interval_desc.floating_point_range[0].from_value = 1.0;
    reconnect_interval_desc.floating_point_range[0].to_value = 60.0;
    reconnect_interval_desc.floating_point_range[0].step = 0.0;
    node_->declare_parameter<float>("reconnection_interval_seconds", 5.0, reconnect_interval_desc);
}
```

---

### 7. Coast Motors When Disconnected

**Problem:** If we lose connection mid-drive, motors may stay powered (depending on RoboClaw serial timeout).

**Solution:** Before marking DISCONNECTED, attempt to coast motors.

**Implementation:**
```cpp
void RoboClaw::setConnectionState(ConnectionState new_state, const char* reason) {
    if (new_state != connection_state_) {
        if (new_state == DISCONNECTED && connection_state_ == CONNECTED) {
            // Transitioning to disconnected - try to coast motors for safety
            RCUTILS_LOG_WARN("[RoboClaw] Connection lost: %s. Attempting to coast motors...", reason);
            try {
                coast();  // Send coast command
                RCUTILS_LOG_INFO("[RoboClaw] Motors coasted successfully before disconnect");
            } catch (...) {
                RCUTILS_LOG_ERROR("[RoboClaw] Failed to coast motors during disconnect");
                // Continue with disconnect anyway
            }
        } else if (new_state == CONNECTED && connection_state_ == DISCONNECTED) {
            RCUTILS_LOG_INFO("[RoboClaw] Connection restored: %s", reason);
        }
        
        connection_state_ = new_state;
    }
}
```

---

## Implementation Checklist

### Phase 1: Core Resilience (Critical)
- [ ] Wrap `readSensorGroup()` entire body in try-catch to prevent crashes
- [ ] Wrap motor command calls in `cmdVelCallback()` in try-catch
- [ ] Add `attemptReconnection()` non-throwing function
- [ ] Update `readSensorGroup()` to use `attemptReconnection()`
- [ ] Test: Unplug USB cable while driving → node stays alive, stops motors
- [ ] Test: Replug USB cable → node reconnects within 5 seconds
- [ ] Update diagnostics to show DISCONNECTED state clearly

### Phase 2: Safety Enhancements
- [ ] Check `getConnectionState()` in `cmdVelCallback()` before sending commands
- [ ] Add rate-limited warnings when commands dropped due to DISCONNECTED state
- [ ] Call `coast()` in `setConnectionState()` when transitioning to DISCONNECTED
- [ ] Test: Verify motors coast when connection lost
- [ ] Test: Verify cmd_vel is ignored when disconnected (no crash, just warnings)

### Phase 3: Configuration & Tuning
- [ ] Add `max_command_retries` parameter to RoboClaw constructor
- [ ] Add `reconnection_interval_seconds` parameter
- [ ] Make `error_threshold_for_disconnect` configurable
- [ ] Update `Cmd::execute()` to use configurable retry count
- [ ] Update YAML config with new parameters
- [ ] Document parameters in README.md

### Phase 4: Diagnostics & Monitoring
- [ ] Enhance diagnostics to show:
  - [ ] Connection state (CONNECTED/DISCONNECTED)
  - [ ] Time since last successful communication
  - [ ] Time until next reconnection attempt (when DISCONNECTED)
  - [ ] Consecutive error count
  - [ ] Last error type and message
- [ ] Add Foxglove panel recommendations for monitoring connection health
- [ ] Create test scenario: rapid connect/disconnect cycles

### Phase 5: Code Cleanup & Refactoring (Optional)
- [ ] Remove unused `restartPort()` calls after errors (now handled by state machine)
- [ ] Consider removing `throw new` (leaks memory) - use `throw` without new
- [ ] Audit all `catch (...)` blocks - should catch specific exception types
- [ ] Consider moving retry logic from `Cmd::execute()` to a central retry manager

---

## Testing Strategy

### Unit Tests
1. **Mock serial device** to simulate:
   - CRC errors
   - Timeouts
   - Disconnection (ENODEV)
   - Reconnection after delay

2. **Test cases:**
   - Verify 3 retries before marking DISCONNECTED
   - Verify node stays alive after DISCONNECTED
   - Verify reconnection attempt every 5 seconds
   - Verify motors coast on disconnect
   - Verify cmd_vel ignored when DISCONNECTED

### Integration Tests
1. **Hardware disconnect test:**
   ```bash
   # In one terminal
   ros2 launch roboclaw.launch.py
   
   # In another terminal
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10
   
   # Physically unplug USB cable
   # Expected: Node logs "Connection lost", motors coast, node stays alive
   
   # Wait 10 seconds, replug USB cable
   # Expected: Node logs "Connection restored", motors respond to cmd_vel
   ```

2. **Stress test:**
   - Rapidly plug/unplug USB 10 times over 60 seconds
   - Verify node never crashes
   - Verify node eventually reconnects

3. **Diagnostics test:**
   - Monitor `/diagnostics` topic during disconnect/reconnect
   - Verify status changes: OK → ERROR → OK
   - Verify error messages are clear and actionable

---

## Potential Risks & Mitigation

### Risk 1: Stale Sensor Data
**Issue:** If `readSensorGroup()` fails, diagnostics/status messages show stale data.

**Mitigation:**
- Add timestamp to `g_sensor_value_group_` struct
- In diagnostics, check age of sensor data
- If data > 500ms old, mark diagnostics as STALE

### Risk 2: Race Conditions During Reconnection
**Issue:** If `readSensorGroup()` reconnects while `cmdVelCallback()` is sending a command.

**Mitigation:**
- Use existing `buffered_command_mutex_` to protect all serial operations
- Both sensor reads and motor commands already lock this mutex

### Risk 3: Infinite Retry Loop
**Issue:** If device never comes back, node spams reconnection attempts forever.

**Mitigation:**
- Reconnection attempts every 5 seconds (rate-limited)
- Log at WARN level (not ERROR) to avoid log spam
- Allow user to restart node if needed, but don't crash automatically

### Risk 4: False Disconnection Detection
**Issue:** Transient errors might mark device DISCONNECTED unnecessarily.

**Mitigation:**
- Require `error_threshold` (default 3) consecutive errors before DISCONNECTED
- Single successful command resets consecutive error counter
- Make threshold configurable

---

## Future Enhancements (Beyond This Plan)

1. **Exponential Backoff:** Increase reconnection interval if reconnection repeatedly fails (5s, 10s, 20s, max 60s)

2. **Connection Health Monitoring:** Track error rate over time window, warn if elevated

3. **Alternative Transport:** Support network-based RoboClaw (TCP/IP) as failover

4. **Graceful Degradation:** Reduce sensor update rate when errors frequent

5. **Self-Healing:** Automatically reset RoboClaw device via GPIO if supported by hardware

---

## References

- RoboClaw User Manual: Section on Serial Timeout behavior
- Existing code: `roboclaw.cpp` lines 520-620 (current reconnection logic)
- Existing code: `roboclaw_cmd.h` lines 8-70 (retry logic)
- Repository: `archived_plans/roboclaw_dies_on_serial_errors.md` (related issue)
- Repository: `archived_plans/connection_state_design.md` (connection state machine)

---

## Success Criteria

**Definition of Done:**
1. Node never terminates due to RoboClaw communication errors
2. Motors safely coast when connection lost
3. Node automatically reconnects within 10 seconds of USB replug
4. Diagnostics clearly show connection state
5. No performance degradation during normal (connected) operation
6. All tests pass: unit, integration, stress

**Acceptance Test:**
```bash
# Terminal 1: Start node
ros2 launch roboclaw.launch.py

# Terminal 2: Drive robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10

# Terminal 3: Monitor diagnostics
ros2 topic echo /diagnostics

# Test sequence:
# 1. Verify normal operation - robot moves
# 2. Unplug USB cable
# 3. Verify: motors stop, node stays alive, diagnostics show ERROR
# 4. Wait 10 seconds
# 5. Replug USB cable
# 6. Verify: diagnostics show OK, robot responds to cmd_vel again
# 7. PASS if node never crashes
```

---

## Implementation Notes

**Priority:** HIGH - Node crashes prevent all robot operation

**Estimated Effort:** 
- Phase 1 (Core): 4-6 hours (critical path)
- Phase 2 (Safety): 2-3 hours
- Phase 3 (Config): 2 hours
- Phase 4 (Diagnostics): 3-4 hours
- Phase 5 (Cleanup): 2-3 hours (optional)
- Testing: 4-6 hours

**Total: 17-24 hours**

**Dependencies:**
- No external dependencies
- Builds on existing connection state tracking (already implemented)
- Compatible with current over-current protection system

**Backward Compatibility:**
- No breaking changes to API
- New parameters optional (have defaults)
- Existing YAML configs work unchanged
