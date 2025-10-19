# RoboClaw Command Pattern and Error Handling Refactoring Plan

## Current State Analysis

### Command Pattern Overview

The driver uses a command pattern with a base `Cmd` class and derived command classes. Each command represents a specific RoboClaw operation (read encoder, set speed, etc.).

**Base Command Class** (`roboclaw_cmd.h`):
- `execute()` - retry loop wrapper (3 retries hardcoded)
- `send()` - pure virtual, must be implemented by derived classes
- `process()` - optional post-processing (called after mutex released)

**Typical Command Flow**:
1. User code creates a command object (e.g., `CmdReadEncoder`)
2. Calls `command.execute()`
3. `execute()` acquires mutex, calls `send()`, handles retries
4. After successful send, mutex is released and `process()` is called
5. Records success/failure for diagnostics

### Current Error Handling Problems

#### 1. **Throwing Pointers Instead of Values**

**Problem**: Code throws pointer to exception instead of exception object.

```cpp
throw new TRoboClawException("message");  // WRONG - throws pointer
```

**Issues**:
- Causes undefined behavior when caught by value or reference
- Memory leak - exception never deleted
- Catch blocks expect `TRoboClawException*` but also have `catch(...)` fallbacks
- Inconsistent catch patterns across codebase

**Current Catch Patterns**:
```cpp
// In Cmd::execute()
catch (RoboClaw::TRoboClawException *e) {  // Catches pointer
    RCUTILS_LOG_ERROR("Exception: %s", e->what());
}
catch (...) {  // Catches everything else
    RCUTILS_LOG_ERROR("Uncaught exception !!!");
}
```

**Examples Found**:
- `roboclaw.cpp`: `throw new TRoboClawException(...)` (12+ occurrences)
- `roboclaw_cmd_*.h`: `throw new RoboClaw::TRoboClawException(...)` (5+ occurrences)
- One correct usage in `roboclaw.cpp:507`: `throw TRoboClawException(...)` (no `new`)

#### 2. **Inconsistent Error Recovery**

**Problem**: Each command handles errors differently, recovery is ad-hoc.

**Patterns Found**:

**Pattern A - Catch and Log Only** (most common):
```cpp
void send() override {
  try {
    // ... do work ...
  } catch (...) {
    RCUTILS_LOG_ERROR("[RoboClaw::CmdXxx] Uncaught exception !!!");
    // Falls through - error is swallowed!
  }
}
```
Examples: `CmdReadEncoderSpeed`, `CmdReadStatus`, `CmdReadEncoder`, `CmdReadTemperature`

**Pattern B - Catch and Rethrow**:
```cpp
void send() override {
  try {
    // ... do work ...
  } catch (...) {
    RCUTILS_LOG_ERROR("[RoboClaw::CmdReadMotorCurrents] Uncaught exception in send() !!!");
    throw;  // Rethrows for execute() to handle
  }
}
```
Examples: `CmdReadMotorCurrents`

**Pattern C - Explicit Throw**:
```cpp
void send() override {
  try {
    // ... do work ...
    if (some_error) {
      throw new RoboClaw::TRoboClawException("message");
    }
  } catch (...) {
    RCUTILS_LOG_ERROR("...");
  }
  RCUTILS_LOG_ERROR("RETRY COUNT EXCEEDED");  // After try-catch!
}
```
Examples: `CmdReadFirmwareVersion`

**Pattern D - No Error Handling**:
```cpp
void send() override {
  // Direct calls to roboclaw methods that may throw
  // No try-catch at all
}
```
Examples: `CmdSetPid`

**Why This Is Bad**:
- Pattern A swallows exceptions - retry mechanism doesn't trigger
- Pattern C has unreachable error messages after catch-all
- Pattern D relies on execute() to catch everything
- Inconsistent - hard to understand what will happen on error

#### 3. **Generic Error Messages**

**Problem**: Error messages don't describe what operation failed or provide actionable info.

**Current Messages**:
```
"[RoboClaw::readByteWithTimeout2 TIMEOUT"  // Which command timed out?
"[RoboClaw::CmdReadEncoder] Uncaught exception !!!"  // What exception?
"INVALID CRC"  // While doing what?
```

**Better Messages Would Be**:
```
"ReadEncoder M1: Communication timeout after 11ms"
"ReadEncoder M1: CRC mismatch (expected 0x1234, got 0x5678)"
"ReadEncoder M1: Failed to read data - device not responding"
```

#### 4. **Low-Level Functions Throw Directly**

**Problem**: `readByteWithTimeout2()`, `writeByte2()`, `writeN2()` throw exceptions directly for low-level errors.

**Examples**:
```cpp
uint8_t RoboClaw::readByteWithTimeout2() {
  // ...
  if (retval == 0) {
    throw new TRoboClawException("[RoboClaw::readByteWithTimeout2 TIMEOUT");
  }
  if (bytesRead != 1) {
    throw TRoboClawException("Failed to read 1 byte");  // Correct (no new)
  }
}
```

**Issues**:
- No context about which command was executing
- Direct throws bypass command-level error handling
- Can't distinguish between timeout, CRC error, device disconnect

#### 5. **Errors Don't Always Reach Diagnostics**

**Problem**: Exception handling in command `send()` methods sometimes swallows errors.

**Current Diagnostic Flow**:
1. `Cmd::execute()` catches exceptions and logs them
2. After 3 retries, calls `recordFailedCommunication()`
3. After threshold, sets `connection_state_ = DISCONNECTED`
4. Diagnostics reads `connection_state_` and cached sensor values

**What Goes Wrong**:
- Pattern A commands (catch and swallow) never propagate exceptions to `execute()`
- `execute()` only records failure if exception is thrown OR if retries succeed
- If `send()` swallows exception, `execute()` thinks it succeeded!
- Diagnostics shows "CONNECTED" but device may be failing intermittently

**Example Flow with Pattern A**:
```
1. CmdReadEncoder::send() tries to read
2. readByteWithTimeout2() throws timeout exception
3. catch (...) in send() catches it, logs error
4. send() returns normally (no exception propagated)
5. execute() thinks command succeeded
6. recordSuccessfulCommunication() is called!
7. Diagnostics never know about the error
```

#### 6. **Retry Logic is Inflexible**

**Problem**: Hardcoded 3 retries in `Cmd::execute()`.

```cpp
for (int retry = 0; retry < 3 /*### maxCommandRetries_*/; retry++) {
```

**Issues**:
- Cannot configure per deployment
- Same retry count for critical vs non-critical commands
- Comment indicates this was known issue: `/*### maxCommandRetries_*/`

## Recommendations

### 1. Fix Exception Throwing (CRITICAL)

**Change all exception throws from**:
```cpp
throw new TRoboClawException("message");
```

**To**:
```cpp
throw TRoboClawException("message");
```

**Update all catch blocks from**:
```cpp
catch (RoboClaw::TRoboClawException *e) { ... }
```

**To**:
```cpp
catch (const RoboClaw::TRoboClawException& e) { ... }
```

**Files to Update**:
- `src/roboclaw.cpp` - All low-level I/O functions
- All `include/roboclaw_cmd_*.h` - All command implementations
- `include/roboclaw_cmd.h` - Base execute() catch blocks
- `src/motor_driver_node.cpp` - Diagnostic callback

**Rationale**: Throwing by value is the C++ standard practice. Prevents memory leaks and undefined behavior.

### 2. Create Exception Hierarchy

**Define specific exception types for different failure modes**:

```cpp
// Base exception (already exists)
struct TRoboClawException : public std::exception { ... };

// Communication failures
struct CommunicationException : public TRoboClawException {
    CommunicationException(const char* operation, const char* reason, ...)
        : TRoboClawException("[%s] Communication error: %s", operation, reason) {}
};

// Timeout specifically
struct TimeoutException : public CommunicationException {
    TimeoutException(const char* operation, int timeout_ms)
        : CommunicationException(operation, "Timeout after %dms", timeout_ms) {}
};

// CRC/checksum errors
struct CrcException : public CommunicationException {
    uint16_t expected;
    uint16_t actual;
    CrcException(const char* operation, uint16_t exp, uint16_t act)
        : CommunicationException(operation, "CRC mismatch (expected 0x%04X, got 0x%04X)", exp, act),
          expected(exp), actual(act) {}
};

// Device not responding
struct DeviceNotRespondingException : public CommunicationException {
    DeviceNotRespondingException(const char* operation)
        : CommunicationException(operation, "Device not responding") {}
};

// Invalid response format
struct InvalidResponseException : public CommunicationException {
    InvalidResponseException(const char* operation, const char* details, ...)
        : CommunicationException(operation, details, ...) {}
};
```

**Rationale**: Allows higher-level code to handle different errors differently. E.g., CRC errors might warrant immediate retry, timeouts might need longer delays.

### 3. Standardize Command Error Handling

**Establish ONE pattern for all commands**:

```cpp
void send() override {
  // Step 1: Log what we're about to do
  roboclaw_.appendToWriteLog("ReadEncoder M%d: WROTE: ", 
                             motor_ == RoboClaw::kM1 ? 1 : 2);
  
  // Step 2: Do the actual work (may throw from low-level functions)
  uint16_t crc = 0;
  roboclaw_.updateCrc(crc, roboclaw_.portAddress_);
  roboclaw_.updateCrc(crc, command);
  roboclaw_.writeN2(false, 2, roboclaw_.portAddress_, command);
  
  uint8_t datum = roboclaw_.readByteWithTimeout2(name_);  // May throw TimeoutException
  // ... read more bytes, each may throw ...
  
  // Step 3: Validate response
  if (responseCrc != crc) {
    throw CrcException(name_, crc, responseCrc);
  }
  
  // Step 4: Log success
  roboclaw_.appendToReadLog(", RESULT: %d", value);
  
  // Note: NO try-catch blocks! Let exceptions propagate to execute()
}
```

**Rules**:
1. **No try-catch in send()** - let exceptions propagate naturally
2. Low-level functions (`readByteWithTimeout2()`, `writeN2()`, etc.) throw specific exceptions
3. Commands only validate results and throw validation exceptions (CRC, invalid format)
4. `execute()` is the ONLY place that catches and retries

**Rationale**: 
- Simplest possible code - no error handling boilerplate in every command
- Low-level functions already have context (passed operation name)
- Consistent behavior - all errors propagate the same way
- Diagnostics always updated via `execute()`'s catch block

### 4. Improve Error Messages with Context

**Low-level functions should include context**:

```cpp
uint8_t RoboClaw::readByteWithTimeout2(const char* operation_context) {
  int retval = poll(ufd, 1, 11);
  if (retval == 0) {
    throw TimeoutException(operation_context, 11);
  }
  if (ufd[0].revents & POLLERR) {
    throw CommunicationException(operation_context, "Socket error");
  }
  // ...
}
```

**Commands pass their name as context**:

```cpp
void CmdReadEncoder::send() override {
  // ...
  uint8_t datum = roboclaw_.readByteWithTimeout2(name_);  // "ReadEncoder"
}
```

**Error messages become**:
```
[ReadEncoder] Communication error: Timeout after 11ms
[ReadEncoder] Communication error: CRC mismatch (expected 0x1234, got 0x5678)
[SetPID M1] Communication error: Invalid ACK response (expected 0xFF, got 0x00)
```

**Rationale**: Immediately clear which operation failed, easier to debug.

### 5. Create Error Classification System

**Add error severity levels**:

```cpp
enum class ErrorSeverity {
  TRANSIENT,    // Likely to succeed on retry (timeout, busy)
  CRITICAL,     // Unlikely to recover (device unplugged, wrong device)
  FATAL         // Cannot continue (invalid configuration)
};

struct TRoboClawException : public std::exception {
  ErrorSeverity severity;
  // ...
};
```

**Use severity for retry decisions**:

```cpp
void Cmd::execute() {
  for (int retry = 0; retry < maxCommandRetries_; retry++) {
    try {
      // ...
    } catch (const TRoboClawException& e) {
      if (e.severity == ErrorSeverity::FATAL) {
        // Don't retry fatal errors
        throw;
      }
      if (e.severity == ErrorSeverity::CRITICAL && retry > 0) {
        // Only retry CRITICAL errors once
        throw;
      }
      // TRANSIENT errors get full retry count
      continue;
    }
  }
}
```

**Rationale**: Reduces unnecessary retries, fails faster on unrecoverable errors.

### 6. Make Retry Count Configurable

**Add to RoboClaw constructor**:

```cpp
RoboClaw(/* existing params */, int max_command_retries = 3);
```

**Update Cmd::execute()**:

```cpp
void Cmd::execute() {
  for (int retry = 0; retry < roboclaw_.getMaxCommandRetries(); retry++) {
    // ...
  }
}
```

**Allow per-command override**:

```cpp
class Cmd {
protected:
  int max_retries_ = -1;  // -1 means use RoboClaw default
  
  void setMaxRetries(int retries) { max_retries_ = retries; }
  int getMaxRetries() const {
    return max_retries_ >= 0 ? max_retries_ : roboclaw_.getMaxCommandRetries();
  }
};
```

**Rationale**: Allows tuning for specific environments, critical commands can have more retries.

### 7. Improve Diagnostic Error Reporting

**Track error types in RoboClaw**:

```cpp
struct ErrorStats {
  uint32_t total_timeouts = 0;
  uint32_t total_crc_errors = 0;
  uint32_t total_ack_errors = 0;
  uint32_t total_device_not_responding = 0;
  std::string last_error_message;
  std::chrono::steady_clock::time_point last_error_time;
};
```

**Update in recordFailedCommunication()**:

```cpp
void RoboClaw::recordFailedCommunication(const TRoboClawException& e) {
  error_stats_.last_error_message = e.what();
  error_stats_.last_error_time = std::chrono::steady_clock::now();
  
  if (dynamic_cast<const TimeoutException*>(&e)) {
    error_stats_.total_timeouts++;
  } else if (dynamic_cast<const CrcException*>(&e)) {
    error_stats_.total_crc_errors++;
  }
  // ...
}
```

**Add to diagnostics**:

```cpp
stat.add("Total Timeouts", error_stats_.total_timeouts);
stat.add("Total CRC Errors", error_stats_.total_crc_errors);
stat.add("Last Error", error_stats_.last_error_message);
stat.add("Last Error Time", /* format time */);
```

**Rationale**: Provides visibility into what types of errors are occurring, helps diagnose hardware vs software issues.

### 8. Add Communication State Machine

**States**:
```cpp
enum CommunicationHealth {
  HEALTHY,           // All commands succeeding
  DEGRADED,          // Some errors but still functional
  FAILING,           // Frequent errors, limited functionality
  DISCONNECTED       // Device not responding
};
```

**Transitions**:
- HEALTHY -> DEGRADED: First error after string of successes
- DEGRADED -> FAILING: Error rate exceeds threshold (e.g., >20% fail)
- FAILING -> DISCONNECTED: Multiple consecutive failures (current behavior)
- DISCONNECTED -> FAILING: First successful command after reconnect
- FAILING -> DEGRADED -> HEALTHY: Error rate decreases over time

**Use in diagnostics**:
```cpp
if (comm_health == DISCONNECTED) {
  stat.summary(DiagnosticStatus::ERROR, "Device disconnected");
} else if (comm_health == FAILING) {
  stat.summary(DiagnosticStatus::ERROR, "Communication failing");
} else if (comm_health == DEGRADED) {
  stat.summary(DiagnosticStatus::WARN, "Communication degraded");
} else {
  stat.summary(DiagnosticStatus::OK, "OK");
}
```

**Rationale**: Provides gradual degradation visibility, early warning before complete failure.

## Implementation Plan

### Phase 1: Fix Critical Issues (Safety)

1. [x] Fix exception throwing (pointer -> value) across all files
2. [x] Update all catch blocks to catch by reference
3. [x] Standardize command error handling (remove swallowing)
4. [x] Test that exceptions propagate to diagnostics

**Deliverable**: Errors always reach diagnostics, no undefined behavior. ✅ COMPLETE

### Phase 2: Improve Error Messages (Debuggability)

1. [x] Create exception hierarchy (Timeout, CRC, DeviceNotResponding, etc.)
2. [x] Update low-level functions to use specific exceptions
3. [x] Add operation context to all exceptions
4. [x] Update command classes to use new exception types

**Deliverable**: Clear, actionable error messages. ✅ COMPLETE

### Phase 3: Enhanced Diagnostics (Visibility)

1. [x] Add error statistics tracking to RoboClaw
2. [x] Update recordFailedCommunication() to track error types
3. [x] Enhance diagnostic callback with error details
4. [x] Add recordError() method for retry tracking
5. [x] Change recoverable errors to WARN level
6. [x] Add command name to error messages
7. [x] Remove redundant timeout log messages

**Deliverable**: Rich diagnostic information for troubleshooting. ✅ COMPLETE



## Testing Strategy

### Unit Tests
- Test each exception type is thrown correctly
- Test exception propagation through command layers
- Test retry logic with different error types
- Test diagnostic state transitions

### Integration Tests
- Simulate device timeout (unplug during operation)
- Simulate CRC errors (corrupted data)
- Simulate device busy (rapid commands)
- Verify diagnostics update correctly in each case

### Regression Tests
- Verify existing functionality still works
- Verify motor control works after error recovery
- Verify all sensors continue reading after transient errors

## Metrics for Success

- [x] Zero catch blocks that swallow exceptions
- [x] 100% of errors visible in diagnostics
- [x] Error messages include operation context
- [x] Diagnostic topic shows error type breakdown
- [x] No memory leaks from exception handling
- [x] Device recovery works within 5 seconds of reconnect
- [x] Recoverable errors logged as WARN, failures as ERROR
- [x] Command name included in all error messages
- [x] Error statistics track all errors including recovered ones

## References

- C++ exception best practices: https://isocpp.org/wiki/faq/exceptions
- ROS2 diagnostics: https://docs.ros.org/en/rolling/Tutorials/Demos/Logging-and-logger-configuration.html
- Current implementation: See files in `src/ros2_roboclaw_driver/`

---

## Implementation Summary (Completed October 19, 2025)

### ✅ Phases 1-3 COMPLETE

All critical bug fixes and enhanced diagnostics have been successfully implemented and tested:

**Critical Fixes:**
- Fixed all exception throwing from pointers to values (C++ standard practice)
- Updated all catch blocks to catch by reference
- Removed all exception-swallowing patterns
- Eliminated memory leaks in exception handling

**Improved Error Reporting:**
- Created exception hierarchy: TimeoutException, CrcException, DeviceNotRespondingException, InvalidResponseException
- All error messages now include command context
- Low-level I/O functions throw specific exception types
- Recoverable errors logged at WARN level, failures at ERROR level

**Enhanced Diagnostics:**
- Added ErrorStats struct tracking all error types
- recordError() method tracks all errors including recovered ones
- Diagnostics show: total timeouts, CRC errors, ACK errors, device not responding, communication errors
- Last error message and timestamp visible in diagnostics
- Error counters increment even for errors recovered by retry

**Code Quality:**
- Zero catch blocks that swallow exceptions
- Consistent error handling across all 16+ command classes
- Clean separation: commands do work, execute() handles errors
- Package builds with zero errors or warnings

### 📋 Phase 4 Not Implemented (Future Enhancement)

The following items remain as future enhancements if needed:
- Configurable retry count (currently hardcoded to 3)
- Error severity levels for smarter retry logic
- Per-command retry override capability

These were deemed lower priority as the current implementation handles errors robustly.
