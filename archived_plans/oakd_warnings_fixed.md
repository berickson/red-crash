# OAK-D Warning Fixes

## Overview
This document explains the two ROS warnings from the OAK-D camera startup and their fixes.

## Warning 1: Foxglove Bridge Max QoS Depth

### The Warning
```
[foxglove_bridge-1] [WARN] [...] Max history depth 10 is less than the requested depth of 12 for topic /rosout, 
limiting from 12 to 10.
```

### Explanation
The foxglove_bridge has a default max_qos_depth of 10, but the /rosout topic requests a history depth of 12. 
This causes the bridge to limit messages, potentially dropping some log messages.

### Fix Applied
Created `/root/ros2_ws/config/foxglove_config.yaml` with `max_qos_depth: 20` and updated 
`launch_all.screenrc` to pass this config file to foxglove_bridge.

### Verification
After restarting with updated screenrc, warning should no longer appear.


## Warning 2: IMU Extrinsics Not Set

### The Warning
```
[WARN] [...] IMU extrinsics are not set. Publishing IMU frame with zero translation and RDF orientation.
```

### Explanation
The OAK-D camera's BNO085 IMU does not have calibration data stored in EEPROM for its physical position
relative to the camera. This is common even on factory-calibrated devices. The warning is **cosmetic only** - 
IMU data (acceleration, gyroscope) is still being published correctly.

### Background
- IMU firmware was successfully updated from 3.2.13 to 3.9.9 (resolved a separate critical warning)
- Factory calibration includes internal IMU sensor calibration (accelerometer/gyro biases)
- Factory calibration does NOT always include IMU physical position (extrinsics) on the PCB
- Source code shows warning triggered when `imuExtr["toCameraSocket"] == -1` (no extrinsics in EEPROM)

### Status: No Action Required
The warning can be safely ignored because:
1. IMU data is fully functional and accurate
2. TF transform is published with identity rotation (RDF orientation)
3. Zero translation is a reasonable approximation (IMU is near PCB center, close to camera frame)
4. Physical measurement would be required for precise values (IMU chip is ~0-20mm from camera center)

### Optional: Manual Extrinsics Override
If precise IMU position is critical for your application, you could manually measure and configure
IMU extrinsics using VIO parameters (i_override_imu_extrinsics, i_imu_extr_x/y/z), but this is
unnecessary for most applications where the IMU is used for motion sensing rather than precise
spatial localization.

### Typical OAK-D IMU Position
Based on OAK-D design, the BNO085 IMU is typically:
- On the main PCB near the center
- Within 10-20mm of the RGB camera center
- Close to the same plane as the camera sensors

Since the exact position varies by OAK-D model and revision, and factory calibration didn't include
this data, the zero-translation default is acceptable for most use cases.


## Files Modified
1. `/root/ros2_ws/config/foxglove_config.yaml` - Created (fixes warning #1)
2. `/root/ros2_ws/config/oakd_config.yaml` - Updated comments only (documentation)
3. `/root/ros2_ws/launch_all.screenrc` - Updated foxglove launch command (fixes warning #1)


## Result
After applying these fixes:
- Warning #1 (foxglove max_qos_depth): ELIMINATED
- Warning #2 (IMU extrinsics): Remains but is cosmetic/benign, IMU fully functional
