#!/usr/bin/env python3

import depthai as dai

device = dai.Device()

imuVersion = device.getConnectedIMU()
imuFirmwareVersion = device.getIMUFirmwareVersion()
embeddedIMUFirmwareVersion = device.getEmbeddedIMUFirmwareVersion()

print(f"IMU type: {imuVersion}")
print(f"IMU firmware version: {imuFirmwareVersion}")
print(f"Embedded IMU firmware version: {embeddedIMUFirmwareVersion}")

# Also check calibration
calib = device.readCalibration()
print(f"\nCalibration data available: {calib is not None}")
if calib:
    print(f"IMU to camera extrinsics available: {calib.getImuToCameraExtrinsics is not None}")
