## Issue
Bluetooth can fail on car boot

## Summary
Sometimes when the Orange Pi 5b boots, the bluetooth won't work. Investigation points to it being because the car docker is starting and trying to use bluetooth before bluetooth starts, and it is messing things up (race condition)

AI suggested that the solution could be to create a service with appropriate dependencies that launches docker/start at the appropriate time. This seems to cause other problems with the docker not running properly.