# OAK-D Camera Migration Plan

## Overview
Replace custom noetic OAK-D node with official DepthAI ROS2 driver for better maintainability and feature support.

## Current State
- Location: `noetic/ws/src/oakd/`
- Custom Python node using legacy DepthAI v2 API
- Publishes RGB, stereo, depth, and object detection topics
- Uses MobileNet-SSD for object detection

## Target State
- Use official `depthai-ros` ROS2 driver
- Standard topic names and message types
- Modern DepthAI API with better performance
- Maintained by Luxonis with regular updates

## Implementation Steps

### 1. Install Official Driver (IN DOCKER)
```bash
# Option A: Binary install (preferred)
apt install ros-jazzy-depthai-ros-driver ros-jazzy-depthai-ros-msgs

# Option B: Build from source if customization needed
cd /root/ros2_ws/src
git clone https://github.com/luxonis/depthai-ros.git
cd ..
colcon build --symlink-install --packages-select depthai_ros_driver
```

### 2. Create Configuration File
- Create `config/oakd_config.yaml` in red-crash root
- Port camera calibration from existing `left.yaml`, `right.yaml`, `rgb.yaml`
- Configure neural network pipeline for object detection
- Set topic names to match existing or update dependent nodes

### 3. Create Launch File
- Create `launch/oakd.launch.py`
- Load configuration from `config/oakd_config.yaml`
- Enable RGB, stereo, depth pipelines
- Configure MobileNet-SSD or YOLO detection if needed

### 4. Update Topic Names
**Current topics → New topics mapping:**
- `/cameras/rgb/image_raw` → `/oak/rgb/image_raw`
- `/cameras/left/image_raw` → `/oak/left/image_raw`
- `/cameras/right/image_raw` → `/oak/right/image_raw`
- `/cameras/stereo/depth` → `/oak/stereo/image_raw`
- `/cameras/detections` → `/oak/nn/detections`

**Action:** Search codebase for any nodes subscribing to old topics and update

### 5. Update Launch System
- Add OAK-D launch to appropriate screenrc file (launch_all.screenrc)
- Command: `ros2 launch launch/oakd.launch.py`
- Test that camera streams are visible in Foxglove

### 6. Verify Neural Network
- Copy or reference `mobilenet-ssd.blob` model file
- Or use built-in models from depthai-ros
- Test object detection output

### 7. Build and Test
```bash
# If building from source
docker exec -it car bash -c "source /opt/ros/jazzy/setup.bash && cd /root/ros2_ws && colcon build --symlink-install"

# Test camera streams
ros2 topic list | grep oak
ros2 topic echo /oak/rgb/image_raw --once

# View in Foxglove Studio
# Check RGB, depth, stereo images
# Verify object detection if enabled
```

### 8. Cleanup
- Keep `noetic/ws/src/oakd/` until fully verified
- Document any parameter or feature differences
- Update README if needed

## Key Lessons from Previous Migrations

1. **Source ROS before building**: Always `source /opt/ros/jazzy/setup.bash` in docker
2. **Use symlink-install**: Enables faster iteration with Python changes
3. **Keep old code until verified**: Don't delete legacy code until new version fully tested
4. **Test incrementally**: Verify each step before moving to next
5. **Update launch system**: Don't forget to add to screenrc files
6. **Document changes**: Note any API or topic name changes for future reference

## Dependencies to Verify
- DepthAI Python library (should come with ros-jazzy-depthai-ros)
- OpenCV (already available)
- camera_info_manager (for calibration)

## Estimated Effort
4-6 hours (including testing and integration)

## Risk Level
MEDIUM
- Topic name changes may affect downstream nodes
- Neural network configuration may differ
- Camera calibration needs proper porting

## Success Criteria
- [x] Official depthai-ros driver installed in docker
- [x] Configuration file created with proper calibration
- [x] Launch file working and camera streaming
- [x] All image topics publishing correctly
- [x] Object detection working (verified /oak/nn/spatial_detections exists )
- [x] Added to launch_all.screenrc  
- [x] Verified in Foxglove Studio
- [x] No errors in logs during operation
- [x] Upgrade camera, test and xplaPlay with IMU

## Notes
- Official driver is more actively maintained than custom implementation
- Supports newer OAK camera models for future upgrades
- Better integration with ROS2 ecosystem
- Performance improvements over legacy DepthAI v2 API
