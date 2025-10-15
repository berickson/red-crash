This project is in the process of migrating from ROS noetic to ROS 2 Jazzy.  Everything should be kept working during this migration.

**Project Structure:**
- **Root directory**: ROS2 Jazzy development (current/main)
- **noetic/ folder**: Legacy ROS Noetic content (will be removed after migration)

# Pre Migration Checks
Each folder in src should be checked for migratability and have a basic plan for each. Each plan should be kept to around under 15 lines

## speech_ros

location: noetic/ws/src/speech_ros

**Status:** Ready for migration - LOW complexity

**Package Type:** Pure Python (ament_python)

**Main Files:**
- `speech.py` - Speech recognition and text-to-speech node
- `command_interpreter.py` - Voice command processing node
- `say.py` - Test/utility script

**ROS Dependencies (All verified for ROS2 Jazzy):**
- `rclpy` - ROS2 Python client library (fully supported)
- `std_msgs` - Standard messages (v4.9.0, Quality Level 1)
- `diagnostic_msgs` - Diagnostics messages (v4.9.0, Quality Level 1)

**External Python Dependencies:**
- `speech_recognition` - Google Speech Recognition API
- `gtts` (Google Text-to-Speech) - TTS generation
- `pyaudio` - Microphone input

**Migration Plan:**
1. Convert package structure from catkin to ament_python
2. Update Python code: replace `rospy` with `rclpy`
   - `rospy.init_node()` → `rclpy.init()`
   - `rospy.Publisher()` → `node.create_publisher()`
   - `rospy.Subscriber()` → `node.create_subscription()`
   - `rospy.spin()` → `rclpy.spin(node)`
   - `rospy.loginfo()` → `node.get_logger().info()`
   - `rospy.get_param()` → `node.declare_parameter()` / `node.get_parameter()`
3. Update CMakeLists.txt → Remove (not needed for ament_python)
4. Update package.xml format 2 → format 3, change build type to ament_python
5. Create setup.py with entry points for nodes
6. Create requirements.txt or add Python dependencies to package.xml
7. Update launch files from .launch (XML) to Python launch files

**Estimated Effort:** 2-4 hours
**Risk Level:** Low - No custom messages, no C++ code, standard dependencies

## oakd

location: noetic/ws/src/oakd

**Status:** Replace with official ROS2 driver - MEDIUM complexity

**Package Type:** Custom Python node using DepthAI library

**Main Files:**
- `oakd.py` - Main camera node for OAK-D stereo camera with AI detection
- Camera calibration files: `left.yaml`, `right.yaml`, `rgb.yaml`
- Neural network model: `mobilenet-ssd/mobilenet-ssd.blob`

**Current Dependencies:**
- `roscpp`, `rospy`, `std_msgs` (ROS1 packages)
- `depthai` - Luxonis DepthAI Python API
- `cv_bridge` - ROS/OpenCV image conversion
- `sensor_msgs` - Camera and image messages
- OpenCV, NumPy for image processing

**Functionality:**
- Publishes RGB, left/right stereo images (raw + compressed)
- Generates depth maps and disparity images
- MobileNet-SSD object detection on RGB stream
- Camera calibration info publishing
- Manual focus control

**Migration Strategy: REPLACE with Official Driver**

**Recommended Approach:**
1. **Use Official DepthAI ROS2 Driver**: `depthai_ros_driver`
   - Available for ROS2 Humble, Iron, Jazzy
   - Install: `sudo apt install ros-jazzy-depthai-ros`
   - Full feature parity with current custom implementation

2. **Alternative: Build from Source** (if specific customizations needed)
   - Repository: `https://github.com/luxonis/depthai-ros`
   - Supports ROS2 Humble/Iron with Jazzy compatibility

**Official Driver Features:**
- Multiple pipeline types: RGB, RGBD, Stereo, Depth
- Neural network support (YOLO, MobileNet, Segmentation)
- Camera parameter configuration via YAML
- Launch files for various use cases
- Better performance and maintenance than custom implementation

**Migration Steps:**
1. Install official DepthAI ROS2 packages
2. Create configuration YAML files (port existing calibration)
3. Replace custom launch files with official driver launch
4. Update any dependent nodes to use standard topic names:
   - `/cameras/rgb/image_raw` → `/oak/rgb/image_raw`
   - `/cameras/stereo/depth` → `/oak/stereo/image_raw`
   - `/cameras/left/image_raw` → `/oak/left/image_raw`
   - `/cameras/right/image_raw` → `/oak/right/image_raw`
5. Configure neural network pipeline if object detection needed

**Benefits of Official Driver:**
- Regular updates and bug fixes from Luxonis
- Better integration with ROS2 ecosystem
- Support for newer OAK camera models
- Standardized interfaces and message types
- Performance optimizations

**Estimated Effort:** 4-8 hours (including testing and integration)
**Risk Level:** Medium - Topic name changes may affect dependent nodes, calibration data needs porting

**Notes:**
- Current package uses DepthAI v2 API (legacy)
- Official driver supports both DepthAI v2 and newer v3 API
- Migration is opportunity to leverage latest camera features

## [x] roboclaw_ros
done, replaced with a customized ros2_roboclaw_driver

## [x] docker
done, moved docker and screen system to ros2

## [x] joy_soundboard_ros

**Location:** `noetic/ws/src/joy_soundboard_ros` → `src/joy_soundboard_ros`

**Status:** ✅ MIGRATED - LOW complexity

**Package Type:** Pure Python (ament_python)

**Main Files:**
- `joy_soundboard.py` - Main node that plays sounds on joystick button press
- `sounds/` - Directory containing numbered sound files (e.g., `0-sound.mp3`, `1-sound.wav`)

**ROS Dependencies (All verified for ROS2 Jazzy):**
- `rclpy` - ROS2 Python client library (fully supported)
- `sensor_msgs` - Joy message (v4.9.0, Quality Level 1)

**External Dependencies:**
- `sox` - Command-line audio player (system package, no changes needed)
- Standard Python: `os`, `glob`

**Migration Plan:**
1. Create new package structure in `src/joy_soundboard_ros/`
2. Copy `sounds/` directory to new location
3. Convert Python code: replace `rospy` with `rclpy`
   - `rospy.init_node()` → `rclpy.init()` + `Node` class
   - `rospy.Subscriber()` → `self.create_subscription()`
   - `rospy.spin()` → `rclpy.spin(node)`
   - `RosPack().get_path()` → `get_package_share_directory()` (from `ament_index_python`)
4. Update `package.xml`: format 2 → format 3, add `ament_python` build type
5. Remove `CMakeLists.txt` (not needed for Python)
6. Create `setup.py` with entry point for `joy_soundboard` node
7. Create `resource/joy_soundboard_ros` marker file (ament requirement)
8. Add entry to `launch_all.screenrc` for the soundboard node
9. Test button press → sound playback functionality

**Estimated Effort:** 1-2 hours (✅ Completed in ~1 hour)
**Risk Level:** Low - Simple Python node, no custom messages, minimal ROS interaction

**Migration Completed:** October 15, 2025

**Build Command:**
```bash
docker exec car bash -c "cd /root/ros2_ws && colcon build --packages-select joy_soundboard_ros --symlink-install --paths src/joy_soundboard_ros"
```

**Run Command:**
```bash
docker exec car bash -c "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 run joy_soundboard_ros joy_soundboard"
```

**Notes:**
- Sound files can be copied as-is (format-agnostic)
- `sox` command-line tool works identically in ROS2 environment
- Button timing logic (1-second message age check) should be preserved
start time 6:51 pm, estimate 

# Migration Tracker

[x] Reorganize project structure (Jazzy to root, Noetic to legacy folder)
[x] Implement docker for use with ros2 jazzy in docker/
[x] Launch file for joystick
[x] Manually test and verify (Brian)
[x] Test with Foxglove studio
[x] Implement motor control for teleop (roboclaw)
[x] Manually test and verify roboclaw + joystick integration (Brian)
[x] Get soundboard working
[ ] Get speech module working
[ ] Get depthai-ros working