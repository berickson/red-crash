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
- `say.py` - Test/utility script (likely not needed in ROS2 version)

**ROS Dependencies (All verified for ROS2 Jazzy):**
- `rclpy` - ROS2 Python client library (fully supported)
- `std_msgs` - Standard messages (v4.9.0, Quality Level 1)
- `diagnostic_msgs` - Diagnostics messages (v4.9.0, Quality Level 1)

**External Python Dependencies:**
- `speech_recognition` - Google Speech Recognition API
- `gtts` (Google Text-to-Speech) - TTS generation
- `pyaudio` - Microphone input

**Detailed Migration Plan:**

1. **Create Package Structure**
   - Create `src/speech_ros/` directory
   - Create `src/speech_ros/speech_ros/` subdirectory for Python modules
   - Create `src/speech_ros/speech_ros/__init__.py`
   - Create `src/speech_ros/resource/speech_ros` (empty marker file)
   - Create `src/speech_ros/setup.cfg` (standard boilerplate)
   - Copy `secrets/` folder (maintain .gitignore for secrets.json)

2. **Create package.xml (format 3)**
   - Set build type to `ament_python`
   - Add buildtool_depend: `ament_python`
   - Add dependencies: `rclpy`, `std_msgs`, `diagnostic_msgs`
   - Add test dependencies (ament_copyright, ament_flake8, ament_pep257, python3-pytest)

3. **Create setup.py**
   - Define entry points for two nodes:
     - `speech = speech_ros.speech:main`
     - `command_interpreter = speech_ros.command_interpreter:main`
   - Include secrets folder in data_files (similar to sounds folder pattern)

4. **Migrate speech.py**
   - Convert to class-based node: `class SpeechNode(Node)`
   - Replace `rospy.init_node()` with node initialization in `__init__()`
   - Replace `rospy.Publisher()` with `self.create_publisher()`
   - Replace `rospy.Subscriber()` with `self.create_subscription()`
   - Replace `rospy.loginfo()` with `self.get_logger().info()`
   - Replace `rospy.get_param()` with `self.declare_parameter()` / `self.get_parameter()`
   - Replace `rospy.Time.now()` with `self.get_clock().now()`
   - Replace `rospy.Duration()` with `rclpy.duration.Duration()`
   - Replace `rospy.spin()` with `rclpy.spin(node)` in main()
   - Add proper main() function with rclpy.init/shutdown

5. **Migrate command_interpreter.py**
   - Convert to class-based node: `class CommandInterpreterNode(Node)`
   - Apply all rospy → rclpy conversions as above
   - Update diagnostics callback to handle ROS2 message structure
   - Update regex patterns and response logic (no changes needed)
   - Add proper main() function

6. **Create Launch File**
   - Create `launch/speech.launch.py`
   - Launch both `speech` and `command_interpreter` nodes
   - Declare parameter for speaker_volume_percent (default 35)

7. **Update launch_all.screenrc**
   - Add new screen for speech module
   - Add command: `ros2 launch launch/speech.launch.py`

8. **Build and Test**
   - Run `colcon build --symlink-install --packages-select speech_ros`
   - Test speech recognition (utterances published to /speech/utterances)
   - Test TTS (publish to /speech/say)
   - Test command interpreter wake word detection
   - Test diagnostics monitoring

9. **Documentation**
   - Update README if needed
   - Document any parameter changes
   - Note: say.py test script uses pyttsx3 instead of gtts - may not be needed

**Estimated Effort:** 2-4 hours
**Risk Level:** Low - No custom messages, no C++ code, standard dependencies

**Notes:**
- The package uses Google Speech Recognition API (no auth required for basic usage)
- secrets/ folder is for optional Google OAuth credentials
- Audio output uses `play` command (sox) - ensure installed in docker
- Background listening uses threading - should work similarly in ROS2

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
done

## speech_ros

# Migration Tracker

[x] Reorganize project structure (Jazzy to root, Noetic to legacy folder)
[x] Implement docker for use with ros2 jazzy in docker/
[x] Launch file for joystick
[x] Manually test and verify (Brian)
[x] Test with Foxglove studio
[x] Implement motor control for teleop (roboclaw)
[x] Manually test and verify roboclaw + joystick integration (Brian)
[x] Get soundboard working
[x] Migrate speech_ros package to ROS2
[x] Test speech_ros (install Python dependencies, test speech recognition and TTS)
[ ] Get depthai-ros working