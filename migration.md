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

## roboclaw_ros

location: noetic/ws/src/roboclaw_ros

**Status:** Replace with modern ROS2 implementation - MEDIUM complexity

**Package Type:** Custom Python driver using roboclaw_driver library

**Main Files:**
- `roboclaw_node.py` - Main motor controller node with odometry
- `roboclaw_driver.py` - Low-level serial communication driver (1213 lines)
- `roboclaw.launch` - Launch file with motor/robot parameters

**Current Dependencies (ROS1):**
- `rospy`, `roscpp`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `tf` 
- `diagnostic_msgs`, `diagnostic_updater` - Robot diagnostics
- Serial communication over USB/UART (/dev/ttyACM0)

**Functionality:**
- Differential drive motor control via `/cmd_vel` subscription
- Wheel encoder-based odometry publishing on `/odom`
- TF transforms between `odom` and `base_link` frames
- Motor current monitoring and battery voltage reporting
- Diagnostic status reporting and safety timeouts

**Migration Strategy: REPLACE with Modern ROS2 Implementation**

**Recommended Approach:**
1. **Primary Option**: Use `ros2_roboclaw_driver` by wimblerobotics
   - Mature C++ implementation with active maintenance
   - Repository: `https://github.com/wimblerobotics/ros2_roboclaw_driver`
   - Full feature parity with current Python implementation

2. **Alternative**: Use `roboclaw_hardware_interface` by dumbotics
   - Integrates with ROS2 Control framework
   - Repository: `https://github.com/dumbotics/roboclaw_hardware_interface`
   - Better for complex robot configurations

**Recommended Package Features:**
- **Motor Control**: Velocity commands, acceleration limiting, safety timeouts
- **Odometry**: Encoder-based position estimation with configurable parameters
- **Diagnostics**: Battery voltage, motor current, temperature monitoring
- **Configuration**: YAML-based parameter setup for robot dimensions
- **Safety**: Automatic motor stop on communication timeout

**Migration Steps:**
1. **Install ROS2 roboclaw driver**:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/wimblerobotics/ros2_roboclaw_driver.git
   ```

2. **Port configuration parameters** from current launch file:
   - `dev: /dev/ttyACM0` → `device_name: "/dev/ttyACM0"`
   - `baud: 115200` → `baud_rate: 115200`  
   - `address: 128` → `device_port: 128`
   - `max_speed: 2.0` → `max_linear_velocity: 2.0`
   - `ticks_per_meter: 4342.2` → `quad_pulses_per_meter: 4342`
   - `base_width: 0.315` → `wheel_separation: 0.315`

3. **Configure RoboClaw PID settings** using Motion Studio (Windows required):
   - Calibrate velocity PID for both motors (M1/M2)
   - Note QPPS (max speed) values for configuration
   - Export PID settings to driver configuration YAML

4. **Update dependent nodes**:
   - Same topic interfaces (`/cmd_vel`, `/odom`) - minimal changes needed
   - Diagnostic topics may have different message formats

**Benefits of Modern Implementation:**
- **Better Performance**: C++ implementation vs Python
- **Modern ROS2 Patterns**: Uses rclcpp, proper lifecycle management
- **Enhanced Safety**: More robust error handling and recovery
- **Active Development**: Regular updates and community support
- **Better Documentation**: Comprehensive setup guides

**Estimated Effort:** 6-10 hours (including PID calibration and testing)
**Risk Level:** Medium - Requires Windows for PID tuning, potential parameter differences

**Prerequisites:**
- Windows computer with IonMotion/Motion Studio for PID calibration
- RoboClaw firmware configured for packet serial mode
- Motor encoder direction and QPPS values determined

## docker

location: noetic/docker/ (legacy), docker/ (current ROS2)

**Status:** Update for ROS2 when needed - LOW complexity

**Package Type:** Docker containerization for ROS environment

**Main Files:**
- `Dockerfile` - Container definition based on ROS Noetic
- `build` - Build script that references back to docker directory
- `start` - Runtime script that mounts workspace from `~/red-crash/ws`

**Current Setup:**
- Base image: `ros:noetic-perception` 
- Workspace mounting: `-v ~/red-crash/ws:/root/ws`
- Device access: Joystick, RoboClaw, OAK-D, LiDAR via `/dev/*`
- Screen-based launch system with configurable launch files

**Migration Strategy: UPDATE WHEN NEEDED**

**Approach:**
- **Keep ROS1 container operational** during migration period
- **Create ROS2 container only when first ROS2 package is ready**
- **Maintain parallel operation** until migration is complete

**Future ROS2 Container Plan:**
1. **Base Image**: Change to `ros:jazzy-perception`
2. **Workspace Structure**: 
   - Mount: `-v ~/red-crash/ros2:/root/ros2_ws` 
   - Keep existing: `-v ~/red-crash/ws:/root/ws` for ROS1 compatibility
3. **Package Dependencies**: Replace ROS1 packages with ROS2 equivalents:
   - `ros-noetic-*` → `ros-jazzy-*`
   - `ros-noetic-rosbridge-suite` → `ros-jazzy-rosbridge-suite`
   - `ros-noetic-hector-*` → `ros-jazzy-hector-slam`
   - Navigation: `ros-jazzy-navigation2` instead of `ros-noetic-move-base`
4. **Launch System**: Update for ROS2 launch files when needed

**Build Script Pattern (maintain reference-back approach):**
```bash
#!/bin/bash
docker_dir=$(dirname $0)
pushd ${docker_dir}
docker build . -t brianerickson/ros2-jazzy
popd
```

**Runtime Script Pattern (dual workspace mounting):**
```bash
# Mount both workspaces for transition period
-v ~/red-crash/ws:/root/ws \
-v ~/red-crash/ros2:/root/ros2_ws \
```

**Key Principles:**
- **No premature Docker changes** - wait until ROS2 packages are ready
- **Maintain existing functionality** - current container must keep working
- **Reference-back pattern** - scripts use relative paths from docker folder
- **Incremental adoption** - add ROS2 support alongside existing ROS1

**Estimated Effort:** 2-3 hours (when first ROS2 package is ready)
**Risk Level:** Low - Existing container remains untouched until needed

# Migration Tracker

[x] Reorganize project structure (Jazzy to root, Noetic to legacy folder)
[x] Implement docker for use with ros2 jazzy in docker/
[x] Launch file for joystick
[ ] Manually test and verify (Brian)
[ ] Test with Foxglove studio
[ ] Implement motor control for teleop
[ ] Manually test and verify (Brian)