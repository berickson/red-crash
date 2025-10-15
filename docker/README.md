# ROS2 Jazzy Docker Configuration

This directory contains the Docker configuration for building the ROS2 Jazzy environment for the red-crash robot.

## Building the Image

```bash
./build
```

This creates the `brianerickson/ros2-jazzy` image.

## Starting the Container

```bash
./start [mode]
```

Modes:
- `all` (default) - Launch all nodes via `launch_all.screenrc`
- `basic` - Launch basic nodes via `launch_basic.screenrc`
- `none` - Start container without launching nodes

For usage and container management, see the main README.md.

## Docker Image Configuration

### Base Image
- **ros:jazzy-perception** - Provides ROS2 Jazzy with perception packages

### Installed ROS2 Packages
- `ros-jazzy-rosbridge-suite` - Web bridge for ROS2
- `ros-jazzy-diagnostics` - System diagnostics
- `ros-jazzy-joy` - Joystick support
- `ros-jazzy-teleop-twist-joy` - Joystick teleoperation
- `ros-jazzy-navigation2` - Nav2 navigation stack
- `ros-jazzy-nav2-bringup` - Navigation launch files
- `ros-jazzy-slam-toolbox` - SLAM implementation
- `ros-jazzy-robot-localization` - Robot localization
- `ros-jazzy-depthai-ros` - Official OAK-D camera driver
- `ros-jazzy-depthai-bridge` - DepthAI bridge
- `ros-jazzy-depthai-descriptions` - Camera descriptions
- `ros-jazzy-foxglove-bridge` - Foxglove visualization

### Development Tools
- `python3-colcon-common-extensions` - ROS2 build tool
- `git`, `screen`, `htop`, `nano` - Development utilities

### Python Libraries
- `SpeechRecognition`, `gtts`, `PyAudio` - Speech capabilities
- `opencv-python`, `numpy` - Computer vision
- `psutil` - System monitoring
- `sox` - Audio playback (play command)

## Container Configuration

### Volume Mounts
- `~/red-crash:/root/ros2_ws` - ROS2 workspace (main)
- `~/red-crash/noetic/ws:/root/ws` - Legacy ROS1 workspace

### Device Access
- `/dev/ps3-joystick` - PlayStation controller
- `/dev/input/js0` - Generic joystick
- `/dev/roboclaw` - Motor controller
- `/dev/oak-d` - OAK-D stereo camera
- `/dev/lidar` - LiDAR sensor

### Container Features
- **Working directory**: `/root/ros2_ws`
- **Container name**: `car`
- **Restart policy**: `unless-stopped`
- **Network mode**: Host networking for rosbridge/web interfaces
- **Privileges**: Runs with `--privileged` for device access
- **Audio**: Added to audio group with ALSA support

### Screen Integration
The container uses GNU Screen to launch multiple ROS2 nodes in separate tabs. Screen configuration files (`launch_*.screenrc`) define which nodes to start automatically.

## Migration Support

This Docker environment supports parallel ROS1/ROS2 operation during migration:
- ROS2: `/root/ros2_ws` (primary workspace)
- ROS1: `/root/ws` (legacy, for reference)

Both workspaces are accessible for gradual migration. See `migration.md` for status.