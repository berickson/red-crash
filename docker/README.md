# ROS2 Jazzy Docker Setup for Red-Crash2

This directory contains the Docker configuration for running ROS2 Jazzy with the red-crash2 project.

## Quick Start

1. **Build the Docker image:**
   ```bash
   cd /home/pi/red-crash/red-crash2/docker
   ./build
   ```

2. **Start the container:**
   ```bash
   ./start
   ```

## Docker Configuration

### Base Image
- **ros:jazzy-perception** - Provides ROS2 Jazzy with perception packages

### Installed Packages

#### ROS2 Core Packages
- `ros-jazzy-rosbridge-suite` - Web bridge for ROS2
- `ros-jazzy-diagnostics` - System diagnostics
- `ros-jazzy-joy` - Joystick support
- `ros-jazzy-teleop-twist-joy` - Joystick teleoperation

#### Navigation Stack
- `ros-jazzy-navigation2` - Nav2 navigation stack
- `ros-jazzy-nav2-bringup` - Navigation launch files
- `ros-jazzy-slam-toolbox` - SLAM implementation
- `ros-jazzy-robot-localization` - Robot localization

#### Camera Support
- `ros-jazzy-depthai-ros` - Official OAK-D camera driver
- `ros-jazzy-depthai-bridge` - DepthAI bridge
- `ros-jazzy-depthai-descriptions` - Camera descriptions

#### Development Tools
- `python3-colcon-common-extensions` - ROS2 build tool
- `git`, `screen`, `htop`, `nano` - Development utilities

#### Python Libraries
- `SpeechRecognition`, `gtts`, `pyttsx3` - Speech capabilities
- `opencv-python`, `numpy` - Computer vision
- `psutil` - System monitoring

### Volume Mounts
- `~/red-crash/red-crash2:/root/ros2_ws` - ROS2 workspace
- `~/red-crash/ws:/root/ws` - Legacy ROS1 workspace (for transition)

### Device Access
- `/dev/ps3-joystick` - PlayStation controller
- `/dev/input/js0` - Generic joystick
- `/dev/roboclaw` - Motor controller
- `/dev/oak-d` - OAK-D stereo camera
- `/dev/lidar` - LiDAR sensor

## Container Features

### Workspace Setup
- **Working directory**: `/root/ros2_ws`
- **ROS2 sourcing**: Automatic setup in `.bashrc`
- **Build system**: colcon (ROS2 standard)

### Network Configuration
- **Host networking**: Direct access to host network interfaces
- **Port access**: Full access for rosbridge and web interfaces

### Audio Support
- **ALSA**: Audio system integration
- **Group membership**: Added to audio group
- **Device access**: `/dev` mounted for audio hardware

## Usage Notes

### Building ROS2 Packages
```bash
# Inside container
cd /root/ros2_ws
colcon build
source install/setup.bash
```

### Running ROS2 Nodes
```bash
# Example: Launch joystick node
ros2 launch teleop_twist_joy teleop-launch.py
```

### Screen usage
The container uses GNU Screen to launch multiple ROS2 nodes in separate tabs. Configuration files like `launch_all.screenrc` automatically start all nodes (joy, roboclaw, foxglove, soundboard, speech) in individual screens that you can switch between with Ctrl-A followed by the screen number.

### Parallel ROS1/ROS2 Operation
During migration, both ROS1 and ROS2 workspaces are accessible:
- ROS2: `/root/ros2_ws` (primary)
- ROS1: `/root/ws` (legacy, read-only access)

## Migration Status
This Docker setup supports the ROS2 Jazzy migration as outlined in `migration.md`. It provides:

✅ **Modern ROS2 base** - Jazzy perception stack  
✅ **Navigation2** - Replaces ROS1 move_base  
✅ **Official OAK-D driver** - Replaces custom implementation  
✅ **Development tools** - colcon, debugging utilities  
✅ **Parallel operation** - Can run alongside ROS1 container  

## Container Management

### Container Name
- **ROS2**: `car-ros2` (this container)
- **ROS1**: `car` (existing container)

Both containers can run simultaneously during migration.

### Restart Policy
- **unless-stopped**: Container restarts automatically unless explicitly stopped

### Troubleshooting

#### Device Access Issues
Ensure device symlinks exist on host:
```bash
ls -la /dev/oak-d /dev/roboclaw /dev/lidar
```

#### Permission Issues
Run container with `--privileged` flag (already configured).

#### Audio Issues
Check audio group membership and ALSA configuration:
```bash
# Inside container
groups
aplay -l
```