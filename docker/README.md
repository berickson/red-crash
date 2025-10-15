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


### Screen usage
The container uses GNU Screen to launch multiple ROS2 nodes in separate tabs. Configuration files like `launch_all.screenrc` automatically start all nodes (joy, roboclaw, foxglove, soundboard, speech) in individual screens that you can switch between with Ctrl-A followed by the screen number.

### Parallel ROS1/ROS2 Operation
During migration, both ROS1 and ROS2 workspaces are accessible:
- ROS2: `/root/ros2_ws` (primary)
- ROS1: `/root/ws` (legacy, read-only access)

## Migration Status

This project is currently being ported from Noetic to Jazzy, See `migration.md`.


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