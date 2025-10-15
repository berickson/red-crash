# Red-Crash ROS2 Jazzy

This is the ROS2 Jazzy workspace for the red-crash robotics project.

## Quick Start

### 1. Build the Docker Image

```bash
cd docker
./build
```

This creates the `brianerickson/ros2-jazzy` image with ROS2 Jazzy, Navigation2, DepthAI drivers, and speech recognition.

### 2. Start the Container

```bash
cd docker
./start
```

This starts the `car` container with:
- Working directory: `/root/ros2_ws`
- ROS2 workspace: `~/red-crash -> /root/ros2_ws`
- Hardware device access: joystick, roboclaw, oak-d, lidar
- Host networking for rosbridge
- GNU Screen launching all nodes automatically

### Container Management

**Start interactive session:**
```bash
docker exec -it car /bin/bash
```

**Stop container:**
```bash
docker stop car
```

**Remove container:**
```bash
docker rm car
```

**View logs:**
```bash
docker logs car
```

## Screen Usage

The container uses GNU Screen to launch multiple ROS2 nodes in separate tabs. Configuration files like `launch_all.screenrc` automatically start all nodes (joy, roboclaw, foxglove, soundboard, speech) in individual screens that you can switch between with Ctrl-A followed by the screen number.

**Screen commands:**
- `Ctrl-A 0-5` - Switch to screen 0-5
- `Ctrl-A "` - List all screens
- `Ctrl-A d` - Detach from screen session

## ROS2 Workspace Usage

### Building Packages

```bash
# Inside container
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Running ROS2 Commands

```bash
# List available nodes
ros2 node list

# Check topics
ros2 topic list

# View topic data
ros2 topic echo /joy

# Launch files
ros2 launch launch/joy.launch.py
ros2 launch launch/roboclaw.launch.py
ros2 launch launch/speech.launch.py
```

### Development Workflow

1. **Edit code** on host in `~/red-crash/src/`
2. **Build inside container:**
   ```bash
   docker exec car bash -c "cd /root/ros2_ws && colcon build --symlink-install --packages-select your_package"
   ```
3. **Source and test:**
   ```bash
   docker exec -it car bash -c "source install/setup.bash && ros2 launch your_package your_launch.py"
   ```

## Migration Status

This workspace is actively migrating from ROS Noetic to ROS2 Jazzy. See `migration.md` for detailed status.

## Troubleshooting

### Container Won't Start
Check device permissions and ensure devices exist:
```bash
ls -la /dev/oak-d /dev/roboclaw /dev/lidar /dev/ps3-joystick
```

### Build Issues
Clean and rebuild workspace:
```bash
docker exec car bash -c "cd /root/ros2_ws && rm -rf build install log && colcon build"
```

### Network Issues
Verify host networking and rosbridge:
```bash
docker exec car bash -c "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
```