# Red-Crash2 ROS2 Migration

This directory contains the ROS2 Jazzy migration workspace for the red-crash robotics project.

## Docker Setup

The Docker configuration provides a complete ROS2 Jazzy environment with all necessary dependencies for the red-crash robot.

### Building the Docker Image

```bash
cd docker
./build
```

This creates the `brianerickson/ros2-jazzy` image with:
- ROS2 Jazzy perception stack
- Navigation2 packages
- Official DepthAI OAK-D driver
- Speech recognition libraries
- Development tools

### Running the Docker Container

```bash
cd docker
./start
```

This starts the `car-ros2` container with:
- Working directory: `/root/ros2_ws`
- ROS2 workspace mounted: `~/red-crash/red-crash2 -> /root/ros2_ws`
- Legacy ROS1 workspace: `~/red-crash/ws -> /root/ws`
- Hardware device access: joystick, roboclaw, oak-d, lidar
- Host networking for rosbridge

### Container Management

**Start interactive session:**
```bash
docker exec -it car-ros2 /bin/bash
```

**Stop container:**
```bash
docker stop car-ros2
```

**Remove container:**
```bash
docker rm car-ros2
```

**View logs:**
```bash
docker logs car-ros2
```

## ROS2 Workspace Usage

### Building Packages

```bash
# Inside container
cd /root/ros2_ws
colcon build
source install/setup.bash
```

### Running ROS2 Commands

```bash
# List available nodes
ros2 node list

# Check topics
ros2 topic list

# Launch navigation
ros2 launch nav2_bringup navigation_launch.py

# Joystick control
ros2 launch teleop_twist_joy teleop-launch.py
```

### Development Workflow

1. **Edit code** on host in `~/red-crash/red-crash2/src/`
2. **Build inside container:**
   ```bash
   docker exec -it car-ros2 colcon build
   ```
3. **Source and test:**
   ```bash
   docker exec -it car-ros2 bash -c "source install/setup.bash && ros2 launch your_package your_launch.py"
   ```

## Migration Status

This workspace supports the ROS1 to ROS2 Jazzy migration:

- **Docker Environment**: Complete ROS2 Jazzy setup
- **Parallel Operation**: Runs alongside existing ROS1 container
- **Hardware Support**: All devices accessible (camera, motors, sensors)
- **Navigation**: Navigation2 stack ready for testing

## Troubleshooting

### Container Won't Start
Check device permissions and ensure devices exist:
```bash
ls -la /dev/oak-d /dev/roboclaw /dev/lidar /dev/ps3-joystick
```

### Build Issues
Clean and rebuild workspace:
```bash
docker exec -it car-ros2 bash -c "cd /root/ros2_ws && rm -rf build install log && colcon build"
```

### Network Issues
Verify host networking and rosbridge:
```bash
docker exec -it car-ros2 bash -c "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
```