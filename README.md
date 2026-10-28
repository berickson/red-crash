# Red-Crash ROS2 Jazzy

This is the ROS2 Jazzy workspace for the red-crash robotics project.

## Prerequisites

Install Git LFS (required for voice model files):
```bash
sudo apt-get install git-lfs
git lfs install
```

Then clone normally - LFS files will be downloaded automatically:
```bash
git clone https://github.com/berickson/red-crash.git
```

## Quick Start

### 1. Build the Docker Image

```bash
cd docker
./build
```

This creates the `brianerickson/ros2-jazzy` image with ROS2 Jazzy, Navigation2, DepthAI drivers, and speech recognition.

### 2. Start the Container

Before starting the container, make sure all ports on the USB hub are switched on.

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

## Connecting the 8BitDo Pro 3 Controller

1. Plug the 2.4 GHz adapter into a USB port on the Raspberry Pi.
2. Set the Mode switch on the back of the controller to S/X (all the way to
   the right). S/X is the only mode that works with the 2.4 GHz adapter.
3. Press the controller's Start button.

## Controller Keymap

![8BitDo Pro 3 button layout](media/8bitdo-pro3-buttons.png)

The face buttons use the Nintendo layout: X top, A right, B bottom, Y left.
The Star button is on the left below the D-pad, and the Home button (8BitDo
logo) is on the right below the face buttons. L4/R4 sit on the back top edge,
next to the triggers, and PL/PR are the back paddles.

The joystick launch config maps the left stick's horizontal axis to turning and
its vertical axis to forward/reverse movement. Hold B to enable normal-speed
movement (0.5 linear / 0.05 turning). Hold A to enable fast/turbo movement
(1.5 linear / 0.1 turning); A enables movement on its own and does not require
B to be held.

Hold D-pad Down for push-to-talk speech recognition (`ptt_button_index: 12`).

The soundboard plays the clip whose filename starts with the `/joy` button
index (files in `src/joy_soundboard_ros/sounds/`, named
`<index>-<button>-<text>`). After adding or renaming a clip, rebuild
`joy_soundboard_ros` so the installed copy picks it up. Mapping verified on the
8BitDo Pro 3 in Switch mode (it shows up as a Nintendo Pro Controller), which
`joy_node` reports using the SDL game controller layout:

| Button | `/joy` index | Action |
| --- | --- | --- |
| A | 0 | Turbo drive enable |
| B | 1 | Normal drive enable |
| X | 2 | Says "no" |
| Y | 3 | Says "yes" |
| Select (−) | 4 | Says "no" |
| Home | 5 | Says "You're welcome" |
| Start (+) | 6 | Says "I don't know very much, I'm just a robot after all" |
| L3 (left stick click) | 7 | Says "I don't know" |
| R3 (right stick click) | 8 | Introduces itself as Ivy, the Halloween Pumpkin Robot |
| L1 | 9 | Plays the Mario Kart race-start sound |
| R1 | 10 | Says "You can take a candy from my head" |
| D-pad Up | 11 | Says "Happy Halloween" |
| D-pad Down | 12 | Push-to-talk |
| D-pad Left | 13 | — |
| D-pad Right | 14 | Says "You have taken the forbidden candy" |
| Star | 15 | — |

D-pad Left (13) and Star (15) are free for new clips.

In S/X mode the controller acts like a Nintendo Switch Pro Controller, which
has no extra buttons, so L4, R4, PL, PR and Profile never appear in `/joy`.
To use L4, R4, PL or PR, map it on the controller to copy another button: hold
the extra button and the target button, then press Star (e.g. hold PR + D-pad
Left, press Star). Hold the extra button and press Star to clear its mapping.
L2 and R2 are analog axes (4 and 5; 1.0 released, -1.0 fully pressed), not
buttons, so they cannot trigger soundboard clips. To check the mapping, run
`ros2 topic echo /joy` and press buttons. Speech recognition and voice
commands run separately from these controller sound effects.

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