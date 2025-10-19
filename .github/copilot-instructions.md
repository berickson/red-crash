# Copilot Instructions (repo-specific)

1. Use ASCII text only and no emoticons.
2. Stick to the user's explicit request; do not implement extra features without asking.
3. Ask for explicit confirmation before marking checklist items complete (e.g., "May I now mark <item> as complete?").
4. Never modify completion status ([x] or [ ]) in any document without explicit user permission.

---

Repository-specific guidance

- Container & workspaces
  - Development happens inside a Docker container named `car` (see `docker/start` and `docker/README.md`).
  - Primary ROS2 workspace: `/root/ros2_ws` (host mount: `~/red-crash`).
  - Legacy ROS1 workspace (for migration reference): `/root/ws` (host mount: `~/red-crash/noetic/ws`).

- Common commands (run inside `car`)
  - Build full workspace: `cd /root/ros2_ws && colcon build --symlink-install`
  - Build single package: `cd /root/ros2_ws && colcon build --symlink-install --packages-select <pkg>`
  - Clean and rebuild: `rm -rf build install log && colcon build`

- Launch & run patterns
  - The container uses GNU Screen to start runtime nodes. Screen configs are at repo root: `launch_all.screenrc`, `launch_basic.screenrc`, `launch_none.screenrc`.
  - Top-level launch files are under `launch/`. Individual packages may also include their own `launch` directories.
  - Example: run a single launch from host in container:

    docker exec -it car bash -c "source /root/ros2_ws/install/setup.bash && ros2 launch <package> <launch.py>"

- Devices & permissions
  - Devices commonly accessed: `/dev/ps3-joystick`, `/dev/input/js0`, `/dev/roboclaw`, `/dev/oak-d`, `/dev/lidar`.
  - If a node can't access hardware, confirm device nodes exist on the host and that the container was started with the expected privileges (see `docker/start`).

- Migration context
  - This repository is actively migrating from ROS Noetic to ROS2 Jazzy. Consult `migration.md`, `noetic/`, and package-level READMEs for migration notes and examples.

- Conventions & patterns
  - Edit code on the host (under `~/red-crash`), build and test inside the `car` container.
  - Prefer incremental builds with `--symlink-install` for faster iteration.
  - GNU Screen is used for multi-node sessions; use `Ctrl-A` to switch or detach.
  - Logs and build artifacts are under `log/`, `build/`, and `install/` at the repo root.

- Files to inspect when working here
  - `docker/` — Docker build/start scripts and image config
  - `launch_*screenrc` and `launch/` — runtime startup scripts and launch files
  - `install/` — local setup helpers (`setup.sh`, `local_setup.*`)
  - `noetic/ws/src/` — legacy ROS1 packages (for migration reference)
  - `src/ros2_roboclaw_driver/README.md` and `src/ros2_roboclaw_driver/config/` — example package config and driver usage

- How an AI coding agent should behave in this repo
  - Preserve existing instructions; merge rather than overwrite valuable content.
  - Run builds and runtime tests inside the `car` container where devices and environment are configured.
  - Use `colcon build --symlink-install` for iterative development.
  - When suggesting changes, reference exact file paths and show minimal patches (use repository tools to apply patches).

If you'd like more rules (coding style, CI, test harnesses), tell me what to include and I'll update this file.
# Copilot Instructions

1. Use ASCII text only and no emoticons
2. Never go beyond what was explicitly asked for - stick to the specific request
3. Get explicit confirmation before marking an item as complete by saying "May I now mark xxx as complete?"
4. NEVER modify completion status ([x] or [ ]) in any document without explicit user permission
5. There is a docker named car that has the ROS2 workspace at /root/ros2_ws
6. Use colcon build --symlink-install to build packages within the ROS2 docker container
example docker exec car bash -c "cd /root/ros2_ws && colcon build --symlink-install --packages-select ros2_roboclaw_driver"
