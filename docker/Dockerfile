FROM ros:jazzy-perception

# Install system dependencies including Python development tools
RUN apt-get update && apt-get install -y \
  git \
  screen \
  htop \
  nano \
  curl \
  wget \
  usbutils \
  udev \
  build-essential \
  cmake \
  python3-pip \
  python3-venv \
  python3-dev \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-rosdep \
  python3-vcstool \
  && rm -rf /var/lib/apt/lists/*

# Install ROS2 Jazzy packages
RUN apt-get update && apt-get install -y \
  ros-jazzy-rosbridge-suite \
  ros-jazzy-diagnostics \
  ros-jazzy-joy \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro \
  && rm -rf /var/lib/apt/lists/*

# Install Navigation2 stack
RUN apt-get update && apt-get install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  && rm -rf /var/lib/apt/lists/*

# Install audio and speech dependencies (prefer system packages)
RUN apt-get update && apt-get install -y \
  sox \
  libsox-fmt-mp3 \
  alsa-utils \
  python3-pyaudio \
  python3-espeak \
  ffmpeg \
  flac \
  python3-numpy \
  python3-opencv \
  python3-serial \
  python3-yaml \
  python3-requests \
  && rm -rf /var/lib/apt/lists/*

# Install espeak-ng for phonemization (required by piper)
RUN apt-get update && apt-get install -y \
  espeak-ng \
  && rm -rf /var/lib/apt/lists/*

# Install Python packages that aren't available via apt (using --break-system-packages for container)
RUN python3 -m pip install --break-system-packages \
  psutil \
  SpeechRecognition \
  pyttsx3 \
  gtts \
  depthai \
  depthai-pipeline-graph \
  openai

# Install piper-tts (includes embedded espeak-ng for phonemization) and sounddevice
RUN python3 -m pip install --break-system-packages \
  piper-tts \
  sounddevice

# Install DepthAI for ROS2 (official driver)
RUN apt-get update && apt-get install -y \
  ros-jazzy-depthai-ros \
  ros-jazzy-depthai-bridge \
  ros-jazzy-depthai-descriptions \
  && rm -rf /var/lib/apt/lists/*

# Install Node.js and http-server for web interface
RUN apt-get update && apt-get install -y nodejs npm && \
  npm install http-server -g && \
  rm -rf /var/lib/apt/lists/*

# Setup USB rules for OAK-D camera
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' > /etc/udev/rules.d/80-movidius.rules

# Create workspace directory
RUN mkdir -p /root/ros2_ws/src

# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi" >> /root/.bashrc
RUN echo "if [ -f /root/ros2_ws/secrets.env ]; then source /root/ros2_ws/secrets.env; fi" >> /root/.bashrc

# Install foxglove_bridge and ament packages for building (temporary location for fast rebuild - TODO: move to ROS2 packages section)
RUN apt-get update && apt-get install -y \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-ament-cmake \
  ros-jazzy-ament-cmake-python \
  ros-jazzy-rclcpp \
  ros-jazzy-rclcpp-components \
  && rm -rf /var/lib/apt/lists/*

# Set default working directory
WORKDIR /root/ros2_ws