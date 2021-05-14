#FROM osrf/ros:noetic-desktop-full
FROM ros:noetic-perception
# RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y \
  git \
  screen \
  # because DeserializationError cannot deserialize: unknown error handler name 'rosmsg' 
  ros-noetic-genpy \
  ros-noetic-rosbridge-suite \
  ros-noetic-hector-*

RUN apt-get update && apt-get install -y \
  htop \
  nano \
  python3-catkin-tools

#####################################

RUN apt-get update && apt-get install -y \
  python3-pip \
  && python3 -m pip install --upgrade pip

RUN pip install psutil platformio

RUN python3 -m pip install numpy opencv-python
RUN python3 -m pip install depthai==2.1.0.0.dev0+df16f1724b2969896fd39c086978da1da816a1c0 --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-snapshot-local

RUN apt-get update && apt-get install -y \
  ros-noetic-rosserial-arduino \
  #ros-noetic-rosserial-server \
  ros-noetic-rosserial-python \
  ros-noetic-rosserial-* 
  #ros-noetic-tf2-web-republisher
  

  # ros-noetic-rosserial-embeddedlinux ros-kinetic-rosserial-windows ros-kinetic-rosserial-server ros-kinetic-rosserial-python


# Install special version of node because of conflict, see
# https://github.com/ros/rosdistro/issues/27302
#RUN apt-get install -y nodejs=8.10.0~dfsg-2ubuntu0.2 nodejs-dev=8.10.0~dfsg-2ubuntu0.2 npm
RUN apt-get install -y nodejs npm

#RUN apt-get install npm
RUN npm install http-server -g

RUN apt-get update && apt-get install -y \
    python3-catkin-pkg-modules \
    python3-rospkg-modules \
    ros-noetic-diagnostics \
    ros-noetic-joy \
    ros-noetic-joystick-drivers \
    ros-noetic-teleop-twist-joy \
    sox \
    libsox-fmt-mp3 \
    alsa-utils \
    python3-pyaudio \
    python3-espeak \
    ffmpeg \
    flac

RUN python3 -m pip install pathlib SpeechRecognition pyttsx3 gtts catkin_tools

# Mapping

RUN apt-get update --fix-missing && \
    apt-get install -y \
    ros-noetic-gmapping \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-move-base \
    ros-noetic-global-planner \
    ros-noetic-dwa-local-planner \
    ros-noetic-vision-msgs

# special stuff for luxonis, see https://gist.github.com/atinfinity/c3255077e23164438c19544687e05b84

RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo \
        less \
        emacs \
        build-essential \
        cmake \
        git \
        tmux \
        bash-completion \
        command-not-found \
        software-properties-common \
        xsel \
        xdg-user-dirs \
        wget \
        curl \
        usbutils \
        udev \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

  RUN wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/noetic-devel/install_dependencis.sh | bash; return 0

# # https://github.com/luxonis/depthai-docker/blob/master/Dockerfile-depthai#L11
# RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules

# USER $USERNAME
# WORKDIR /home/${USERNAME}
# SHELL ["/bin/bash", "-c"]
# RUN sudo wget -qO- http://docs.luxonis.com/_static/install_dependencies.sh | bash
# RUN git clone https://github.com/luxonis/depthai.git -b v0.4.0.0 && \
#     cd depthai && \
#     python3 install_requirements.py
# RUN wget https://github.com/libusb/libusb/releases/download/v1.0.24/libusb-1.0.24.tar.bz2 && \
#     tar xf libusb-1.0.24.tar.bz2 && \
#     cd libusb-1.0.24 && \
#     ./configure --disable-udev --prefix="$PWD/install_dir" && \
#     make -j && \
#     make install
# ENV LD_LIBRARY_PATH="$PWD/libusb-1.0.24/install_dir/lib":$LD_LIBRARY_PATH


# #
# ######################################

# #RUN apt-get install -y \
# #  ros-noetic-xv-11-laser-driver
  
# # commenting out for now because it takes a long time
# #RUN git clone https://github.com/RainerKuemmerle/g2o.git
# #RUN cd g2o && mkdir build && cd build && cmake -j4 .. && make && make install
