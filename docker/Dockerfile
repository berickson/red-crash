#FROM osrf/ros:melodic-desktop-full
FROM ros:melodic-perception
# RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y \
  git \
  screen \
  # because DeserializationError cannot deserialize: unknown error handler name 'rosmsg' 
  ros-melodic-genpy \
  ros-melodic-rosbridge-suite \
  ros-melodic-hector-*

RUN apt-get update && apt-get install -y \
  htop \
  nano \
  python-catkin-tools

#####################################

RUN apt-get update && apt-get install -y \
  python-pip \
  python3-pip \
  && pip install --upgrade pip \
  && python3 -m pip install --upgrade pip

RUN pip install psutil platformio

RUN python3 -m pip install numpy opencv-python depthai

RUN apt-get update && apt-get install -y \
  ros-melodic-rosserial-arduino \
  ros-melodic-rosserial-server \
  ros-melodic-rosserial-python \
  ros-melodic-rosserial-* \
  ros-melodic-tf2-web-republisher
  

  # ros-melodic-rosserial-embeddedlinux ros-kinetic-rosserial-windows ros-kinetic-rosserial-server ros-kinetic-rosserial-python


# Install special version of node because of conflict, see
# https://github.com/ros/rosdistro/issues/27302
RUN apt-get install -y nodejs=8.10.0~dfsg-2ubuntu0.2 nodejs-dev=8.10.0~dfsg-2ubuntu0.2 npm
#RUN apt-get install npm
RUN npm install http-server -g

RUN apt-get update && apt-get install -y \
    python3-catkin-pkg-modules \
    python3-rospkg-modules \
    ros-melodic-diagnostics \
    ros-melodic-joy \
    ros-melodic-joystick-drivers \
    ros-melodic-teleop-twist-joy

RUN python3 -m pip install pathlib

# special stuff for luxonis, see https://gist.github.com/atinfinity/c3255077e23164438c19544687e05b84

# RUN apt-get update && apt-get install -y --no-install-recommends \
#         sudo \
#         less \
#         emacs \
#         build-essential \
#         cmake \
#         git \
#         tmux \
#         bash-completion \
#         command-not-found \
#         software-properties-common \
#         xsel \
#         xdg-user-dirs \
#         wget \
#         curl \
#         usbutils \
#         udev \
#         && \
#     apt-get clean && \
#     rm -rf /var/lib/apt/lists/*

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
