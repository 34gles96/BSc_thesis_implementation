FROM ros:humble AS base

ARG USERNAME=base
ARG USER_UID=1000
ARG USER_GID=1000
ARG DEBIAN_FRONTEND=noninteractive

ENV DOCKER_RUNNING=true

# Ros environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DOMAIN_ID=0

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID >/dev/null 2>&1; then userdel $(id -un $USER_UID); fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

# Add sudo support.
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && apt-get upgrade -y \
    && rm -rf /var/lib/apt/lists/*

# Camera driver things, GStreamer & ROS vision tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-stereo-image-proc \
    ros-humble-camera-calibration \
    ros-humble-image-proc \
    && rm -rf /var/lib/apt/lists/*

# RTAB-Map (stereo SLAM, visual odometry)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rtabmap \
    ros-humble-rtabmap-msgs \
    ros-humble-rtabmap-odom \
    ros-humble-rtabmap-slam \
    ros-humble-rtabmap-sync \
    ros-humble-rtabmap-util \
    ros-humble-rtabmap-viz \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Convenience tools (rqt, pip, nano)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    nano \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-graph \
    ros-humble-rqt-image-view \
    && rm -rf /var/lib/apt/lists/* 

USER ${USERNAME}
# Configure ROS2 to be usable every time we open a terminal
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Symlink for the repo mount point
RUN ln -s /ADC_27 /home/${USERNAME}/ADC_27
WORKDIR /home/${USERNAME}/ADC_27

# BeamNG Ros2 Bridge install
FROM base AS beamng-ros2

# beamngpy dependencies
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-msgpack \
    python3-seaborn \
    && sudo rm -rf /var/lib/apt/lists/*

# BeamNG Ros2 Bridge install
FROM base AS beamng-ros2

# beamngpy dependencies
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-msgpack \
    python3-seaborn \
    git \
    && sudo rm -rf /var/lib/apt/lists/*

# Clone beamng-ros2-integration AND the missing radar_msgs source
RUN git clone https://github.com/BeamNG/beamng-ros2-integration.git /home/$USERNAME/beamng-ros2-ws/src/beamng-ros2-integration && \
    git clone https://github.com/ros-perception/radar_msgs.git -b ros2 /home/$USERNAME/beamng-ros2-ws/src/radar_msgs

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    sudo rosdep init; \
    fi

WORKDIR /home/$USERNAME/beamng-ros2-ws
RUN sudo apt-get update && \
    . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build && \
    sudo rm -rf /var/lib/apt/lists/*

RUN echo "source /home/$USERNAME/beamng-ros2-ws/install/setup.bash" >> /home/$USERNAME/.bashrc

# Install beamngpy 1.36 for BeamNG.tech 0.39 compatibility
# Régi sor:
# RUN python3 -m pip install beamngpy==1.36 python-rapidjson --no-deps

# ÚJ sor:
RUN python3 -m pip install "numpy<2" beamngpy==1.36 python-rapidjson jinja2 Pillow scipy

WORKDIR /home/${USERNAME}/ADC_27