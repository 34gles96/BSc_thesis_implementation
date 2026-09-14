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
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y

# camera driver things
RUN apt-get install -y aravis-tools
RUN apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
RUN apt-get install -y ros-humble-camera-aravis2
RUN apt-get install -y ros-humble-rosbag2-storage-mcap
RUN apt-get install -y ros-humble-stereo-image-proc
RUN apt-get install -y ros-humble-camera-calibration
RUN apt-get install -y ros-humble-image-proc
# RTAB-Map (stereo SLAM, visual odometry).
# Do not use ros-humble-rtabmap-ros: it depends on rtabmap_demos →
# turtlebot3_gazebo, which is often missing (arm64) and breaks the meta-package.
# Install only the stack we use in beamng_launch.py.
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

# For convinience (rqt, pip, nano)
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    nano \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-graph \
    ros-humble-rqt-image-view && \
    # To avoid conflicts with ROS2 numpy version
    pip3 uninstall -y numpy 

USER ${USERNAME}
# Configure ROS2 to be usable every time we open a terminal
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Symlink for the repo mount point
RUN ln -s /ADC_27 /home/${USERNAME}/ADC_27
WORKDIR /home/${USERNAME}/ADC_27

# BeamNG Ros2 Bridge install
FROM base AS beamng-ros2

# beamngpy dependencies
RUN sudo apt install -y \
    python3-colcon-common-extensions \
    python3-msgpack \
    python3-seaborn 

# install beamng-ros2-integration
RUN git clone https://github.com/BeamNG/beamng-ros2-integration.git /home/$USERNAME/beamng-ros2-ws/src/beamng-ros2-integration

# ÚJ:
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    sudo rosdep init; \
    fi

WORKDIR /home/$USERNAME/beamng-ros2-ws
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src -r -y && \
    colcon build

RUN . /home/$USERNAME/beamng-ros2-ws/install/setup.sh
RUN echo "source /home/$USERNAME/beamng-ros2-ws/install/setup.bash" >> /home/$USERNAME/.bashrc

# Install beamngpy 1.36 for BeamNG.tech 0.39 compatibility
RUN python3 -m pip install beamngpy==1.36 python-rapidjson --no-deps

WORKDIR /home/${USERNAME}/ADC_27