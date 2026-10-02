#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Default to 'humble' if ROS_DISTRO is not set in the environment
ROS_DISTRO=${ROS_DISTRO:-humble}

echo "Updating apt package index..."
sudo apt update

echo "========================================"
echo "Installing standard build tools..."
echo "========================================"
sudo apt install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    git

echo "========================================"
echo "Installing external C++ libraries (OpenCV, yaml-cpp)..."
echo "========================================"
sudo apt install -y \
    libopencv-dev \
    libyaml-cpp-dev


echo "========================================"
echo "Installing yaml lib"
echo "========================================"
sudo apt install -y libyaml-cpp-dev


echo "========================================"
echo "Installing python 3rd party packages"
pip install numpy scipy
echo "========================================"


echo "========================================"
echo "Done! All apt dependencies are installed."
echo "========================================"
echo "NOTE: The MATLAB Coder library  is a local"
echo "dependency. Ensure it is compiled and placed in:"
echo "./codegen/lib/"
