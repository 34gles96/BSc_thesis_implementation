#!/bin/bash
# Exit immediately if a command exits with a non-zero status
set -e

echo "=========================================="
echo "      ROS 2 Workspace Build Script        "
echo "=========================================="

# 1. Source ROS 2 Underlay (Humble/Rolling/Iron - adjust distro if needed)
ROS_DISTRO_PATH="/opt/ros/humble/setup.bash"

if [ -f "$ROS_DISTRO_PATH" ]; then
    echo "[1/3] Sourcing ROS 2 base environment ($ROS_DISTRO_PATH)..."
    source "$ROS_DISTRO_PATH"
else
    echo "[!] Warning: Main ROS 2 setup.bash not found at $ROS_DISTRO_PATH."
    echo "    Continuing with system environment..."
fi

# 2. Run colcon build
echo "[2/3] Building ROS 2 workspace packages..."
colcon build --symlink-install

# 3. Source Workspace Overlay
echo "[3/3] Sourcing local workspace setup..."
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "=========================================="
    echo " Workspace built & sourced successfully!  "
    echo "=========================================="
else
    echo "[!] Build failed or install/setup.bash missing."
    exit 1
fi