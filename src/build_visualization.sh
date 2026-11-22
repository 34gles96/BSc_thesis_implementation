
#!/bin/bash

echo "=========================================="
echo "      ROS 2 Workspace Build Script        "
echo "=========================================="

# Get directory containing this script
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$WORKSPACE_DIR" || exit 1

echo "Workspace: $WORKSPACE_DIR"

# 1. Source ROS 2 Humble
ROS_DISTRO_PATH="/opt/ros/humble/setup.bash"

if [ -f "$ROS_DISTRO_PATH" ]; then
    echo "[1/3] Sourcing ROS 2 Humble..."
    source "$ROS_DISTRO_PATH"
else
    echo "[ERROR] ROS 2 setup.bash not found:"
    echo "$ROS_DISTRO_PATH"
    exit 1
fi

# 2. Build visualization package
echo "[2/3] Building visualization..."

colcon build \
    --symlink-install \
    --packages-select visualization

if [ $? -ne 0 ]; then
    echo "[ERROR] Build failed."
    exit 1
fi

# 3. Source workspace overlay
echo "[3/3] Sourcing workspace..."

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then

    source "$WORKSPACE_DIR/install/setup.bash"

    echo "============ Source the environment ================"
    echo "source /opt/ros/humble/setup.bash"
    echo "source $WORKSPACE_DIR/install/setup.bash"
    echo "===================================================="

    echo ""
    echo "Available executables:"
    ros2 pkg executables visualization

else
    echo "[ERROR] install/setup.bash not found."
    exit 1
fi

echo ""
echo "=========================================="
echo "      Build completed successfully        "
echo "=========================================="
