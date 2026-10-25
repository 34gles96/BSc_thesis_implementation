
#!/bin/bash

set -e

# Directory where this script is located
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "======================================"
echo " Building MPC Controller"
echo " Workspace: $WORKSPACE_DIR"
echo "======================================"

# ROS 2 base environment
source /opt/ros/humble/setup.bash

# Build from workspace root
cd "$WORKSPACE_DIR"

colcon build \
    --symlink-install \
    --packages-select mpc_node

echo ""
echo "Build finished successfully."

# Source overlay inside this script
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then

    source "$WORKSPACE_DIR/install/setup.bash"

    echo ""
    echo "======================================"
    echo " MPC build environment is ready"
    echo "======================================"

    echo ""
    echo "Available MPC executables:"
    ros2 pkg executables mpc_node

    echo ""
    echo "To use the package in your CURRENT terminal, run:"
    echo "source /opt/ros/humble/setup.bash"
    echo "source $WORKSPACE_DIR/install/setup.bash"

else

    echo "[ERROR] $WORKSPACE_DIR/install/setup.bash not found."
    exit 1

fi
