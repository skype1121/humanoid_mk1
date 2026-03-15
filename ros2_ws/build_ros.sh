#!/bin/bash

set -e

source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="/root/ros2_ws"

if [ ! -d "$WORKSPACE_DIR/src" ]; then
  WORKSPACE_DIR="$SCRIPT_DIR"
fi

cd "$WORKSPACE_DIR" || exit 1

echo "=== Building ROS workspace: $WORKSPACE_DIR ==="
colcon build --symlink-install

echo "=== Build finished ==="
echo "=== Run this next in the current shell ==="
echo "source $WORKSPACE_DIR/install/setup.bash"
