#!/bin/bash

source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="/root/ros2_ws"

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  source "$WORKSPACE_DIR/install/setup.bash"
elif [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
  source "$SCRIPT_DIR/install/setup.bash"
fi
