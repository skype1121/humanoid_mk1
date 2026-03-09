#!/bin/bash

source /opt/ros/humble/setup.bash
cd /root/ros2_ws || exit 1

echo "=== Building ROS workspace ==="
colcon build --symlink-install
BUILD_EXIT=$?

if [ $BUILD_EXIT -ne 0 ]; then
  echo "=== Build failed ==="
  return $BUILD_EXIT 2>/dev/null || exit $BUILD_EXIT
fi

echo "=== Build finished ==="
echo "=== Run this next in the current shell ==="
echo "source /root/ros2_ws/install/setup.bash"
