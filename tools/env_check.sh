echo "===== OS ====="
lsb_release -a 2>/dev/null || cat /etc/os-release

echo
echo "===== ARCH ====="
uname -m

echo
echo "===== PYTHON ====="
python3 --version
which python3

echo
echo "===== PIP ====="
python3 -m pip --version

echo
echo "===== ROS2 ====="
printenv | grep ROS_DISTRO
which ros2

echo
echo "===== COLCON ====="
which colcon

echo
echo "===== PIP FREEZE ====="
python3 -m pip freeze | sort

echo
echo "===== APT ROS PACKAGES ====="
dpkg -l | grep ros-humble | sort

echo
echo "===== CUDA ====="
nvcc --version 2>/dev/null || echo "nvcc not found"
dpkg -l | grep -E 'cuda|tensorrt|nvinfer' | sort

echo
echo "===== ENV ====="
printenv | sort | grep -E 'ROS|AMENT|COLCON|PYTHON|CUDA'
