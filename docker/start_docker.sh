docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/humanoid_mk1/ros2_ws:/root/ros2_ws \
  ros2-humble-dev
