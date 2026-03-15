#!/usr/bin/env bash
set -euo pipefail

xhost +local:

XSOCK=/tmp/.X11-unix
XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
HOST_UID="$(id -u)"
HOST_RUNTIME_DIR="/run/user/${HOST_UID}"
CONTAINER_RUNTIME_DIR="${HOST_RUNTIME_DIR}"

docker_args=(
  -it
  --rm
  --network host
  --user root
  -e "DISPLAY=${DISPLAY:-:0}"
  -e "XAUTHORITY=/root/.Xauthority"
  -e "QT_X11_NO_MITSHM=1"
  -v "${XAUTH}:/root/.Xauthority:ro"
  -v "${XSOCK}:${XSOCK}:rw"
  -v "${HOME}/humanoid_mk1/ros2_ws:/root/ros2_ws"
)

if [ -d "${HOST_RUNTIME_DIR}" ]; then
  docker_args+=(
    -e "XDG_RUNTIME_DIR=${CONTAINER_RUNTIME_DIR}"
    -v "${HOST_RUNTIME_DIR}:${CONTAINER_RUNTIME_DIR}:rw"
  )

  if [ -S "${HOST_RUNTIME_DIR}/bus" ]; then
    docker_args+=(
      -e "DBUS_SESSION_BUS_ADDRESS=unix:path=${CONTAINER_RUNTIME_DIR}/bus"
    )
  fi

  if [ -n "${WAYLAND_DISPLAY:-}" ] && [ -S "${HOST_RUNTIME_DIR}/${WAYLAND_DISPLAY}" ]; then
    docker_args+=(
      -e "WAYLAND_DISPLAY=${WAYLAND_DISPLAY}"
    )
  fi
fi

docker run "${docker_args[@]}" \
  ros2-humble-dev \
  bash -lc "source /opt/ros/humble/setup.bash && if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi && exec bash"
