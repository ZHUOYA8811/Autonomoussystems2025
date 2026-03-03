#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="jazzy-as3"
IMAGE_NAME="osrf/ros:jazzy-desktop"
WORKSPACE_HOST="$HOME/AS/autonomous_system-group05"
WORKSPACE_CONT="/workspace"

# --- ROS settings (inside container) ---
ROS_DOMAIN_ID="42"
FOXGLOVE_PORT="9000"

# ---- helper ----
have_container() {
  docker ps -a -q -f "name=^/${CONTAINER_NAME}$" >/dev/null 2>&1 && \
  [ -n "$(docker ps -a -q -f "name=^/${CONTAINER_NAME}$")" ]
}

container_running() {
  [ -n "$(docker ps -q -f "name=^/${CONTAINER_NAME}$")" ]
}

# 允许 docker 访问 X server（只对本机有效）
xhost +local:docker >/dev/null 2>&1 || true

# 1) create container if not exists
if ! have_container; then
  echo "[start] Creating new container: ${CONTAINER_NAME}"
  docker run -dit \
    --name "${CONTAINER_NAME}" \
    --network host \
    --gpus all \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "${WORKSPACE_HOST}:${WORKSPACE_CONT}" \
    "${IMAGE_NAME}" \
    bash
else
  echo "[start] Container exists: ${CONTAINER_NAME}"
fi

# 2) start if stopped
if ! container_running; then
  echo "[start] Starting container..."
  docker start "${CONTAINER_NAME}" >/dev/null
fi

# 3) run everything inside container (tmux manages 3 sessions)
echo "[start] Launching simulation + mapping + foxglove_bridge (port ${FOXGLOVE_PORT}) ..."
docker exec -it "${CONTAINER_NAME}" bash -lc "
  set -e

  source /opt/ros/jazzy/setup.bash

  # prefer workspace overlay if present
  if [ -f ${WORKSPACE_CONT}/ros2_ws/install/setup.bash ]; then
    source ${WORKSPACE_CONT}/ros2_ws/install/setup.bash
  elif [ -f ${WORKSPACE_CONT}/install/setup.bash ]; then
    source ${WORKSPACE_CONT}/install/setup.bash
  fi

  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  unset ROS_LOCALHOST_ONLY

  # restart ros2 daemon for clean graph
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true

  # ensure tmux exists
  command -v tmux >/dev/null 2>&1 || (apt-get update && apt-get install -y tmux)

  # kill previous tmux session if exists
  tmux kill-session -t as3 >/dev/null 2>&1 || true
  tmux new-session -d -s as3 -n sim

  # Pane 1: simulation
  tmux send-keys -t as3:sim \"cd ${WORKSPACE_CONT} && ros2 launch simulation simulation.launch.py\" C-m

  # Pane 2: mapping
  tmux new-window -t as3 -n map
  tmux send-keys -t as3:map \"cd ${WORKSPACE_CONT}/ros2_ws && ros2 launch mapping_pkg mapping.launch.py\" C-m

  # Pane 3: foxglove bridge
  tmux new-window -t as3 -n fox
  tmux send-keys -t as3:fox \"apt-get update && apt-get install -y ros-jazzy-foxglove-bridge && ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=${FOXGLOVE_PORT}\" C-m

  echo
  echo \"[OK] Started in tmux session: as3\"
  echo \"     - sim:  ros2 launch simulation simulation.launch.py\"
  echo \"     - map:  ros2 launch mapping_pkg mapping.launch.py\"
  echo \"     - fox:  ros2 launch foxglove_bridge ... port:=${FOXGLOVE_PORT}\"
  echo
  echo \"Attach:   tmux attach -t as3\"
  echo \"Detach:  Ctrl+b then d\"
  echo
  exec bash
"