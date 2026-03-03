#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="jazzy-as3"

if ! docker ps -a -q -f "name=^/${CONTAINER_NAME}$" >/dev/null 2>&1; then
  echo "[stop] Container not found: ${CONTAINER_NAME}"
  exit 0
fi

if [ -z "$(docker ps -q -f "name=^/${CONTAINER_NAME}$")" ]; then
  echo "[stop] Container already stopped: ${CONTAINER_NAME}"
  exit 0
fi

echo "[stop] Stopping ROS processes in container..."
docker exec "${CONTAINER_NAME}" bash -lc '
  tmux kill-session -t as3 >/dev/null 2>&1 || true

  # extra safety: kill common processes if any remain
  pkill -f "ros2 launch simulation" >/dev/null 2>&1 || true
  pkill -f "Simulation\.x86_64" >/dev/null 2>&1 || true
  pkill -f "simulation/unity_ros" >/dev/null 2>&1 || true
  pkill -f "ros2 launch mapping_pkg" >/dev/null 2>&1 || true
  pkill -f "foxglove_bridge" >/dev/null 2>&1 || true

  ros2 daemon stop >/dev/null 2>&1 || true
  echo "[OK] ROS processes stopped."
' || true

echo "[stop] Stopping container..."
docker stop "${CONTAINER_NAME}" >/dev/null
echo "[OK] Container stopped: ${CONTAINER_NAME}"