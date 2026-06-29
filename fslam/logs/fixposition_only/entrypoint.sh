#!/bin/bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

PIDS=()
cleanup() {
  echo ""
  echo "[INFO] Shutting down..."
  for pid in "${PIDS[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  sleep 3
  for pid in "${PIDS[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  echo "[INFO] Shutdown complete."
}
trap cleanup SIGINT SIGTERM

echo "[STEP 0/2] Launching Fixposition driver..."
ros2 launch /data/fixposition_config/node.launch \
  config:=/data/fixposition_config/config_fp_only.yaml \
  > /data/fixposition.log 2>&1 &
PIDS+=($!)
sleep 3

echo "[STEP 1/2] Launching robot_state_publisher..."
robot_description="$(xacro /data/fixposition_config/robot.urdf.xacro)"
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="${robot_description}" \
  > /data/robot_state_publisher.log 2>&1 &
PIDS+=($!)

echo "[INFO] Started PIDs: ${PIDS[*]}"
echo "[INFO] Logs: /data/fixposition.log /data/robot_state_publisher.log"

wait
