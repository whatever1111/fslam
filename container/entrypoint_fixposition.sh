#!/bin/bash
# ============================================================================
# entrypoint_fixposition.sh —— fixposition-only 模式容器载荷
#                              container payload for fixposition-only mode
# ============================================================================
# 由 run_fixposition_prod.sh 只读挂载为 /entrypoint.sh 后执行,不要在容器外跑。
# Mounted read-only at /entrypoint.sh by run_fixposition_prod.sh; not meant to
# run outside the container.
# 挂载 | mounts: /fixposition_config(driver 配置,ro)  /data(日志 | logs)
# ============================================================================
set -o pipefail
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

PIDS=()
cleanup() {
  echo ""
  echo "[INFO] Shutting down..."
  for pid in "${PIDS[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
  sleep 3
  for pid in "${PIDS[@]}"; do kill "${pid}" 2>/dev/null || true; done
  wait 2>/dev/null || true
  echo "[INFO] Shutdown complete."
}
trap cleanup SIGINT SIGTERM

echo "[STEP 1/2] Fixposition driver 启动中 | launching..."
ros2 launch /fixposition_config/node.launch \
  config:=/fixposition_config/config_fp_only.yaml \
  > /data/fixposition.log 2>&1 &
PIDS+=($!)
sleep 3

echo "[STEP 2/2] robot_state_publisher 启动中 | launching..."
robot_description="$(xacro /fixposition_config/robot.urdf.xacro)"
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="${robot_description}" \
  > /data/robot_state_publisher.log 2>&1 &
PIDS+=($!)

echo "[INFO] PIDs: ${PIDS[*]}  logs: /data/{fixposition,robot_state_publisher}.log"
wait
