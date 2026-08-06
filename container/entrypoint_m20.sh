#!/bin/bash
# ============================================================================
# entrypoint_m20.sh —— M20 定位模式容器载荷 | container payload for M20 mode
# ============================================================================
# 由 run_m20_prod.sh 只读挂载为 /entrypoint.sh 后执行,不要在容器外跑。
# Mounted read-only at /entrypoint.sh by run_m20_prod.sh; not meant to run
# outside the container.
#
# 与 entrypoint_fixposition.sh 的区别:驱动自己发 /ODOM、/LOC_BODY_POINTS、
# /LOCATION_STATUS 并直接吃 /MOTION_INFO,所以宿主机那两个 Python 节点没了。
# Unlike entrypoint_fixposition.sh, the driver itself publishes the three OEM
# topics and consumes /MOTION_INFO directly, so the two host-side Python nodes
# are gone.
#
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

# 镜像自检:叠加层没生效就早点炸,别等到现场查话题。
# Fail loudly here rather than during a field debug session if the M20 layer is
# missing from this image.
# 不能用 `ros2 pkg list | grep -q`:pipefail 下 grep -q 提前退出会让 ros2 收到
# BrokenPipe 而"失败",导致这条检查随机误报(实测踩中)。
# Not `ros2 pkg list | grep -q`: under pipefail, grep -q's early exit gives ros2
# a BrokenPipeError and the pipeline randomly "fails" (bitten in production).
if ! ros2 pkg prefix fixposition_driver_m20 > /dev/null 2>&1; then
  echo "[ERROR] 镜像里没有 fixposition_driver_m20 —— 用 tools/build_m20_image.sh 构建的镜像吗?" >&2
  echo "[ERROR] fixposition_driver_m20 is not in this image; was it built by tools/build_m20_image.sh?" >&2
  exit 1
fi

echo "[STEP 1/2] Fixposition driver (M20) 启动中 | launching..."
ros2 launch /fixposition_config/node.launch \
  config:=/fixposition_config/config_m20.yaml \
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
