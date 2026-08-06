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
  pkill -INT -f "cloud_relay_(in|out)\.py" 2>/dev/null || true
  sleep 3
  for pid in "${PIDS[@]}"; do kill "${pid}" 2>/dev/null || true; done
  pkill -f "cloud_relay_(in|out)\.py" 2>/dev/null || true
  wait 2>/dev/null || true
  echo "[INFO] Shutdown complete."
}
trap cleanup SIGINT SIGTERM

# 镜像自检:叠加层没生效就早点炸,别等到现场查话题。
# Fail loudly here rather than during a field debug session if the M20 layer is
# missing from this image.
if ! ros2 pkg list | grep -qx "fixposition_driver_m20"; then
  echo "[ERROR] 镜像里没有 fixposition_driver_m20 —— 用 tools/build_m20_image.sh 构建的镜像吗?" >&2
  echo "[ERROR] fixposition_driver_m20 is not in this image; was it built by tools/build_m20_image.sh?" >&2
  exit 1
fi

# /LIDAR/POINTS 只存在于一个 127.0.0.1-only 的 OEM participant 上,驱动的多网卡
# participant 永远发现不了它(FastDDS 2.6 不再随真实网卡一起通告环回 locator)。
# 用一对管道桥接的中继把它搬到 /M20/LIDAR_POINTS —— 详见 cloud_relay_in.py 头注释。
# The OEM cloud writer lives on a loopback-only participant which the driver's
# multi-interface participant can never discover (FastDDS 2.6 stops announcing
# a loopback locator once real NICs exist). A pipe-bridged relay pair moves it
# to /M20/LIDAR_POINTS — see the header of cloud_relay_in.py.
echo "[STEP 1/3] /LIDAR/POINTS 环回中继 | loopback cloud relay..."
if [[ -f /cloud_relay_in.py && -f /cloud_relay_out.py && -f /fastdds_lo.xml ]]; then
  (
    while true; do
      FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_lo.xml python3 /cloud_relay_in.py 2>> /data/cloud_relay.log \
        | python3 /cloud_relay_out.py 2>> /data/cloud_relay.log
      echo "[WARN] relay pipeline exited at $(date -Is), respawning in 3 s" >> /data/cloud_relay.log
      sleep 3
    done
  ) &
  PIDS+=($!)
else
  echo "[WARN] relay 文件未挂载,/LIDAR/POINTS 将收不到(lidar 输入常 0)| relay files not mounted, lidar input stays dead" >&2
fi

echo "[STEP 2/3] Fixposition driver (M20) 启动中 | launching..."
ros2 launch /fixposition_config/node.launch \
  config:=/fixposition_config/config_m20.yaml \
  > /data/fixposition.log 2>&1 &
PIDS+=($!)
sleep 3

echo "[STEP 3/3] robot_state_publisher 启动中 | launching..."
robot_description="$(xacro /fixposition_config/robot.urdf.xacro)"
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="${robot_description}" \
  > /data/robot_state_publisher.log 2>&1 &
PIDS+=($!)

echo "[INFO] PIDs: ${PIDS[*]}  logs: /data/{fixposition,robot_state_publisher}.log"
wait
