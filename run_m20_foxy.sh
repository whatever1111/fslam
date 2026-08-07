#!/usr/bin/env bash
# ============================================================================
# run_m20_foxy.sh —— M20 定位模式(Foxy 原生,无容器)
#                    M20 mode, native on Foxy — no container
# ============================================================================
# 驱动进程自己发 /ODOM、/LOC_BODY_POINTS、/LOCATION_STATUS,并直接吃 /MOTION_INFO。
# 和 run_m20_prod.sh(Humble 容器版)是二选一,别同时跑 —— 两套都发 /ODOM。
#
# The driver publishes the three OEM topics itself and consumes /MOTION_INFO
# directly. Mutually exclusive with run_m20_prod.sh (the Humble container build)
# and with the Python deployment: all of them publish /ODOM.
#
# 不设 FASTRTPS_DEFAULT_PROFILES_FILE —— 用狗自己的默认 DDS 配置。Foxy 原生进程
# 就能看到 OEM 那个 127.0.0.1-only 的点云 writer,这正是走原生的理由。
# Deliberately does NOT set FASTRTPS_DEFAULT_PROFILES_FILE: the robot's own default
# applies. A native Foxy process can see the OEM's loopback-only cloud writer, which
# is the entire reason this path exists.
#
# 环境变量 | environment overrides:
#   WS ROS_SETUP FIXPOSITION_CONFIG_DIR LOG_DIR ROS_DOMAIN_ID
# 停止 | stop: systemctl stop m20_loc_foxy   (或 kill 掉 pid 文件里的进程)
# ============================================================================
set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/deploy_common.sh
source "${BASE_DIR}/lib/deploy_common.sh"

WS="${WS:-/home/user/m20_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
SDK_PREFIX="${SDK_PREFIX:-${WS}/fpsdk}"
FIXPOSITION_CONFIG_DIR="${FIXPOSITION_CONFIG_DIR:-${BASE_DIR}/config/fixposition}"
LOG_DIR="${LOG_DIR:-${BASE_DIR}/logs/m20_foxy}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ---- 预检 | pre-checks ------------------------------------------------------
[[ -f "${ROS_SETUP}" ]] || { echo "[ERROR] 没找到 ROS2 | ROS 2 not found: ${ROS_SETUP}" >&2; exit 1; }
[[ -f "${WS}/install/setup.bash" ]] \
  || { echo "[ERROR] 工作区没编 | workspace not built: ${WS}" >&2
       echo "[ERROR] 先编译 | build it first: tools/build_m20_foxy.sh --source <driver checkout>" >&2; exit 1; }
for f in node.launch config_m20.yaml robot.urdf; do
  [[ -f "${FIXPOSITION_CONFIG_DIR}/${f}" ]] \
    || { echo "[ERROR] 配置缺失 | missing config: ${FIXPOSITION_CONFIG_DIR}/${f}" >&2; exit 1; }
done

mkdir -p "${LOG_DIR}"
LOG_DIR="$(realpath "${LOG_DIR}")"

# ---- 停掉别的部署 | stop the other deployments -------------------------------
# 容器版(Humble)和 Python 版都发 /ODOM,同时跑会给导航两个打架的位姿源。
# The container (Humble) and Python deployments also publish /ODOM; running any two
# would leave navigation with competing pose sources.
if command -v docker >/dev/null 2>&1; then
  for c in m20-runtime fixposition-runtime; do
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${c}"; then
      echo "[INFO] 停掉容器部署 | stopping container deployment: ${c}"
      docker stop "${c}" >/dev/null 2>&1 || true
    fi
  done
fi
for f in fp_to_odom motion_info_bridge watcher; do
  fslam_stop_pidfile "${BASE_DIR}/logs/fixposition_only/${f}.pid"
done
fslam_stop_pidfile "${LOG_DIR}/driver.pid"

# ---- 起驱动 | start the driver ----------------------------------------------
# ROS 的 setup.bash 会引用未定义变量(AMENT_TRACE_SETUP_FILES 等),在 set -u 下
# 直接把脚本打死,所以 source 前后要临时关掉 -u。
# ROS's setup.bash references unbound variables (AMENT_TRACE_SETUP_FILES and
# friends), which is fatal under set -u — relax it just around the sourcing.
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u
export LD_LIBRARY_PATH="${SDK_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

echo "[INFO] 启动 M20 驱动(Foxy 原生)| starting the M20 driver (native Foxy)..."
ros2 launch "${FIXPOSITION_CONFIG_DIR}/node.launch" \
  config:="${FIXPOSITION_CONFIG_DIR}/config_m20.yaml" \
  > "${LOG_DIR}/fixposition.log" 2>&1 &
DRIVER_PID=$!
echo "${DRIVER_PID}" > "${LOG_DIR}/driver.pid"

# robot_state_publisher:用预展开的 URDF —— 狗上 Foxy 没装 xacro,而且没有公网可装。
# robot_state_publisher: uses the pre-expanded URDF — the robot's Foxy has no xacro
# package and no internet to install one.
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat "${FIXPOSITION_CONFIG_DIR}/robot.urdf")" \
  > "${LOG_DIR}/robot_state_publisher.log" 2>&1 &
echo "$!" > "${LOG_DIR}/rsp.pid"

sleep 5
if ! kill -0 "${DRIVER_PID}" 2>/dev/null; then
  echo "[ERROR] 驱动起来就死了,看日志 | driver died on startup, see ${LOG_DIR}/fixposition.log" >&2
  tail -20 "${LOG_DIR}/fixposition.log" >&2 || true
  exit 1
fi

echo "[INFO] 已启动 | started (pid ${DRIVER_PID}). 日志 | logs: ${LOG_DIR}/"
echo "[INFO] 验证 | verify: ros2 topic hz /ODOM; ros2 topic echo --once /LOCATION_STATUS"
echo "[INFO] 关键检查 | key check: input_val.lidar 应为 1 | should be 1"
wait "${DRIVER_PID}"
