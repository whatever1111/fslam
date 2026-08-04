#!/usr/bin/env bash
# ============================================================================
# run_m20_prod.sh —— M20 定位模式(驱动内完成三话题,无宿主机 Python 节点)
#                    M20 mode: the three OEM topics come out of the driver
# ============================================================================
# 容器内:Fixposition driver(含 M20 模块)+ robot_state_publisher。
# 宿主机:什么都不跑 —— fp_to_odom.py 和 motion_info_to_twist.py 的活儿都进了
#         驱动进程。run_fixposition_prod.sh(Python 版)保留作回退路径。
#
# In-container: the Fixposition driver with the M20 module, plus
# robot_state_publisher. Nothing runs on the host any more: what fp_to_odom.py
# and motion_info_to_twist.py did now happens inside the driver process. The
# Python deployment (run_fixposition_prod.sh) is kept as the fallback path.
#
# 镜像 | image: tools/build_m20_image.sh 构建的 fslam-m20:<arch>
#
# 环境变量覆盖 | environment overrides:
#   DOCKER_IMAGE CONTAINER_NAME DOCKER_MEM_LIMIT DOCKER_MEM_SWAP LOG_DIR
#   FIXPOSITION_CONFIG_DIR ROS_DOMAIN_ID FASTDDS_CONFIG
# 停止 | stop:
#   docker stop <container> && docker rm <container>
# ============================================================================
set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # 仓库根 | repo root
# shellcheck source=lib/deploy_common.sh
source "${BASE_DIR}/lib/deploy_common.sh"

DOCKER_IMAGE="${DOCKER_IMAGE:-fslam-m20:$(fslam_arch_tag)}"
CONTAINER_NAME="${CONTAINER_NAME:-m20-runtime}"
DOCKER_MEM_LIMIT="${DOCKER_MEM_LIMIT:-8g}"
DOCKER_MEM_SWAP="${DOCKER_MEM_SWAP:-10g}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

LOG_DIR="${LOG_DIR:-${BASE_DIR}/logs/m20}"
FIXPOSITION_CONFIG_DIR="${FIXPOSITION_CONFIG_DIR:-${BASE_DIR}/config/fixposition}"
ENTRYPOINT="${BASE_DIR}/container/entrypoint_m20.sh"
# DDS:默认 profile(全接口)。M20 模块要收 103 的 /LIDAR/POINTS、别的 CPU 的
# /IMU 与 /MOTION_INFO,还要把三话题发给导航 —— OEM 白名单 profile 只留 lo +
# 10.21.33.x,会把这些全挡掉。留空即用 FastDDS 默认。
# DDS: default profile (all interfaces). The M20 module has to receive
# /LIDAR/POINTS from CPU 103 and /IMU + /MOTION_INFO from the other units, and
# its three topics have to reach the navigation stack. The OEM whitelist
# profile (lo + 10.21.33.x only) would cut all of that off. Empty = defaults.
FASTDDS_CONFIG="${FASTDDS_CONFIG:-}"

# ---- 预检 | pre-checks ------------------------------------------------------
command -v docker >/dev/null 2>&1 || { echo "[ERROR] docker 不可用 | docker not available" >&2; exit 1; }
docker image inspect "${DOCKER_IMAGE}" >/dev/null 2>&1 \
  || { echo "[ERROR] 镜像不存在 | image not found: ${DOCKER_IMAGE}" >&2
       echo "[ERROR] 先构建 | build it first: tools/build_m20_image.sh" >&2; exit 1; }

for f in node.launch config_m20.yaml robot.urdf.xacro; do
  [[ -f "${FIXPOSITION_CONFIG_DIR}/${f}" ]] \
    || { echo "[ERROR] FP 配置缺失 | missing FP config: ${FIXPOSITION_CONFIG_DIR}/${f}" >&2; exit 1; }
done
[[ -f "${ENTRYPOINT}" ]] || { echo "[ERROR] 缺容器载荷 | missing entrypoint: ${ENTRYPOINT}" >&2; exit 1; }
ENTRYPOINT="$(realpath "${ENTRYPOINT}")"
if [[ -n "${FASTDDS_CONFIG}" && ! -f "${FASTDDS_CONFIG}" ]]; then
  echo "[WARN] fastdds 配置不存在 | fastdds profile not found: ${FASTDDS_CONFIG}" >&2
fi

mkdir -p "${LOG_DIR}"
LOG_DIR="$(realpath "${LOG_DIR}")"
FIXPOSITION_CONFIG_DIR="$(realpath "${FIXPOSITION_CONFIG_DIR}")"

# ---- 清理上一次部署 | stop any previous deployment --------------------------
# 也要停 Python 版容器:两者会抢同一批话题。
# Also stops the Python-mode container: the two would fight over the same topics.
fslam_stop_previous "${CONTAINER_NAME}" "${LOG_DIR}"

# Python 版部署必须一起停:fp_to_odom.py 也发 /ODOM 和 /LOCATION_STATUS,两套
# 同时跑等于给导航两个互相打架的位姿源。容器看门狗本来会回收宿主机节点,但重启
# 过的机器上它可能已经不在了,所以按 pid 文件再收一遍。
# The Python deployment has to go too: fp_to_odom.py also publishes /ODOM and
# /LOCATION_STATUS, and running both would leave navigation with two competing
# pose sources. Its container watcher normally reaps the host nodes, but it may
# not have survived a reboot, so reap them by pid file as well.
PY_LOG_DIR="${PY_LOG_DIR:-${BASE_DIR}/logs/fixposition_only}"
if docker ps -a --format '{{.Names}}' | grep -qx "fixposition-runtime"; then
  echo "[INFO] 停掉 Python 版部署 | stopping the Python deployment: fixposition-runtime"
  docker stop fixposition-runtime >/dev/null 2>&1 || true
  docker rm fixposition-runtime >/dev/null 2>&1 || true
fi
for f in fp_to_odom motion_info_bridge watcher; do
  fslam_stop_pidfile "${PY_LOG_DIR}/${f}.pid"
done
if pgrep -f "fp_to_odom.py" >/dev/null 2>&1; then
  echo "[WARN] fp_to_odom.py 仍在跑且没有 pid 文件,手动确认 | still running with no pid file, check it:" >&2
  pgrep -af "fp_to_odom.py" >&2 || true
fi

# ---- 起容器 | start the container -------------------------------------------
DOCKER_ARGS=(
  --name "${CONTAINER_NAME}"
  --network host --ipc=host --pid=host
  --restart unless-stopped
  --memory="${DOCKER_MEM_LIMIT}" --memory-swap="${DOCKER_MEM_SWAP}"
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  -v /dev:/dev
  -v /dev/shm:/dev/shm
  -v /etc/localtime:/etc/localtime:ro
  -v "${ENTRYPOINT}:/entrypoint.sh:ro"
  -v "${FIXPOSITION_CONFIG_DIR}:/fixposition_config:ro"
  -v "${LOG_DIR}:/data"
)
if [[ -n "${FASTDDS_CONFIG}" && -f "${FASTDDS_CONFIG}" ]]; then
  DOCKER_ARGS+=(-e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml -v "$(realpath "${FASTDDS_CONFIG}"):/fastdds.xml:ro")
fi

echo "[INFO] Starting ${CONTAINER_NAME} (${DOCKER_IMAGE})..."
docker run -d "${DOCKER_ARGS[@]}" "${DOCKER_IMAGE}" bash /entrypoint.sh

docker ps --filter "name=${CONTAINER_NAME}"
echo "[INFO] 已启动 | started. 日志 | logs: ${LOG_DIR}/"
echo "[INFO] 验证 | verify: 三话题同 stamp、/LOCATION_STATUS 2 Hz、total_status=1"
echo "[INFO] 停止 | stop: docker stop ${CONTAINER_NAME} && docker rm ${CONTAINER_NAME}"
