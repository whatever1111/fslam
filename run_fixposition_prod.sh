#!/usr/bin/env bash
# ============================================================================
# run_fixposition_prod.sh —— fixposition-only 模式(纯 RTK,无 SLAM)
#                            fixposition-only mode (pure RTK, no SLAM)
# ============================================================================
# 容器内:Fixposition driver + robot_state_publisher。
# 宿主机:motion_info_to_twist.py(轮速 → FP 设备融合)
#         fp_to_odom.py(/fixposition/odometry_enu → /ODOM 中继,
#                        + /LOC_BODY_POINTS 去畸变点云 + /LOCATION_STATUS)。
#
# In-container: Fixposition driver + robot_state_publisher.
# On the host : motion_info_to_twist.py (wheel speed → FP device fusion) and
#               fp_to_odom.py (/fixposition/odometry_enu → /ODOM relay, plus
#               /LOC_BODY_POINTS deskewed cloud + /LOCATION_STATUS).
#
# 所有路径从本 checkout 推导,均可用环境变量覆盖 | all paths derive from this
# checkout and every one can be overridden via environment:
#   DOCKER_IMAGE CONTAINER_NAME DOCKER_MEM_LIMIT DOCKER_MEM_SWAP LOG_DIR
#   FIXPOSITION_CONFIG_DIR HOST_BRIDGE_SCRIPT FP_TO_ODOM_SCRIPT
#   CYCLONEDDS_CONFIG FASTDDS_CONFIG ROS_DOMAIN_ID ENABLE_MOTION_INFO_BRIDGE
#   FP_TO_ODOM_ARGS(透传给 fp_to_odom.py 的 --ros-args | extra ros-args)
# 停止 | stop:
#   docker stop <container> && docker rm <container>   (看门狗随之回收宿主机节点)
# ============================================================================
set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # 仓库根 | repo root
# shellcheck source=lib/deploy_common.sh
source "${BASE_DIR}/lib/deploy_common.sh"

DOCKER_IMAGE="${DOCKER_IMAGE:-wanderer123/fslam-humble:$(fslam_arch_tag)}"
CONTAINER_NAME="${CONTAINER_NAME:-fixposition-runtime}"
DOCKER_MEM_LIMIT="${DOCKER_MEM_LIMIT:-8g}"
DOCKER_MEM_SWAP="${DOCKER_MEM_SWAP:-10g}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

LOG_DIR="${LOG_DIR:-${BASE_DIR}/logs/fixposition_only}"
FIXPOSITION_CONFIG_DIR="${FIXPOSITION_CONFIG_DIR:-${BASE_DIR}/config/fixposition}"
HOST_BRIDGE_SCRIPT="${HOST_BRIDGE_SCRIPT:-${BASE_DIR}/host/motion_info_to_twist.py}"
FP_TO_ODOM_SCRIPT="${FP_TO_ODOM_SCRIPT:-${BASE_DIR}/host/fp_to_odom.py}"
CYCLONEDDS_CONFIG="${CYCLONEDDS_CONFIG:-${BASE_DIR}/config/dds/cyclonedds.xml}"
FASTDDS_CONFIG="${FASTDDS_CONFIG:-${BASE_DIR}/config/dds/fastdds.xml}"
ENTRYPOINT="${BASE_DIR}/container/entrypoint_fixposition.sh"   # 容器载荷 | container payload
ENABLE_MOTION_INFO_BRIDGE="${ENABLE_MOTION_INFO_BRIDGE:-1}"
FP_TO_ODOM_ARGS="${FP_TO_ODOM_ARGS:-}"

# ---- 预检 | pre-checks ------------------------------------------------------
command -v docker >/dev/null 2>&1 || { echo "[ERROR] docker 不可用 | docker not available" >&2; exit 1; }
docker image inspect "${DOCKER_IMAGE}" >/dev/null 2>&1 \
  || { echo "[ERROR] 镜像不存在 | image not found: ${DOCKER_IMAGE}  (docker pull 之)" >&2; exit 1; }

for f in node.launch config_fp_only.yaml robot.urdf.xacro; do
  [[ -f "${FIXPOSITION_CONFIG_DIR}/${f}" ]] \
    || { echo "[ERROR] FP 配置缺失 | missing FP config: ${FIXPOSITION_CONFIG_DIR}/${f}" >&2; exit 1; }
done
[[ -f "${FP_TO_ODOM_SCRIPT}" ]] || { echo "[ERROR] 缺 fp_to_odom | missing: ${FP_TO_ODOM_SCRIPT}" >&2; exit 1; }
[[ -f "${ENTRYPOINT}" ]] || { echo "[ERROR] 缺容器载荷 | missing entrypoint: ${ENTRYPOINT}" >&2; exit 1; }
ENTRYPOINT="$(realpath "${ENTRYPOINT}")"
if [[ "${ENABLE_MOTION_INFO_BRIDGE}" == "1" && ! -f "${HOST_BRIDGE_SCRIPT}" ]]; then
  echo "[ERROR] 缺桥脚本 | missing bridge script: ${HOST_BRIDGE_SCRIPT}" >&2; exit 1
fi
[[ -f "${CYCLONEDDS_CONFIG}" ]] || echo "[WARN] 缺 cyclonedds 配置 | missing: ${CYCLONEDDS_CONFIG}" >&2
[[ -f "${FASTDDS_CONFIG}" ]]    || echo "[WARN] 缺 fastdds 配置 | missing: ${FASTDDS_CONFIG}" >&2
# shellcheck disable=SC2034  # 由 fslam_start_host_node 消费 | consumed by fslam_start_host_node
HOST_ROS_SETUP="$(fslam_host_ros_setup)" \
  || { echo "[ERROR] 宿主机无 ROS2 环境 | no host ROS2 env under /opt/ros" >&2; exit 1; }

mkdir -p "${LOG_DIR}"
LOG_DIR="$(realpath "${LOG_DIR}")"
FIXPOSITION_CONFIG_DIR="$(realpath "${FIXPOSITION_CONFIG_DIR}")"
FP_TO_ODOM_SCRIPT="$(realpath "${FP_TO_ODOM_SCRIPT}")"
[[ "${ENABLE_MOTION_INFO_BRIDGE}" == "1" ]] && HOST_BRIDGE_SCRIPT="$(realpath "${HOST_BRIDGE_SCRIPT}")"

# ---- 清理上一次部署 | stop any previous deployment --------------------------
fslam_stop_previous "${CONTAINER_NAME}" "${LOG_DIR}" \
  "${LOG_DIR}/motion_info_bridge.pid" "${LOG_DIR}/fp_to_odom.pid"

# ---- 起容器 | start the container -------------------------------------------
# 容器载荷是随仓库的 entrypoint_fixposition.sh(只读挂载,restart 时重读)。
# The container payload is the checked-in entrypoint_fixposition.sh,
# mounted read-only and re-read on every container restart.
DOCKER_ARGS=(
  --name "${CONTAINER_NAME}"
  --network host --ipc=host --pid=host
  --restart unless-stopped
  --memory="${DOCKER_MEM_LIMIT}" --memory-swap="${DOCKER_MEM_SWAP}"
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  -v /dev:/dev
  -v /dev/shm:/dev/shm
  -v /etc/localtime:/etc/localtime:ro
  -v "${ENTRYPOINT}:/entrypoint.sh:ro"
  -v "${FIXPOSITION_CONFIG_DIR}:/fixposition_config:ro"
  -v "${LOG_DIR}:/data"
)
if [[ -f "${CYCLONEDDS_CONFIG}" ]]; then
  DOCKER_ARGS+=(-e CYCLONEDDS_URI=/cyclonedds.xml -v "$(realpath "${CYCLONEDDS_CONFIG}"):/cyclonedds.xml:ro")
fi

echo "[INFO] Starting ${CONTAINER_NAME} (${DOCKER_IMAGE})..."
docker run -d "${DOCKER_ARGS[@]}" "${DOCKER_IMAGE}" bash /entrypoint.sh

# ---- 宿主机节点 | host-side nodes -------------------------------------------
HOST_PID_FILES=()
if [[ "${ENABLE_MOTION_INFO_BRIDGE}" == "1" ]]; then
  fslam_start_host_node motion_info_bridge "${HOST_BRIDGE_SCRIPT}" "${LOG_DIR}"
  HOST_PID_FILES+=("${LOG_DIR}/motion_info_bridge.pid")
fi
# fp_to_odom 默认即 fixposition-only 模式(relay_enable=true)
# shellcheck disable=SC2086  # FP_TO_ODOM_ARGS 有意按词拆分 | intentional word-split
fslam_start_host_node fp_to_odom "${FP_TO_ODOM_SCRIPT}" "${LOG_DIR}" \
  ${FP_TO_ODOM_ARGS:+--ros-args ${FP_TO_ODOM_ARGS}}
HOST_PID_FILES+=("${LOG_DIR}/fp_to_odom.pid")

fslam_start_container_watcher "${CONTAINER_NAME}" "${LOG_DIR}" "${HOST_PID_FILES[@]}"

docker ps --filter "name=${CONTAINER_NAME}"
echo "[INFO] 已启动 | started. 日志 | logs: ${LOG_DIR}/"
echo "[INFO] 停止 | stop: docker stop ${CONTAINER_NAME} && docker rm ${CONTAINER_NAME}"
