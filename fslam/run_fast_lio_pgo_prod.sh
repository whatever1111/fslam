#!/usr/bin/env bash
# ============================================================================
# run_fast_lio_pgo_prod.sh —— fslam 生产部署 · 云深处 M20 机器狗
#                             fslam production deploy · Deep Robotics M20 dog
# ============================================================================
# 用 CI/CD 发布镜像 wanderer123/fslam-humble:{arm64,amd64} 在实机拉起完整链路：
#   ① 容器内先启 Fixposition driver（开启 RTK/FPA 输出：rawimu/imubias/odomenu）
#   ② 等到 /fixposition/fpa/odomenu 出现后，经镜像内四象限 run_prod_native.sh
#      --profile m20 拉起 canonical 管线（prod 模式，/odom @100Hz）
#   ③ 可选 /odom → /ODOM 别名中继（狗上导航栈按旧 fslam 约定消费 /ODOM）
#   ④ 可选宿主机 motion_info 桥（/MOTION_INFO → /fixposition/motion_info_twist，
#      喂 FP 设备内部轮速融合；需狗主机 ROS2 + drdds 消息，容器内无法运行）
#
# Runs the full chain on the robot with the CI/CD release image:
#   ① Fixposition driver first (brings up the RTK/FPA streams), ② then the canonical
#   pipeline via the IN-IMAGE four-quadrant run_prod_native.sh --profile m20,
#   ③ optional /odom→/ODOM alias relay, ④ optional host-side motion-info bridge.
#
# 旧版脚本的 fp_imu_relay / rename_pointcloud_field / ecef_to_enu_bridge 手工链
# 已被 canonical 适配器链（config/profiles/m20 + 三层配置）取代，不再需要。
# The legacy fp_imu_relay / field-rename / ecef bridge chain is fully replaced by
# the canonical adapter chain (config/profiles/m20 + 3-layer config).
#
# 用法 | usage:
#   tools/run_fast_lio_pgo_prod.sh [选项]
# 选项 | options:
#   --profile <name>     profile（默认 m20）| default m20
#   --image <img>        镜像（默认 wanderer123/fslam-humble:<按 uname -m 选 arm64/amd64>）
#   --name <n>           容器名（默认 fslam-runtime）| container name
#   --mem/--swap <sz>    内存/交换上限（默认 24g/28g）| memory/swap limits
#   --domain <id>        ROS_DOMAIN_ID（默认 0，实机部署）| default 0
#   --fp-config <yaml>   宿主机 Fixposition 驱动配置（默认用镜像内 config/fixposition/m20.yaml）
#                        host FP driver yaml (default: the baked-in image copy)
#   --config-dir <dir>   用宿主机 config/ 目录（含 profiles/）只读覆盖镜像内配置。
#                        缺省自动发现：脚本同级或上一级的 config/（即部署仓库 checkout
#                        自带的配置树）；都没有才用镜像内烘焙副本。
#                        overlay-mount a host config/ tree over the baked-in one.
#                        Auto-discovered by default: ./config or ../config next to this
#                        script (the deploy-repo checkout's tree); falls back to the
#                        baked-in image copy when absent.
#   --fp-stream <uri>    覆盖 FP 传感器流地址（如 tcpcli://10.21.31.66:21000）| override stream URI
#   --no-fixposition     不启驱动（RTK 已由别处提供）| skip the driver (RTK provided elsewhere)
#   --odom-alias <t>     /odom 别名话题（默认 /ODOM；空串禁用）| alias topic ("" disables)
#   --foxglove           开 Foxglove 监控 | enable the Foxglove monitor
#   --bridge-port <p>    Foxglove bridge 端口（默认 8765）
#   --param-overlay <f>  在线改参 overlay（最高优先级层）| highest-priority param overlay
#   --foreground         前台运行（--rm，Ctrl-C 停；默认后台 -d --restart unless-stopped）
#                        run attached (--rm); default detached with restart policy
#   --check              对运行中的容器做链路体检（逐话题测频 + 关键日志签名），
#                        第一个无数据的话题即断点 | health-check a RUNNING deployment:
#                        per-topic rate probe + log signatures; first silent topic = the break
# 环境 | env:
#   LOG_DIR                     日志/数据目录（默认 ~/fslam/logs；须可跨重启持久，
#                               容器 restart 会重读其中的 entrypoint）| must persist across reboots
#   RMW_IMPLEMENTATION          默认 rmw_cyclonedds_cpp（与旧 fslam 实机部署一致）
#   HOST_BRIDGE_SCRIPT          motion_info_to_twist.py 路径（默认 /home/user/fslam/motion_info_to_twist.py）
#   ENABLE_MOTION_INFO_BRIDGE   1/0/auto（默认 auto=脚本存在则启）| default auto
# ============================================================================
#!/usr/bin/env bash
source /opt/ros/foxy/setup.bash 
set -euo pipefail

# 获取当前脚本所在目录的绝对路径，实现任意位置运行
BASE_DIR="$(dirname "$(realpath "$0")")"

DOCKER_IMAGE="${DOCKER_IMAGE:-wanderer123/fslam-humble:arm64}"
DOCKER_MEM_LIMIT="${DOCKER_MEM_LIMIT:-8g}"
DOCKER_MEM_SWAP="${DOCKER_MEM_SWAP:-10g}"
CONTAINER_NAME="${CONTAINER_NAME:-fixposition-runtime}"

# 将硬编码的路径替换为基于 BASE_DIR 的相对路径
LOG_DIR="${LOG_DIR:-${BASE_DIR}/logs/fixposition_only}"                   # 对应原 /home/user/fslam/fslam/logs/...
FIXPOSITION_CONFIG_DIR="${FIXPOSITION_CONFIG_DIR:-${BASE_DIR}/../fixposition}"       # 对应原 /home/user/fslam/fixposition
HOST_BRIDGE_SCRIPT="${HOST_BRIDGE_SCRIPT:-${BASE_DIR}/../motion_info_to_twist.py}"   # 对应原 /home/user/fslam/motion_info_to_twist.py
FP_TO_ODOM_SCRIPT="${FP_TO_ODOM_SCRIPT:-${BASE_DIR}/../fp_to_odom.py}"             # 对应原 /home/user/fslam/fp_to_odom.py
CYCLONEDDS_CONFIG="${CYCLONEDDS_CONFIG:-${BASE_DIR}/../cyclonedds.xml}"            # 对应原 /home/user/fslam/cyclonedds.xml
FASTDDS_CONFIG="${FASTDDS_CONFIG:-${BASE_DIR}/../fastdds.xml}"

ENABLE_MOTION_INFO_BRIDGE="${ENABLE_MOTION_INFO_BRIDGE:-1}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
HOST_BRIDGE_PID_FILE=""
HOST_BRIDGE_WATCHER_PID_FILE=""
FP_TO_ODOM_PID_FILE=""

if ! docker image inspect "${DOCKER_IMAGE}" >/dev/null 2>&1; then
  echo "[ERROR] Docker image not found: ${DOCKER_IMAGE}" >&2
  exit 1
fi

for f in node.launch config_fp_only.yaml robot.urdf.xacro; do
  if [[ ! -f "${FIXPOSITION_CONFIG_DIR}/${f}" ]]; then
    echo "[ERROR] Missing Fixposition config file: ${FIXPOSITION_CONFIG_DIR}/${f}" >&2
    exit 1
  fi
done

if [[ "${ENABLE_MOTION_INFO_BRIDGE}" == "1" && ! -f "${HOST_BRIDGE_SCRIPT}" ]]; then
  echo "[ERROR] Missing host bridge script: ${HOST_BRIDGE_SCRIPT}" >&2
  exit 1
fi

if [[ ! -f "${FP_TO_ODOM_SCRIPT}" ]]; then
  echo "[ERROR] Missing fp_to_odom script: ${FP_TO_ODOM_SCRIPT}" >&2
  exit 1
fi

if [[ ! -f "${CYCLONEDDS_CONFIG}" ]]; then
  echo "[WARNING] Missing cyclonedds config: ${CYCLONEDDS_CONFIG}" >&2
fi

mkdir -p "${LOG_DIR}"
LOG_DIR="$(realpath "${LOG_DIR}")"
FIXPOSITION_CONFIG_DIR="$(realpath "${FIXPOSITION_CONFIG_DIR}")"
if [[ "${ENABLE_MOTION_INFO_BRIDGE}" == "1" ]]; then
  HOST_BRIDGE_SCRIPT="$(realpath "${HOST_BRIDGE_SCRIPT}")"
fi
FP_TO_ODOM_SCRIPT="$(realpath "${FP_TO_ODOM_SCRIPT}")"

HOST_BRIDGE_PID_FILE="${LOG_DIR}/motion_info_bridge.pid"
HOST_BRIDGE_WATCHER_PID_FILE="${LOG_DIR}/motion_info_bridge_watcher.pid"
FP_TO_ODOM_PID_FILE="${LOG_DIR}/fp_to_odom.pid"

cleanup_host_bridge() {
  if [[ -f "${HOST_BRIDGE_PID_FILE}" ]]; then
    local pid
    pid="$(<"${HOST_BRIDGE_PID_FILE}")"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
      sleep 2
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
    rm -f "${HOST_BRIDGE_PID_FILE}"
  else
    pkill -f "python3 ${HOST_BRIDGE_SCRIPT}" >/dev/null 2>&1 || true
  fi
}

cleanup_fp_to_odom() {
  if [[ -f "${FP_TO_ODOM_PID_FILE}" ]]; then
    local pid
    pid="$(<"${FP_TO_ODOM_PID_FILE}")"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
      sleep 2
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
    rm -f "${FP_TO_ODOM_PID_FILE}"
  else
    pkill -f "python3 ${FP_TO_ODOM_SCRIPT}" >/dev/null 2>&1 || true
  fi
}

cleanup_host_bridge_watcher() {
  if [[ -f "${HOST_BRIDGE_WATCHER_PID_FILE}" ]]; then
    local watcher_pid
    watcher_pid="$(<"${HOST_BRIDGE_WATCHER_PID_FILE}")"
    if [[ -n "${watcher_pid}" ]] && kill -0 "${watcher_pid}" 2>/dev/null; then
      kill "${watcher_pid}" 2>/dev/null || true
      wait "${watcher_pid}" 2>/dev/null || true
    fi
    rm -f "${HOST_BRIDGE_WATCHER_PID_FILE}"
  fi
}

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  docker stop "${CONTAINER_NAME}" >/dev/null 2>&1 || true
  docker rm "${CONTAINER_NAME}" >/dev/null 2>&1 || true
fi
cleanup_host_bridge_watcher
cleanup_host_bridge
cleanup_fp_to_odom

cat > "${LOG_DIR}/entrypoint.sh" <<'EOFSCRIPT'
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
EOFSCRIPT
chmod +x "${LOG_DIR}/entrypoint.sh"

echo "[INFO] Starting ${CONTAINER_NAME}..."
docker run -d \
  --name "${CONTAINER_NAME}" \
  --network host \
  --ipc=host \
  --pid=host \
  --restart unless-stopped \
  --memory="${DOCKER_MEM_LIMIT}" --memory-swap="${DOCKER_MEM_SWAP}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=/data/cyclonedds.xml \
  -v "${CYCLONEDDS_CONFIG}:/data/cyclonedds.xml:ro" \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  -v /etc/localtime:/etc/localtime:ro \
  -v "${FIXPOSITION_CONFIG_DIR}:/data/fixposition_config:ro" \
  -v "${LOG_DIR}:/data" \
  "${DOCKER_IMAGE}" \
  bash /data/entrypoint.sh


cleanup_host_bridge_watcher
cleanup_host_bridge
cleanup_fp_to_odom

if [[ "${ENABLE_MOTION_INFO_BRIDGE}" == "1" ]]; then
  echo "[INFO] Restarting host MOTION_INFO bridge..."
  bash -lc "source /opt/ros/foxy/setup.bash && export ROS_DOMAIN_ID=\"${ROS_DOMAIN_ID}\" && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export FASTRTPS_DEFAULT_PROFILES_FILE=\"${FASTDDS_CONFIG}\" && exec python3 \"${HOST_BRIDGE_SCRIPT}\"" \
    > "${LOG_DIR}/motion_info_bridge.log" 2>&1 &
  host_bridge_pid=$!
  printf '%s\n' "${host_bridge_pid}" > "${HOST_BRIDGE_PID_FILE}"
fi

echo "[INFO] Starting fp_to_odom..."
bash -lc "source /opt/ros/foxy/setup.bash && export ROS_DOMAIN_ID=\"${ROS_DOMAIN_ID}\" && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export FASTRTPS_DEFAULT_PROFILES_FILE=\"${FASTDDS_CONFIG}\" && exec python3 \"${FP_TO_ODOM_SCRIPT}\"" \
  > "${LOG_DIR}/fp_to_odom.log" 2>&1 &
fp_to_odom_pid=$!
printf '%s\n' "${fp_to_odom_pid}" > "${FP_TO_ODOM_PID_FILE}"

# Watcher logic (now monitors both host_bridge and fp_to_odom)
(
  while docker inspect -f '{{if or .State.Running .State.Restarting}}up{{else}}down{{end}}' "${CONTAINER_NAME}" >/dev/null 2>&1; do
    state="$(docker inspect -f '{{if or .State.Running .State.Restarting}}up{{else}}down{{end}}' "${CONTAINER_NAME}" 2>/dev/null || true)"
    [[ "${state}" == "up" ]] || break
    sleep 2
  done
  cleanup_host_bridge
  cleanup_fp_to_odom
  rm -f "${HOST_BRIDGE_WATCHER_PID_FILE}"
) &
watcher_pid=$!
printf '%s\n' "${watcher_pid}" > "${HOST_BRIDGE_WATCHER_PID_FILE}"


echo "[INFO] Container started: ${CONTAINER_NAME}"
docker ps --filter "name=${CONTAINER_NAME}"
wait

