#!/usr/bin/env bash
source /opt/ros/foxy/setup.bash 
set -euo pipefail

DOCKER_IMAGE="${DOCKER_IMAGE:-wanderer123/fslam-humble:arm64}"
DOCKER_MEM_LIMIT="${DOCKER_MEM_LIMIT:-8g}"
DOCKER_MEM_SWAP="${DOCKER_MEM_SWAP:-10g}"
CONTAINER_NAME="${CONTAINER_NAME:-fixposition-runtime}"
LOG_DIR="${LOG_DIR:-/home/user/fslam/fslam/logs/fixposition_only}"
FIXPOSITION_CONFIG_DIR="${FIXPOSITION_CONFIG_DIR:-/home/user/fslam/fixposition}"
HOST_BRIDGE_SCRIPT="${HOST_BRIDGE_SCRIPT:-/home/user/fslam/motion_info_to_twist.py}"
FP_TO_ODOM_SCRIPT="${FP_TO_ODOM_SCRIPT:-/home/user/fslam/fp_to_odom.py}"
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
  -v /home/user/fslam/cyclonedds.xml:/data/cyclonedds.xml:ro \
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
  bash -lc "source /opt/ros/foxy/setup.bash && FASTDDS_URI=/opt/robot/fastdds.xml && exec python3 \"${HOST_BRIDGE_SCRIPT}\"" \
    > "${LOG_DIR}/motion_info_bridge.log" 2>&1 &
  host_bridge_pid=$!
  printf '%s\n' "${host_bridge_pid}" > "${HOST_BRIDGE_PID_FILE}"
fi

echo "[INFO] Starting fp_to_odom..."
bash -lc "source /opt/ros/foxy/setup.bash && FASTDDS_URI=/opt/robot/fastdds.xml && exec python3 \"${FP_TO_ODOM_SCRIPT}\"" \
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