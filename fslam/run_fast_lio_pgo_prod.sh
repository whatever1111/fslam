#!/usr/bin/env bash
set -euo pipefail

DOCKER_IMAGE="${DOCKER_IMAGE:-wanderer123/fslam-humble:arm64}"
DOCKER_MEM_LIMIT="${DOCKER_MEM_LIMIT:-24g}"
DOCKER_MEM_SWAP="${DOCKER_MEM_SWAP:-28g}"
CONTAINER_NAME="${CONTAINER_NAME:-fslam-runtime}"
CONFIG_DIR="${CONFIG_DIR:-/home/user/fslam_test/fslam/config}"
LOG_DIR="${LOG_DIR:-/home/user/fslam_test/fslam/logs}"
FIXPOSITION_CONFIG_DIR="${FIXPOSITION_CONFIG_DIR:-/home/user/fslam_test/fixposition}"
POINTCLOUD_REMAP_SCRIPT="${POINTCLOUD_REMAP_SCRIPT:-/home/user/fslam_test/fslam/rename_pointcloud_field.py}"
PGO_CLOUD_RELAY_SCRIPT="${PGO_CLOUD_RELAY_SCRIPT:-/home/user/fslam_test/fslam/pointcloud_relay.py}"
HOST_BRIDGE_SCRIPT="${HOST_BRIDGE_SCRIPT:-/home/user/fslam_test/motion_info_to_twist.py}"
ENABLE_MOTION_INFO_BRIDGE="${ENABLE_MOTION_INFO_BRIDGE:-1}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
HOST_BRIDGE_PID_FILE=""
HOST_BRIDGE_WATCHER_PID_FILE=""

if ! docker image inspect "${DOCKER_IMAGE}" >/dev/null 2>&1; then
  echo "[ERROR] Docker image not found: ${DOCKER_IMAGE}" >&2
  exit 1
fi

for f in params_fast_lio_pgo.yaml params_ros2.yaml mower.yaml; do
  if [[ ! -f "${CONFIG_DIR}/${f}" ]]; then
    echo "[ERROR] Missing config file: ${CONFIG_DIR}/${f}" >&2
    exit 1
  fi
done

for f in node.launch config.yaml; do
  if [[ ! -f "${FIXPOSITION_CONFIG_DIR}/${f}" ]]; then
    echo "[ERROR] Missing Fixposition config file: ${FIXPOSITION_CONFIG_DIR}/${f}" >&2
    exit 1
  fi
done

if [[ -f "${FIXPOSITION_CONFIG_DIR}/robot.urdf.xacro" ]]; then
  echo "[INFO] Found FP-only URDF at ${FIXPOSITION_CONFIG_DIR}/robot.urdf.xacro; FAST-LIO startup will ignore it."
fi

if [[ ! -f "${POINTCLOUD_REMAP_SCRIPT}" ]]; then
  echo "[ERROR] Missing pointcloud remap script: ${POINTCLOUD_REMAP_SCRIPT}" >&2
  exit 1
fi


if [[ ! -f "${PGO_CLOUD_RELAY_SCRIPT}" ]]; then
  echo "[ERROR] Missing PGO cloud relay script: ${PGO_CLOUD_RELAY_SCRIPT}" >&2
  exit 1
fi

if [[ "${ENABLE_MOTION_INFO_BRIDGE}" == "1" && ! -f "${HOST_BRIDGE_SCRIPT}" ]]; then
  echo "[ERROR] Missing host bridge script: ${HOST_BRIDGE_SCRIPT}" >&2
  exit 1
fi

mkdir -p "${LOG_DIR}"
LOG_DIR="$(realpath "${LOG_DIR}")"
CONFIG_DIR="$(realpath "${CONFIG_DIR}")"
FIXPOSITION_CONFIG_DIR="$(realpath "${FIXPOSITION_CONFIG_DIR}")"
POINTCLOUD_REMAP_SCRIPT="$(realpath "${POINTCLOUD_REMAP_SCRIPT}")"
PGO_CLOUD_RELAY_SCRIPT="$(realpath "${PGO_CLOUD_RELAY_SCRIPT}")"
HOST_BRIDGE_PID_FILE="${LOG_DIR}/motion_info_bridge.pid"
HOST_BRIDGE_WATCHER_PID_FILE="${LOG_DIR}/motion_info_bridge_watcher.pid"

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

cat > "${LOG_DIR}/entrypoint.sh" <<'EOFSCRIPT'
#!/bin/bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

mkdir -p /data/PCD /data/Log
rm -rf /root/ros2_ws/src/FAST_LIO/PCD /root/ros2_ws/src/FAST_LIO/Log
ln -sf /data/PCD /root/ros2_ws/src/FAST_LIO/PCD
ln -sf /data/Log /root/ros2_ws/src/FAST_LIO/Log

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

echo "[STEP 0/8] Launching Fixposition driver..."
ros2 launch /data/fixposition_config/node.launch \
  config:=/data/fixposition_config/config.yaml \
  > /data/fixposition.log 2>&1 &
PIDS+=($!)
sleep 5

echo "[STEP 1/8] Launching FP IMU relay..."
python3 /root/ros2_ws/install/lio_slam/share/lio_slam/tools/fp_imu_relay.py \
  --ros-args \
  --params-file /config/params_fast_lio_pgo.yaml \
  > /data/fp_imu_relay.log 2>&1 &
PIDS+=($!)
sleep 2

echo "[STEP 2/8] Launching PointCloud field remap..."
python3 /data/rename_pointcloud_field.py \
  --ros-args \
  -p input_topic:=/LIDAR/POINTS \
  -p output_topic:=/LIDAR/POINTS_REMAP \
  -p from_field:=timestamp \
  -p to_field:=time \
  > /data/pointcloud_remap.log 2>&1 &
PIDS+=($!)
sleep 2

echo "[STEP 3/8] Launching FAST-LIO2..."
ros2 launch fast_lio mapping.launch.py \
  config_path:=/config \
  config_file:=mower.yaml \
  rviz:=false \
  > /data/fastlio.log 2>&1 &
PIDS+=($!)
sleep 5


echo "[STEP 5/8] Launching PGO cloud relay..."
python3 /data/pointcloud_relay.py \
  --ros-args \
  -p input_topic:=/cloud_registered_body \
  -p output_topic:=/cloud_registered_body_pgo \
  > /data/pgo_cloud_relay.log 2>&1 &
PIDS+=($!)
sleep 2

echo "[STEP 6/8] Launching ECEF-to-ENU bridge..."
python3 /root/ros2_ws/install/lio_slam/share/lio_slam/tools/ecef_to_enu_bridge.py \
  --ros-args \
  -p input_topic:=/fixposition/odometry_ecef \
  -p output_topic:=/odometry/gps \
  > /data/gps_converter.log 2>&1 &
PIDS+=($!)
sleep 2

echo "[STEP 7/8] Launching FastLioPGO..."
ros2 run lio_slam lio_slam_fastLioPGO \
  --ros-args \
  --params-file /config/params_fast_lio_pgo.yaml \
  -r /Odometry:=/Odometry \
  -r /cloud_registered_body:=/cloud_registered_body_pgo \
  -r /odom:=/ODOM \
  > /data/pgo.log 2>&1 &
PIDS+=($!)
sleep 3

echo "[INFO] Started PIDs: ${PIDS[*]}"
echo "[INFO] Logs: /data/fixposition.log /data/fp_imu_relay.log /data/pointcloud_remap.log /data/fastlio.log /data/pgo_odom_relay.log /data/pgo_cloud_relay.log /data/gps_converter.log /data/pgo.log"

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
  -e OMP_NUM_THREADS=4 \
  -e OPENBLAS_NUM_THREADS=1 \
  -e MKL_NUM_THREADS=1 \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  -v /etc/localtime:/etc/localtime:ro \
  -v "${CONFIG_DIR}:/config:ro" \
  -v "${FIXPOSITION_CONFIG_DIR}:/data/fixposition_config:ro" \
  -v "${POINTCLOUD_REMAP_SCRIPT}:/data/rename_pointcloud_field.py:ro" \
  -v "${PGO_CLOUD_RELAY_SCRIPT}:/data/pointcloud_relay.py:ro" \
  -v "${LOG_DIR}:/data" \
  "${DOCKER_IMAGE}" \
  bash /data/entrypoint.sh

if [[ "${ENABLE_MOTION_INFO_BRIDGE}" == "1" ]]; then
  cleanup_host_bridge_watcher
  cleanup_host_bridge

  echo "[INFO] Restarting host MOTION_INFO bridge..."
  bash -lc "source /opt/ros/foxy/setup.bash && exec python3 \"${HOST_BRIDGE_SCRIPT}\"" \
    > "${LOG_DIR}/motion_info_bridge.log" 2>&1 &
  host_bridge_pid=$!
  printf '%s\n' "${host_bridge_pid}" > "${HOST_BRIDGE_PID_FILE}"

  (
    while docker inspect -f '{{if or .State.Running .State.Restarting}}up{{else}}down{{end}}' "${CONTAINER_NAME}" >/dev/null 2>&1; do
      state="$(docker inspect -f '{{if or .State.Running .State.Restarting}}up{{else}}down{{end}}' "${CONTAINER_NAME}" 2>/dev/null || true)"
      [[ "${state}" == "up" ]] || break
      sleep 2
    done
    cleanup_host_bridge
    rm -f "${HOST_BRIDGE_WATCHER_PID_FILE}"
  ) &
  watcher_pid=$!
  printf '%s\n' "${watcher_pid}" > "${HOST_BRIDGE_WATCHER_PID_FILE}"
fi

echo "[INFO] Container started: ${CONTAINER_NAME}"
docker ps --filter "name=${CONTAINER_NAME}"
