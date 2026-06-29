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
