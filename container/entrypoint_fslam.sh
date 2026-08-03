#!/bin/bash
# ============================================================================
# entrypoint_fslam.sh —— fslam 模式容器载荷 | container payload for fslam mode
# ============================================================================
# 由 run_fast_lio_pgo_prod.sh 只读挂载为 /entrypoint.sh 后执行,不要在容器外跑。
# Mounted read-only at /entrypoint.sh by run_fast_lio_pgo_prod.sh; not meant
# to run outside the container.
#
# 环境 | env: PROFILE START_FIXPOSITION FP_STREAM FP_CONFIG_FILE LIDAR_TOPIC
#             RUNNER_ARGS SLAM_PARAM_OVERLAY(经 run_prod_native 消费)
# 挂载 | mounts: /fixposition_driver.launch(ro) /data(日志+运行时配置 | logs)
# ============================================================================
set -o pipefail
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
SHARE=/root/ros2_ws/install/lio_slam/share/lio_slam

# ---- ① Fixposition driver:先开 RTK 输出 | bring up the RTK streams FIRST ----
launch_fixposition() {
  : "${FP_CONFIG_FILE:=${SHARE}/config/fixposition/m20.yaml}"
  if [[ ! -f "${FP_CONFIG_FILE}" ]]; then
    echo "[ERROR] FP 驱动配置缺失 | FP driver config missing: ${FP_CONFIG_FILE}" >&2
    exit 1
  fi
  # 写运行时副本再按需覆盖 stream(原件只读)| copy, then optionally override
  cp "${FP_CONFIG_FILE}" /data/fixposition_runtime.yaml
  if [[ -n "${FP_STREAM:-}" ]]; then
    sed -i "s|^\([[:space:]]*stream:\).*|\1 ${FP_STREAM}|" /data/fixposition_runtime.yaml
    echo "[INFO] FP stream 覆盖 | overridden: ${FP_STREAM}"
  fi
  echo "[STEP 1/2] Fixposition driver 启动中 | launching..."
  setsid ros2 launch /fixposition_driver.launch > /data/fixposition.log 2>&1 &
}

wait_rtk_streams() {
  local up=0
  for _ in $(seq 1 30); do
    if timeout 5 ros2 topic list 2>/dev/null | grep -qx '/fixposition/fpa/odomenu'; then
      up=1
      break
    fi
    sleep 1
  done
  if [[ "${up}" != "1" ]]; then
    echo "[WARN] 30s 内未见 /fixposition/fpa/odomenu(驱动仍在 respawn,管线继续)| driver still respawning; continuing"
    return 0
  fi
  # 话题存在 ≠ 有数据在流:实收一条才算真就绪。
  # Topic presence ≠ data flowing: only a received message counts as ready.
  if timeout 5 ros2 topic echo --once /fixposition/fpa/corrimu >/dev/null 2>&1; then
    echo "[INFO] RTK 输出就绪(实收数据)| RTK streams up: /fixposition/fpa/{corrimu,odomenu}"
  else
    echo "[WARN] FPA 话题存在但 corrimu 5s 无数据 | FPA topics exist but no corrimu data in 5s (see /data/fixposition.log)"
  fi
}

if [[ "${START_FIXPOSITION:-1}" == "1" ]]; then
  launch_fixposition
  wait_rtk_streams
fi

# 雷达是狗自带服务,本脚本不启动 —— 缺席只能告警。
# The lidar driver is the dog's own service, never started here — warn only.
if ! timeout 5 ros2 topic list 2>/dev/null | grep -qx "${LIDAR_TOPIC:-/LIDAR/POINTS}"; then
  echo "[WARN] 未发现雷达话题 | lidar topic missing: ${LIDAR_TOPIC:-/LIDAR/POINTS}"
fi

# ---- ② canonical 管线 | the in-image four-quadrant prod entry ---------------
# exec 后本进程即 run_prod_native.sh(容器 PID1 链):其 trap 负责管线清理,
# 容器退出时运行时统一回收 driver 子进程。SLAM_PARAM_OVERLAY 注入 odom_topic
# 直出层。 After exec this process IS run_prod_native.sh (container PID1
# chain): its trap cleans the pipeline; the runtime reaps the driver child on
# exit. SLAM_PARAM_OVERLAY injects the direct-odom_topic layer.
echo "[STEP 2/2] canonical 管线 | pipeline: run_prod_native.sh --profile ${PROFILE:-m20} ${RUNNER_ARGS:-}"
# shellcheck disable=SC2086  # RUNNER_ARGS 有意按词拆分 | intentional word-split
exec bash "${SHARE}/tools/run_prod_native.sh" --profile "${PROFILE:-m20}" --log /data ${RUNNER_ARGS:-}
