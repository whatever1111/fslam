#!/usr/bin/env bash
# ============================================================================
# run_fast_lio_pgo_prod.sh —— fslam 模式生产部署 · 云深处 M20 机器狗
#                             fslam-mode production deploy · Deep Robotics M20
# ============================================================================
# 容器内 | in-container:
#   ① Fixposition driver(RTK/FPA 流)→ ② canonical 管线(镜像内
#      run_prod_native.sh --profile m20)。融合节点经参数 overlay 直接以
#      ${ODOM_TOPIC}(默认 /ODOM)发布——无别名中继。
#      The fusion node publishes ${ODOM_TOPIC} (default /ODOM) DIRECTLY via a
#      param overlay — the old /odom→/ODOM alias relay is gone.
# 宿主机 | on the host:
#   ③ motion_info_to_twist.py(轮速 → FP 设备融合,需狗侧 drdds)
#   ④ fp_to_odom.py --ros-args -p relay_enable:=false
#      (不中继 /ODOM,仅订阅之作为位姿/时戳源;发布 /LOC_BODY_POINTS
#       去畸变点云 + /LOCATION_STATUS。No /ODOM relay — it only tracks the
#       pipeline's /ODOM and publishes the deskewed cloud + status.)
#
# 直接 /ODOM 依赖镜像内融合节点支持 `odom_topic` 参数(LIO-SLAM ≥ 对应提交,
# docker pull 最新镜像)。旧镜像会忽略该参数继续发 /odom → 用 --check 可见
# /ODOM SILENT。 Direct /ODOM needs an image whose fusion node has the
# `odom_topic` parameter; older images ignore it (pipeline keeps /odom, /ODOM
# stays silent — visible via --check).
#
# 用法 | usage: fslam/run_fast_lio_pgo_prod.sh [选项]
#   --profile <name>     profile(默认 m20)
#   --image <img>        镜像(默认 wanderer123/fslam-humble:<arch>)
#   --name <n>           容器名(默认 fslam-runtime)
#   --mem/--swap <sz>    内存/交换上限(默认 24g/28g)
#   --domain <id>        ROS_DOMAIN_ID(默认 0)
#   --fp-config <yaml>   宿主机 FP 驱动配置(默认镜像内 m20.yaml)
#   --config-dir <dir>   宿主机 config/ 树只读覆盖镜像配置(须含 profiles/)
#   --fp-stream <uri>    覆盖 FP 传感器流地址(如 tcpcli://10.21.31.66:21000)
#   --no-fixposition     不启驱动(RTK 已由别处提供)
#   --odom-topic <t>     管线直出话题(默认 /ODOM;空串 = 保持 /odom)
#                        direct output topic ("" keeps /odom)
#   --foxglove           开 Foxglove 监控    --bridge-port <p> 端口(默认 8765)
#   --param-overlay <f>  在线改参 overlay(最高优先级层,与 odom_topic 合并)
#   --foreground         前台运行(--rm,Ctrl-C 停;默认后台 + restart)
#   --check              对运行中的部署做链路体检(逐话题测频)
# 环境 | env: DOCKER_IMAGE CONTAINER_NAME LOG_DIR ROS_DOMAIN_ID LIDAR_TOPIC
#   RMW_IMPLEMENTATION CYCLONEDDS_CONFIG FASTDDS_CONFIG HOST_BRIDGE_SCRIPT
#   FP_TO_ODOM_SCRIPT ENABLE_MOTION_INFO_BRIDGE(1/0/auto) FP_TO_ODOM_ARGS
# ============================================================================
set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # <checkout>/fslam
REPO_DIR="$(cd "${BASE_DIR}/.." && pwd)"                   # <checkout>
# shellcheck source=deploy_common.sh
source "${BASE_DIR}/deploy_common.sh"

IMAGE="${DOCKER_IMAGE:-wanderer123/fslam-humble:$(fslam_arch_tag)}"
NAME="${CONTAINER_NAME:-fslam-runtime}"
MEM="${DOCKER_MEM_LIMIT:-24g}"
SWAP="${DOCKER_MEM_SWAP:-28g}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
LOG_DIR="${LOG_DIR:-${BASE_DIR}/logs/fslam_prod}"
LIDAR_TOPIC="${LIDAR_TOPIC:-/LIDAR/POINTS2}"
CYCLONEDDS_CONFIG="${CYCLONEDDS_CONFIG:-${REPO_DIR}/cyclonedds.xml}"
FASTDDS_CONFIG="${FASTDDS_CONFIG:-${REPO_DIR}/fastdds.xml}"
HOST_BRIDGE_SCRIPT="${HOST_BRIDGE_SCRIPT:-${REPO_DIR}/motion_info_to_twist.py}"
FP_TO_ODOM_SCRIPT="${FP_TO_ODOM_SCRIPT:-${REPO_DIR}/fp_to_odom.py}"
ENABLE_MOTION_INFO_BRIDGE="${ENABLE_MOTION_INFO_BRIDGE:-auto}"
FP_TO_ODOM_ARGS="${FP_TO_ODOM_ARGS:-}"

PROFILE="m20"
FP_CONFIG=""
CONFIG_DIR=""
FP_STREAM="${FP_STREAM:-}"
START_FIXPOSITION=1
ODOM_TOPIC="/ODOM"
USER_OVERLAY=""
DETACH=1
RUNNER_ARGS=""
DO_CHECK=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)        PROFILE="$2"; shift 2 ;;
    --image)          IMAGE="$2"; shift 2 ;;
    --name)           NAME="$2"; shift 2 ;;
    --mem)            MEM="$2"; shift 2 ;;
    --swap)           SWAP="$2"; shift 2 ;;
    --domain)         ROS_DOMAIN_ID="$2"; shift 2 ;;
    --fp-config)      FP_CONFIG="$2"; shift 2 ;;
    --config-dir)     CONFIG_DIR="$2"; shift 2 ;;
    --fp-stream)      FP_STREAM="$2"; shift 2 ;;
    --no-fixposition) START_FIXPOSITION=0; shift ;;
    --odom-topic)     ODOM_TOPIC="$2"; shift 2 ;;
    --odom-alias)
      echo "[ERROR] --odom-alias 已移除:管线现直出 /ODOM,用 --odom-topic 改名。" >&2
      echo "[ERROR] --odom-alias is gone — the pipeline now publishes /ODOM directly; use --odom-topic." >&2
      exit 2 ;;
    --foxglove)       RUNNER_ARGS+=" --foxglove"; shift ;;
    --bridge-port)    RUNNER_ARGS+=" --bridge-port $2"; shift 2 ;;
    --param-overlay)  USER_OVERLAY="$2"; shift 2 ;;
    --foreground)     DETACH=0; shift ;;
    --check)          DO_CHECK=1; shift ;;
    -h|--help) sed -n '2,45p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "[ERROR] 未知参数 | unknown arg: $1" >&2; exit 2 ;;
  esac
done

# 管线实际输出话题(--odom-topic "" = 保持 /odom)| the pipeline's actual output
PIPE_ODOM_TOPIC="${ODOM_TOPIC:-/odom}"

# ---- 预检 | pre-checks ------------------------------------------------------
command -v docker >/dev/null 2>&1 || { echo "[ERROR] docker 不可用 | not available" >&2; exit 1; }
if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "[ERROR] 镜像不存在 | image not found: ${IMAGE}  (docker pull ${IMAGE})" >&2; exit 1
fi
if [[ -n "${FP_CONFIG}" ]]; then
  [[ -f "${FP_CONFIG}" ]] || { echo "[ERROR] FP 配置不存在 | not found: ${FP_CONFIG}" >&2; exit 1; }
  FP_CONFIG="$(realpath "${FP_CONFIG}")"
fi
if [[ -n "${USER_OVERLAY}" ]]; then
  [[ -f "${USER_OVERLAY}" ]] || { echo "[ERROR] overlay 不存在 | not found: ${USER_OVERLAY}" >&2; exit 1; }
  USER_OVERLAY="$(realpath "${USER_OVERLAY}")"
fi
if [[ -n "${CONFIG_DIR}" ]]; then
  [[ -d "${CONFIG_DIR}/profiles" ]] || { echo "[ERROR] --config-dir 须含 profiles/ | must contain profiles/: ${CONFIG_DIR}" >&2; exit 1; }
  CONFIG_DIR="$(realpath "${CONFIG_DIR}")"
else
  # 自动发现:配置树随部署仓库 checkout 走 | auto-discover from this checkout
  for d in "${BASE_DIR}/config" "${REPO_DIR}/config"; do
    if [[ -d "${d}/profiles" ]]; then CONFIG_DIR="$(realpath "${d}")"; break; fi
  done
fi
[[ -f "${FP_TO_ODOM_SCRIPT}" ]] || { echo "[ERROR] 缺 fp_to_odom | missing: ${FP_TO_ODOM_SCRIPT}" >&2; exit 1; }
if [[ "${ENABLE_MOTION_INFO_BRIDGE}" == "auto" ]]; then
  [[ -f "${HOST_BRIDGE_SCRIPT}" ]] && ENABLE_MOTION_INFO_BRIDGE=1 || ENABLE_MOTION_INFO_BRIDGE=0
fi
if [[ "${ENABLE_MOTION_INFO_BRIDGE}" == "1" && ! -f "${HOST_BRIDGE_SCRIPT}" ]]; then
  echo "[ERROR] 缺桥脚本 | missing bridge script: ${HOST_BRIDGE_SCRIPT}" >&2; exit 1
fi
HOST_ROS_SETUP="$(fslam_host_ros_setup)" \
  || { echo "[ERROR] 宿主机无 ROS2 环境 | no host ROS2 env under /opt/ros" >&2; exit 1; }
FP_TO_ODOM_SCRIPT="$(realpath "${FP_TO_ODOM_SCRIPT}")"
[[ "${ENABLE_MOTION_INFO_BRIDGE}" == "1" ]] && HOST_BRIDGE_SCRIPT="$(realpath "${HOST_BRIDGE_SCRIPT}")"

mkdir -p "${LOG_DIR}"
LOG_DIR="$(realpath "${LOG_DIR}")"

# ============================================================================
# --check:对运行中的部署做链路体检 | health-check a RUNNING deployment
# 顺数据流逐话题测频:第一个无数据的话题 = 断点。
# ============================================================================
if [[ "${DO_CHECK}" == "1" ]]; then
  docker ps --format '{{.Names}}' | grep -qx "${NAME}" \
    || { echo "[ERROR] 容器未运行 | container not running: ${NAME}" >&2; exit 1; }
  echo "== 数据流体检(容器 ${NAME},每话题探测 4s)| data-flow check =="
  docker exec -i "${NAME}" bash -s -- "${LIDAR_TOPIC}" "${PIPE_ODOM_TOPIC}" <<'EOFCHECK'
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source /root/ros2_ws/install/setup.bash >/dev/null 2>&1
for t in "$1" /fixposition/fpa/corrimu /fixposition/fpa/odomenu \
         /lio/points /lio/imu /lio/gps_odom \
         /Odometry /fast_lio_pgo/odometry "$2" /LOC_BODY_POINTS; do
  printf '  %-32s ' "$t"
  rate="$(timeout 4 ros2 topic hz "$t" 2>/dev/null | grep -m1 -oE 'average rate: [0-9.]+')"
  echo "${rate:-无数据 | SILENT}"
done
EOFCHECK
  # /LOCATION_STATUS 是 drdds 类型,镜像内无该消息包 → 必须在宿主机测
  # /LOCATION_STATUS is a drdds type absent from the image → probe on the host
  echo "== 宿主机侧 | host-side probe =="
  bash -c "source '${HOST_ROS_SETUP}' \
    && export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && export FASTRTPS_DEFAULT_PROFILES_FILE='${FASTDDS_CONFIG}' \
    && printf '  %-32s ' /LOCATION_STATUS \
    && rate=\"\$(timeout 4 ros2 topic hz /LOCATION_STATUS 2>/dev/null | grep -m1 -oE 'average rate: [0-9.]+')\" \
    && echo \"\${rate:-无数据 | SILENT}\"" || true
  echo ""
  echo "== 关键日志签名 | log signatures (${LOG_DIR}) =="
  grep -hE 'first imu cb|first lidar|first GPS cb|alignment transform|No Effective|xy_floor' \
    "${LOG_DIR}/pipeline.log" 2>/dev/null | tail -6 || true
  for f in fixposition fp_to_odom motion_info_bridge; do
    echo "-- ${f}.log 尾部 | tail --"
    tail -3 "${LOG_DIR}/${f}.log" 2>/dev/null || echo "  (无 | none)"
  done
  echo ""
  echo "解读 | how to read:"
  echo "  ${LIDAR_TOPIC} SILENT       → 雷达驱动(狗自带服务)没在跑,本脚本不启动它"
  echo "  corrimu/odomenu SILENT      → FP 设备没出数据:查 fixposition.log(TCP/RTK/基站)"
  echo "  /lio/* SILENT               → adapter 异常:查 pipeline.log"
  echo "  /Odometry SILENT            → 前端未初始化(需点云+IMU 同时就绪)"
  echo "  仅 ${PIPE_ODOM_TOPIC} SILENT → PGO 未对齐(等 'alignment transform');若 /odom 有数据"
  echo "                                而 ${PIPE_ODOM_TOPIC} 无 → 镜像太旧,无 odom_topic 参数,请更新镜像"
  echo "  /LOC_BODY_POINTS SILENT     → 宿主机 fp_to_odom 未跑或雷达缺席:查 fp_to_odom.log"
  exit 0
fi

# ---- 清理上一次部署 | stop any previous deployment --------------------------
fslam_stop_previous "${NAME}" "${LOG_DIR}" \
  "${LOG_DIR}/motion_info_bridge.pid" "${LOG_DIR}/fp_to_odom.pid"

# ============================================================================
# 参数 overlay:管线直出 ${PIPE_ODOM_TOPIC}(最高优先级层,含用户 overlay)
# Param overlay: direct pipeline output topic (+ the user's overlay, merged).
# ============================================================================
OVERLAY_FILE="${LOG_DIR}/param_overlay.yaml"
: > "${OVERLAY_FILE}"
NEED_ODOM_SECTION=1
if [[ -n "${USER_OVERLAY}" ]]; then
  cat "${USER_OVERLAY}" >> "${OVERLAY_FILE}"
  echo "" >> "${OVERLAY_FILE}"
  if grep -qE '^fast_lio_pgo_fusion:' "${USER_OVERLAY}"; then
    # YAML 顶层键不可重复,无法安全追加同名段 | top-level keys must be unique
    if ! grep -qE '^[[:space:]]+odom_topic:' "${USER_OVERLAY}"; then
      echo "[ERROR] --param-overlay 已含 fast_lio_pgo_fusion 段但未设 odom_topic;" >&2
      echo "        请在该段内自行加:  odom_topic: ${PIPE_ODOM_TOPIC}" >&2
      echo "        (your overlay has a fast_lio_pgo_fusion section — add odom_topic there yourself)" >&2
      exit 1
    fi
    NEED_ODOM_SECTION=0
  fi
fi
if [[ -n "${ODOM_TOPIC}" && "${NEED_ODOM_SECTION}" == "1" ]]; then
  cat >> "${OVERLAY_FILE}" <<EOF
# injected by run_fast_lio_pgo_prod.sh —— 管线直出 ${ODOM_TOPIC},无别名中继
# direct ${ODOM_TOPIC} output from the fusion node (no alias relay)
fast_lio_pgo_fusion:
  ros__parameters:
    odom_topic: ${ODOM_TOPIC}
EOF
fi

# ============================================================================
# 容器 entrypoint(写入 LOG_DIR → /data;容器 restart 时重读,LOG_DIR 必须持久)
# ============================================================================
cat > "${LOG_DIR}/entrypoint.sh" <<'EOFSCRIPT'
#!/bin/bash
set -o pipefail
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
SHARE=/root/ros2_ws/install/lio_slam/share/lio_slam

# ---- ① Fixposition driver:先开 RTK 输出 | bring up the RTK streams FIRST ----
if [[ "${START_FIXPOSITION:-1}" == "1" ]]; then
  : "${FP_CONFIG_FILE:=${SHARE}/config/fixposition/m20.yaml}"
  if [[ ! -f "${FP_CONFIG_FILE}" ]]; then
    echo "[ERROR] FP 驱动配置缺失 | FP driver config missing: ${FP_CONFIG_FILE}" >&2
    exit 1
  fi
  cp "${FP_CONFIG_FILE}" /data/fixposition_runtime.yaml
  if [[ -n "${FP_STREAM:-}" ]]; then
    sed -i "s|^\([[:space:]]*stream:\).*|\1 ${FP_STREAM}|" /data/fixposition_runtime.yaml
    echo "[INFO] FP stream 覆盖 | overridden: ${FP_STREAM}"
  fi
  cat > /data/fixposition.launch <<'EOFLAUNCH'
<launch>
  <node name="fixposition_driver_ros2" pkg="fixposition_driver_ros2"
        exec="fixposition_driver_ros2_exec" output="screen"
        respawn="true" respawn_delay="5">
    <param from="/data/fixposition_runtime.yaml"/>
  </node>
</launch>
EOFLAUNCH
  echo "[STEP 1/2] Fixposition driver 启动中 | launching..."
  setsid ros2 launch /data/fixposition.launch > /data/fixposition.log 2>&1 &

  fp_up=0
  for _ in $(seq 1 30); do
    if timeout 5 ros2 topic list 2>/dev/null | grep -qx '/fixposition/fpa/odomenu'; then
      fp_up=1; break
    fi
    sleep 1
  done
  if [[ "${fp_up}" == "1" ]]; then
    if timeout 5 ros2 topic echo --once /fixposition/fpa/corrimu >/dev/null 2>&1; then
      echo "[INFO] RTK 输出就绪(实收数据)| RTK streams up: /fixposition/fpa/{corrimu,odomenu}"
    else
      echo "[WARN] FPA 话题存在但 corrimu 5s 无数据 | FPA topics exist but no corrimu data in 5s (see /data/fixposition.log)"
    fi
  else
    echo "[WARN] 30s 内未见 /fixposition/fpa/odomenu(驱动仍在 respawn,管线继续)| driver still respawning; continuing"
  fi
fi

# 雷达是狗自带服务,本脚本不启动 —— 缺席只能告警。
# The lidar driver is the dog's own service, never started here — warn only.
if ! timeout 5 ros2 topic list 2>/dev/null | grep -qx "${LIDAR_TOPIC:-/LIDAR/POINTS2}"; then
  echo "[WARN] 未发现雷达话题 | lidar topic missing: ${LIDAR_TOPIC:-/LIDAR/POINTS2}"
fi

# ---- ② canonical 管线(exec → 本进程即 run_prod_native.sh)-------------------
# SLAM_PARAM_OVERLAY(/data/param_overlay.yaml)注入 odom_topic 直出层。
echo "[STEP 2/2] canonical 管线 | pipeline: run_prod_native.sh --profile ${PROFILE} ${RUNNER_ARGS:-}"
# shellcheck disable=SC2086  # RUNNER_ARGS 有意按词拆分 | intentional word-split
exec bash "${SHARE}/tools/run_prod_native.sh" --profile "${PROFILE}" --log /data ${RUNNER_ARGS:-}
EOFSCRIPT
chmod +x "${LOG_DIR}/entrypoint.sh"

# ============================================================================
# 启动容器 | start the container
# ============================================================================
DOCKER_ARGS=(
  --name "${NAME}"
  --network host --ipc=host --pid=host
  --memory="${MEM}" --memory-swap="${SWAP}"
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
  # 与实测过的 fslam 实机部署一致:容器 CycloneDDS、宿主机 FastDDS(跨厂商 RTPS 互通)。
  # Matches the validated field deployment: CycloneDDS in-container, FastDDS on host.
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  -e OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
  -e OPENBLAS_NUM_THREADS=1 -e MKL_NUM_THREADS=1
  -e PROFILE="${PROFILE}"
  -e START_FIXPOSITION="${START_FIXPOSITION}"
  -e FP_STREAM="${FP_STREAM}"
  -e LIDAR_TOPIC="${LIDAR_TOPIC}"
  -e RUNNER_ARGS="${RUNNER_ARGS}"
  -v /dev:/dev
  -v /dev/shm:/dev/shm
  -v /etc/localtime:/etc/localtime:ro
  -v "${LOG_DIR}:/data"
)
[[ -s "${OVERLAY_FILE}" ]] && DOCKER_ARGS+=(-e SLAM_PARAM_OVERLAY=/data/param_overlay.yaml)
[[ -f "${CYCLONEDDS_CONFIG}" ]] && DOCKER_ARGS+=(-e CYCLONEDDS_URI=/data/cyclonedds.xml -v "$(realpath "${CYCLONEDDS_CONFIG}"):/data/cyclonedds.xml:ro")
[[ -n "${FP_CONFIG}" ]]  && DOCKER_ARGS+=(-v "${FP_CONFIG}:/data/fixposition_config.yaml:ro" -e FP_CONFIG_FILE=/data/fixposition_config.yaml)
[[ -n "${CONFIG_DIR}" ]] && DOCKER_ARGS+=(-v "${CONFIG_DIR}:/root/ros2_ws/install/lio_slam/share/lio_slam/config:ro")

echo "=========================================="
echo " fslam 生产部署 | production deploy"
echo "   image    : ${IMAGE}"
echo "   profile  : ${PROFILE}   domain: ${ROS_DOMAIN_ID}"
echo "   fp驱动   : $([[ "${START_FIXPOSITION}" == "1" ]] && echo "on (${FP_CONFIG:-镜像内 m20.yaml | baked-in})" || echo off)"
echo "   输出     : ${PIPE_ODOM_TOPIC} 直出(无中继)| direct, no relay"
echo "   雷达     : ${LIDAR_TOPIC}"
echo "   宿主机   : fp_to_odom(/LOC_BODY_POINTS + /LOCATION_STATUS)"
echo "              motion_info 桥 | bridge: $([[ "${ENABLE_MOTION_INFO_BRIDGE}" == "1" ]] && echo on || echo off)"
echo "   模式     : $([[ "${DETACH}" == "1" ]] && echo 'detached + restart unless-stopped' || echo 'foreground (--rm)')"
echo "   log      : ${LOG_DIR}"
echo "=========================================="

# fslam 模式下 fp_to_odom 不中继:仅跟踪管线直出的 ${PIPE_ODOM_TOPIC},
# 发布 /LOC_BODY_POINTS + /LOCATION_STATUS;matching 健康监控用前端 /Odometry。
FP_TO_ODOM_MODE_ARGS="-p relay_enable:=false -p output_topic:=${PIPE_ODOM_TOPIC}"
FP_TO_ODOM_MODE_ARGS+=" -p lidar_input_topic:=${LIDAR_TOPIC} -p matching_odom_topic:=/Odometry"

start_host_nodes() {
  HOST_PID_FILES=()
  if [[ "${ENABLE_MOTION_INFO_BRIDGE}" == "1" ]]; then
    fslam_start_host_node motion_info_bridge "${HOST_BRIDGE_SCRIPT}" "${LOG_DIR}"
    HOST_PID_FILES+=("${LOG_DIR}/motion_info_bridge.pid")
  fi
  # shellcheck disable=SC2086  # ros-args 有意按词拆分 | intentional word-split
  fslam_start_host_node fp_to_odom "${FP_TO_ODOM_SCRIPT}" "${LOG_DIR}" \
    --ros-args ${FP_TO_ODOM_MODE_ARGS} ${FP_TO_ODOM_ARGS}
  HOST_PID_FILES+=("${LOG_DIR}/fp_to_odom.pid")
}

if [[ "${DETACH}" == "1" ]]; then
  docker run -d --restart unless-stopped "${DOCKER_ARGS[@]}" "${IMAGE}" bash /data/entrypoint.sh
  start_host_nodes
  fslam_start_container_watcher "${NAME}" "${LOG_DIR}" "${HOST_PID_FILES[@]}"
  docker ps --filter "name=${NAME}"
  echo "[INFO] 已后台启动 | started detached. 日志 | logs:"
  echo "       docker logs -f ${NAME}"
  echo "       ${LOG_DIR}/{fixposition,pipeline,fp_to_odom,motion_info_bridge}.log"
  echo "[INFO] 体检 | check: $0 --check     停止 | stop: docker stop ${NAME} && docker rm ${NAME}"
else
  cleanup_foreground() {
    for f in "${HOST_PID_FILES[@]:-}"; do fslam_stop_pidfile "${f}"; done
    command -v docker >/dev/null 2>&1 || return 0
    docker rm -f "${NAME}" >/dev/null 2>&1 || true
  }
  trap cleanup_foreground EXIT INT TERM
  start_host_nodes
  echo "[INFO] 前台运行,Ctrl-C 停止 | running attached; Ctrl-C to stop."
  docker run --rm "${DOCKER_ARGS[@]}" "${IMAGE}" bash /data/entrypoint.sh
fi
