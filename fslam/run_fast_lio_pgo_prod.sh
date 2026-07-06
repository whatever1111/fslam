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
set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

case "$(uname -m)" in
  aarch64|arm64) ARCH_TAG="arm64" ;;
  x86_64)        ARCH_TAG="amd64" ;;
  *)             ARCH_TAG="amd64" ;;
esac

IMAGE="${DOCKER_IMAGE:-wanderer123/fslam-humble:${ARCH_TAG}}"
NAME="${CONTAINER_NAME:-fslam-runtime}"
MEM="${DOCKER_MEM_LIMIT:-24g}"; SWAP="${DOCKER_MEM_SWAP:-28g}"
LOG_DIR="${LOG_DIR:-${HOME}/fslam/logs}"
PROFILE="m20"
FP_CONFIG=""            # 宿主机 FP 驱动 yaml；空 = 镜像内烘焙副本 | empty = baked-in image copy
CONFIG_DIR=""           # 宿主机 config/ 覆盖挂载；空 = 镜像内烘焙副本 | empty = baked-in image copy
FP_STREAM="${FP_STREAM:-}"
START_FIXPOSITION=1
ODOM_ALIAS="/ODOM"
OVERLAY=""
DETACH=1
RUNNER_ARGS=""          # 透传给镜像内 run_prod_native.sh | passed through to the in-image runner
LIDAR_TOPIC="${LIDAR_TOPIC:-/LIDAR/POINTS}"   # 雷达话题（体检用；驱动是狗自带服务）| for checks; driver is the dog's own service
DO_CHECK=0
HOST_BRIDGE_SCRIPT="${HOST_BRIDGE_SCRIPT:-}"
ENABLE_MOTION_INFO_BRIDGE="${ENABLE_MOTION_INFO_BRIDGE:-auto}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)        PROFILE="$2"; shift 2 ;;
    --image)          IMAGE="$2"; shift 2 ;;
    --name)           NAME="$2"; shift 2 ;;
    --mem)            MEM="$2"; shift 2 ;;
    --swap)           SWAP="$2"; shift 2 ;;
    --domain)         export ROS_DOMAIN_ID="$2"; shift 2 ;;
    --fp-config)      FP_CONFIG="$2"; shift 2 ;;
    --config-dir)     CONFIG_DIR="$2"; shift 2 ;;
    --fp-stream)      FP_STREAM="$2"; shift 2 ;;
    --no-fixposition) START_FIXPOSITION=0; shift ;;
    --odom-alias)     ODOM_ALIAS="$2"; shift 2 ;;
    --foxglove)       RUNNER_ARGS+=" --foxglove"; shift ;;
    --bridge-port)    RUNNER_ARGS+=" --bridge-port $2"; shift 2 ;;
    --param-overlay)  OVERLAY="$2"; shift 2 ;;
    --foreground)     DETACH=0; shift ;;
    --check)          DO_CHECK=1; shift ;;
    -h|--help) sed -n '2,58p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "[ERROR] 未知参数 | unknown arg: $1" >&2; exit 2 ;;
  esac
done

# ============================================================================
# 预检 | pre-checks
# ============================================================================
command -v docker >/dev/null 2>&1 || { echo "[ERROR] 本机无 docker | docker not available" >&2; exit 1; }
if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "[ERROR] 镜像不存在 | image not found: ${IMAGE}" >&2
  echo "        docker pull ${IMAGE}  （或从 release 资产 docker load）| or docker load a release tar" >&2
  exit 1
fi
if [[ -n "${FP_CONFIG}" ]]; then
  [[ -f "${FP_CONFIG}" ]] || { echo "[ERROR] FP 配置不存在 | FP config not found: ${FP_CONFIG}" >&2; exit 1; }
  FP_CONFIG="$(realpath "${FP_CONFIG}")"
fi
if [[ -n "${OVERLAY}" ]]; then
  [[ -f "${OVERLAY}" ]] || { echo "[ERROR] overlay 不存在 | overlay not found: ${OVERLAY}" >&2; exit 1; }
  OVERLAY="$(realpath "${OVERLAY}")"
fi
if [[ -n "${CONFIG_DIR}" ]]; then
  [[ -d "${CONFIG_DIR}/profiles" ]] || { echo "[ERROR] --config-dir 需含 profiles/ | must contain profiles/: ${CONFIG_DIR}" >&2; exit 1; }
  CONFIG_DIR="$(realpath "${CONFIG_DIR}")"
else
  # 自动发现：配置树随部署仓库 checkout 走（脚本同级或上一级的 config/）。
  # Auto-discover: the config tree travels with the deploy-repo checkout.
  for d in "${SCRIPT_DIR}/config" "${SCRIPT_DIR}/../config"; do
    if [[ -d "${d}/profiles" ]]; then CONFIG_DIR="$(realpath "${d}")"; break; fi
  done
fi

# motion_info 桥脚本同样随 checkout 自动发现 | the bridge script is auto-discovered too
if [[ -z "${HOST_BRIDGE_SCRIPT}" ]]; then
  for f in "${SCRIPT_DIR}/../motion_info_to_twist.py" "${SCRIPT_DIR}/motion_info_to_twist.py" \
           /home/user/fslam/motion_info_to_twist.py; do
    if [[ -f "${f}" ]]; then HOST_BRIDGE_SCRIPT="${f}"; break; fi
  done
fi

mkdir -p "${LOG_DIR}"; LOG_DIR="$(realpath "${LOG_DIR}")"

# ============================================================================
# --check：对运行中的部署做链路体检,不动容器 | health-check a RUNNING deployment
# 顺数据流逐话题测频:第一个无数据的话题 = 断点。
# Probe topic rates along the data flow: the FIRST silent topic is the break.
# ============================================================================
if [[ "${DO_CHECK}" == "1" ]]; then
  if ! docker ps --format '{{.Names}}' | grep -qx "${NAME}"; then
    echo "[ERROR] 容器未运行 | container not running: ${NAME}" >&2; exit 1
  fi
  echo "== 数据流体检(容器 ${NAME},每话题探测 4s)| data-flow check =="
  docker exec -i "${NAME}" bash -s -- "${LIDAR_TOPIC}" "${ODOM_ALIAS:-/ODOM}" <<'EOFCHECK'
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source /root/ros2_ws/install/setup.bash >/dev/null 2>&1
for t in "$1" /fixposition/fpa/corrimu /fixposition/fpa/odomenu \
         /lio/points /lio/imu /lio/gps_odom \
         /Odometry /fast_lio_pgo/odometry /odom "$2"; do
  printf '  %-32s ' "$t"
  rate="$(timeout 4 ros2 topic hz "$t" 2>/dev/null | grep -m1 -oE 'average rate: [0-9.]+')"
  echo "${rate:-无数据 | SILENT}"
done
EOFCHECK
  echo ""
  echo "== 关键日志签名 | log signatures (${LOG_DIR}) =="
  grep -hE 'first imu cb|first lidar|first GPS cb|alignment transform|No Effective|xy_floor' \
    "${LOG_DIR}/pipeline.log" 2>/dev/null | tail -6 || true
  echo "-- fixposition.log 尾部 | tail --"
  tail -5 "${LOG_DIR}/fixposition.log" 2>/dev/null || echo "  (无 | none)"
  echo "-- odom_alias.log 尾部 | tail --"
  tail -3 "${LOG_DIR}/odom_alias.log" 2>/dev/null || echo "  (无 | none)"
  echo ""
  echo "解读 | how to read:"
  echo "  ${LIDAR_TOPIC} SILENT      → 雷达驱动(狗自带服务)没在跑,本脚本不启动它"
  echo "  corrimu/odomenu SILENT     → FP 设备没出数据:查 fixposition.log(TCP/RTK/基站改正)"
  echo "  /lio/* SILENT              → adapter 异常:查 pipeline.log"
  echo "  /Odometry SILENT           → 前端未初始化(需点云+IMU 同时就绪)"
  echo "  仅 /odom(及别名) SILENT    → PGO 尚未对齐:等 'alignment transform' 日志行(需 GPS 因子+移动)"
  exit 0
fi

# 同名旧容器先清 | drop any pre-existing container with the same name
docker ps -a --format '{{.Names}}' | grep -qx "${NAME}" && {
  docker stop "${NAME}" >/dev/null 2>&1 || true
  docker rm "${NAME}" >/dev/null 2>&1 || true
}

# ============================================================================
# 宿主机 motion_info 桥（狗轮速 → FP 设备融合）| host-side motion-info bridge
# 需要狗主机自带的 ROS2 环境 + drdds 消息包，容器内不可用。
# Needs the dog's host ROS2 env + drdds message package; unavailable in-container.
# ============================================================================
HOST_BRIDGE_PID_FILE="${LOG_DIR}/motion_info_bridge.pid"
HOST_BRIDGE_WATCHER_PID_FILE="${LOG_DIR}/motion_info_bridge_watcher.pid"

cleanup_host_bridge() {
  if [[ -f "${HOST_BRIDGE_PID_FILE}" ]]; then
    local pid; pid="$(<"${HOST_BRIDGE_PID_FILE}")"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true; sleep 2
      kill "${pid}" 2>/dev/null || true
    fi
    rm -f "${HOST_BRIDGE_PID_FILE}"
  fi
}
cleanup_host_bridge_watcher() {
  if [[ -f "${HOST_BRIDGE_WATCHER_PID_FILE}" ]]; then
    local pid; pid="$(<"${HOST_BRIDGE_WATCHER_PID_FILE}")"
    if [[ -n "${pid}" ]]; then kill "${pid}" 2>/dev/null || true; fi
    rm -f "${HOST_BRIDGE_WATCHER_PID_FILE}"
  fi
}
cleanup_host_bridge_watcher
cleanup_host_bridge

if [[ "${ENABLE_MOTION_INFO_BRIDGE}" == "auto" ]]; then
  [[ -f "${HOST_BRIDGE_SCRIPT}" ]] && ENABLE_MOTION_INFO_BRIDGE=1 || ENABLE_MOTION_INFO_BRIDGE=0
fi
if [[ "${ENABLE_MOTION_INFO_BRIDGE}" == "1" && ! -f "${HOST_BRIDGE_SCRIPT}" ]]; then
  echo "[ERROR] 桥脚本不存在 | bridge script not found: ${HOST_BRIDGE_SCRIPT}" >&2; exit 1
fi

# ============================================================================
# 容器 entrypoint（写入 LOG_DIR，挂载为 /data；容器 restart 时重读，故 LOG_DIR 必须持久）
# container entrypoint (written to LOG_DIR → /data; re-read on container restart,
# hence LOG_DIR must survive reboots)
# ============================================================================
cat > "${LOG_DIR}/entrypoint.sh" <<'EOFSCRIPT'
#!/bin/bash
set -o pipefail
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
SHARE=/root/ros2_ws/install/lio_slam/share/lio_slam

# ---- ① Fixposition driver：先开 RTK 输出 | bring up the RTK streams FIRST ----
if [[ "${START_FIXPOSITION:-1}" == "1" ]]; then
  : "${FP_CONFIG_FILE:=${SHARE}/config/fixposition/m20.yaml}"
  if [[ ! -f "${FP_CONFIG_FILE}" ]]; then
    echo "[ERROR] FP 驱动配置缺失 | FP driver config missing: ${FP_CONFIG_FILE}" >&2
    exit 1
  fi
  # 写副本再按需覆盖 stream（镜像内原件只读）| copy then optionally override the stream
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

  # 等 RTK ENU 输出出现（odom_adapter 的输入）| wait for the RTK ENU stream (odom_adapter input)
  fp_up=0
  for _ in $(seq 1 30); do
    if timeout 5 ros2 topic list 2>/dev/null | grep -qx '/fixposition/fpa/odomenu'; then
      fp_up=1; break
    fi
    sleep 1
  done
  if [[ "${fp_up}" == "1" ]]; then
    # 话题存在 ≠ 有数据在流:实收一条 IMU 才算真就绪。
    # Topic presence ≠ data flowing: only a received IMU message counts as ready.
    if timeout 5 ros2 topic echo --once /fixposition/fpa/corrimu >/dev/null 2>&1; then
      echo "[INFO] RTK 输出就绪(实收数据)| RTK streams up (data verified): /fixposition/fpa/{corrimu,odomenu}"
    else
      echo "[WARN] FPA 话题存在但 corrimu 5s 无数据 —— FP 设备可能没在流(查 /data/fixposition.log)"
      echo "[WARN] FPA topics exist but corrimu delivered nothing in 5s — device likely not streaming"
    fi
  else
    echo "[WARN] 30s 内未见 /fixposition/fpa/odomenu（驱动仍在 respawn 重试，管线继续启动）"
    echo "[WARN] no /fixposition/fpa/odomenu within 30s (driver keeps respawning; continuing)"
    echo "[WARN] 查 | check: /data/fixposition.log（stream 地址? RTK 天线/基站?）"
  fi
fi

# 雷达是狗自带服务,本脚本不启动 —— 缺席只能告警。
# The LiDAR driver is the dog's own service, never started here — absence is warn-only.
if ! timeout 5 ros2 topic list 2>/dev/null | grep -qx "${LIDAR_TOPIC:-/LIDAR/POINTS}"; then
  echo "[WARN] 未发现雷达话题 ${LIDAR_TOPIC:-/LIDAR/POINTS} —— 确认狗上的雷达驱动服务在运行"
  echo "[WARN] lidar topic missing — make sure the dog's own lidar driver service is running"
fi

# ---- 可选 /odom → 别名中继（狗导航栈消费 /ODOM）| optional /odom alias relay ----
if [[ -n "${ODOM_ALIAS:-}" ]]; then
  cat > /data/odom_alias.py <<'EOFPY'
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


class OdomAlias(Node):
    def __init__(self):
        super().__init__("odom_alias")
        alias = os.environ.get("ODOM_ALIAS", "/ODOM")
        # /odom 为 SensorDataQoS(best-effort) → 订阅端必须 best-effort；
        # 发布端用 reliable，兼容下游 reliable/best-effort 两类订阅者。
        self.pub = self.create_publisher(Odometry, alias, 10)
        self.sub = self.create_subscription(Odometry, "/odom", self.cb, qos_profile_sensor_data)
        self.get_logger().info(f"relaying /odom -> {alias}")

    def cb(self, msg):
        self.pub.publish(msg)


rclpy.init()
rclpy.spin(OdomAlias())
EOFPY
  echo "[INFO] /odom 别名中继 | alias relay: /odom -> ${ODOM_ALIAS}"
  setsid python3 /data/odom_alias.py > /data/odom_alias.log 2>&1 &
fi

# ---- ② canonical 管线：镜像内四象限 prod 入口 | the in-image four-quadrant prod entry ----
# exec 后本进程即 run_prod_native.sh（容器 PID1 链）：其 trap 负责管线清理；
# 容器退出时运行时统一回收 fixposition/中继子进程。
# After exec this process IS run_prod_native.sh (container PID1 chain): its trap cleans
# the pipeline; the container runtime reaps the driver/relay children on exit.
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
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  # 与旧 fslam 实机部署一致用 CycloneDDS（镜像内已装）；可用环境变量换回 FastDDS。
  # CycloneDDS to match the validated fslam field deployment (shipped in the image);
  # override via RMW_IMPLEMENTATION for FastDDS.
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  -e OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
  -e OPENBLAS_NUM_THREADS=1 -e MKL_NUM_THREADS=1
  -e PROFILE="${PROFILE}"
  -e START_FIXPOSITION="${START_FIXPOSITION}"
  -e FP_STREAM="${FP_STREAM}"
  -e ODOM_ALIAS="${ODOM_ALIAS}"
  -e LIDAR_TOPIC="${LIDAR_TOPIC}"
  -e RUNNER_ARGS="${RUNNER_ARGS}"
  -v /dev:/dev
  -v /dev/shm:/dev/shm
  -v /etc/localtime:/etc/localtime:ro
  -v "${LOG_DIR}:/data"
)
[[ -n "${FP_CONFIG}" ]]  && DOCKER_ARGS+=(-v "${FP_CONFIG}:/data/fixposition_config.yaml:ro" -e FP_CONFIG_FILE=/data/fixposition_config.yaml)
[[ -n "${CONFIG_DIR}" ]] && DOCKER_ARGS+=(-v "${CONFIG_DIR}:/root/ros2_ws/install/lio_slam/share/lio_slam/config:ro")
[[ -n "${OVERLAY}" ]]    && DOCKER_ARGS+=(-v "${OVERLAY}:${OVERLAY}:ro" -e SLAM_PARAM_OVERLAY="${OVERLAY}")

echo "=========================================="
echo " fslam 生产部署 | production deploy"
echo "   image   : ${IMAGE}"
echo "   profile : ${PROFILE}   domain: ${ROS_DOMAIN_ID:-0}"
echo "   fp驱动  : $([[ "${START_FIXPOSITION}" == "1" ]] && echo "on (${FP_CONFIG:-镜像内 m20.yaml | baked-in})" || echo off)"
echo "   /odom→  : ${ODOM_ALIAS:-（禁用 | disabled）}"
echo "   模式    : $([[ "${DETACH}" == "1" ]] && echo 'detached + restart unless-stopped' || echo 'foreground (--rm)')"
echo "   log     : ${LOG_DIR}"
echo "=========================================="

if [[ "${DETACH}" == "1" ]]; then
  docker run -d --restart unless-stopped "${DOCKER_ARGS[@]}" "${IMAGE}" bash /data/entrypoint.sh
else
  # 前台按标签兜底回收（同 run_prod_docker.sh）| label-scoped reap like run_prod_docker.sh
  CONTAINER_LABEL="lio_slam.leader=$$"
  DOCKER_ARGS+=(--label "${CONTAINER_LABEL}")
  cleanup_container() {
    cleanup_host_bridge_watcher; cleanup_host_bridge
    command -v docker >/dev/null 2>&1 || return 0
    docker ps -q --filter "label=${CONTAINER_LABEL}" | xargs -r docker rm -f >/dev/null 2>&1 || true
  }
  trap cleanup_container EXIT INT TERM
fi

# ---- 宿主机桥 + 看门狗 | host bridge + watcher ------------------------------
if [[ "${ENABLE_MOTION_INFO_BRIDGE}" == "1" ]]; then
  HOST_ROS_SETUP=""
  for d in /opt/ros/foxy /opt/ros/humble; do
    [[ -f "${d}/setup.bash" ]] && HOST_ROS_SETUP="${d}/setup.bash" && break
  done
  if [[ -n "${HOST_ROS_SETUP}" ]]; then
    echo "[INFO] 宿主机 motion_info 桥 | host bridge: ${HOST_BRIDGE_SCRIPT} (${HOST_ROS_SETUP})"
    bash -c "source '${HOST_ROS_SETUP}' && exec python3 '${HOST_BRIDGE_SCRIPT}'" \
      > "${LOG_DIR}/motion_info_bridge.log" 2>&1 &
    printf '%s\n' "$!" > "${HOST_BRIDGE_PID_FILE}"
    # 看门狗：容器停则收桥 | watcher: reap the bridge when the container stops
    (
      while true; do
        state="$(docker inspect -f '{{if or .State.Running .State.Restarting}}up{{else}}down{{end}}' "${NAME}" 2>/dev/null || echo down)"
        [[ "${state}" == "up" ]] || break
        sleep 2
      done
      cleanup_host_bridge
      rm -f "${HOST_BRIDGE_WATCHER_PID_FILE}"
    ) &
    printf '%s\n' "$!" > "${HOST_BRIDGE_WATCHER_PID_FILE}"
  else
    echo "[WARN] 宿主机无 ROS2 环境，跳过 motion_info 桥 | no host ROS2 env, skipping the bridge" >&2
  fi
fi

if [[ "${DETACH}" == "1" ]]; then
  docker ps --filter "name=${NAME}"
  echo "[INFO] 已后台启动 | started detached. 日志 | logs:"
  echo "       docker logs -f ${NAME}"
  echo "       ${LOG_DIR}/{fixposition,pipeline,odom_alias}.log"
  echo "[INFO] 停止 | stop: docker stop ${NAME} && docker rm ${NAME}"
else
  echo "[INFO] 前台运行，Ctrl-C 停止 | running attached; Ctrl-C to stop."
  docker run --rm "${DOCKER_ARGS[@]}" "${IMAGE}" bash /data/entrypoint.sh
fi
