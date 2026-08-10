#!/usr/bin/env bash
# ============================================================================
# run_fslam_native.sh —— fslam 模式生产部署 · Foxy 原生(无容器)
#                        fslam-mode production deploy · native on Foxy
# ============================================================================
# 为什么有原生版:OEM 雷达的点云 writer 只挂 127.0.0.1(FastDDS 2.0 才发
# loopback locator),Humble 容器(FastDDS 2.6)看得见拓扑收不到数据,只能靠
# 中继。原生 Foxy 进程直接收 —— 无中继、无容器。
# Why native: the OEM cloud writer is loopback-only; Humble's Fast DDS 2.6
# cannot receive it (container mode needs the relay). A native Foxy process
# receives it directly — no relay, no container.
#
# 进程拓扑 | process topology (all native, all FastDDS defaults):
#   ① Fixposition driver(驱动 fork 的 Foxy 二进制,上游节点 + fslam 配置,
#      出 /fixposition/fpa/* 流,不出 /ODOM)
#      the driver fork's Foxy binaries running the UPSTREAM node with the
#      fslam config — FPA streams only, no /ODOM
#   ② canonical 管线(LIO-SLAM 原生 tarball 的 run_prod_native.sh --profile m20;
#      融合节点经 overlay 直出 ${ODOM_TOPIC},雷达 adapter 直吃 ${LIDAR_TOPIC})
#   ③ motion_info_to_twist.py(轮速 → FP 设备融合,需狗侧 drdds)
#   ④ fp_to_odom.py -p relay_enable:=false(跟踪 /ODOM,发 /LOC_BODY_POINTS
#      去畸变点云 + /LOCATION_STATUS)
#
# 用法 | usage: ./run_fslam_native.sh [选项]
#   --profile <name>     LIO-SLAM profile(默认 m20)
#   --slam-dir <dir>     LIO-SLAM 原生 tarball 解包目录(默认 /home/user/lio-slam;
#                        须含 env.sh —— 见 SLAM_NATIVE_RELEASE + DEPLOYMENT.md)
#   --driver-ws <dir>    驱动 fork 原生安装(默认 /home/user/m20_ws,与 rtk_only 共用)
#   --fp-config <yaml>   FP 驱动配置(默认 tarball 内 config/fixposition/m20.yaml)
#   --fp-stream <uri>    覆盖 FP 传感器流地址(如 tcpcli://10.21.31.66:21000)
#   --no-fixposition     不启驱动(RTK 已由别处提供)
#   --lidar-topic <t>    雷达话题(默认 /LIDAR/POINTS2 —— 当前固件;旧 bag 是 /LIDAR/POINTS)
#   --odom-topic <t>     管线直出话题(默认 /ODOM;空串 = 保持 /odom)
#   --domain <id>        ROS_DOMAIN_ID(默认 0)
#   --param-overlay <f>  在线改参 overlay(最高优先级层,与注入段合并)
#   --check              对运行中的部署做链路体检(逐话题测频)
#   --stop               停止本部署的全部进程
# 环境 | env: LOG_DIR ROS_DOMAIN_ID RMW_IMPLEMENTATION OMP_NUM_THREADS
#   HOST_BRIDGE_SCRIPT FP_TO_ODOM_SCRIPT ENABLE_MOTION_INFO_BRIDGE FP_TO_ODOM_ARGS
#
# 与 rtk_only(两种形态)、fslam docker 互斥 —— 都发 /ODOM。systemd 单元里
# Conflicts= 已写死;手动跑请自觉。 Mutually exclusive with rtk_only (either
# form) and fslam-docker — they all publish /ODOM.
# ============================================================================
set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/deploy_common.sh
source "${BASE_DIR}/lib/deploy_common.sh"

SLAM_DIR="${SLAM_DIR:-/home/user/lio-slam}"
DRIVER_WS="${DRIVER_WS:-/home/user/m20_ws}"
LOG_DIR="${LOG_DIR:-${BASE_DIR}/logs/fslam_native}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
HOST_BRIDGE_SCRIPT="${HOST_BRIDGE_SCRIPT:-${BASE_DIR}/host/motion_info_to_twist.py}"
FP_TO_ODOM_SCRIPT="${FP_TO_ODOM_SCRIPT:-${BASE_DIR}/host/fp_to_odom.py}"
ENABLE_MOTION_INFO_BRIDGE="${ENABLE_MOTION_INFO_BRIDGE:-auto}"
FP_TO_ODOM_ARGS="${FP_TO_ODOM_ARGS:-}"

PROFILE="m20"
FP_CONFIG=""
FP_STREAM="${FP_STREAM:-}"
START_FIXPOSITION=1
LIDAR_TOPIC="${LIDAR_TOPIC:-/LIDAR/POINTS2}"
ODOM_TOPIC="/ODOM"
USER_OVERLAY=""
DO_CHECK=0
DO_STOP=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)        PROFILE="$2"; shift 2 ;;
    --slam-dir)       SLAM_DIR="$2"; shift 2 ;;
    --driver-ws)      DRIVER_WS="$2"; shift 2 ;;
    --fp-config)      FP_CONFIG="$2"; shift 2 ;;
    --fp-stream)      FP_STREAM="$2"; shift 2 ;;
    --no-fixposition) START_FIXPOSITION=0; shift ;;
    --lidar-topic)    LIDAR_TOPIC="$2"; shift 2 ;;
    --odom-topic)     ODOM_TOPIC="$2"; shift 2 ;;
    --domain)         ROS_DOMAIN_ID="$2"; shift 2 ;;
    --param-overlay)  USER_OVERLAY="$2"; shift 2 ;;
    --check)          DO_CHECK=1; shift ;;
    --stop)           DO_STOP=1; shift ;;
    -h|--help) sed -n '2,47p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "[ERROR] 未知参数 | unknown arg: $1" >&2; exit 2 ;;
  esac
done

PIPE_ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
mkdir -p "${LOG_DIR}"
LOG_DIR="$(realpath "${LOG_DIR}")"
PID_FILES=("${LOG_DIR}/driver.pid" "${LOG_DIR}/pipeline.pid"
           "${LOG_DIR}/fp_to_odom.pid" "${LOG_DIR}/motion_info_bridge.pid")

stop_all() {
  # 管线是 setsid 进程组(component_container 等经 launch 派生)——整组回收。
  # The pipeline is a setsid process GROUP (launch spawns the container etc.).
  if [[ -f "${LOG_DIR}/pipeline.pid" ]]; then
    local pg
    pg="$(<"${LOG_DIR}/pipeline.pid")"
    if [[ -n "${pg}" ]] && kill -0 "${pg}" 2>/dev/null; then
      kill -INT -- "-${pg}" 2>/dev/null || true
      sleep 3
      kill -KILL -- "-${pg}" 2>/dev/null || true
    fi
    rm -f "${LOG_DIR}/pipeline.pid"
  fi
  local f
  for f in "${PID_FILES[@]}"; do fslam_stop_pidfile "${f}"; done
}

if [[ "${DO_STOP}" == "1" ]]; then
  stop_all
  echo "[INFO] fslam native 已停止 | stopped."
  exit 0
fi

# ---- 预检 | pre-checks ------------------------------------------------------
HOST_ROS_SETUP="$(fslam_host_ros_setup)" \
  || { echo "[ERROR] 宿主机无 ROS2 环境 | no host ROS2 env under /opt/ros" >&2; exit 1; }
[[ -f "${SLAM_DIR}/env.sh" ]] || {
  echo "[ERROR] LIO-SLAM 原生包缺失 | native bundle missing: ${SLAM_DIR}/env.sh" >&2
  echo "        版本见 SLAM_NATIVE_RELEASE,部署步骤见 DEPLOYMENT.md(fslam · native)" >&2
  echo "        (pinned in SLAM_NATIVE_RELEASE; deploy steps: DEPLOYMENT.md, fslam · native)" >&2
  exit 1
}
if [[ "${START_FIXPOSITION}" == "1" ]]; then
  [[ -f "${DRIVER_WS}/install/setup.bash" ]] || {
    echo "[ERROR] 驱动 fork 原生安装缺失 | driver-fork native install missing: ${DRIVER_WS}" >&2
    echo "        rtk_only 的原生部署与此共用一份 —— 见 DEPLOYMENT.md(rtk_only · native)" >&2
    exit 1
  }
fi
[[ -f "${FP_TO_ODOM_SCRIPT}" ]] || { echo "[ERROR] 缺 fp_to_odom | missing: ${FP_TO_ODOM_SCRIPT}" >&2; exit 1; }
if [[ "${ENABLE_MOTION_INFO_BRIDGE}" == "auto" ]]; then
  [[ -f "${HOST_BRIDGE_SCRIPT}" ]] && ENABLE_MOTION_INFO_BRIDGE=1 || ENABLE_MOTION_INFO_BRIDGE=0
fi
if [[ -n "${USER_OVERLAY}" ]]; then
  [[ -f "${USER_OVERLAY}" ]] || { echo "[ERROR] overlay 不存在 | not found: ${USER_OVERLAY}" >&2; exit 1; }
  USER_OVERLAY="$(realpath "${USER_OVERLAY}")"
fi

# ---- --check:链路体检 | data-flow health check ------------------------------
if [[ "${DO_CHECK}" == "1" ]]; then
  echo "== 数据流体检(原生,每话题探测 4s)| data-flow check (native) =="
  set +u
  # shellcheck disable=SC1090,SC1091
  source "${HOST_ROS_SETUP}"
  # shellcheck disable=SC1091
  source "${SLAM_DIR}/env.sh"
  set -u
  export ROS_DOMAIN_ID RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  for t in "${LIDAR_TOPIC}" /fixposition/fpa/corrimu /fixposition/fpa/odomenu \
           /lio/points /lio/imu /lio/gps_odom \
           /Odometry /fast_lio_pgo/odometry "${PIPE_ODOM_TOPIC}" \
           /LOC_BODY_POINTS /LOCATION_STATUS; do
    printf '  %-32s ' "${t}"
    rate="$(timeout 4 ros2 topic hz "${t}" 2>/dev/null | grep -m1 -oE 'average rate: [0-9.]+')"
    echo "${rate:-无数据 | SILENT}"
  done
  echo ""
  echo "== 关键日志签名 | log signatures (${LOG_DIR}) =="
  grep -hE 'first imu cb|first lidar|first GPS cb|alignment transform|No Effective|xy_floor' \
    "${LOG_DIR}"/pipeline_run/pipeline.log 2>/dev/null | tail -6 || true
  for f in fixposition fp_to_odom motion_info_bridge; do
    echo "-- ${f}.log 尾部 | tail --"
    tail -3 "${LOG_DIR}/${f}.log" 2>/dev/null || echo "  (无 | none)"
  done
  exit 0
fi

# ---- 停掉别的部署 | stop competing deployments -------------------------------
# 全都发 /ODOM:fslam docker、rtk_only 原生、rtk_only docker、旧名部署。
if command -v docker >/dev/null 2>&1; then
  for c in fslam-runtime rtk-only-runtime m20-runtime fixposition-runtime; do
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "${c}"; then
      echo "[INFO] 停掉容器部署 | stopping container deployment: ${c}"
      docker stop "${c}" >/dev/null 2>&1 || true
    fi
  done
fi
for d in rtk_only fixposition_only fslam; do
  for f in driver rsp fp_to_odom motion_info_bridge watcher; do
    fslam_stop_pidfile "${BASE_DIR}/logs/${d}/${f}.pid"
  done
done
stop_all

# ---- 参数 overlay:雷达直吃 + /ODOM 直出 | the injected top layer ------------
OVERLAY_FILE="${LOG_DIR}/param_overlay.yaml"
: > "${OVERLAY_FILE}"
if [[ -n "${USER_OVERLAY}" ]]; then
  cat "${USER_OVERLAY}" >> "${OVERLAY_FILE}"
  echo "" >> "${OVERLAY_FILE}"
  for sec in fast_lio_pgo_fusion lidar_adapter; do
    if grep -qE "^${sec}:" "${USER_OVERLAY}"; then
      echo "[ERROR] --param-overlay 已含 ${sec} 段;请把注入键并进该段后重试:" >&2
      echo "        lidar_adapter.input_topic: ${LIDAR_TOPIC} / fast_lio_pgo_fusion.odom_topic: ${PIPE_ODOM_TOPIC}" >&2
      echo "        (your overlay already has a ${sec} section — merge the injected keys yourself)" >&2
      exit 1
    fi
  done
fi
cat >> "${OVERLAY_FILE}" <<EOF
# injected by run_fslam_native.sh —— 原生直吃雷达话题 + 直出 ${PIPE_ODOM_TOPIC}
# native direct lidar input + direct odom output (no relay anywhere)
lidar_adapter:
  ros__parameters:
    input_topic: ${LIDAR_TOPIC}
EOF
if [[ -n "${ODOM_TOPIC}" ]]; then
  cat >> "${OVERLAY_FILE}" <<EOF
fast_lio_pgo_fusion:
  ros__parameters:
    odom_topic: ${ODOM_TOPIC}
EOF
fi

echo "=========================================="
echo " fslam 生产部署 · 原生 | production deploy · native"
echo "   slam     : ${SLAM_DIR} ($(cat "${SLAM_DIR}/.ros_distro" 2>/dev/null || echo '?'))"
echo "   profile  : ${PROFILE}   domain: ${ROS_DOMAIN_ID}"
echo "   fp驱动   : $([[ "${START_FIXPOSITION}" == "1" ]] && echo "on (${DRIVER_WS})" || echo off)"
echo "   雷达     : ${LIDAR_TOPIC}(直吃,无中继 | direct, no relay)"
echo "   输出     : ${PIPE_ODOM_TOPIC} 直出 + /LOC_BODY_POINTS + /LOCATION_STATUS"
echo "   桥       : motion_info $([[ "${ENABLE_MOTION_INFO_BRIDGE}" == "1" ]] && echo on || echo off)"
echo "   log      : ${LOG_DIR}"
echo "=========================================="

export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# ---- ① FP 驱动(fork 二进制 · 上游节点 · fslam 配置)| the FP driver ---------
if [[ "${START_FIXPOSITION}" == "1" ]]; then
  # 默认配置在 tarball 里(fslam 语义:FPA 流,无 /ODOM)。写运行时副本再覆盖
  # stream。 Default config ships in the tarball (FPA streams, no /ODOM);
  # copy, then optionally override the sensor stream.
  : "${FP_CONFIG:=${SLAM_DIR}/install/lio_slam/share/lio_slam/config/fixposition/m20.yaml}"
  [[ -f "${FP_CONFIG}" ]] || { echo "[ERROR] FP 配置缺失 | FP config missing: ${FP_CONFIG}" >&2; exit 1; }
  cp "${FP_CONFIG}" "${LOG_DIR}/fixposition_runtime.yaml"
  if [[ -n "${FP_STREAM}" ]]; then
    sed -i "s|^\([[:space:]]*stream:\).*|\1 ${FP_STREAM}|" "${LOG_DIR}/fixposition_runtime.yaml"
    echo "[INFO] FP stream 覆盖 | overridden: ${FP_STREAM}"
  fi
  # 直接起可执行文件,不走 launch XML(Foxy 方言问题,同 run_rtk_only_native.sh)
  setsid bash -c "set +u \
    && source '${HOST_ROS_SETUP}' \
    && source '${DRIVER_WS}/install/setup.bash' \
    && export LD_LIBRARY_PATH='${DRIVER_WS}/fpsdk/lib:'\"\${LD_LIBRARY_PATH:-}\" \
    && export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && exec ros2 run fixposition_driver_ros2 fixposition_driver_ros2_exec \
         --ros-args -r __node:=fixposition_driver_ros2 \
         --params-file '${LOG_DIR}/fixposition_runtime.yaml'" \
    > "${LOG_DIR}/fixposition.log" 2>&1 &
  echo "$!" > "${LOG_DIR}/driver.pid"
  echo "[INFO] FP driver up (pid $(<"${LOG_DIR}/driver.pid"))"
fi

# ---- ② canonical 管线 | the canonical pipeline ------------------------------
mkdir -p "${LOG_DIR}/pipeline_run"
setsid bash -c "set +u \
  && source '${SLAM_DIR}/env.sh' \
  && export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION}' \
  && export OMP_NUM_THREADS='${OMP_NUM_THREADS:-4}' OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  && export SLAM_PARAM_OVERLAY='${OVERLAY_FILE}' \
  && exec bash \"\${LIO_SLAM_SHARE}/tools/run_prod_native.sh\" \
       --profile '${PROFILE}' --log '${LOG_DIR}/pipeline_run'" \
  > "${LOG_DIR}/pipeline.log" 2>&1 &
echo "$!" > "${LOG_DIR}/pipeline.pid"
echo "[INFO] pipeline up (pgid $(<"${LOG_DIR}/pipeline.pid"), log ${LOG_DIR}/pipeline_run/pipeline.log)"

# ---- ③④ 宿主机胶水(与 docker 版同一套脚本)| the same host glue ------------
if [[ "${ENABLE_MOTION_INFO_BRIDGE}" == "1" ]]; then
  fslam_start_host_node motion_info_bridge "${HOST_BRIDGE_SCRIPT}" "${LOG_DIR}"
fi
# shellcheck disable=SC2086  # ros-args 有意按词拆分 | intentional word-split
fslam_start_host_node fp_to_odom "${FP_TO_ODOM_SCRIPT}" "${LOG_DIR}" \
  --ros-args -p relay_enable:=false -p output_topic:="${PIPE_ODOM_TOPIC}" \
  -p lidar_input_topic:="${LIDAR_TOPIC}" -p matching_odom_topic:=/Odometry \
  ${FP_TO_ODOM_ARGS}

sleep 5
for n in driver pipeline; do
  [[ "${n}" == "driver" && "${START_FIXPOSITION}" != "1" ]] && continue
  pid="$(cat "${LOG_DIR}/${n}.pid" 2>/dev/null || true)"
  if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
    echo "[ERROR] ${n} 起来就死了 | died on startup — see ${LOG_DIR}/${n/driver/fixposition}*.log" >&2
    stop_all
    exit 1
  fi
done
echo "[INFO] fslam native 已启动 | started. 体检 | check: $0 --check   停止 | stop: $0 --stop"
