# shellcheck shell=bash
# ============================================================================
# deploy_common.sh —— fslam 部署脚本共享函数 | shared helpers for the deploy
# scripts (sourced by run_fixposition_prod.sh / run_fast_lio_pgo_prod.sh).
# 所有路径由调用方从 checkout 位置推导,本文件不含任何绝对路径。
# All paths are derived by the callers from the checkout location — this file
# contains no absolute paths.
# ============================================================================

fslam_arch_tag() {
  case "$(uname -m)" in
    aarch64|arm64) echo arm64 ;;
    *)             echo amd64 ;;
  esac
}

# 宿主机 ROS2 环境自动发现(狗上通常是 foxy)| first available host ROS2 env
fslam_host_ros_setup() {
  local d
  for d in /opt/ros/foxy /opt/ros/galactic /opt/ros/humble; do
    if [[ -f "${d}/setup.bash" ]]; then
      echo "${d}/setup.bash"
      return 0
    fi
  done
  return 1
}

# 按 pid 文件停进程:先 INT 给 rclpy 清理机会,2s 后强杀。
# Stop a process by pid file: INT first (rclpy cleanup), hard kill after 2 s.
fslam_stop_pidfile() {
  local f="$1" pid
  [[ -f "${f}" ]] || return 0
  pid="$(<"${f}")"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -INT "${pid}" 2>/dev/null || true
    sleep 2
    kill "${pid}" 2>/dev/null || true
  fi
  rm -f "${f}"
}

# 起一个宿主机 ROS2 python 节点(狗侧 FastDDS 约定),日志/记 pid 到 LOG 目录。
# setsid → 不随 ssh 会话挂断。$4 起为透传给节点的 --ros-args 参数。
# Launch a host-side ROS2 python node under the dog's FastDDS convention,
# logging + pid file into the log dir. setsid → survives ssh hangup.
# Args $4+ are passed to the node verbatim (e.g. --ros-args -p k:=v).
# Env: ROS_DOMAIN_ID, FASTDDS_CONFIG, HOST_ROS_SETUP.
fslam_start_host_node() {
  local name="$1" script="$2" logdir="$3"
  shift 3
  local profile_env=""
  if [[ -n "${FASTDDS_CONFIG:-}" && -f "${FASTDDS_CONFIG}" ]]; then
    profile_env="export FASTRTPS_DEFAULT_PROFILES_FILE='${FASTDDS_CONFIG}' && "
  fi
  setsid bash -c "source '${HOST_ROS_SETUP}' \
    && export ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' \
    && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && ${profile_env}exec python3 '${script}' $*" \
    > "${logdir}/${name}.log" 2>&1 &
  printf '%s\n' "$!" > "${logdir}/${name}.pid"
  echo "[INFO] host node '${name}' up (pid $(<"${logdir}/${name}.pid"), log ${logdir}/${name}.log)"
}

# 看门狗:容器停则回收宿主机节点。先等容器出现(最多 60s),再盯运行状态。
# Watcher: reap the host nodes when the container stops. Waits for the
# container to APPEAR first (≤60 s) so a slow docker start is not read as
# "already stopped". Args: $1 container  $2 log dir  $3+ pid files to reap.
fslam_start_container_watcher() {
  local cname="$1" logdir="$2"
  shift 2
  local files=("$@")
  setsid bash -c '
    cname="$1"; logdir="$2"; shift 2
    state() { docker inspect -f "{{if or .State.Running .State.Restarting}}up{{else}}down{{end}}" "${cname}" 2>/dev/null || echo down; }
    for _ in $(seq 1 30); do [[ "$(state)" == "up" ]] && break; sleep 2; done
    while [[ "$(state)" == "up" ]]; do sleep 2; done
    for f in "$@"; do
      pid="$(cat "${f}" 2>/dev/null)"
      if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
        kill -INT "${pid}" 2>/dev/null; sleep 2; kill "${pid}" 2>/dev/null
      fi
      rm -f "${f}"
    done
    rm -f "${logdir}/watcher.pid"
  ' watcher "${cname}" "${logdir}" "${files[@]}" &
  printf '%s\n' "$!" > "${logdir}/watcher.pid"
}

# 停旧容器 + 旧宿主机节点(按 pid 文件)| stop a previous deployment
fslam_stop_previous() {
  local cname="$1" logdir="$2"
  shift 2
  if docker ps -a --format '{{.Names}}' | grep -qx "${cname}"; then
    docker stop "${cname}" >/dev/null 2>&1 || true
    docker rm "${cname}" >/dev/null 2>&1 || true
  fi
  local f
  for f in "$@" "${logdir}/watcher.pid"; do
    fslam_stop_pidfile "${f}"
  done
}
