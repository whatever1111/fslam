#!/usr/bin/env bash
# ============================================================================
# verify_drive.sh —— 一次遛狗把该验的都验了:录包 + 雷达时戳模式探针 + FE 偏航探针
#                    + 管线日志里的 [REFIX]/守卫事件,结束时给一页报告(狗上跑,root)
# One drive, every open verification at once: recording + lidar header-stamp mode
# probe + FE-yaw-vs-FP probe + the pipeline log's [REFIX]/guard events, one report
# at the end. Runs on the robot as root.
# ============================================================================
# 用法 | usage:
#   sudo ./tools/verify_drive.sh [-d 秒|0] [-o bag目录] [--no-record] [--start-slam [--slam-args "..."]]
#     -d            时长,0 = 一直跑到 Ctrl-C(默认 0)| duration, 0 = until Ctrl-C (default)
#     -o            bag 输出目录(默认 /home/user/bags)| bag out dir
#     --no-record   不录包(只跑探针/看日志)| probes + log only
#     --start-slam  同时拉起 fslam native 管线(--no-fixposition,直出话题改成
#                   /lio_verify_odom,不碰 rtk_only 的 /ODOM),结束时停掉;不加则只
#                   观察已在跑的管线(如果有)| also start the native pipeline (FP driver
#                   from rtk_only, odom on /lio_verify_odom so /ODOM keeps one owner) and
#                   stop it at the end; without it, only watch a running pipeline
#     --slam-args   透传给 run_fslam_native.sh 的额外参数 | extra run_fslam_native.sh args
#     --force-record-with-slam  允许录包和管线同时跑(4 核狗实测 load 15、FE 时延 2.3 s、
#                   丢帧——结果不可用;默认 --start-slam 自动关录包)| let recording and
#                   the pipeline run together (measured on the 4-core dog: load 15, FE
#                   latency 2.3 s, dropped scans — useless numbers; by default
#                   --start-slam turns recording off)
#
# 两种用法 | two ways to use it:
#   A) 录包遛狗(默认):录 + 雷达时戳探针;拿包回开发机跑离线/融合评测。
#      recording drive (default): record + lidar-stamp probe; evaluate the bag offline.
#   B) 实时遛狗:--start-slam(自动不录):[REFIX] 拉回 + [YAW] 漂移 + 时戳探针,不用录包。
#      live drive: --start-slam (recording off): [REFIX] pull + [YAW] drift + stamp probe, no bag.
#
# 结束时的报告 | the report at the end:
#   [LIDAR-STAMP]  header 时戳模式 A/B 占比、翻转次数与时刻(A = 末点+13 ms,应为 0)
#   [YAW]          FIX 段内 FE−FP 偏航偏移随里程的漂移(应 <2°/100 m)
#   [REFIX]        管线自己打的重固定拉回(pull2D/along/cross;≥10 s 断 FIX 后第一条)
#   [GUARD]        DIVERGENCE/HOLE 守卫次数、clouds in/out、FE 时延
#   [BAG]          bag 路径(拿回开发机跑 run_offline_replay / run_eval_docker)
#   [MANUAL]       人工项:量 RoboSense↔VRTK2 杆臂(FE 在线估到 ≈(−0.25,+0.04,−0.02) m)
# ============================================================================
set -uo pipefail
if [[ ${EUID} -ne 0 ]]; then
  echo "[ERROR] 必须 root(SHM 不跨 UID)| must run as root" >&2
  exit 1
fi
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FSLAM="$(cd "${HERE}/.." && pwd)"
DUR=0; OUTDIR=/home/user/bags; RECORD=1; START_SLAM=0; SLAM_ARGS=""; FORCE_BOTH=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    -d) DUR="$2"; shift 2 ;;
    -o) OUTDIR="$2"; shift 2 ;;
    --no-record) RECORD=0; shift ;;
    --start-slam) START_SLAM=1; shift ;;
    --slam-args) SLAM_ARGS="$2"; shift 2 ;;
    --force-record-with-slam) FORCE_BOTH=1; shift ;;
    -h|--help) sed -n '2,39p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "[ERROR] unknown arg: $1" >&2; exit 2 ;;
  esac
done
if [[ ${START_SLAM} -eq 1 && ${RECORD} -eq 1 && ${FORCE_BOTH} -eq 0 ]]; then
  echo "[WARN] --start-slam: recording turned OFF (pipeline + two recorders saturate the 4-core dog:"
  echo "       measured load 15, FE latency 2.3 s, dropped scans). Use --force-record-with-slam to override."
  RECORD=0
fi
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG="${FSLAM}/logs/verify_${STAMP}"
mkdir -p "${LOG}"
PIPE_LOG="${FSLAM}/logs/fslam_native/pipeline_run/pipeline.log"
SLAM_ENV=/home/user/lio-slam/env.sh
DRIVER_WS=/home/user/m20_ws/install/setup.bash

# ROS env for the probes: Foxy + driver msgs (fixposition_driver_msgs for gnss status)
ros_env() {
  set +u
  source /opt/ros/foxy/setup.bash >/dev/null 2>&1
  [[ -f "${DRIVER_WS}" ]] && source "${DRIVER_WS}" >/dev/null 2>&1
  [[ -f "${SLAM_ENV}" ]] && source "${SLAM_ENV}" >/dev/null 2>&1
  set -u
}

PIDS=()
cleanup_done=0
finish() {
  [[ ${cleanup_done} -eq 1 ]] && return
  cleanup_done=1
  echo; echo "[INFO] finishing — stopping probes/recorder, collecting the report ..."
  # probes: SIGINT → they print their summaries
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
  # recorder (its own process group; it finalizes metadata/journal/chown on SIGINT)
  if [[ -n "${REC_PGID:-}" ]]; then kill -INT -- "-${REC_PGID}" 2>/dev/null || true; fi
  for _ in $(seq 1 60); do
    alive=0
    for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
    [[ -n "${REC_PGID:-}" ]] && kill -0 -- "-${REC_PGID}" 2>/dev/null && alive=1
    [[ ${alive} -eq 0 ]] && break
    sleep 1
  done
  [[ -n "${TAIL_PID:-}" ]] && kill "${TAIL_PID}" 2>/dev/null || true
  if [[ ${START_SLAM} -eq 1 ]]; then "${FSLAM}/run_fslam_native.sh" --stop >/dev/null 2>&1 || true; fi
  report
}
trap finish INT TERM

report() {
  local R="${LOG}/REPORT.txt"
  {
    echo "===================== verify_drive report ${STAMP} ====================="
    echo "log dir: ${LOG}"
    echo
    echo "--- [LIDAR-STAMP] OEM header-stamp mode (want: mode A 0 %, flips 0) ---"
    grep -a "LIDAR-STAMP" "${LOG}/lidar_stamp.log" 2>/dev/null || echo "(no probe output)"
    echo
    echo "--- [YAW] FE yaw vs FP heading (want: FIX-epoch drift < 2°/100 m; needs the pipeline running) ---"
    grep -a "\[YAW\]" "${LOG}/yaw.log" 2>/dev/null || echo "(no probe output)"
    echo
    echo "--- [REFIX] live re-fix pull from the pipeline (want: pull2D ≲ 0.6 m) ---"
    if [[ -f "${PIPE_LOG}" ]]; then
      grep -a "\[REFIX\]" "${PIPE_LOG}" | sed 's/^\[[^]]*\] //' | tail -20 || true
      [[ $(grep -ac "\[REFIX\]" "${PIPE_LOG}") -eq 0 ]] && echo "(none — no ≥10 s FIX gap during the drive, or pipeline not running)"
      echo
      echo "--- [GUARD] pipeline health ---"
      echo "guard starved resets: $(grep -ac 'DIVERGENCE-GUARD\] starved' "${PIPE_LOG}")   engulf released: $(grep -ac 'engulfment released' "${PIPE_LOG}")   hole-guard holds: $(grep -ac 'HOLE-GUARD' "${PIPE_LOG}")   NEP: $(grep -ac 'No Effective' "${PIPE_LOG}")"
      echo "adapter header-stamp warnings: $(grep -ac 'header stamp is' "${PIPE_LOG}")   out-of-order drops: $(grep -ac 'out of order' "${PIPE_LOG}")"
      grep -a "clouds in=" "${PIPE_LOG}" | tail -1 | sed 's/^.*lidar_adapter: /clouds: /' || true
      grep -a "SENSOR_TO_ODOM" "${PIPE_LOG}" | tail -1 | sed 's/^.*\[LATENCY\]/FE latency:/' || true
    else
      echo "(no pipeline log at ${PIPE_LOG} — SLAM was not running)"
    fi
    echo
    echo "--- [BAG] ---"
    if [[ ${RECORD} -eq 1 ]]; then
      grep -a -iE "bag|录制|record" "${LOG}/record.log" | tail -6 || true
      ls -dt "${OUTDIR}"/fslam_inputs_* 2>/dev/null | head -1 | sed 's/^/newest: /'
    else
      echo "(recording disabled)"
    fi
    echo
    echo "--- [MANUAL] ---"
    echo "1. Measure the RoboSense ↔ VRTK2 lever arm (lidar origin in the VRTK2/IMU frame). FE online estimate"
    echo "   converges to ≈ (−0.25, +0.04, −0.02) m; if the tape agrees, put it in profiles/m20/fast_lio.yaml extrinsic_T."
    echo "2. Note the drive: where FIX was lost/regained (building, trees), in-place turns, speed. Bring the bag back:"
    echo "   tools/run_offline_replay.sh --profile m20 --bag <bag> (FE, deterministic) / run_eval_docker.sh (fused)."
    echo "======================================================================="
  } | tee "${R}"
  echo "[INFO] report saved: ${R}"
}

echo "[INFO] verify_drive ${STAMP}: dur=${DUR}s record=${RECORD} start_slam=${START_SLAM} log=${LOG}"

if [[ ${START_SLAM} -eq 1 ]]; then
  # shellcheck disable=SC2086
  "${FSLAM}/run_fslam_native.sh" --no-fixposition --odom-topic /lio_verify_odom ${SLAM_ARGS} 2>&1 | tail -3
  sleep 5
fi
if [[ ! -f "${PIPE_LOG}" ]] || ! pgrep -f "^/home/user/lio-slam/install" >/dev/null 2>&1; then
  echo "[WARN] no running SLAM pipeline detected — [REFIX]/[YAW] will be empty (use --start-slam or start run_fslam_native.sh first)"
fi

# probes (nice: never compete with the pipeline)
( ros_env; exec nice -n 10 python3 "${HERE}/lidar_stamp_probe.py" /LIDAR/POINTS 0 "${LOG}/lidar_stamp.csv" ) > "${LOG}/lidar_stamp.log" 2>&1 &
PIDS+=($!)
( ros_env; exec nice -n 10 python3 "${HERE}/yaw_probe.py" 0 "${LOG}/yaw.csv" ) > "${LOG}/yaw.log" 2>&1 &
PIDS+=($!)

# pipeline event tail (what to look at while driving)
if [[ -f "${PIPE_LOG}" ]]; then
  ( tail -n0 -F "${PIPE_LOG}" | grep --line-buffered -E "REFIX|DIVERGENCE-GUARD|HOLE-GUARD|header stamp|No Effective" ) > "${LOG}/pipeline_events.log" 2>&1 &
  TAIL_PID=$!
fi

# recorder
if [[ ${RECORD} -eq 1 ]]; then
  setsid bash "${HERE}/record_fslam_inputs.sh" -d "${DUR}" -o "${OUTDIR}" > "${LOG}/record.log" 2>&1 &
  REC_PGID=$!
  sleep 3
  if ! kill -0 "${REC_PGID}" 2>/dev/null; then
    echo "[ERROR] recorder exited early:"; tail -5 "${LOG}/record.log"
    finish; exit 1
  fi
fi

echo "[INFO] running. Live events: tail -f ${LOG}/pipeline_events.log    Stop: Ctrl-C (or wait ${DUR}s)"
if [[ "${DUR}" != "0" ]]; then
  end=$(( $(date +%s) + DUR ))
  while [[ $(date +%s) -lt ${end} ]]; do sleep 1; done
  finish
else
  while :; do sleep 1; done
fi
