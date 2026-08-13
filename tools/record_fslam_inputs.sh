#!/usr/bin/env bash
# ============================================================================
# record_fslam_inputs.sh —— 录制 fslam 离线评测所需的全部输入话题(狗上跑)
#                           record every input topic fslam needs, on the robot
# ============================================================================
# 录的是 fslam 模式的**输入**(雷达/IMU/轮速/FP 流)+ 可选的 rtk_only **输出**
# 作参考真值(/ODOM 三话题 + handler 别名)。回放评测在开发机做,不在狗上。
# Records the fslam-mode INPUTS (lidar/IMU/wheel/FP streams) plus, optionally,
# the live rtk_only OUTPUTS as reference (the /ODOM triplet + handler aliases).
# Replay/eval happens on the dev machine, not on the dog.
#
# 用法 | usage:
#   ./record_fslam_inputs.sh [-d 秒 seconds] [-o 输出目录 outdir] [--no-reference]
#     -d  录制时长,默认 120s | duration, default 120 s
#     -o  bag 输出目录,默认 /home/user/bags | output dir, default /home/user/bags
#     --no-reference  不录 rtk_only 输出参考 | skip the rtk_only reference outputs
#
# 体积预估 | size estimate: 输入 ~14 MB/s(雷达为主),含参考 ~24 MB/s。
# 默认 120s ≈ 1.7/2.9 GB。脚本要求磁盘余量 ≥ 预估 ×1.5,不够直接拒绝。
# Inputs ~14 MB/s (lidar-dominated), ~24 MB/s with reference. 120 s ≈ 1.7/2.9 GB.
# The script refuses to start unless free disk ≥ 1.5× the estimate.
#
# QoS:统一 best_effort/volatile 订阅 —— 对任何 writer 都兼容(BE reader 可配
# Reliable writer),避免 Foxy rosbag2 的 QoS 不匹配静默丢录。
# Subscribes best_effort/volatile across the board — compatible with any writer
# and immune to Foxy rosbag2's silent QoS-mismatch no-record trap.
# ============================================================================
set -euo pipefail

DURATION=120
OUTDIR=/home/user/bags
WITH_REF=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    -d) DURATION="$2"; shift 2 ;;
    -o) OUTDIR="$2"; shift 2 ;;
    --no-reference) WITH_REF=0; shift ;;
    -h|--help) sed -n '2,24p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "[ERROR] unknown arg: $1" >&2; exit 2 ;;
  esac
done

INPUT_TOPICS=(
  /LIDAR/POINTS
  /IMU
  /MOTION_INFO
  /tf_static
  /fixposition/fpa/odomenu
  /fixposition/fpa/corrimu
  /fixposition/fpa/odomstatus
  /fixposition/fpa/eoe
  /fixposition/odometry_enu
  /fixposition/odometry_ecef
  /fixposition/odometry_llh
  /fixposition/poiimu
  /fixposition/ypr
)
REF_TOPICS=( /ODOM /LOC_BODY_POINTS /LOCATION_STATUS /LIO_ODOM /cloud_nav )

TOPICS=( "${INPUT_TOPICS[@]}" )
RATE_MBS=14
if [[ "${WITH_REF}" == "1" ]]; then
  TOPICS+=( "${REF_TOPICS[@]}" )
  RATE_MBS=24
fi

# 磁盘余量检查 | disk guard
need_mb=$(( DURATION * RATE_MBS * 3 / 2 ))
free_mb=$(df -m --output=avail "${OUTDIR%/*}" 2>/dev/null | tail -1 | tr -d ' ')
mkdir -p "${OUTDIR}"
free_mb=$(df -m --output=avail "${OUTDIR}" | tail -1 | tr -d ' ')
if [[ "${free_mb}" -lt "${need_mb}" ]]; then
  echo "[ERROR] 磁盘不够 | not enough disk: need ~${need_mb} MB, free ${free_mb} MB (${OUTDIR})" >&2
  exit 1
fi

STAMP=$(date +%Y%m%d_%H%M%S)
BAG="${OUTDIR}/fslam_inputs_${STAMP}"

# QoS overrides:全部 BE/volatile | all best_effort/volatile
QOS_FILE="${OUTDIR}/.qos_overrides_${STAMP}.yaml"
{
  for t in "${TOPICS[@]}"; do
    printf '%s:\n  reliability: best_effort\n  durability: volatile\n  history: keep_last\n  depth: 50\n' "${t}"
  done
} > "${QOS_FILE}"

set +u
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "[INFO] 录制 ${DURATION}s → ${BAG}(话题 ${#TOPICS[@]} 个,预估 ~$(( DURATION * RATE_MBS )) MB)"
echo "[INFO] recording ${DURATION}s, ${#TOPICS[@]} topics, ~$(( DURATION * RATE_MBS )) MB expected"
# nice + 单独进程组;超时后 SIGINT 让 rosbag2 干净收尾(写 metadata)。
# nice + own process group; SIGINT lets rosbag2 finalize metadata cleanly.
nice -n 10 timeout --signal=INT "${DURATION}" \
  ros2 bag record -o "${BAG}" --qos-profile-overrides-path "${QOS_FILE}" \
  "${TOPICS[@]}" || true
rm -f "${QOS_FILE}"

echo ""
echo "[INFO] 完成 | done:"
du -sh "${BAG}" 2>/dev/null || { echo "[ERROR] bag 目录缺失 | bag missing" >&2; exit 1; }
echo "[INFO] 各话题条数 | per-topic counts:"
grep -A2 "topic_metadata" "${BAG}/metadata.yaml" 2>/dev/null | grep -E "name:|message_count" | paste - - | sed 's/^ *//' || true
echo ""
echo "[INFO] 取回 | fetch: tar czf - -C ${OUTDIR} $(basename "${BAG}") | ssh <dev> 'tar xzf - -C ~/bags'"
