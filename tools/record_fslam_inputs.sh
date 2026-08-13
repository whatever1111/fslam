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

# 必须 root:所有 writer(OEM 栈 + 我们的驱动)都是 root,Fast DDS SHM 不跨
# UID —— 非 root 的 recorder 订阅成功但收不到任何数据,产出空 bag(实测 52 KB)。
# Must be root: every writer (OEM stack + our driver) runs as root, and Fast DDS
# SHM does not cross UIDs — a non-root recorder subscribes fine yet receives
# nothing and writes an empty bag (measured: 52 KB).
if [[ ${EUID} -ne 0 ]]; then
  echo "[ERROR] 必须以 root 运行(SHM 不跨 UID,非 root 录出来是空包)| must run as root" >&2
  exit 1
fi

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
BAG_REF="${OUTDIR}/fslam_ref_${STAMP}"

# QoS overrides:全部 BE/volatile | all best_effort/volatile
# 例外 /tf_static:它的 writer 是 transient_local 锁存,volatile reader 永远收不到
# 已发布的静态 TF —— 必须用 transient_local 才能录到锁存历史。
# Exception /tf_static: its writers are transient_local latches; a volatile
# reader receives NOTHING already published — transient_local gets the latch.
QOS_FILE="${OUTDIR}/.qos_overrides_${STAMP}.yaml"
{
  for t in "${TOPICS[@]}"; do
    dur=volatile
    [[ "${t}" == "/tf_static" ]] && dur=transient_local
    printf '%s:\n  reliability: best_effort\n  durability: %s\n  history: keep_last\n  depth: 50\n' "${t}" "${dur}"
  done
} > "${QOS_FILE}"

set +u
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
# fpa 话题的 typesupport 在 m20_ws(fixposition_driver_msgs)—— 不 source 的话
# rosbag2 静默跳过这些话题(实测只订到 7/18)。drdds 装在系统路径,无需额外 source。
# The fpa topics' typesupport lives in m20_ws (fixposition_driver_msgs) — without
# it rosbag2 silently skips them (measured: only 7/18 subscribed). drdds is on
# the system path already.
# shellcheck disable=SC1091
[[ -f /home/user/m20_ws/install/setup.bash ]] && source /home/user/m20_ws/install/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "[INFO] 录制 ${DURATION}s → ${BAG}(话题 ${#TOPICS[@]} 个,预估 ~$(( DURATION * RATE_MBS )) MB)"
echo "[INFO] recording ${DURATION}s, ${#TOPICS[@]} topics, ~$(( DURATION * RATE_MBS )) MB expected"
# nice + 单独进程组;超时后 SIGINT 让 rosbag2 干净收尾(写 metadata)。
# nice + own process group; SIGINT lets rosbag2 finalize metadata cleanly.
# 双 recorder 拆分:单进程录三路 ~1MB@10Hz 点云会丢 ~15% 雷达帧(A/B 实测:
# 18 话题一起录 = 雷达 112/128,仅输入话题 = 131/131 无损)。输入 bag(回放评测
# 的原料)单独一个 recorder、正常优先级保无损;参考 bag(对照轨迹)另一个
# recorder 加 nice,丢帧无所谓。不用 --max-cache-size:Foxy 语义是条数不是字节,
# 且 SIGINT 不冲刷缓存 —— metadata 计数与 db 内容不符,尾巴整批丢失(实测)。
# Two-recorder split: one process recording three ~1MB@10Hz clouds drops ~15%
# of lidar frames (A/B measured: 18 topics together = 112/128 lidar, inputs
# only = 131/131 lossless). The inputs bag (replay material) gets its own
# recorder at normal priority; the reference bag (comparison track) records
# under nice where losses don't matter. No --max-cache-size: Foxy counts
# MESSAGES not bytes, and SIGINT does not flush the cache — metadata counts
# diverge from db contents and the tail batch is lost (measured).
# 参考 recorder 必须 UDP-only:第二个并发 rosbag2(同名 rosbag2_recorder 节点、
# 默认 SHM)订阅成功但收 0 条(实测);挂 handler 的 UDP-only profile 后全量收齐
# (103/103,连 /LOC_BODY_POINTS 都比单独 SHM 录得全)。参考话题的 writer 都是
# 我们的驱动,UDP 通路已被 handler 每天验证。输入 recorder 保持默认 transports
# (OEM 雷达 writer 只走同版本 SHM)。
# The reference recorder must be UDP-only: a second concurrent rosbag2 (same
# rosbag2_recorder node name, default SHM) subscribes fine yet receives ZERO
# (measured); with the handler's UDP-only profile it captures everything
# (103/103 — even /LOC_BODY_POINTS records more completely than solo over
# SHM). All reference writers are our driver, whose UDP path the handler
# exercises daily. The inputs recorder keeps default transports (the OEM
# lidar writer delivers over same-version SHM only).
FSLAM_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
UDP_PROFILE="${FSLAM_ROOT}/config/fixposition/fastdds_handler_udp.xml"
REF_PID=""
if [[ "${WITH_REF}" == "1" ]]; then
  if [[ ! -f "${UDP_PROFILE}" ]]; then
    echo "[ERROR] UDP profile 缺失 | missing: ${UDP_PROFILE}(参考 recorder 必需)" >&2
    exit 1
  fi
  FASTRTPS_DEFAULT_PROFILES_FILE="${UDP_PROFILE}" \
  nice -n 10 timeout --signal=INT "${DURATION}" \
    ros2 bag record -o "${BAG_REF}" --qos-profile-overrides-path "${QOS_FILE}" \
    "${REF_TOPICS[@]}" >/dev/null 2>&1 &
  REF_PID=$!
fi
timeout --signal=INT "${DURATION}" \
  ros2 bag record -o "${BAG}" --qos-profile-overrides-path "${QOS_FILE}" \
  "${INPUT_TOPICS[@]}" || true
[[ -n "${REF_PID}" ]] && wait "${REF_PID}" 2>/dev/null || true
rm -f "${QOS_FILE}"

print_counts() {
  grep -E "name:|message_count:" "$1/metadata.yaml" 2>/dev/null \
    | sed 's/^ *//' | paste - - | grep -v topics_with || true
}
# Foxy rosbag2 落盘是 WAL journal 模式 —— 只读挂载(docker 评测 -v :ro)下
# sqlite 无法建 -wal/-shm 附属文件,回放直接 SQLITE_CANTOPEN(14)。落盘后
# 切回 DELETE 模式(瞬时操作,不重写数据),取回即可只读回放。
# Foxy rosbag2 writes WAL-journal databases — replay through a read-only
# mount (docker eval -v :ro) fails with SQLITE_CANTOPEN(14) because sqlite
# cannot create the -wal/-shm sidecars. Flip to DELETE mode after recording
# (instant, no data rewrite) so fetched bags replay read-only as-is.
fix_journal() {
  local db
  for db in "$1"/*.db3; do
    [[ -f "${db}" ]] && python3 -c "import sqlite3; c=sqlite3.connect('${db}'); c.execute('PRAGMA journal_mode=DELETE'); c.close()" 2>/dev/null || true
  done
}
echo ""
echo "[INFO] 完成 | done:"
du -sh "${BAG}" 2>/dev/null || { echo "[ERROR] bag 目录缺失 | bag missing" >&2; exit 1; }
fix_journal "${BAG}"
[[ "${WITH_REF}" == "1" ]] && fix_journal "${BAG_REF}"
# 录制以 root 跑,归还给 user 方便取回 | recorded as root; hand back to user
id user >/dev/null 2>&1 && chown -R user:user "${BAG}"
echo "[INFO] 输入话题条数 | input topic counts:"
print_counts "${BAG}"
if [[ "${WITH_REF}" == "1" ]]; then
  du -sh "${BAG_REF}" 2>/dev/null || echo "[WARN] 参考 bag 缺失 | reference bag missing" >&2
  id user >/dev/null 2>&1 && chown -R user:user "${BAG_REF}" 2>/dev/null
  echo "[INFO] 参考话题条数 | reference topic counts:"
  print_counts "${BAG_REF}"
fi
echo ""
echo "[INFO] 取回 | fetch: tar czf - -C ${OUTDIR} $(basename "${BAG}")$([[ "${WITH_REF}" == "1" ]] && echo " $(basename "${BAG_REF}")") | ssh <dev> 'tar xzf - -C ~/bags'"
