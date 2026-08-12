#!/usr/bin/env bash
# ============================================================================
# wait_boot_settled.sh —— 开机就绪门:等系统"定"了再创建 DDS participant
#                         boot readiness gate: create DDS participants only
#                         after the system has settled
# ============================================================================
# 根因(2026-08-12,journald 全量取证):开机后 ~47s chrony 选中 PHC0 并阶跃系统
# 时钟;OEM DDS 节点群在其后 10-35s 内陆续起来。在这之前创建的 participant
# (locator 集在创建时冻结;Foxy Fast RTPS 2.1.4 的定时事件未完成 steady_clock
# 迁移)与其后到达的对端配对会**永久楔死**,每次开机受害者不同 —— 实测三对:
# /GRID_MAP→astar、驱动的雷达 reader、驱动的 UDP egress。平静系统上的配对实测
# 100% 可靠。所以:所有我们的 DDS 进程一律等这道门 —— 一次起对,不靠事后重启。
#
# Root cause (2026-08-12, full journald forensics): chrony selects PHC0 and
# STEPS the system clock ~47 s after boot; the OEM DDS crowd starts 10-35 s
# after that. Participants created before that point (locator sets freeze at
# creation; Foxy's Fast RTPS 2.1.4 predates the steady_clock migration for
# timed events) wedge PERMANENTLY against peers arriving later — a different
# victim pair each boot (three observed). Calm-system pairing measured 100%
# reliable. Hence: every DDS process of ours waits for this gate — start
# correctly once instead of healing with restarts.
#
# 用法 | usage:  wait_boot_settled.sh [max_wait_seconds]   (默认 | default 150)
# 永不失败(超时告警放行)—— 宁可退化启动也不卡死开机。
# Never fails (times out with a warning) — a degraded start beats a hung boot.
# ============================================================================
set -u
MAX_WAIT="${1:-150}"
t0=$(date +%s)

left() { echo $(( MAX_WAIT - ( $(date +%s) - t0 ) )); }

# ① 时钟已同步(chrony 选源完成,阶跃已经发生)| clock synchronised (source
#    selected — the step, if any, is behind us)
while [ "$(left)" -gt 0 ]; do
  if chronyc tracking 2>/dev/null | grep -q "Leap status.*Normal"; then
    echo "[gate] clock: synchronised (Leap status Normal)"
    break
  fi
  sleep 2
done

# ② OEM DDS 节点群已就位 | the OEM DDS crowd is up
for s in rsdriver yesense planner global_planner passable_area handler; do
  while [ "$(left)" -gt 0 ]; do
    state="$(systemctl is-active "$s" 2>/dev/null)"
    [ "$state" = "active" ] && break
    sleep 2
  done
  echo "[gate] $s: $(systemctl is-active "$s" 2>/dev/null)"
done

# ③ 静置:让上面这批的 participant/端口全部建完 | settle: let their
#    participants and ports finish forming
sleep 10

el=$(( $(date +%s) - t0 ))
if [ "$el" -ge "$MAX_WAIT" ]; then
  echo "[gate][WARN] 就绪门超时(${el}s)—— 放行,可能需要人工体检 | timed out, proceeding; a manual check may be needed"
else
  echo "[gate] 系统已定(${el}s)| settled after ${el}s"
fi
exit 0
