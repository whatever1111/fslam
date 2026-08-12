#!/usr/bin/env bash
# ============================================================================
# wait_boot_settled.sh —— 开机就绪门:等时钟阶跃过去再创建 DDS participant
#                         boot readiness gate: create DDS participants only
#                         after the clock step is behind us
# ============================================================================
# 根因(2026-08-12,四次开机全量取证):chrony 首次启动死于尚未出现的 /dev/ptp1,
# 第二次启动在开机 ~25-47s 选中 PHC0 并**阶跃系统时钟**;在阶跃前创建的 DDS
# participant(Foxy Fast RTPS 2.1.4 的定时事件未完成 steady_clock 迁移)与对端的
# 配对会**永久楔死**,每次开机受害者不同(实测三对)。四次开机的唯一判别因子就是
# participant 创建时间在阶跃之前还是之后 —— 之后创建的配对 100% 可靠,与 OEM
# 节点群的启动顺序无关。
# 注意:不要等 OEM 服务 —— OEM 启动管理器反过来在等我们(两次实测:我们的门一放
# 行,OEM 群 ~10s 内跟着起来),等它们就是互相死锁。
#
# Root cause (2026-08-12, four fully-journaled boots): chrony's first start
# dies on a not-yet-present /dev/ptp1; its second start selects PHC0 and STEPS
# the system clock ~25-47 s after boot. DDS participants created before the
# step (Foxy's Fast RTPS 2.1.4 predates the steady_clock migration for timed
# events) wedge PERMANENTLY against their peers — a different victim pair each
# boot (three observed). Across all four boots the single discriminator is
# whether the participant was created before or after the step: after-step
# pairings were 100% reliable regardless of OEM start order.
# Do NOT wait for OEM services — their startup manager waits for US (measured
# twice: the OEM crowd follows ~10 s after our gate releases), so waiting on
# them is a mutual deadlock.
#
# 用法 | usage: wait_boot_settled.sh [clock_wait_cap_seconds]  (默认 | default 120)
# 永不失败(超时告警放行)—— 宁可退化启动也不卡死开机。
# Never fails (times out with a warning) — a degraded start beats a hung boot.
# ============================================================================
set -u
MAX_WAIT="${1:-120}"
t0=$(date +%s)

# ① 时钟已同步(chrony 选源完成,阶跃已经发生)| clock synchronised (source
#    selected — the step, if any, is behind us)
while [ $(( $(date +%s) - t0 )) -lt "${MAX_WAIT}" ]; do
  if chronyc tracking 2>/dev/null | grep -q "Leap status.*Normal"; then
    echo "[gate] clock: synchronised (Leap status Normal, $(( $(date +%s) - t0 ))s)"
    break
  fi
  sleep 2
done
if ! chronyc tracking 2>/dev/null | grep -q "Leap status.*Normal"; then
  echo "[gate][WARN] 时钟未同步(${MAX_WAIT}s)—— 放行,配对风险自负 | clock not synchronised after ${MAX_WAIT}s — proceeding at pairing risk"
fi

# ② 静置:让 ptp4l/phc2sys 的收尾抖动过去 | settle margin for ptp4l/phc2sys tail
sleep 30

echo "[gate] 系统已定($(( $(date +%s) - t0 ))s)| settled after $(( $(date +%s) - t0 ))s"
exit 0
