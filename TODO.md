# TODO — deep-robotics-m20-foxy

## 1. `leg_odom_bridge` — /MOTION_INFO → 标准腿式里程计 sidecar | leg-odometry sidecar (DESIGN, not started)

### 为什么 | Why
三个消费者都要轮速/腿速,而今天各读各的、各自无时间戳:
Three consumers need the leg speed today and each reads it its own way, none with a timestamp:
- fixposition 驱动 M20 模块直接订阅 `/MOTION_INFO`(`wheelspeed_enable`),或走 `converter` 吃 `host/motion_info_to_twist.py` 的裸 `Twist`;
- LIO 前端 `twist_adapter`(`driver: raw_twist`,裸 `Twist`,只能"最新样本"对齐);
- LIO 后端 PGO 还没有腿式里程计输入。

`/MOTION_INFO` 本身**没有可用时间**:`header.stamp` 恒 0,`header.frame_id` 计数器也恒 0(2026-08-16 106 实测)。
但 DDS 层的 **`source_timestamp`**(写端发送时刻,rclcpp `MessageInfo` 可读)是完美的时钟:
- 严格 50.000 ms 周期(线性拟合残差 σ 0.02 ms,max 0.1 ms),对齐整秒(相位恒 33.25 ms mod 50);
- 写端 → 收端时延中位 0.46 ms / p99 3.0 ms / max 7.8 ms(稳态;启动前 ~14 帧是 reliable 历史回放,忽略);
- 从不为负 ⇒ 运动主板(GUID host `06.66`,与导航机 `67.42` 不同主机)的时钟已跟导航机 PTP 对齐到亚毫秒;
- 收端到达抖动 p95 50.3 ms / max 57 ms(4 核负载 4.7 下,进程内测)。此前 bag 里量到的 50/127/215 ms 是**录制路径**的伪影。

**Foxy 的 rclpy 拿不到 `MessageInfo`**(`rclpy_take` 只返回消息;Galactic 才有),所以现有 python 桥无法打时间戳 → 必须 C++。

### 契约 | Contract
输入 | input: `/MOTION_INFO` (`drdds/msg/MotionInfo`,20 Hz,OEM reliable/volatile;读端 best-effort 兼容)

输出 | outputs (all `frame_id = base_link`, stamp = DDS `source_timestamp` + `latency_offset_sec`):
| topic | type | 用途 | consumer |
|---|---|---|---|
| `/leg_odom/twist` | `geometry_msgs/TwistStamped` | 体坐标 vx (× `speed_scale`)、vy(默认 0)、wz | LIO `twist_adapter driver: twist_stamped` → `/lio/twist`;FE `wheel_odom_en`;PGO 速度先验 |
| `/leg_odom/odom` | `nav_msgs/Odometry` | 2D 航位推算(∫vx,wz),`frame_id=leg_odom`,pose 协方差随里程增长 | PGO 关键帧间 between-factor;robot_localization/nav 通用形态 |
| `/fixposition/motion_info_twist` | `geometry_msgs/Twist`(≤ `max_rate_hz`) | 兼容 FP `converter`(FP 轮速协议本身无时戳,不受益于 stamp,只受益于"同一来源同一标定") | fixposition 驱动;届时关掉 M20 模块直读,让整机只有**一个** `/MOTION_INFO` 读者 |

参数 | params: `speed_scale`(默认 1.0;RTK 标定慢走 ≈1.0、快走 ×1.06–1.09,步态相关 → 先不硬编码)、`sigma_vx` 0.15、`sigma_vy` 0.05、`sigma_wz`、`use_y`、`stamp_source: dds|receive`(dds 默认;`receive` 供无 source_timestamp 的 RMW 退化)、`latency_offset_sec`(控制器内部延迟,RTK 互相关标定,当前 <25 ms)、`max_rate_hz`、`stale_sec` 0.2。

诊断 | diagnostics(节流日志):source gap > `stale_sec`;`|now − source_timestamp| > 50 ms` ⇒ "motion board clock unsynced"(退回 receive 时戳并告警);`motion_state`/gait 透传到 covariance(站立态 vx≡0 高置信)。

### 部署 | Deployment
- 位置:fslam `host/leg_odom_bridge/`(ament C++ 包,依赖 rclcpp、drdds、geometry_msgs、nav_msgs;drdds 只在狗上有 ⇒ 只能狗上编,`colcon build` 单包 ≈36 s/4 核,`nice -n 10 --parallel-workers 1`)。
- `run_fslam_native.sh` 的 `motion_info_bridge` 槽位改起 C++ 二进制(pid/log 名不变);`run_rtk_only_native.sh` 同。
- `tools/record_fslam_inputs.sh` 追加 `/leg_odom/twist`、`/leg_odom/odom`,回放才带真时戳。
- 参考实现:2026-08-16 探针 `106:/home/user/probe_mi/`(rclcpp `MessageInfo::get_rmw_message_info().source_timestamp`)。

### 后续依赖 | Downstream work it unblocks
1. LIO FE `wheel_odom_en` 由"最新样本"升级为"插值到 scan end"(FAST_LIO fork);
2. PGO 腿式 between-factor / 速度先验(outage 段的航向仍无观测 → 只约束里程,不约束航向);
3. 老 bag 离线重打时戳:收端时刻吸附到 50 ms 格(逐帧计数,≥75 ms 间隙记丢帧),用于先在 dev 机 A/B。

### 验收 | Acceptance
`ros2 topic hz /leg_odom/twist` ≈20 Hz;stamp 与 `/fixposition/fpa/corrimu` 同基;`ros2 topic info -v /MOTION_INFO` 只剩 OEM 读者 + 本 bridge;3× degrade 回放对比 baseline(pull 1.6–3.2 m 带内噪声,须看均值)。注意 08-15 实测:腿速在 m20_0814_degrade 的室内段(建筑物内,慢速多转弯)∫vx 少 19%,室外少 10-12% —— 不是恒定尺度;前端融合此路已证不通,本 bridge 的价值在 FP 驱动/PGO 共用一个带时戳标定源。
