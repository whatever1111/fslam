# 发行版叶子:deep-robotics-m20-foxy | Distro leaf: deep-robotics-m20-foxy

本分支是 **deep-robotics-m20 产品主干的 ROS 2 Foxy 叶子**:主干载荷 + 本发行版的钉版文件。
发布 tag 打在这里:`deep-robotics-m20-foxy-vX.Y.Z`。
This is the **ROS 2 Foxy leaf of the deep-robotics-m20 trunk**: the trunk payload plus this
distro's pin files. Release tags go here: `deep-robotics-m20-foxy-vX.Y.Z`.

| 分支 branch | 角色 role | driver fork 配对 pairing | 发布 tag |
|---|---|---|---|
| `main` | 通用机制 generic machinery | — | — |
| `deep-robotics-m20` | 产品主干(载荷)product trunk | `m20`(trunk)| — |
| **`deep-robotics-m20-foxy`(本分支)** | Foxy 叶子(钉版)| `m20-foxy` / release `foxy-v*` | `deep-robotics-m20-foxy-v*` |
| `deep-robotics-m20-humble`(未建 not yet)| Humble 叶子 | `m20-humble` / `humble-v*` | `deep-robotics-m20-humble-v*` |

本分支独有的文件 | files unique to this leaf:`DRIVER_RELEASE`(rtk_only 用的驱动 release 钉版)、
本 `DISTRO.md`。模式载荷(启动脚本、unit、配置)都在主干,部署方法见 `DEPLOYMENT.md`。
Unique to this leaf: `DRIVER_RELEASE` (the driver-release pin for rtk_only) and this file. The mode
payload lives on the trunk; deployment steps in `DEPLOYMENT.md`.

## 为什么 M20 必须 Foxy | Why Foxy is required

OEM 雷达的 `/LIDAR/POINTS` writer 绑在 127.0.0.1;Humble 的 Fast DDS 2.6 不再为多网卡参与者通告
loopback locator,**能在 graph 里看到话题、永远收不到数据**(2026-08-08 实测,已排除 `/dev/shm`
变量);Foxy 的 2.0 可以。详见 driver 仓库 `m20-foxy` 分支的 DISTRO.md。
The OEM cloud writer is loopback-only; Humble's Fast DDS 2.6 no longer announces a loopback locator
for multi-NIC participants — **it sees the topic in the graph and never receives a message**
(measured 2026-08-08 with the `/dev/shm` variable ruled out); Foxy's 2.0 receives fine. Full story
in DISTRO.md on the driver's `m20-foxy` branch.

## 本叶子的 Foxy 专属坑 | Foxy-specific traps on this leaf

- `node.launch` 是 Galactic+ 方言,Foxy `launch_xml` 直接拒载 → `run_rtk_only_native.sh` 直接
  `ros2 run` 可执行文件,守护交给 systemd `Restart=`。
  `node.launch` is Galactic+ dialect and Foxy's `launch_xml` rejects it → the launcher `ros2 run`s
  the executable directly; systemd `Restart=` supervises.
- Foxy 的 rcl 不接受含换行的 `-p` 值 → robot_state_publisher 的 URDF 走生成的 params 文件
  (`rsp_params.yaml`),不走 `-p robot_description:=`。
  Foxy's rcl rejects multiline `-p` values → the URDF goes in via a generated params file.
- 狗上 Foxy 无 xacro → 用预展开的 `config/fixposition/robot.urdf`。
  No xacro on the robot's Foxy → the pre-expanded URDF ships in config.
- Foxy 的 `ros2 topic echo` 没有 `--once`/`--field`(Galactic+ 才有);探针还会阶段性"聋" ——
  验证以 graph info + `/LOCATION_STATUS` 的 `meta.frame_id` 计数为准。
  Foxy's `ros2 topic echo` lacks `--once`/`--field`; probes go deaf in phases — trust graph info
  and the `meta.frame_id` counter.
- rtk_only 的 foxy 驱动包**不含 drdds**(狗上是系统包,带了会遮蔽)、无 python typesupport
  (fixposition 自定义消息不能 `ros2 topic echo`)。
  The foxy driver tarball ships no drdds (system package on the robot) and no python typesupport.
- distro→driver 分支映射:`lib/deploy_common.sh` 的 `fslam_driver_branch()`。

## 发布 | Cutting a release

```bash
# 在本分支上 | on this leaf:
echo foxy-v1.0.1 > DRIVER_RELEASE            # 需要换驱动版本时 | when bumping the driver
git tag deep-robotics-m20-foxy-v1.2.0 deep-robotics-m20-foxy
git push origin deep-robotics-m20-foxy deep-robotics-m20-foxy-v1.2.0
```

主干原则:能可移植的进 `main` 或产品主干,叶子只留真正分叉的东西(钉版 + 本文件)。
模式(rtk_only/fslam)是运行时维度,靠 systemd 单元切换 —— 永远不设模式分支。
Trunk rule: portable things go to `main` or the product trunk; a leaf carries only true divergence
(pins + this file). Modes (rtk_only/fslam) are a runtime axis switched by systemd unit — never
branches.
