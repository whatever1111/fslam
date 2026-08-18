# deep-robotics-m20 — 云深处 M20 定位部署 | Deep Robotics M20 localization deployment

本分支是 **deep-robotics-m20 产品主干**:M20 的全部定位部署载荷。两个模式 × 两种形态,
同一个 OEM 三话题契约(`/ODOM` + `/LOC_BODY_POINTS` + `/LOCATION_STATUS`)。上游算法仓库都是
通用组件 —— 部署逻辑全在这里。通用机制与分支模型见 `main` 分支的 README。
This is the **deep-robotics-m20 product trunk**: the M20's entire localization deployment payload.
Two modes × two forms, one OEM three-topic contract. The upstream algorithm repos are generic
components — deployment logic lives here. The generic machinery and branch model are on `main`.

## 模式 × 形态 | Mode × form

| | native | docker |
|---|---|---|
| **`rtk_only`**(RTK/INS:fork 驱动直接交付契约 the forked driver delivers the contract)| ✅ `run_rtk_only_native.sh` + `rtk_only.service` — **线上 live** | ✅ release 镜像一行 `docker run`(§DEPLOYMENT.md)— 已在 106 验证 validated |
| **`fslam`**(SLAM:LIO-SLAM + 内部上游原版 fixposition 驱动 upstream driver inside)| ✅ `run_fslam_native.sh` + `fslam_native.service` — 原生 Foxy 直收 OEM 点云,无中继 direct OEM cloud, no relay | ✅ `run_fslam.sh` + `fslam.service` — Humble 容器 + 中继 container + relay |

- 一次只能跑一个:所有模式都发 `/ODOM`,unit 间 `Conflicts=` 互斥;手工进程绕得过互斥,自己当心。
  One at a time: every mode publishes `/ODOM`; units conflict pairwise, hand-started processes bypass that.
- **必须 Foxy**:Humble 的 Fast DDS 2.6 永远收不到 OEM 的 loopback-only 雷达云(见 `DISTRO.md`)。
  **Foxy required**: Humble's Fast DDS 2.6 can never receive the OEM's loopback-only cloud (`DISTRO.md`).
- 钉版 | pins:`DRIVER_RELEASE`(rtk_only ← fork 驱动 release)· `SLAM_IMAGE`(fslam · docker ← LIO-SLAM 镜像,私有 registry)· `SLAM_NATIVE_RELEASE`(fslam · native ← LIO-SLAM 仓库 `foxy-v*` release 的原生 tarball)。

## 快速开始 | Quick start

完整步骤在 **[DEPLOYMENT.md](DEPLOYMENT.md)**;最短路径:
Full instructions in **[DEPLOYMENT.md](DEPLOYMENT.md)**; shortest paths:

```bash
# rtk_only · native(线上路径 live)—— release 捆绑包免编译 | no compile
tar -xzf rtk_only_<ver>_foxy_arm64.tar.gz && cd rtk_only-foxy
WS="$(pwd)/driver" ./fslam/run_rtk_only_native.sh

# rtk_only · docker
docker run -d --name rtk_only --restart unless-stopped \
    --network host -v /dev/shm:/dev/shm ghcr.io/whatever1111/rtk_only:foxy-<ver>

# fslam · docker(镜像按 SLAM_IMAGE 先离线搬上狗 | ship the pinned image first)
./run_fslam.sh && ./run_fslam.sh --check

# fslam · native(LIO-SLAM tarball 按 SLAM_NATIVE_RELEASE 解到 /home/user/lio-slam)
# (extract the pinned LIO-SLAM native tarball to /home/user/lio-slam first)
./run_fslam_native.sh && ./run_fslam_native.sh --check
```

## 数据流 | Data flow

| 话题 | 类型 | rtk_only 来源 | fslam 来源 |
|------|------|--------------|-----------|
| `/ODOM` | nav_msgs/Odometry | fork 驱动直发 driver-native, no relay | canonical 管线 fusion 直出(`odom_topic:=/ODOM`)|
| `/LOC_BODY_POINTS` | sensor_msgs/PointCloud2 | 驱动内去畸变(`/LIDAR/POINTS`→base_link,对齐最新 `/ODOM`)| 宿主机 `fp_to_odom.py` |
| `/LOCATION_STATUS` | drdds/LocationStatus | 驱动内 2 Hz(total_status: 0 未初始化/1 正常/2 低质量/3 丢失)| 宿主机 `fp_to_odom.py`(同戳)|

## 目录 | Layout

```
DEPLOYMENT.md                部署指南 | the deployment guide
DISTRO.md                    发行版叶子说明(在叶子分支上)| distro notes (on leaf branches)
DRIVER_RELEASE SLAM_IMAGE SLAM_NATIVE_RELEASE   各模式·形态的钉版 | per mode·form pins(叶子上 on the leaf)
run_rtk_only_native.sh       rtk_only · native 启动器(线上 live)
run_fslam.sh                 fslam · docker 启动器
run_fslam_native.sh          fslam · native 启动器(LIO-SLAM 原生 tarball + fork 驱动上游节点)
tools/build_rtk_only_native.sh  （已下线，仅开发机应急重现；部署只用 release 二进制）| retired: dev-box emergency rebuild only, deployments use release binaries
tools/build_m20_image.sh     旧 Humble 容器镜像构建(humble 叶子备用)| legacy Humble image build
lib/deploy_common.sh         共享函数 | shared helpers
container/                   fslam 容器载荷(只读挂载)| fslam container payloads (ro mounts)
host/                        fslam 宿主机胶水 | fslam host glue(fp_to_odom.py, motion_info_to_twist.py)
config/
  dds/                       cyclonedds.xml(容器)/ fastdds.xml(宿主机)
  fixposition/               config_m20.yaml(rtk_only 线上配置)/ config_fp_only.yaml / robot.urdf
  slam/                      canonical 三层配置树(源:LIO-SLAM 仓库,勿在此直改)
systemd/
  rtk_only.service           rtk_only · native(线上 live)
  fslam.service              fslam · docker
  fslam_native.service       fslam · native
release/                     rtk_only docker 镜像的 Dockerfile 等 | release payloads
legacy/                      旧代启动器/单元 + lio-slam-era 迁移存档,勿用 | superseded, do not use
logs/                        运行时日志(git 忽略)| runtime logs (git-ignored)
```

## 旧名对照 | Old-name decoder

2026-08-08 改名前的旧名(狗上已装的 unit 可能还是旧名,迁移步骤见 DEPLOYMENT.md):
Names before the 2026-08-08 rename (robots may still have old units installed; migration in
DEPLOYMENT.md):

| 旧 old | 新 new | 模式·形态 mode·form |
|---|---|---|
| `run_m20_foxy.sh` / `m20_loc_foxy.service` | `run_rtk_only_native.sh` / `rtk_only.service` | rtk_only · native |
| `run_fast_lio_pgo_prod.sh` / `fslam_loc.service` | `run_fslam.sh` / `fslam.service` | fslam · docker |
| `run_m20_prod.sh` / `m20_loc.service` | `legacy/`(受 loopback 墙限制 broken by the wall)| rtk_only · docker(旧代 Humble)|
| `run_fixposition_prod.sh` / `rtk_loc.service` | `legacy/` | rtk_only(旧代 Python)|
| 资产 assets `fslam-m20_*` / `fslam-rtk_*` | `rtk_only_*` / `fslam_*` | — |

## 注意 | Notes

- 路径全部由脚本从 checkout 位置推导,环境变量可覆盖 —— 无硬编码。
  All paths derive from the checkout location, env-overridable — nothing hard-coded.
- `config/slam/` 与镜像内烘焙配置不一致时以本 checkout 为准(挂载优先);改动在 LIO-SLAM 仓库改并同步。
  Mounted `config/slam/` wins over the image's baked config; edit in LIO-SLAM and sync here.
- `/LOCATION_STATUS` 子字段(exec/loss/input)语义待 OEM 确认:rtk_only 版见驱动仓库
  `status_monitor.hpp`,fslam 版见 `host/fp_to_odom.py`,两处取值一致,规范到位一起改。
  Sub-field semantics await OEM confirmation; both implementations use identical values.
- 狗上 docker 只允许 `--network host`/`none`,且必须 `-v /dev/shm:/dev/shm`(实测见 DEPLOYMENT.md)。
  Docker on the robot: host/none networking only, and the /dev/shm mount is mandatory.
