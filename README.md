# fslam — 云深处 M20 定位部署项目 | Deep Robotics M20 localization deployment

M20 的**部署项目**:两种定位模式,同一个 OEM 三话题契约(`/ODOM` + `/LOC_BODY_POINTS` +
`/LOCATION_STATUS`)。两个上游算法仓库都是通用组件,不含部署逻辑 —— 部署逻辑全在这里。
The M20 **deployment project**: two localization modes, one OEM three-topic contract. Both
upstream algorithm repos are generic components with no deployment logic — all of it lives here.

| 模式 mode | 定位来源 | 上游 upstream | 状态 status |
|---|---|---|---|
| **rtk**(fixposition-only)| VRTK2 RTK/INS,fork 驱动直接交付三话题 the forked driver delivers the contract itself | [fixposition_driver](https://github.com/whatever1111/fixposition_driver) 二进制 release(`DRIVER_RELEASE` 钉版)| **线上 live**:原生 Foxy,`m20_loc_foxy.service`(2026-08-08 起)|
| **fslam**(SLAM)| LIO-SLAM(FAST-LIO + PGO),容器内用镜像自带的上游原版 fixposition 驱动 | LIO-SLAM 镜像(`SLAM_IMAGE` 钉版,私有 registry)| 工具链就绪:`fslam_loc.service` |

两种模式共存于狗上、靠 systemd 单元切换(全部发 `/ODOM`,互斥)——所以模式不是分支;分支轴是
ROS 发行版(见 `DISTRO.md`)。**M20 上定位进程必须跑在 Foxy**(Humble 的 Fast DDS 2.6 永远收不到
OEM 的 loopback-only 雷达云,详见 `DISTRO.md`)。
Modes coexist on the robot and switch by systemd unit (all publish `/ODOM`, mutually exclusive) —
so modes are not branches; the branch axis is the ROS distro (see `DISTRO.md`). **Localization on
the M20 must run on Foxy** (Humble's Fast DDS 2.6 can never receive the OEM's loopback-only cloud).

## 快速开始 | Quick start

**完整步骤看 [DEPLOYMENT.md](DEPLOYMENT.md)** —— 这里只给最短路径:
Full instructions in **[DEPLOYMENT.md](DEPLOYMENT.md)**; the shortest paths:

```bash
# rtk 模式(线上路径)· 免编译:从 release 取捆绑包 | rtk mode, no compile
tar -xzf fslam-rtk_<ver>_foxy_arm64.tar.gz && cd fslam-rtk-foxy
WS="$(pwd)/driver" ./fslam/run_m20_foxy.sh

# rtk 模式 · 狗上编译 | or build on the robot
BUILD_CPUS=2,3 BUILD_JOBS=2 tools/build_m20_foxy.sh --source /home/user/m20_src
sudo systemctl enable --now m20_loc_foxy

# fslam(SLAM)模式 | fslam (SLAM) mode
./run_fast_lio_pgo_prod.sh            # 镜像先按 SLAM_IMAGE 离线搬上狗,见 DEPLOYMENT.md §3.4
./run_fast_lio_pgo_prod.sh --check    # 链路体检 | per-topic pipeline check
```

发布 | releases:一个 `foxy-v*` release 同时带 rtk 捆绑包/镜像和 slam 部署层捆绑包
(SLAM 镜像只钉版不分发)。One `foxy-v*` release ships the rtk bundle/image plus the slam
deployment-layer bundle (the SLAM image is pinned, never attached).

## 数据流 | Data flow

| 话题 | 类型 | rtk 模式来源 | fslam 模式来源 |
|------|------|-------------|----------------|
| `/ODOM` | nav_msgs/Odometry | fork 驱动解析线程内直发 driver-native, no relay | canonical 管线 fusion 直出(`odom_topic:=/ODOM`)|
| `/LOC_BODY_POINTS` | sensor_msgs/PointCloud2 | 驱动内去畸变(`/LIDAR/POINTS` → base_link,对齐最新 `/ODOM`)| 宿主机 `fp_to_odom.py` |
| `/LOCATION_STATUS` | drdds/LocationStatus | 驱动内 2 Hz(total_status: 0 未初始化/1 正常/2 低质量/3 丢失)| 宿主机 `fp_to_odom.py`(同戳)|

## 目录 | Layout

```
DEPLOYMENT.md              部署指南(两种模式)| the deployment guide
DISTRO.md                  分支/发行版配对(仅发行版分支)| branch pairing (distro branches)
DRIVER_RELEASE  SLAM_IMAGE 两个模式的钉版文件 | the per-mode pin files
run_m20_foxy.sh            rtk 模式启动器(原生 Foxy,线上)| rtk launcher (native Foxy, live)
run_fast_lio_pgo_prod.sh   fslam 模式启动器 | fslam-mode launcher
run_fixposition_prod.sh    旧 Python rtk(回退用)| legacy Python rtk (fallback)
run_m20_prod.sh            旧 Humble 容器 rtk(受 loopback 墙限制)| legacy Humble-container rtk
tools/build_m20_foxy.sh    狗上编译 rtk 驱动 | build the rtk driver on the robot
tools/build_m20_image.sh   构建 Humble 容器镜像(humble 分支用)| Humble container image
lib/deploy_common.sh       共享函数(含 distro→驱动分支映射)| shared helpers
container/                 容器载荷(只读挂载)| container payloads (mounted ro)
host/                      fslam 模式宿主机胶水 | fslam-mode host glue
  fp_to_odom.py              · /LOC_BODY_POINTS + /LOCATION_STATUS(+旧版 /ODOM 中继)
  motion_info_to_twist.py    · 轮速桥 /MOTION_INFO → FP 设备融合
config/
  dds/                       · cyclonedds.xml(容器)/ fastdds.xml(宿主机)
  fixposition/               · 驱动配置 config_m20.yaml / config_fp_only.yaml / robot.urdf
  slam/                      · canonical 三层配置树(源:LIO-SLAM 仓库,保持同步,勿在此直改)
systemd/
  m20_loc_foxy.service       · rtk 模式(线上)| rtk mode (live)
  fslam_loc.service          · fslam 模式 | fslam mode
  rtk_loc.service m20_loc.service · 旧链回退 | legacy fallbacks
release/                   发布流水线载荷 | release pipeline payloads
legacy/                    存档勿用(含从 LIO-SLAM 迁来的部署遗产 lio-slam-era/)
logs/                      运行时日志(git 忽略)| runtime logs (git-ignored)
```

## 注意 | Notes

- 所有路径由脚本从 checkout 位置推导,环境变量可覆盖 —— 无硬编码。
  All paths derive from the checkout location, env-overridable — nothing hard-coded.
- `config/slam/` 与镜像内烘焙配置不一致时以本 checkout 为准(挂载优先);配置改动在 LIO-SLAM
  仓库改并同步过来,保持单一事实源。`config/slam/` (mounted) wins over the image's baked config;
  edit in the LIO-SLAM repo and sync here — single source of truth.
- `/LOCATION_STATUS` 子字段(exec/loss/input)语义仍待 OEM 规范确认:rtk 版见驱动仓库
  `status_monitor.hpp` 顶部,fslam 版见 `host/fp_to_odom.py` 顶部,两处取值一致,规范到位一起改。
  The exec/loss/input sub-field semantics still await OEM confirmation; the two implementations
  (driver `status_monitor.hpp`, host `fp_to_odom.py`) use identical values — change both together.
- 狗上 docker 只允许 `--network host` 或 `none`,并且必须 `-v /dev/shm:/dev/shm`(细节与实测见
  `DEPLOYMENT.md`)。Docker on the robot: `--network host`/`none` only, and `-v /dev/shm:/dev/shm`
  is mandatory — details and measurements in `DEPLOYMENT.md`.
