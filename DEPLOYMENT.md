# M20 定位部署指南 | M20 Localization Deployment

fslam 是 M20 的**部署项目**:两种定位模式,同一个 OEM 三话题契约
`/ODOM` + `/LOC_BODY_POINTS` + `/LOCATION_STATUS`。上游两个算法仓库都是通用组件,不带部署逻辑
—— 部署逻辑全在这里。
fslam is the M20 **deployment project**: two localization modes, one OEM three-topic contract.
Both upstream algorithm repos are generic components carrying no deployment logic — all of that
lives here.

**模式 × 形态 | mode × form** —— 全局只有两个模式名 `rtk_only` / `fslam`,每个模式给出 native 和
docker 形态(能给的都给)。Only two mode names exist globally; each ships in native and docker
form where possible:

| | native | docker |
|---|---|---|
| **`rtk_only`**(VRTK2 RTK/INS:fork 驱动直接交付三话题 the forked `whatever1111/fixposition_driver` delivers the contract itself)| §1/§2:`run_rtk_only_native.sh` — **线上 live**(2026-08-08 起)| §3:release 镜像一行命令 — 已在 106 验证 validated |
| **`fslam`**(SLAM:LIO-SLAM 容器,内用**上游原版** fixposition 驱动 the container's internal **upstream** driver;`SLAM_IMAGE` 钉版)| —(SLAM 按设计容器化 containerized by design)| §3.4:`run_fslam.sh` — 工具链就绪 toolchain ready |

只想跑裸驱动、自己给配置:看 fork 驱动仓库的 `DEPLOYMENT.md`。
For the bare driver with your own configuration, see `DEPLOYMENT.md` in the driver fork.

> **为什么模式不是分支 | Why modes are not branches:** 两种模式共存于同一台狗上(SLAM 主用时 rtk
> 是回退),切换靠 systemd 单元,不靠 checkout;分支轴留给 ROS 发行版(foxy/humble/jazzy),那才
> 是和驱动 release 配对的维度。The two modes coexist on one robot (rtk stays as fallback when SLAM
> is primary) and are switched by systemd unit, not by checkout; the branch axis is the ROS distro,
> which is what pairs with driver releases.

---

## 0. 动手前必须知道的三件事 | Three things to know first

1. **所有定位部署互斥。** `rtk_only.service`、`fslam.service`、以及旧代的
   `m20_loc`/`rtk_loc`/`m20_loc_foxy` 都发 `/ODOM`,同时跑就是给导航塞多个打架的位姿源。unit 里已写
   `Conflicts=`(含旧名),但手工起进程绕得过去。
   **Every localization deployment is mutually exclusive.** `rtk_only.service`, `fslam.service`, and
   the legacy `m20_loc`/`rtk_loc`/`m20_loc_foxy` units all publish `/ODOM`. `Conflicts=` covers them
   (old names included), but hand-started processes bypass that.

2. **必须是 Foxy。** OEM 雷达点云 writer 只绑 127.0.0.1,Humble 的 Fast DDS 2.6 能在 graph 里看到它、
   永远收不到它的数据(2026-08-08 实测,已排除 `/dev/shm` 变量)。理由见 `DISTRO.md`。
   **Foxy is required.** The OEM cloud writer is loopback-only; Humble's Fast DDS 2.6 sees it in the
   graph and never receives from it (measured 2026-08-08 with the `/dev/shm` variable ruled out).
   Rationale in `DISTRO.md`.

3. **狗上没有公网**(GitHub 不可达),发布包/源码一律 `scp` 送过去。别在狗上跑重编译把 CPU 吃满 ——
   会把 sshd 饿死。
   **The robot has no public internet** (GitHub unreachable); ship releases and sources over `scp`.
   Never saturate its CPU with a heavy build — that starves sshd.

---

## 1. rtk_only · native — 发布包(推荐,免编译)| Release bundle (recommended, no compile)

从 fslam 的 `foxy-v*` release 取 `rtk_only_<ver>_foxy_arm64.tar.gz`。包里 `driver/` 是驱动二进制树,
`fslam/` 是启动脚本 + 配置 + unit。`driver/` 的布局就是 `run_rtk_only_native.sh` 认的 `WS`,可以直接当工作区。

Take `rtk_only_<ver>_foxy_arm64.tar.gz` from an fslam `foxy-v*` release. Inside, `driver/` is the
driver binary tree and `fslam/` is the launcher, config and unit. `driver/` matches the `WS` layout
`run_rtk_only_native.sh` expects, so it works as a drop-in workspace.

```bash
sha256sum -c SHA256SUMS.txt
tar -xzf rtk_only_<ver>_foxy_arm64.tar.gz
cd rtk_only-foxy

# 手动跑一次看看 | run it by hand first
WS="$(pwd)/driver" ./fslam/run_rtk_only_native.sh
```

装成开机服务 | install as a boot service:

```bash
sudo systemctl disable --now rtk_loc m20_loc        # 先关掉别的 | stop the others first
sudo cp fslam/systemd/rtk_only.service /etc/systemd/system/
sudo mkdir -p /etc/systemd/system/rtk_only.service.d
sudo tee /etc/systemd/system/rtk_only.service.d/bundle.conf >/dev/null <<EOF
[Service]
Environment=WS=/path/to/rtk_only-foxy/driver
ExecStart=
ExecStart=/bin/bash /path/to/rtk_only-foxy/fslam/run_rtk_only_native.sh
EOF
sudo systemctl daemon-reload && sudo systemctl enable --now rtk_only
```

---

## 2. rtk_only · native — 狗上编译(现行线上形态)| Build on the robot (the current live setup)

线上就是这条:源码在 `/home/user/m20_src`(驱动 `m20-foxy` 分支),工作区 `/home/user/m20_ws`,
仓库工作副本 `/home/user/fslam`。
This is what's running now: sources at `/home/user/m20_src` (driver branch `m20-foxy`), workspace
`/home/user/m20_ws`, repo working copy `/home/user/fslam`.

```bash
# 源码整棵送过去(含 fixposition-sdk 子模块)| ship the whole tree, submodule included
#   tar czf - . | ssh <robot> 'mkdir -p /home/user/m20_src && tar xzf - -C /home/user/m20_src'

# 编译钉在 2 个核上,别把狗跑满 | pin the build to 2 cores; never saturate the robot
BUILD_CPUS=2,3 BUILD_JOBS=2 tools/build_rtk_only_native.sh --source /home/user/m20_src

sudo cp systemd/rtk_only.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now rtk_only
```

> 源码同步务必**整棵覆盖**,别一个文件一个文件挑。曾经因为漏同步一个 `.cpp` 而头文件里还留着声明,
> 直接编出 `undefined reference`。
> Always sync the **whole tree** rather than picking files: a missed `.cpp` once left a declaration
> in the header with no definition and produced an `undefined reference`.

---

## 3. rtk_only · docker | rtk_only in docker

镜像已把 `config_m20.yaml` 烘进 `/config/fixposition.yaml`,不用挂配置。
The image bakes `config_m20.yaml` at `/config/fixposition.yaml`, so no config mount is needed.

```bash
docker load < rtk_only_<ver>_foxy_arm64_docker-image.tar.gz
docker run -d --name rtk_only --restart unless-stopped \
    --network host -v /dev/shm:/dev/shm \
    ghcr.io/whatever1111/rtk_only:foxy-<ver>
```

**两个 flag 缺一不可,少了就是静默失效** —— 容器照常启动、照常发布,但收不到任何数据:
`--network host` 走宿主网络;`-v /dev/shm:/dev/shm` 因为 Fast DDS 共享内存传输就是 `/dev/shm` 里的
POSIX shm 文件,docker 默认的私有 64 MB `/dev/shm` 会让容器对宿主上所有 publisher 变聋。

**Both flags are mandatory and omitting either fails silently** — the container starts and publishes
but receives nothing. `--network host` for the host network; `-v /dev/shm:/dev/shm` because Fast
DDS's shared-memory transport is POSIX shm files under `/dev/shm`, and docker's default private
64 MB `/dev/shm` makes a container deaf to every host publisher.

> 狗上只允许 `--network host` 或 `--network none`。默认 bridge 网络建 veth 会触发内核 RTNL 死锁 ——
> ssh 断、dockerd 全局卡死,只能整机断电。
> On the robot use only `--network host` or `--network none`: a bridge veth triggers a kernel RTNL
> deadlock — ssh dies, dockerd wedges globally, and only a power cycle recovers.

容器路径已在 106 上验证可用(收到点云、发出 `/ODOM` 与去畸变点云),但**线上仍以原生为准**。
The container path is verified working on 106 (receives the cloud, publishes `/ODOM` and deskewed
clouds), but **native remains the production path**.

---

## 3.2 直接跑启动脚本(日常调试就用这个)| Running the launcher scripts directly (the day-to-day way)

前面三种方式最终都落到仓库根目录的启动脚本上。调试、临时切模式、上手验证,直接跑脚本比装 systemd 快得多
—— 不用装 unit,不用 `daemon-reload`,改完配置重跑一次就行。
All three methods above ultimately run a launcher script from the repo root. For debugging, switching
modes temporarily, or a first hands-on check, running the script directly is much faster than going
through systemd — no unit to install, no `daemon-reload`, just edit the config and run it again.

### 脚本一览 | The launchers

| 脚本 script | 模式·形态 mode·form | 配置 config | 日志目录 log dir |
|---|---|---|---|
| `run_rtk_only_native.sh` | **rtk_only · native(线上 live)** | `config_m20.yaml` | `logs/rtk_only/` |
| `run_fslam.sh` | **fslam · docker**(SLAM 容器 + 宿主胶水)| `config/slam/` 树 | `logs/fslam/` |
| `legacy/run_m20_prod.sh` | 旧:rtk_only · docker(Humble,受 loopback 墙限制)legacy | `config_m20.yaml` | `logs/m20/` |
| `legacy/run_fixposition_prod.sh` | 旧:rtk_only(Python 版)legacy | `config_fp_only.yaml` | `logs/fixposition_only/` |

rtk_only 的 **docker 形态**不需要专用脚本 —— release 镜像自带入口(见 §3 的 `docker run` 一行命令)。
所有模式都发 `/ODOM`,**任何时候只能跑一个**;脚本启动时会自动停掉其它部署(容器 + pid 文件),
换脚本跑就是切模式。
The rtk_only **docker form** needs no dedicated script — the release image carries its entrypoint
(the one-liner in §3). Every mode publishes `/ODOM`, so **only one may run at a time**; each script
stops the other deployments on startup (containers plus pid files), so switching modes is just
running a different script.

### 跑起来 | Running

```bash
cd /home/user/fslam

# 前台跑(会一直占着终端,Ctrl-C 停)| foreground: it holds the terminal, Ctrl-C stops it
./run_rtk_only_native.sh

# 后台跑 | detached
setsid nohup ./run_rtk_only_native.sh > /tmp/rtk_only.log 2>&1 &
```

> `run_rtk_only_native.sh` 最后是 `wait`,所以它**前台阻塞**——systemd 正是靠这点用 `Type=simple` 管着它。
> 想脱离终端就用上面的 `setsid nohup`。`run_fslam.sh` 是容器模式,起完就返回,不阻塞。
> `run_rtk_only_native.sh` ends in `wait`, so it **blocks in the foreground** — which is exactly how systemd
> supervises it with `Type=simple`. Use the `setsid nohup` form to detach. `run_fslam.sh`
> is container-based and returns immediately instead.

### 环境变量覆盖 | Environment overrides

不用改脚本,直接用环境变量指路 —— 发布包当工作区就是这么用的:
No need to edit the scripts; point them elsewhere with environment variables, which is exactly how
the release bundle is used as a workspace:

```bash
WS=/path/to/bundle/driver \
FIXPOSITION_CONFIG_DIR=/path/to/other/config \
LOG_DIR=/tmp/m20_logs \
ROS_DOMAIN_ID=0 \
    ./run_rtk_only_native.sh
```

`run_rtk_only_native.sh` 认:`WS`、`ROS_SETUP`、`SDK_PREFIX`、`FIXPOSITION_CONFIG_DIR`、`LOG_DIR`、
`ROS_DOMAIN_ID`。容器版另有 `DOCKER_IMAGE`、`CONTAINER_NAME` 等。
`run_rtk_only_native.sh` honours `WS`, `ROS_SETUP`, `SDK_PREFIX`, `FIXPOSITION_CONFIG_DIR`, `LOG_DIR` and
`ROS_DOMAIN_ID`. The container launchers add `DOCKER_IMAGE`, `CONTAINER_NAME` and friends.

### 停止 | Stopping

脚本把子进程 pid 写在日志目录里,手工起的就手工停:
The scripts write their children's pids into the log directory; what you start by hand, you stop by
hand:

```bash
kill "$(cat logs/rtk_only/driver.pid)" "$(cat logs/rtk_only/rsp.pid)" 2>/dev/null
# 容器模式 | container mode
docker stop fixposition-runtime && docker rm fixposition-runtime
```

日志:`logs/<模式>/fixposition.log`(驱动)、`robot_state_publisher.log`。
Logs: `logs/<mode>/fixposition.log` for the driver, plus `robot_state_publisher.log`.

> **手工跑和 systemd 混用要小心。** 手工起的进程不受 unit 的 `Conflicts=` 管;`systemctl start
> rtk_only` 也不会知道你手上已经跑了一个 —— 结果就是两个 `/ODOM` publisher。先停掉一边再起另一边。
> **Be careful mixing manual runs with systemd.** A hand-started process is not covered by the unit's
> `Conflicts=`, and `systemctl start rtk_only` will not notice one you already have running — the
> result is two `/ODOM` publishers. Stop one side before starting the other.

> `--check` 只有 `run_fslam.sh` 有,别的脚本没有这个选项;验证按 §4 走。
> Only `run_fslam.sh` has `--check`; the other launchers do not. Verify per §4 instead.

---

## 3.4 fslam(SLAM)模式部署 | Deploying fslam (SLAM) mode

组成 | what runs:**容器**(`SLAM_IMAGE` 钉版的 LIO-SLAM Humble 镜像:FAST-LIO + PGO + 融合 +
镜像自带的上游 fixposition 驱动)+ **宿主机胶水**(`motion_info_to_twist.py` 轮速上行、
`fp_to_odom.py --ros-args -p relay_enable:=false` 出去畸变点云和状态)。融合节点经参数 overlay 直接发
`/ODOM`,无别名中继。
A **container** (the LIO-SLAM Humble image pinned by `SLAM_IMAGE`: FAST-LIO + PGO + fusion + its
internal upstream fixposition driver) plus **host glue** (`motion_info_to_twist.py` for wheelspeed,
`fp_to_odom.py -p relay_enable:=false` for the deskewed cloud and status). The fusion node publishes
`/ODOM` directly via a param overlay — no alias relay.

**镜像获取 | getting the image:** `SLAM_IMAGE` 文件钉住镜像(私有 registry,release **不附带**)。
狗没有公网,从有 registry 权限的机器搬:
The `SLAM_IMAGE` file pins the image (private registry, **not attached** to releases). The robot has
no internet; ship it from any machine with registry access:

```bash
docker pull "$(cat SLAM_IMAGE)"
docker save "$(cat SLAM_IMAGE)" | gzip > slam_image.tar.gz
scp slam_image.tar.gz robot:/home/user/ && ssh robot 'docker load < /home/user/slam_image.tar.gz'
```

**启动 | start:**

```bash
./run_fslam.sh                 # 默认后台 + docker restart 保活
./run_fslam.sh --check         # 逐话题链路体检 | per-topic pipeline check
./run_fslam.sh --foreground    # 调试:前台 + Ctrl-C 停
```

常用选项:`--image <img>`、`--profile <name>`(默认 m20)、`--fp-stream <uri>`、
`--odom-topic <t>`、`--foxglove`;完整列表见脚本头。
Common options: `--image`, `--profile` (default m20), `--fp-stream`, `--odom-topic`, `--foxglove`;
the full list is in the script header.

**开机自启 | boot service:**

```bash
sudo systemctl disable --now rtk_only rtk_loc m20_loc m20_loc_foxy   # 都发 /ODOM | all publish /ODOM
sudo cp systemd/fslam.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now fslam
```

> 容器是 Humble,受 loopback 墙限制**收不到 OEM 雷达云**(§0);雷达数据由宿主机侧供给(去畸变点云在
> 宿主机 `fp_to_odom.py` 产出,SLAM 输入走仓库内已有的中继链路)。rtk_only 模式(原生 Foxy)不受此影响,
> 这正是两种模式并存的原因之一。
> The container is Humble and — per the loopback wall (§0) — **cannot receive the OEM cloud
> directly**; lidar data is supplied host-side (the deskewed cloud comes from host `fp_to_odom.py`,
> and SLAM input goes through the repo's existing relay path). rtk_only mode (native Foxy) is unaffected,
> which is one reason both modes exist.

---

## 3.4b fslam · native — 原生 Foxy(无容器、无中继)| fslam native on Foxy (no container, no relay)

组成 | what runs:**LIO-SLAM 原生二进制**(`SLAM_NATIVE_RELEASE` 钉版的 `foxy-v*` release tarball:
FAST-LIO + PGO + 融合 + canonical 适配器,全部原生 Foxy 进程)+ **驱动 fork 的上游节点**(与 rtk_only
共用 `/home/user/m20_ws` 安装,但跑的是无 m20 包装的 `fixposition_driver_ros2_exec`,fslam 配置,只出
FPA 流不出 `/ODOM`)+ **同一套宿主机胶水**。原生 Foxy(Fast DDS 2.0)直接收 OEM 的 loopback-only
点云 —— 这是 fslam 摆脱 Humble 容器 + 中继链路的形态。
The **LIO-SLAM native binaries** (the `foxy-v*` release tarball pinned by `SLAM_NATIVE_RELEASE`) +
the **driver fork's upstream node** (same `/home/user/m20_ws` install as rtk_only, but running plain
`fixposition_driver_ros2_exec` with the fslam config — FPA streams only, no `/ODOM`) + the **same host
glue**. Native Foxy (Fast DDS 2.0) receives the OEM's loopback-only cloud directly — no relay.

**包获取 | getting the bundle:** LIO-SLAM 仓库(私有)的 `foxy-v*` release 附带
`lio-slam_<ver>_foxy_arm64.tar.gz`;狗没有公网,从有权限的机器搬:
The private LIO-SLAM repo's `foxy-v*` releases carry the tarball; ship it from a machine with access:

```bash
tar -xzf lio-slam_<ver>_foxy_arm64.tar.gz     # 得到 lio-slam/ | yields lio-slam/
scp -r lio-slam robot:/home/user/             # 路径可换,--slam-dir 指过去即可
ssh robot 'source /home/user/lio-slam/env.sh && echo OK'   # 自检 | sanity
```

**启动 | start:**

```bash
./run_fslam_native.sh                # 驱动 + 管线 + 胶水,全后台(pid 文件)
./run_fslam_native.sh --check        # 逐话题链路体检 | per-topic pipeline check
./run_fslam_native.sh --stop         # 整组停止 | stop everything
```

常用选项:`--slam-dir`(默认 `/home/user/lio-slam`)、`--driver-ws`(默认 `/home/user/m20_ws`)、
`--profile`(默认 m20)、`--lidar-topic`(默认 `/LIDAR/POINTS2` —— 当前固件话题,直吃无中继)、
`--fp-stream <uri>`、`--odom-topic`;完整列表见脚本头。
Common options: `--slam-dir`, `--driver-ws`, `--profile` (default m20), `--lidar-topic`
(default `/LIDAR/POINTS2` — the current firmware topic, consumed directly), `--fp-stream`,
`--odom-topic`; full list in the script header.

**开机自启 | boot service:**

```bash
sudo systemctl disable --now fslam rtk_only rtk_loc m20_loc m20_loc_foxy   # 都发 /ODOM
sudo cp systemd/fslam_native.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now fslam_native
```

---

## 3.5 配置:话题名全部可配 | Configuration: every topic name is configurable

线上用的是 `config/fixposition/config_m20.yaml`(镜像已烘进去,原生方式由 `run_rtk_only_native.sh` 传入)。
驱动发布物里也自带一份同名默认配置,见驱动仓库 `DEPLOYMENT.md` §3.1。
The live file is `config/fixposition/config_m20.yaml` (baked into the image; passed by
`run_rtk_only_native.sh` in the native path). The driver release also bundles a same-named default — see
§3.1 of the driver repo's `DEPLOYMENT.md`.

M20 接口的话题名**逐个可配**,不像通用输出那样只能整体换命名空间:
The M20 interface makes its topic names **individually configurable**, unlike the generic outputs
which only offer a namespace switch:

```yaml
/**:
  ros__parameters:
    m20:
      odom_topic: "/ODOM"                  # 输出:融合位姿 | output: fused pose
      lidar_topic: "/LIDAR/POINTS"         # 输入:OEM 点云 | input: OEM cloud
      motion_info_topic: "/MOTION_INFO"    # 输入:腿式里程/轮速 | input: leg odometry
      matching_odom_topic: ""              # SLAM 模式下监控匹配质量,留空禁用 | SLAM-mode matching health, empty disables
      lidar_to_base_xyzrpy: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # M20 上是 identity | identity on the M20
```

> M20 模式下轮速走 `/MOTION_INFO` 直接进模块,**不要**再去开通用的 `converter` 或往 `speed_topic` 发 ——
> 那是给没有 `/MOTION_INFO` 的平台准备的另一条路。
> In M20 mode wheelspeed enters through `/MOTION_INFO` directly. Do **not** also enable the generic
> `converter` or publish to `speed_topic`; those are the alternative route for platforms without
> `/MOTION_INFO`.

改了 `odom_topic` 就等于改了对外契约,导航侧订阅的名字要同步改。
Changing `odom_topic` changes the outward contract — the navigation subscribers must change with it.

---

## 3.6 支持服务:OEM 地图发布 | Support service: the OEM map publisher

OEM 的 `localization.service` 被禁用后(它会起完整原厂定位,和我们的 `/ODOM` 冲突),
它顺带负责的**地图发布**也没了。`systemd/m20-map-server.service` 把这一块单拆出来:
跑 OEM 原生程序 `map_server`(Fast DDS 2.14),把 `/var/opt/robot/data/maps/active/`
的占据栅格 latched 发布到 `/GRID_MAP`(nav_msgs/OccupancyGrid,reliable +
transient-local),供 OEM 导航栈(localPlanner 等)消费。
**与定位模式无关** —— rtk_only / fslam 哪种模式都需要它,装一次、开机自启、别动。

With the OEM `localization.service` disabled (it would start the full OEM localization —
a duplicate `/ODOM` source), its **map publishing** died with it. `systemd/m20-map-server.service`
carves that piece out: it runs the OEM-native `map_server` (Fast DDS 2.14), LATCHING the
occupancy grid from `/var/opt/robot/data/maps/active/` on `/GRID_MAP` (nav_msgs/OccupancyGrid,
reliable + transient-local) for the OEM nav stack (localPlanner etc.). **Mode-independent** — both rtk_only and fslam need it;
install once, enable at boot, leave it alone.

```bash
cp systemd/m20-map-server.service /etc/systemd/system/
systemctl daemon-reload && systemctl enable --now m20-map-server
systemctl is-active m20-map-server                 # active
```

> 注意三个易混名字:`/GRID_MAP` = map_server 的 latched 地图(OccupancyGrid);
> `/grid_map` = passable_area 的 `grid_map_msgs/GridMap`(无关);`occ_grid` = 地图名。
> latched 话题只有 transient-local 订阅者能拿到已发布的地图;原生 writer 在
> `ros2 topic info` 里可能显示 0 publisher —— 以 transient-local 订阅收到为准:
> Three easily-confused names: `/GRID_MAP` = map_server's latched map (OccupancyGrid);
> `/grid_map` = passable_area's `grid_map_msgs/GridMap` (unrelated); `occ_grid` = the map NAME.
> Only transient-local subscribers receive the latched map, and the native writer may show
> 0 publishers in `ros2 topic info` — judge by reception with a transient-local reader.

---

## 4. 验证清单 | Verification checklist

```bash
systemctl is-active rtk_only                       # active
ros2 topic info /ODOM                                  # Publisher count: 1(且只有 1 | exactly one)
ros2 topic echo /LOC_BODY_POINTS | head -3             # 时间戳应当是新鲜的 | stamp should be fresh
ros2 topic echo --qos-profile sensor_data /LOCATION_STATUS
```

预期 | expected:

| 项 item | 预期 expected |
|---|---|
| `/ODOM` publisher | 恰好 1 个 exactly 1 |
| `/LOCATION_STATUS` | 2.00 Hz,`input_val.lidar=1 imu=1 leg_odom=1`,exec flag 全 0 |
| `/LOC_BODY_POINTS` | ≈9.5 Hz,stamp 跟随最新 `/ODOM` |
| `rsdriver` / `hsLidar` 重启数 restart counts | 部署前后不变 unchanged across the deploy |
| `robot_state_publisher` | 活着 alive(和驱动一起发 `/tf_static`) |

`/LOCATION_STATUS.total_status`:0=未初始化,1=正常,2=低质量,3=丢失。
`total_status`: 0 uninitialized, 1 normal, 2 degraded, 3 lost.

**探针会阶段性"聋"。** 任何"没数据"的结论,先用同一条命令在宿主上跑对照;对照也静默就说明是探针的问题,
不是链路的问题。另外 Foxy 的 `ros2 topic echo` 没有 `--once` / `--field`。
**Probes go deaf intermittently.** Before concluding "no data", run the same command as a
host-namespace control; if the control is silent too, the probe is the problem, not the pipeline.
Also, Foxy's `ros2 topic echo` has no `--once` or `--field`.

---

## 5. 回滚与旧名迁移 | Rollback and old-name migration

回滚(rtk_only 出问题时)| rollback when rtk_only misbehaves:

```bash
sudo systemctl disable --now rtk_only
sudo systemctl enable --now rtk_loc      # 旧 Python 版(宿主 Foxy,雷达可用)| legacy Python (host Foxy, lidar OK)
```

也可以回 `m20_loc`(Humble 容器),但它受 loopback 墙限制拿不到雷达云,`input_val.lidar`=0 —— 只在
Python 版也不可用时用。The `m20_loc` Humble container also exists but cannot get the cloud
(`input_val.lidar`=0) — last resort only.

**旧名迁移 | migrating a robot from pre-rename names**(2026-08-08 前部署的狗跑的是
`m20_loc_foxy.service` → `run_m20_foxy.sh`,即 rtk_only · native 的旧名):
Robots deployed before the rename run the OLD names for the same thing:

```bash
# 同步新文件后 | after syncing the renamed files:
sudo systemctl disable --now m20_loc_foxy
sudo cp systemd/rtk_only.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now rtk_only
sudo rm /etc/systemd/system/m20_loc_foxy.service && sudo systemctl daemon-reload
```

新旧 unit 的 `Conflicts=` 互相覆盖,迁移窗口内也不会出现两个 `/ODOM` publisher。
The new and old units conflict with each other, so even mid-migration two `/ODOM` publishers
cannot coexist.

---

## 6. 常见故障 | Troubleshooting

| 现象 symptom | 原因 cause | 处理 fix |
|---|---|---|
| `/LIDAR/POINTS` 没有 publisher no publisher | 整机上电时 rsdriver 早于雷达就绪启动,writer 根本没建(日志仍打 `send success`) | `sudo systemctl restart rsdriver` |
| unit `active` 但 `/ODOM` publisher=0 | 启动脚本里的进程起来就死了 the launched process died at startup | 看 `logs/rtk_only/fixposition.log` |
| `input_val.lidar=0` | 收不到点云 no cloud | 先确认是不是跑在 Humble 上;再查 rsdriver |
| `/ODOM` 有 publisher 但数据全零、`total_status=0` | VRTK2 没有 GNSS 定位,驱动如实扣着不发 | 查 `/fixposition/fpa/odomstatus`,这是传感器/现场问题 |
| 容器收不到任何数据 container receives nothing | 缺 `-v /dev/shm:/dev/shm` | 加上挂载 add the mount |
| ssh 断、dockerd 卡死 ssh dead, dockerd wedged | 用了默认 bridge 网络跑 docker | 只能整机断电重启;以后只用 `--network host` |

---

## 7. 版本配对 | Version pairing

每个模式·形态各有一个钉版文件:
One pin file per mode·form:

| 钉版文件 pin file | 钉什么 pins | 用于 used by |
|---|---|---|
| `DRIVER_RELEASE` | fork 驱动的 release tag(如 `foxy-v1.0.1`)| rtk_only 模式:流水线下载并核 SHA256 |
| `SLAM_IMAGE` | LIO-SLAM 镜像(如 `wanderer123/fslam-humble:arm64`)| fslam · docker:捆绑包记录引用,镜像不随发布分发(私有 registry)|
| `SLAM_NATIVE_RELEASE` | LIO-SLAM 仓库的 `foxy-v*` release tag | fslam · native:原生 tarball 版本(私有仓库,人工搬运,见 §3.4b)|

fslam 发行版分支和驱动分支/发布线一一对应,分支表见 `DISTRO.md`。
Each fslam distro branch pairs with a driver branch and release line; branch table in `DISTRO.md`.

```bash
# 升级 rtk_only 驱动:在叶子分支改 pin,打标签,推 | on the leaf: edit the pin, tag, push
echo foxy-v1.0.2 > DRIVER_RELEASE
git commit -am "chore(deep-robotics-m20-foxy): pin driver foxy-v1.0.2"
git tag deep-robotics-m20-foxy-v1.2.1 deep-robotics-m20-foxy
git push origin deep-robotics-m20-foxy deep-robotics-m20-foxy-v1.2.1
# 升级 SLAM 镜像同理改 SLAM_IMAGE | bump the SLAM image by editing SLAM_IMAGE likewise
```
