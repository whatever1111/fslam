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
| **`rtk_only`**(VRTK2 RTK/INS:fork 驱动直接交付三话题 the forked `whatever1111/fixposition_driver` delivers the contract itself)| §1:`run_rtk_only_native.sh`,release 包 — **线上 live**(2026-08-08 起)| §3:release 镜像一行命令 — 已在 106 验证 validated |
| **`fslam`**(SLAM:LIO-SLAM,上游原版 fixposition 驱动只出 FPA 流 upstream driver, FPA streams only)| §3.4b:`run_fslam_native.sh`,LIO-SLAM `foxy-v*` release 包(`SLAM_NATIVE_RELEASE` 钉版)— 106 已装 foxy-v1.0.3,遛狗验证中 installed on 106, in drive testing | §3.4:`run_fslam.sh`(`SLAM_IMAGE` 钉版)— 工具链就绪,雷达受 loopback 墙限制 toolchain ready, lidar behind the loopback wall |

只想跑裸驱动、自己给配置:看 fork 驱动仓库的 `DEPLOYMENT.md`。
For the bare driver with your own configuration, see `DEPLOYMENT.md` in the driver fork.

> **为什么模式不是分支 | Why modes are not branches:** 两种模式共存于同一台狗上(SLAM 主用时 rtk
> 是回退),切换靠 systemd 单元,不靠 checkout;分支轴留给 ROS 发行版(foxy/humble/jazzy),那才
> 是和驱动 release 配对的维度。The two modes coexist on one robot (rtk stays as fallback when SLAM
> is primary) and are switched by systemd unit, not by checkout; the branch axis is the ROS distro,
> which is what pairs with driver releases.

---

## 0.5 新狗部署清单(rtk_only · native)| New-robot checklist (rtk_only · native)

一台新 M20 上,我们一共装 **3 个自有服务 + 2 个 drop-in**,外加禁用 1 个 OEM 服务。
On a fresh M20 we install **3 services of our own + 2 drop-ins**, and disable 1 OEM service.

| # | 文件(仓库 → 目标)file (repo → target) | 作用 purpose |
|---|---|---|
| 1 | `systemd/rtk_only.service` → `/etc/systemd/system/` | 定位主服务(fork 驱动 + rsp,发三话题 + handler 别名)the localization service |
| 2 | `systemd/m20-map-server.service` → `/etc/systemd/system/` | OEM 地图发布(`/GRID_MAP` latch;OEM localization 被禁后由我们托管)the OEM map publisher |
| 3 | `systemd/chrony-wait.service` → `/etc/systemd/system/` | 让 `time-sync.target` 真正等到时钟同步(DDS 防楔死的根基)the time barrier |
| 4 | `systemd/handler-dds-dropin.conf` → `/etc/systemd/system/handler.service.d/dds.conf` | handler 强制 UDP(跨版本 SHM 丢大样本 → 0x312)handler on UDP |
| 5 | `systemd/chrony-order-dropin.conf` → `/etc/systemd/system/chrony.service.d/m20-order.conf` | chrony 排在 eth1 设备之后(/dev/ptp1 竞速)chrony ordering |
| — | `systemctl disable --now localization` | 禁用 OEM 原厂定位(否则两个 `/ODOM` 打架)disable the OEM localization |

**前置条件 | prerequisites:**
- 驱动二进制:release 包 `rtk_only_<ver>_foxy_arm64.tar.gz` 解到工作区(§1)。**狗上不再编译**(release 二进制唯一来源)。
- 本仓库工作副本在 `/home/user/fslam`(unit 路径写死了它)
- **时钟链已就位**:`chronyc tracking` 显示 `PHC0` + `Leap status: Normal` —— 即 VRTK2→ptp4l(eth1)
  →chrony 的链(见 §3.6 注)。新狗没有这条链先配时钟,再部署定位。
  The VRTK2 time chain must exist first (`chronyc tracking` → PHC0 + Leap Normal).
- 地图已放置:`/var/opt/robot/data/maps/active/occ_grid.yaml`

**每台狗要改的值 | per-robot values(默认写的是 106):**
- `systemd/rtk_only.service`:VRTK 探活 IP(`10.21.31.66:21000`)
- `config/fixposition/fastdds_handler_udp.xml`:白名单里的本机 OEM 网 IP(`10.21.33.106`)
- `config/fixposition/config_m20.yaml`:VRTK stream 地址;`robot.urdf` 如外参不同

**安装 | install:**

```bash
cd /home/user/fslam
sudo cp systemd/rtk_only.service systemd/m20-map-server.service systemd/chrony-wait.service /etc/systemd/system/
sudo install -D systemd/handler-dds-dropin.conf  /etc/systemd/system/handler.service.d/dds.conf
sudo install -D systemd/chrony-order-dropin.conf /etc/systemd/system/chrony.service.d/m20-order.conf
sudo systemctl disable --now localization rtk_loc m20_loc m20_loc_foxy 2>/dev/null || true
sudo systemctl daemon-reload
sudo systemctl enable chrony-wait rtk_only m20-map-server
sudo reboot   # 开机链自会按 time-sync → 定位/地图 → OEM 栈的顺序起来
```

重启后过一遍 §4 验证清单(重点:`handler` 日志 `Odom=10 IMU=200 Cloud=10`、
`global_planner` 出现 `map message`、我们所有单元 `NRestarts=0`)。
After reboot run the §4 checklist (key: handler at full rates, astar's `map message`,
all our units at `NRestarts=0`).

> 106 现状:定位单元仍用旧名 `m20_loc_foxy.service`(内容已同步最新)——迁移到 `rtk_only`
> 见 §5。新狗直接用新名,别再引入旧名。
> On 106 the localization unit still carries the legacy name `m20_loc_foxy.service`
> (content is current); migration is §5. New robots use the new names only.

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

## 2. 狗上编译 —— 已下线 | Building on the robot — retired

驱动和 SLAM 都只从 release 二进制部署(§1 / §3.4b)。狗上没有公网、CPU 只有 4 核、重编译会饿死 sshd,
`tools/build_rtk_only_native.sh` 只留给开发机上的应急重现,不再是任何部署路径的一部分。
Both the driver and the SLAM stack are deployed from release binaries only (§1 / §3.4b). The robot has
no internet and four cores; a rebuild starves sshd. `tools/build_rtk_only_native.sh` remains for
emergency reproduction on a dev box and is no longer part of any deployment path.

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

## 3.4b fslam · native — 发布包(免编译)| fslam native on Foxy — release bundle (no compile)

组成 | what runs:**LIO-SLAM 原生二进制**(`SLAM_NATIVE_RELEASE` 钉版的 LIO-SLAM `foxy-v*` release
tarball:FAST-LIO + PGO + 融合 + canonical 适配器,全部原生 Foxy 进程)+ **驱动 fork 的上游节点**
(与 rtk_only 共用 `/home/user/m20_ws` 安装,但跑的是无 m20 包装的 `fixposition_driver_ros2_exec`,
fslam 配置,只出 FPA 流不出 `/ODOM`)+ **同一套宿主机胶水**。原生 Foxy(Fast DDS 2.0)直接收 OEM 的
loopback-only 点云 —— 这是 fslam 摆脱 Humble 容器 + 中继链路的形态。
The **LIO-SLAM native binaries** (the LIO-SLAM `foxy-v*` release tarball pinned by
`SLAM_NATIVE_RELEASE`) + the **driver fork's upstream node** (same `/home/user/m20_ws` install as
rtk_only, but running plain `fixposition_driver_ros2_exec` with the fslam config — FPA streams only, no
`/ODOM`) + the **same host glue**. Native Foxy (Fast DDS 2.0) receives the OEM's loopback-only cloud
directly — no relay.

**现状 | current state(2026-08-18):** 106 上装的是 `foxy-v1.0.3`(`/home/user/lio-slam`,上一版留在
`/home/user/lio-slam-1.0.2.bak`),**线上服务仍是 `rtk_only`**;fslam · native 按需手工拉起(§3.4b-3 的
遛狗验证),尚未切成开机服务。
On 106 the bundle is `foxy-v1.0.3` (`/home/user/lio-slam`, previous kept as `lio-slam-1.0.2.bak`);
**the live service is still `rtk_only`**; fslam · native is started by hand for drive tests and is not
yet the boot service.

### 3.4b-1 取包与安装 | Get and install the bundle

LIO-SLAM 仓库(私有)的 `foxy-v*` release 附带 `lio-slam_<ver>_foxy_arm64.tar.gz` + `.sha256`
(CI `build-foxy-arm64.yml` 在 arm64 runner 上打包)。狗没有公网,从有权限的机器经跳板搬过去:
The private LIO-SLAM repo's `foxy-v*` releases carry the tarball + `.sha256` (CI
`build-foxy-arm64.yml` on the arm64 runner). Ship it through the jump host:

```bash
# 开发机 | dev box
sha256sum -c lio-slam_<ver>_foxy_arm64.tar.gz.sha256
scp lio-slam_<ver>_foxy_arm64.tar.gz <jump>:/tmp/ && ssh <jump> 'scp /tmp/lio-slam_<ver>_foxy_arm64.tar.gz root@<robot>:/tmp/'

# 狗上,root | on the robot, as root
cd /home/user
sha256sum /tmp/lio-slam_<ver>_foxy_arm64.tar.gz            # 与 .sha256 一致 | must match
[ -d lio-slam ] && mv lio-slam lio-slam-<old>.bak            # 留上一版做回滚 | keep the previous for rollback
tar -xzf /tmp/lio-slam_<ver>_foxy_arm64.tar.gz -C /home/user # 得到 lio-slam/ | yields lio-slam/
chown -R user:user lio-slam
source /home/user/lio-slam/env.sh && echo OK                 # 自检 | sanity
echo foxy-v<ver> > /home/user/fslam/SLAM_NATIVE_RELEASE      # 狗上工作副本没有网,钉版手改 | pin by hand (no git on the robot)
```

同一版本在仓库叶子分支上也要钉(§7):`echo foxy-v<ver> > SLAM_NATIVE_RELEASE && git commit && git push`。
Pin the same version on the leaf branch too (§7).

**每台狗要改的值 | per-robot values:** 同 rtk_only(§0.5):VRTK stream 地址在
`--fp-stream`/驱动配置里,雷达话题 `--lidar-topic`(106 固件是 `/LIDAR/POINTS`,别信 `/LIDAR/POINTS2`)。
Same as rtk_only (§0.5): the VRTK stream (`--fp-stream` / driver config) and the lidar topic
(`--lidar-topic`; the 106 firmware publishes `/LIDAR/POINTS`, not `/LIDAR/POINTS2`).

### 3.4b-2 先手动跑一次(rtk_only 在线也能跑)| Run it by hand first (safe next to a live rtk_only)

```bash
cd /home/user/fslam
# 不起第二个 FP 驱动(吃 rtk_only 已发的 FPA 流),直出话题改名,/ODOM 仍只有 rtk_only 一个 publisher
# no second FP driver (consumes rtk_only's FPA streams); odom on a scratch topic → /ODOM keeps one owner
sudo ./run_fslam_native.sh --no-fixposition --odom-topic /lio_verify_odom
sudo ./run_fslam_native.sh --check          # 逐话题测频 | per-topic rate check
tail -f logs/fslam_native/pipeline_run/pipeline.log   # clouds in=/out=、FE 时延、[REFIX]
sudo ./run_fslam_native.sh --stop
```

健康的样子(106,静止,2026-08-18):`lidar_adapter: clouds in=1107 out=1106` @10 Hz、
`SENSOR_TO_ODOM mean ≈ 70 ms`、PGO 关键帧在走、无 `header stamp is …` 告警。管线 + OEM 栈把 4 核狗顶到
load ≈10,**别同时录包**(实测 load 15、FE 时延 2.3 s、丢帧)。
Healthy (106 at rest, 2026-08-18): clouds in/out at 10 Hz, FE latency ≈70 ms, PGO keyframes running,
no adapter stamp warnings. Pipeline + OEM stack put the 4-core robot at load ≈10 — **do not record at
the same time** (measured: load 15, FE latency 2.3 s, dropped scans).

### 3.4b-3 遛狗验证 | Drive test

`tools/verify_drive.sh` 把该验的都放一起,结束给一页 `REPORT.txt`(§4 有预期值):
`tools/verify_drive.sh` runs every open check together and prints one `REPORT.txt` (expected values in §4):

```bash
sudo tools/verify_drive.sh                # 录包遛狗:录 + 雷达时戳模式探针(包拿回开发机评测)
                                          # recording drive: record + lidar header-stamp probe
sudo tools/verify_drive.sh --start-slam   # 实时遛狗(自动不录):[REFIX] 拉回 + [YAW] 漂移 + 时戳探针
                                          # live drive (recording off): [REFIX] pull + [YAW] drift + stamp probe
```

`[REFIX]` 是管线自己打的日志:FIX 断 ≥10 s 后第一条门内 GPS 到达时的先验残差 `pull2D`(沿/横按估计航向),
5 s 后再打一条 settled —— 与开发机 `evaluate_pgo` 的 fix-recovery pull 同一个量(0814 包 0.97 vs 0.945),
不用录包就能读实时结果。
`[REFIX]` is printed by the pipeline itself: the pre-fit residual `pull2D` (along/cross in the
estimate heading) at the first admitted GPS after a ≥10 s FIX gap, plus a settled line 5 s later — the
same quantity as `evaluate_pgo`'s fix-recovery pull, readable live without a recording.

### 3.4b-4 切成开机服务 | Make it the boot service

只有在 §3.4b-3 的实时遛狗过关后才切(所有定位服务互斥,§0):
Only after the drive test passes (all localization services are mutually exclusive, §0):

```bash
sudo systemctl disable --now fslam rtk_only rtk_loc m20_loc m20_loc_foxy   # 都发 /ODOM | all publish /ODOM
sudo cp systemd/fslam_native.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now fslam_native
```

服务形态下不带 `--no-fixposition`:fslam_native 自己起驱动 fork 的上游节点(FPA 流)+ 管线直出 `/ODOM`。
As a service it runs without `--no-fixposition`: it starts the driver fork's upstream node itself and
the pipeline owns `/ODOM`.

### 3.4b-5 验证与回滚 | Verify and roll back

验证同 §4(`/ODOM` 恰好 1 个 publisher、`/LOCATION_STATUS` 2 Hz、`/LOC_BODY_POINTS` ≈9.5 Hz),外加
`pipeline.log` 里 `clouds in≈out`、FE 时延 <100 ms、`[REFIX]` 数值合理。
Verify as in §4 plus, in `pipeline.log`: clouds in≈out, FE latency <100 ms, sane `[REFIX]` values.

```bash
# 服务回滚 | service rollback
sudo systemctl disable --now fslam_native && sudo systemctl enable --now rtk_only
# 包回滚 | bundle rollback(上一版还在 .bak)
cd /home/user && mv lio-slam lio-slam-<ver>.bad && mv lio-slam-<old>.bak lio-slam
```

`run_fslam_native.sh` 常用选项:`--slam-dir`(默认 `/home/user/lio-slam`)、`--driver-ws`(默认
`/home/user/m20_ws`)、`--profile`(默认 m20)、`--lidar-topic`(默认 `/LIDAR/POINTS`)、`--fp-stream <uri>`、
`--odom-topic`、`--param-overlay`;完整列表见脚本头。
Common options: `--slam-dir`, `--driver-ws`, `--profile` (default m20), `--lidar-topic` (default
`/LIDAR/POINTS`), `--fp-stream`, `--odom-topic`, `--param-overlay`; full list in the script header.

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

> **为什么等 time-sync.target(chrony-wait):** 2026-08-12 五次开机全量取证的根因 ——
> chrony 首启死于尚未出现的 /dev/ptp1(`systemd/chrony-order-dropin.conf` 修此竞速),
> 第二次启动选中 PHC0 后**阶跃系统时钟**;阶跃前创建的 DDS participant(Foxy Fast RTPS
> 2.1.4 定时事件未迁移 steady_clock)与对端的配对**永久楔死**,每次开机受害者不同(实测
> 三对)。阶跃后创建的配对 100% 可靠,与 OEM 启动顺序无关。装上 `systemd/chrony-wait.service`
> 后 `time-sync.target` 真正等到同步才放行,我们的 DDS 单元(rtk_only / fslam /
> m20-map-server)只需声明 `Wants= + After=time-sync.target` —— 纯依赖,无脚本无 sleep,
> **一次起对,开机即稳**。另:不要让单元等 OEM 服务 —— OEM 启动管理器反过来在等我们
> (三次实测),互相等就是死锁。
> **Why wait on time-sync.target (chrony-wait):** root-caused 2026-08-12 over five fully
> journaled boots — chrony's first start dies on a not-yet-present /dev/ptp1 (fixed by
> `systemd/chrony-order-dropin.conf`), and its second start **steps the system clock** on
> selecting PHC0; DDS participants created before the step (Foxy's Fast RTPS 2.1.4 predates
> the steady_clock migration) wedge **permanently**, a different victim pair each boot (three
> observed). After-step pairings are 100% reliable regardless of OEM start order. With
> `systemd/chrony-wait.service` installed, `time-sync.target` genuinely blocks until sync, so
> our DDS units just declare `Wants= + After=time-sync.target` — pure dependencies, no
> scripts, no sleeps, **paired right the first time**. Also: never make units wait for OEM
> services — the OEM startup manager waits for US (measured three times); mutual waiting
> deadlocks.
>
> ```bash
> # 一次性安装时间就绪链 | one-time install of the time-readiness chain
> cp systemd/chrony-wait.service /etc/systemd/system/
> mkdir -p /etc/systemd/system/chrony.service.d
> cp systemd/chrony-order-dropin.conf /etc/systemd/system/chrony.service.d/m20-order.conf
> systemctl daemon-reload && systemctl enable chrony-wait
> ```

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
| fslam · native 附加 extra | `pipeline.log`:`clouds in≈out` @10 Hz,`SENSOR_TO_ODOM mean` <100 ms,无 `header stamp is` 告警;`[REFIX] pull2D` ≲0.6 m(0814 包基线 0.5–0.97) |
| `tools/verify_drive.sh` 报告 report | `[LIDAR-STAMP]` mode A 0 %/flips 0;`[YAW]` FIX 段漂移 <2°/100 m;`[GUARD]` starved 0 |

`/LOCATION_STATUS.total_status`:0=未初始化,1=正常,2=低质量,3=丢失。
`total_status`: 0 uninitialized, 1 normal, 2 degraded, 3 lost.

**探针会阶段性"聋"。** 任何"没数据"的结论,先用同一条命令在宿主上跑对照;对照也静默就说明是探针的问题,
不是链路的问题。另外 Foxy 的 `ros2 topic echo` 没有 `--once` / `--field`。
**Probes go deaf intermittently.** Before concluding "no data", run the same command as a
host-namespace control; if the control is silent too, the probe is the problem, not the pipeline.
Also, Foxy's `ros2 topic echo` has no `--once` or `--field`.

---

## 5. 回滚与旧名迁移 | Rollback and old-name migration

fslam · native 回滚见 §3.4b-5(服务回 rtk_only,包回 `.bak`)。fslam native rollback: §3.4b-5.

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
# 升级原生 SLAM 包:狗上装好(§3.4b-1)后改 SLAM_NATIVE_RELEASE | native SLAM bundle: after §3.4b-1, edit SLAM_NATIVE_RELEASE
echo foxy-v1.0.3 > SLAM_NATIVE_RELEASE
git commit -am "chore(deep-robotics-m20-foxy): pin LIO-SLAM native bundle foxy-v1.0.3"
```
