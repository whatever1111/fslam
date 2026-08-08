# M20 定位部署指南(fslam 版)| M20 Localization Deployment (fslam)

本文讲**整套 M20 定位链路**:驱动 + fslam 产品层,对外交付 OEM 三话题契约
`/ODOM` + `/LOC_BODY_POINTS` + `/LOCATION_STATUS`。只想跑裸驱动、自己给配置,看驱动仓库的
`DEPLOYMENT.md`。

This covers **the whole M20 localization chain** — driver plus the fslam product layer — delivering
the OEM three-topic contract `/ODOM` + `/LOC_BODY_POINTS` + `/LOCATION_STATUS`. For the bare driver
with your own configuration, see `DEPLOYMENT.md` in the driver repo.

**当前线上形态(2026-08-08 起):狗上原生 Foxy,无容器**,unit 是 `m20_loc_foxy.service`。
**Live since 2026-08-08: native Foxy on the robot, no container**, via `m20_loc_foxy.service`.

---

## 0. 动手前必须知道的三件事 | Three things to know first

1. **三套部署互斥。** `m20_loc_foxy`(原生 Foxy)、`m20_loc`(Humble 容器)、`rtk_loc`(Python)
   都发 `/ODOM`,同时跑就是给导航塞多个打架的位姿源。unit 里已写 `Conflicts=`,但手工起进程绕得过去。
   **The three deployments are mutually exclusive.** `m20_loc_foxy` (native Foxy), `m20_loc` (Humble
   container) and `rtk_loc` (Python) all publish `/ODOM`; running two gives navigation competing pose
   sources. The unit declares `Conflicts=`, but hand-started processes bypass that.

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

## 1. 方式 A:发布包(推荐,免编译)| Release bundle (recommended, no compile)

从 fslam 的 `foxy-v*` release 取 `fslam-m20_<ver>_foxy_arm64.tar.gz`。包里 `driver/` 是驱动二进制树,
`fslam/` 是启动脚本 + 配置 + unit。`driver/` 的布局就是 `run_m20_foxy.sh` 认的 `WS`,可以直接当工作区。

Take `fslam-m20_<ver>_foxy_arm64.tar.gz` from an fslam `foxy-v*` release. Inside, `driver/` is the
driver binary tree and `fslam/` is the launcher, config and unit. `driver/` matches the `WS` layout
`run_m20_foxy.sh` expects, so it works as a drop-in workspace.

```bash
sha256sum -c SHA256SUMS.txt
tar -xzf fslam-m20_<ver>_foxy_arm64.tar.gz
cd fslam-m20-foxy

# 手动跑一次看看 | run it by hand first
WS="$(pwd)/driver" ./fslam/run_m20_foxy.sh
```

装成开机服务 | install as a boot service:

```bash
sudo systemctl disable --now rtk_loc m20_loc        # 先关掉别的 | stop the others first
sudo cp fslam/systemd/m20_loc_foxy.service /etc/systemd/system/
sudo mkdir -p /etc/systemd/system/m20_loc_foxy.service.d
sudo tee /etc/systemd/system/m20_loc_foxy.service.d/bundle.conf >/dev/null <<EOF
[Service]
Environment=WS=/path/to/fslam-m20-foxy/driver
ExecStart=
ExecStart=/bin/bash /path/to/fslam-m20-foxy/fslam/run_m20_foxy.sh
EOF
sudo systemctl daemon-reload && sudo systemctl enable --now m20_loc_foxy
```

---

## 2. 方式 B:狗上编译(现行线上形态)| Build on the robot (the current live setup)

线上就是这条:源码在 `/home/user/m20_src`(驱动 `m20-foxy` 分支),工作区 `/home/user/m20_ws`,
仓库工作副本 `/home/user/fslam`。
This is what's running now: sources at `/home/user/m20_src` (driver branch `m20-foxy`), workspace
`/home/user/m20_ws`, repo working copy `/home/user/fslam`.

```bash
# 源码整棵送过去(含 fixposition-sdk 子模块)| ship the whole tree, submodule included
#   tar czf - . | ssh <robot> 'mkdir -p /home/user/m20_src && tar xzf - -C /home/user/m20_src'

# 编译钉在 2 个核上,别把狗跑满 | pin the build to 2 cores; never saturate the robot
BUILD_CPUS=2,3 BUILD_JOBS=2 tools/build_m20_foxy.sh --source /home/user/m20_src

sudo cp systemd/m20_loc_foxy.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now m20_loc_foxy
```

> 源码同步务必**整棵覆盖**,别一个文件一个文件挑。曾经因为漏同步一个 `.cpp` 而头文件里还留着声明,
> 直接编出 `undefined reference`。
> Always sync the **whole tree** rather than picking files: a missed `.cpp` once left a declaration
> in the header with no definition and produced an `undefined reference`.

---

## 3. 方式 C:Docker | Docker

镜像已把 `config_m20.yaml` 烘进 `/config/fixposition.yaml`,不用挂配置。
The image bakes `config_m20.yaml` at `/config/fixposition.yaml`, so no config mount is needed.

```bash
docker load < fslam-m20_<ver>_foxy_arm64_docker-image.tar.gz
docker run -d --name m20-loc --restart unless-stopped \
    --network host -v /dev/shm:/dev/shm \
    ghcr.io/whatever1111/fslam-m20:foxy-<ver>
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

| 脚本 script | 模式 mode | 配置 config | 日志目录 log dir |
|---|---|---|---|
| `run_m20_foxy.sh` | **M20 原生 Foxy(线上)** native Foxy, live | `config_m20.yaml` | `logs/m20_foxy/` |
| `run_m20_prod.sh` | M20 Humble 容器 Humble container | `config_m20.yaml` | `logs/m20/` |
| `run_fixposition_prod.sh` | **fixposition-only**(纯 RTK,无 SLAM;容器驱动 + 宿主 Python 节点) pure RTK, no SLAM | `config_fp_only.yaml` | `logs/fixposition_only/` |
| `run_fast_lio_pgo_prod.sh` | SLAM(FAST-LIO + PGO) | — | `logs/fslam_prod/` |

前三个都发 `/ODOM`,**任何时候只能跑一个**。脚本启动时会自动停掉另外两套(容器 + pid 文件),所以直接换脚本
跑就是切模式,不用先手动清场。
The first three all publish `/ODOM`, so **only one may run at a time**. Each script stops the other
two on startup (containers plus pid files), so switching modes is just running a different script —
no manual teardown needed.

### 跑起来 | Running

```bash
cd /home/user/fslam

# 前台跑(会一直占着终端,Ctrl-C 停)| foreground: it holds the terminal, Ctrl-C stops it
./run_m20_foxy.sh

# 后台跑 | detached
setsid nohup ./run_m20_foxy.sh > /tmp/m20_foxy.log 2>&1 &
```

> `run_m20_foxy.sh` 最后是 `wait`,所以它**前台阻塞**——systemd 正是靠这点用 `Type=simple` 管着它。
> 想脱离终端就用上面的 `setsid nohup`。`run_fixposition_prod.sh` 是容器模式,起完就返回,不阻塞。
> `run_m20_foxy.sh` ends in `wait`, so it **blocks in the foreground** — which is exactly how systemd
> supervises it with `Type=simple`. Use the `setsid nohup` form to detach. `run_fixposition_prod.sh`
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
    ./run_m20_foxy.sh
```

`run_m20_foxy.sh` 认:`WS`、`ROS_SETUP`、`SDK_PREFIX`、`FIXPOSITION_CONFIG_DIR`、`LOG_DIR`、
`ROS_DOMAIN_ID`。容器版另有 `DOCKER_IMAGE`、`CONTAINER_NAME` 等。
`run_m20_foxy.sh` honours `WS`, `ROS_SETUP`, `SDK_PREFIX`, `FIXPOSITION_CONFIG_DIR`, `LOG_DIR` and
`ROS_DOMAIN_ID`. The container launchers add `DOCKER_IMAGE`, `CONTAINER_NAME` and friends.

### 停止 | Stopping

脚本把子进程 pid 写在日志目录里,手工起的就手工停:
The scripts write their children's pids into the log directory; what you start by hand, you stop by
hand:

```bash
kill "$(cat logs/m20_foxy/driver.pid)" "$(cat logs/m20_foxy/rsp.pid)" 2>/dev/null
# 容器模式 | container mode
docker stop fixposition-runtime && docker rm fixposition-runtime
```

日志:`logs/<模式>/fixposition.log`(驱动)、`robot_state_publisher.log`。
Logs: `logs/<mode>/fixposition.log` for the driver, plus `robot_state_publisher.log`.

> **手工跑和 systemd 混用要小心。** 手工起的进程不受 unit 的 `Conflicts=` 管;`systemctl start
> m20_loc_foxy` 也不会知道你手上已经跑了一个 —— 结果就是两个 `/ODOM` publisher。先停掉一边再起另一边。
> **Be careful mixing manual runs with systemd.** A hand-started process is not covered by the unit's
> `Conflicts=`, and `systemctl start m20_loc_foxy` will not notice one you already have running — the
> result is two `/ODOM` publishers. Stop one side before starting the other.

> `--check` 只有 `run_fast_lio_pgo_prod.sh` 有,别的脚本没有这个选项;验证按 §4 走。
> Only `run_fast_lio_pgo_prod.sh` has `--check`; the other launchers do not. Verify per §4 instead.

---

## 3.5 配置:话题名全部可配 | Configuration: every topic name is configurable

线上用的是 `config/fixposition/config_m20.yaml`(镜像已烘进去,原生方式由 `run_m20_foxy.sh` 传入)。
驱动发布物里也自带一份同名默认配置,见驱动仓库 `DEPLOYMENT.md` §3.1。
The live file is `config/fixposition/config_m20.yaml` (baked into the image; passed by
`run_m20_foxy.sh` in the native path). The driver release also bundles a same-named default — see
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

## 4. 验证清单 | Verification checklist

```bash
systemctl is-active m20_loc_foxy                       # active
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

## 5. 回滚 | Rollback

```bash
sudo systemctl disable --now m20_loc_foxy
sudo systemctl enable --now m20_loc          # 回到 Humble 容器版 | back to the Humble container
```

镜像 `fslam-m20:arm64` 仍在狗上。注意:Humble 容器拿不到雷达点云(见 §0),`input_val.lidar` 会是 0。
The `fslam-m20:arm64` image is still on the robot. Note the Humble container cannot get the lidar
cloud (§0), so `input_val.lidar` will read 0.

---

## 6. 常见故障 | Troubleshooting

| 现象 symptom | 原因 cause | 处理 fix |
|---|---|---|
| `/LIDAR/POINTS` 没有 publisher no publisher | 整机上电时 rsdriver 早于雷达就绪启动,writer 根本没建(日志仍打 `send success`) | `sudo systemctl restart rsdriver` |
| unit `active` 但 `/ODOM` publisher=0 | 启动脚本里的进程起来就死了 the launched process died at startup | 看 `logs/m20_foxy/fixposition.log` |
| `input_val.lidar=0` | 收不到点云 no cloud | 先确认是不是跑在 Humble 上;再查 rsdriver |
| `/ODOM` 有 publisher 但数据全零、`total_status=0` | VRTK2 没有 GNSS 定位,驱动如实扣着不发 | 查 `/fixposition/fpa/odomstatus`,这是传感器/现场问题 |
| 容器收不到任何数据 container receives nothing | 缺 `-v /dev/shm:/dev/shm` | 加上挂载 add the mount |
| ssh 断、dockerd 卡死 ssh dead, dockerd wedged | 用了默认 bridge 网络跑 docker | 只能整机断电重启;以后只用 `--network host` |

---

## 7. 版本配对 | Version pairing

fslam 的发行版分支和驱动分支/发布线一一对应,`DRIVER_RELEASE` 文件钉住具体的驱动 release,发布流水线会
按它下载并校验 SHA256。分支表见 `DISTRO.md`。

Each fslam distro branch pairs with a driver branch and release line; the `DRIVER_RELEASE` file pins
the exact driver release, which the pipeline downloads and SHA256-verifies. Branch table in
`DISTRO.md`.

```bash
# 升级驱动:改 pin,打标签,推 | bump the driver: edit the pin, tag, push
echo foxy-v1.0.1 > DRIVER_RELEASE
git commit -am "chore(foxy): pin driver foxy-v1.0.1"
git tag foxy-v1.0.2 && git push origin foxy foxy-v1.0.2
```
