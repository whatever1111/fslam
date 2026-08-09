# 生产部署方法文档

> 适用对象: 把 `lio_slam_fastLioPGO` + FAST-LIO2 仅以「编译产物 + 外挂参数」形式分发到目标机器,
> 通过 `tools/run_fast_lio_pgo_prod.sh` 拉起实时 SLAM。

## 1. 部署模型

### 1.1 制品形式: Docker 镜像 = 唯一交付物

整个仓库 (`Dockerfile` / `Dockerfile.arm64`) 已经实现了「源码不进 runtime 镜像」:

- builder 阶段: 完整源码 + 依赖 + colcon 构建
- runtime 阶段: 只 `COPY --from=builder /root/ros2_ws/install`,并删除 `include/` 与所有源文件
  (`Dockerfile:104-105`, `Dockerfile.arm64:122-125`)

因此目标机上**不需要源码**,只需要:

1. 一个对应架构的镜像 (tar 文件或私有 registry pull)
2. 外挂的算法配置目录 (`params_fast_lio_pgo.yaml` + `mower.yaml`)
3. 启动脚本 `tools/run_fast_lio_pgo_prod.sh` (单文件 bash,可独立 scp)
4. 已就绪的传感器 / 驱动节点(LiDAR、Fixposition、轮速)在 host 或同一 ROS 域内运行

### 1.2 算法不在镜像里参数化什么

镜像不包含 yaml,完全依赖外挂:

| 文件 | 来源 | 容器内挂载点 |
|------|------|-------------|
| `params_fast_lio_pgo.yaml` | `${CONFIG_DIR}` 或 repo `config/` | `/data/params_fast_lio_pgo.yaml` |
| `mower.yaml`(FAST-LIO2) | `${CONFIG_DIR}` (优先) 或 repo `third_party/FAST_LIO/config` | `${FASTLIO_CONFIG_PATH}/mower.yaml` |

`prod` 脚本逻辑: 若 `CONFIG_DIR` 内同时存在两个 yaml,则两份配置都从外部走;否则仅 `params_fast_lio_pgo.yaml` 外挂,`mower.yaml` 仍走仓库内 `third_party/FAST_LIO/config`。

---

## 2. ARM / AMD 通用性分析

### 2.1 脚本本身: 架构中立

`run_fast_lio_pgo_prod.sh` 不含任何 `uname -m` / `x86_64` / `aarch64` 判断,只依赖 `${DOCKER_IMAGE}`,镜像换成什么架构脚本就跑什么架构。**脚本本身天然通用**。

### 2.2 镜像: 必须分别构建

两个 Dockerfile 走不同基础链:

| | `Dockerfile` (amd64) | `Dockerfile.arm64` |
|---|---|---|
| 基础镜像 | `ghcr.io/fixposition/fixposition-sdk:humble-dev` | `ros:humble` |
| Fixposition SDK | 来自基础镜像 | builder 阶段从源码编 |
| GTSAM | `-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF` ✓ | 同上 ✓ |
| Livox-SDK2 | 源码编 | 源码编 |

GTSAM 关闭了 `march=native`,镜像在同架构内可移植。**但跨架构不行**:amd64 镜像不能在 arm64 上跑。

### 2.3 推荐 tag 约定

当前默认 tag `liosam-humble-jammy:latest` 在两端会被各自覆盖,容易混淆。建议:

```bash
# 构建端:
docker build -f Dockerfile        -t liosam-humble-jammy:amd64 .
docker build -f Dockerfile.arm64  -t liosam-humble-jammy:arm64 .

# 部署端:让脚本自动按架构选:
DOCKER_IMAGE="liosam-humble-jammy:$(uname -m | sed 's/x86_64/amd64/;s/aarch64/arm64/')" \
  ./tools/run_fast_lio_pgo_prod.sh
```

或用 `docker buildx` 推一个 multi-arch manifest,让 `:latest` 自动按 host 架构 pull 正确镜像。

---

## 3. 当前 `run_fast_lio_pgo_prod.sh` 的问题清单

### 3.1 Blocker 级(不修会跑错或跑不起来)

#### B1. 配置默认值是「bag 回放专用」,直接拷给生产会算错

| 参数 | 仓库默认 | 生产应有值 | 影响 |
|------|---------|-----------|------|
| `lidar_time_offset` | `37.0` | `0.0` | bag 中是 GPS 闰秒补偿;实时数据本身已对齐,留 37s 会让 PGO 把 LiDAR 时间往前偏 37s,所有 odom-GPS 关联失效 |
| `fast_lio_pgo_imu_preint.imu_time_offset` | `37.0` | `0.0` | 同上,IMU preint 与 FAST-LIO `/Odometry` 时间对不上,IMU 因子全部异常 |
| `mower.yaml: imu_topic` | `/livox/imu` | 视 IMU 源决定 | 若想用 FP IMU,需要改成 `/fixposition/fpa/rawimu` 并把 `imu_input_type` 设为 `fixposition_fpa`。但 prod 脚本没有任何 sed,完全靠外挂 yaml 自行配置 |
| `mower.yaml: wheel_odom_en` | `false` | `true` (推荐基线) | 不开启则 FAST-LIO2 不消费轮速,与测试基线 `USE_TWIST_ODOM=true` 不一致 |

**fix 方案**:在 `${CONFIG_DIR}` 维护一份**生产专用 yaml** (例如 `config_prod/`),把上述值落到文件里,而不是留给运行时 sed。这正符合用户要求的「外挂所有参数」。

#### B2. `prod` 脚本未启动任何传感器驱动

PGO 节点启动后会订阅:

- `/livox/lidar` (CustomMsg) 或 `/livox/points`
- `/livox/imu` 或 `/fixposition/fpa/rawimu` + `/fixposition/fpa/imubias`
- `/fixposition/odometry_enu`
- `/robot/twist` (若 `use_twist_odom: true`)

`tools/run_fast_lio_pgo_prod.sh` 只起了 FAST-LIO2 + PGO 两个进程,**驱动必须由 host 上其他系统启动**(`livox_ros_driver2`、`fixposition_driver_ros2`、轮速发布节点)。需要在文档显式声明,否则部署后会一直等 topic 不出 odom。

#### B3. `entrypoint.sh` 写到 `LOG_DIR`

`LOG_DIR=/tmp/lio_slam_logs` 默认是 logs 目录,把可执行 entrypoint 也放进去,语义混乱、且某些目标机 `/tmp` 是 noexec。

**fix**:把 entrypoint 改成镜像内固定脚本(比如 `COPY` 到 `/usr/local/bin/lio_slam_entrypoint.sh`),或者写到 `${LOG_DIR}/.entrypoint.sh` 并显式 `bash` 调用(不依赖执行位)。

### 3.2 严重(影响生产稳定性)

#### S1. 无重启策略

```diff
docker run \
  --name "${CONTAINER_NAME}" \
+ --restart unless-stopped \
  ...
```

SLAM 偶发崩溃后(GTSAM IndeterminantLinearSystem、ICP 退化等),整车会失去定位直到人工干预。

#### S2. 无 CPU 限制 / 调度策略

只有内存限制 (`--memory=24g`),CPU 没限。在车上常见的多 ROS2 节点共存场景,SLAM 可能挤占规划/控制。建议:

```bash
--cpuset-cpus="0-3"         # 绑核
--cpu-shares=2048           # 提优先级
# 或在 entrypoint 里对关键线程 chrt -f
```

#### S3. PCD 输出无开关,且会写到容器外

`entrypoint.sh` 创建 `/root/ros2_ws/src/FAST_LIO/PCD`(仓库内布局)而不是 `/data/PCD`。当前 prod 模式下,FAST-LIO2 的 `pcd_save_en` 视 `mower.yaml` 而定,如果用户没意识,长时间运行可能写满几十 GB。需要在文档明确:**生产环境强烈建议 `pcd_save_en: false`**,只保留诊断时手动触发 `/map_save` 服务。

#### S4. 输出文件 root 所有

容器以 root 跑,挂载 `LOG_DIR` 后写入文件 host 上是 root:root,运维清理麻烦。建议:

```bash
docker run --user "$(id -u):$(id -g)" ...
```

但要注意 ROS2 humble 镜像内默认用户是 root,需要提前在 builder 里 `useradd` 并 chown 关键路径。

### 3.3 一般(可改可不改)

- `--ipc=host --pid=host --network host` 三件套 OK,但 `--pid=host` 不是必需,删除后容器隔离更干净。
- `OMP_NUM_THREADS=4` 对 4 核车机刚好,8 核 ARM Orin 上偏低,建议做成 env 可覆盖。
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 写死,如果 host 上其它节点用 fastrtps,跨进程通信会失败。建议设成可外部 override 而不是默认。
- 缺乏 `ros2 daemon stop / start` 处理,host 上残留 daemon 可能影响 topic 发现。

---

## 4. 标准操作流程

### 4.1 构建端(开发机)

```bash
# amd64
docker build --network host -f Dockerfile        -t liosam-humble-jammy:amd64 .

# arm64 (在 ARM 机或 buildx 交叉编译)
docker build --network host -f Dockerfile.arm64  -t liosam-humble-jammy:arm64 .

# 导出离线分发包(如果目标机无 registry):
docker save liosam-humble-jammy:arm64 -o liosam_arm64.tar
```

### 4.2 准备生产配置目录

```text
/etc/lio_slam/                       (举例,目标机上)
├── params_fast_lio_pgo.yaml         # 从 config/ 拷贝后改:lidar_time_offset=0.0,
│                                    # imu_time_offset=0.0,
│                                    # use_twist_odom=true,
│                                    # imu_topic 与 imu_input_type 选 livox 或 fp_imu
└── mower.yaml                       # 从 third_party/FAST_LIO/config 拷贝后改:
                                     # imu_topic / imu_input_type / wheel_odom_en
                                     # extrinsic_T / extrinsic_R 按车体标定填
                                     # pcd_save_en=false(生产)
```

`prod` 脚本会把这俩文件拷进 `${LOG_DIR}/params_fast_lio_pgo.yaml`,并把整个 `CONFIG_DIR` 以 `:ro` 挂到 `/config`。

### 4.3 分发与加载镜像

```bash
# A) 离线 tar:
scp liosam_arm64.tar mower@robot:/tmp/
ssh mower@robot 'docker load -i /tmp/liosam_arm64.tar'

# B) 私有 registry:
docker pull registry.internal/liosam-humble-jammy:arm64
docker tag  registry.internal/liosam-humble-jammy:arm64 \
            liosam-humble-jammy:arm64
```

### 4.4 分发启动脚本

```bash
scp tools/run_fast_lio_pgo_prod.sh mower@robot:/usr/local/bin/
ssh mower@robot 'chmod +x /usr/local/bin/run_fast_lio_pgo_prod.sh'
```

> 注意:脚本内 `REPO_ROOT="$(cd ${SCRIPT_DIR}/.. && pwd)"` 用于回退路径(找仓库内 `config/` 和 `third_party/FAST_LIO/config/`)。如果只 scp 单脚本(不带仓库),**必须显式指定 `CONFIG_DIR`**,且 `CONFIG_DIR` 必须同时包含 `params_fast_lio_pgo.yaml` 和 `mower.yaml`,否则 `mower.yaml` 那行 `EXTRA_MOUNTS=-v ${REPO_ROOT}/third_party/FAST_LIO/config:...` 会挂载一个不存在的路径。

### 4.5 目标机依赖检查清单

启动 SLAM 前 host 上必须已经在跑:

```bash
ros2 topic list | grep -E '/livox/(lidar|points|imu)|/fixposition/(odometry_enu|fpa/rawimu|fpa/imubias)|/robot/twist'
```

至少应看到:

- `/livox/points` (或 CustomMsg `/livox/lidar`)
- `/livox/imu` 或 `/fixposition/fpa/rawimu` (取决于 yaml 选哪个 IMU 源)
- `/fixposition/odometry_enu`
- `/robot/twist` (若开启 `use_twist_odom`)

ROS_DOMAIN_ID 必须与驱动一致。

### 4.6 启动

```bash
# 最小调用(用仓库默认 config,适合开发机回归)
sudo run_fast_lio_pgo_prod.sh

# 生产典型调用
DOCKER_IMAGE="liosam-humble-jammy:arm64" \
CONFIG_DIR="/etc/lio_slam" \
LOG_DIR="/var/log/lio_slam/$(date +%Y%m%d_%H%M%S)" \
ROS_DOMAIN_ID=42 \
CONTAINER_NAME=lio_slam \
  run_fast_lio_pgo_prod.sh
```

预期前 30s 输出:

```
[STEP 1/2] Launching FAST-LIO2...
  FAST-LIO2 running (pid ...)
[STEP 2/2] Launching FastLioPGO...
  FastLioPGO running (pid ...)
All nodes running. Ctrl+C to stop.
  Topics:
    /Odometry, /fast_lio_pgo/odometry, /odom ...
```

### 4.7 健康检查

```bash
# 频率
ros2 topic hz /odom                       # ~10Hz
ros2 topic hz /Odometry                   # ~10Hz
ros2 topic hz /fast_lio_pgo/odometry      # ~1Hz (关键帧)

# TF
ros2 run tf2_tools view_frames
# 应包含:map -> odom -> base_link

# 日志
tail -f /var/log/lio_slam/<ts>/fastlio.log
tail -f /var/log/lio_slam/<ts>/pgo.log
```

### 4.8 停止 / 升级

```bash
# 优雅停止
docker stop lio_slam            # entrypoint 的 trap 会先 SIGINT 子进程,sleep 3 再 SIGTERM

# 升级镜像(无停机不可能,SLAM 必须重启)
docker pull registry.internal/liosam-humble-jammy:arm64
docker stop lio_slam && docker rm lio_slam
DOCKER_IMAGE="liosam-humble-jammy:arm64" CONFIG_DIR=/etc/lio_slam run_fast_lio_pgo_prod.sh
```

---

## 5. 必须落到外挂 yaml 的参数清单(Cheat Sheet)

把以下从测试脚本里 sed 出来的覆盖项,**全部固化进 `${CONFIG_DIR}/params_fast_lio_pgo.yaml` 与 `mower.yaml`**:

`params_fast_lio_pgo.yaml`(prod 推荐值):

```yaml
fast_lio_pgo:
  ros__parameters:
    lidar_time_offset: 0.0          # 实时为 0.0
    use_twist_odom: true            # 推荐基线
    pgo_smooth_tau: 0.2             # 已验证最优
    loop_hessian_eigenvalue_ratio: 0.0
    enu_yaw_tau: 5.0
fast_lio_pgo_imu_preint:
  ros__parameters:
    imu_topic: "/fixposition/fpa/rawimu"   # 生产用 FP IMU
    imu_input_type: "fixposition_fpa"
    imu_acc_scale: 1.0                     # FP 已是 m/s²
    imu_time_offset: 0.0                   # 实时为 0.0
fast_lio_pgo_fusion:
  ros__parameters:
    use_relocalization: false              # 默认不启用 NDT 重定位
diagnostics:
  latency_enable: false                    # 生产关闭埋点
```

`mower.yaml`(prod 推荐值):

```yaml
common:
  imu_topic: "/fixposition/fpa/rawimu"    # 与 PGO 一致
  imu_input_type: "fixposition_fpa"
  imu_bias_topic: "/fixposition/fpa/imubias"
  imu_time_offset: 0.0
mapping:
  extrinsic_est_en: false                 # 标定后关闭在线估计
  wheel_odom_en: true
  prior_map_pcd: ""                       # 不预加载先验地图
pcd_save:
  pcd_save_en: false                      # 生产关闭以免写满磁盘
runtime_pos_log_enable: false
```

---

## 6. 一句话结论

> 当前 `run_fast_lio_pgo_prod.sh` 在「镜像分发 + 配置外挂 + 架构无关」**架构骨架上是对的**,
> 但默认配置是为 bag 回放调的,直接落到生产会算错时间偏移、还差一个重启策略和驱动依赖文档;
> 把上面 §3 的 Blocker 级修掉、§5 的 yaml override 落到 `CONFIG_DIR`,这条链路即可作为正式部署流程使用。
