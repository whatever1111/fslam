# 宿主机原生二进制部署文档

> 适用对象: 不使用 Docker,直接把编译后的 ROS2 二进制 + 必要共享库部署到目标机器,
> 通过 systemd 或 shell 脚本拉起 SLAM。
> 配套 Docker 部署见 `docs/deployment_guide.md`。

## 1. 路线选择

宿主机部署最快的路径**不是**在目标机上重新 colcon build,而是:

> **从 amd64/arm64 builder 镜像里把 install 树和系统库直接拷出来,目标机只装 ROS2 + 系统依赖。**

理由:

- 仓库已经在 Dockerfile 里把跨发行版差异封死(GTSAM 4.2 源码编、Livox-SDK2 源码编、Fixposition SDK 源码编),目标机上手工复现这些步骤每次都耗 30 分钟以上,且容易因 GCC/PCL 小版本差异挂掉。
- builder 出来的 `install/` 已经是「无源码、含 .so 和 share」的合规交付物,RPATH 里也已经写好了 `$ORIGIN/../lib`,直接拷过去就能 source。

因此本指南的核心动作是「**docker cp 拷文件 → tar 打包 → scp 到目标机 → 装系统包 → source → 跑**」,中间不在目标机上跑任何编译。

如果目标机连 Docker 都装不了 / 没有任何 builder 机,只能在目标机上从零编译,见 §8。

---

## 2. 目标机前置依赖

### 2.1 操作系统 ABI 必须对齐

| 项 | 要求 |
|---|---|
| 发行版 | Ubuntu 22.04 (Jammy) |
| 架构 | amd64 或 arm64,**必须与 builder 镜像架构一致** |
| ROS2 | Humble |
| libstdc++ | builder 与 target 的 GCC major version 必须一致(Jammy 默认 GCC 11)|

跨发行版部署(Ubuntu 20.04 / 24.04 / Debian)**不要走这个路径**,直接走 Docker 路线更可靠。

### 2.2 系统包安装(目标机一次性)

```bash
sudo apt update && sudo apt install -y \
    ros-humble-ros-base \
    ros-humble-pcl-conversions \
    ros-humble-pcl-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosbag2 \
    libpcl-dev libapr1 libtbb12 libeigen3-dev libyaml-cpp0.7 \
    libssl3 zlib1g python3-pip
```

> `libpcl-dev` 装的是 PCL 1.12 全套(运行只需 runtime,但开发包能拉齐 deps)。

### 2.3 不要从 apt 装 GTSAM

apt 上没有 GTSAM 4.2 的 Jammy 包,且我们的 BetweenFactor / ISAM2 相关 API 与 4.0 系列不兼容。**GTSAM 必须从 builder 镜像里整套 .so 拷过来,放到 `/usr/local/lib`。**

---

## 3. 从 builder 镜像导出制品

下面假设你有一个已经构建完的 `liosam-humble-jammy:amd64` 或 `:arm64` 镜像。

```bash
# 起一个临时容器(不真启动)
CID=$(docker create liosam-humble-jammy:arm64)

# 1) ROS2 install 树(算法二进制 + share + .so)
docker cp ${CID}:/root/ros2_ws/install ./_export/ros2_ws_install

# 2) /usr/local 下的第三方 .so(GTSAM / Livox / Fixposition SDK)
docker cp ${CID}:/usr/local/lib  ./_export/usr_local_lib_raw
mkdir -p ./_export/usr_local_lib
cp -a ./_export/usr_local_lib_raw/libgtsam*    ./_export/usr_local_lib/
cp -a ./_export/usr_local_lib_raw/libmetis*    ./_export/usr_local_lib/
cp -a ./_export/usr_local_lib_raw/liblivox*    ./_export/usr_local_lib/
cp -a ./_export/usr_local_lib_raw/libfpsdk*    ./_export/usr_local_lib/ 2>/dev/null || true
rm -rf ./_export/usr_local_lib_raw

# 3) 仓库内的 yaml 与 launch(可选,生产建议外挂自己的)
docker cp ${CID}:/root/ros2_ws/install/lio_slam/share/lio_slam/config ./_export/config_template
docker cp ${CID}:/root/ros2_ws/install/fast_lio/share/fast_lio       ./_export/fast_lio_share

docker rm ${CID}

# 4) 打包
tar czf liosam-native-arm64-$(date +%Y%m%d).tar.gz -C ./_export .
```

打出来的 tar 大概 80–150MB,内容:

```
ros2_ws_install/                # colcon install,完整 install/ 树
├── lio_slam/
│   ├── lib/lio_slam/lio_slam_fastLioPGO          # ← 主二进制
│   ├── lib/lio_slam/lio_slam_ndtRelocalization
│   ├── share/lio_slam/launch/run_fast_lio_pgo.launch.py
│   └── share/lio_slam/config/params_fast_lio_pgo.yaml
├── fast_lio/
│   ├── lib/fast_lio/fastlio_mapping              # ← 前端二进制
│   └── share/fast_lio/...
├── fixposition_driver_msgs/...
└── setup.bash, local_setup.bash, ...
usr_local_lib/                  # 第三方 .so
├── libgtsam.so.*, libgtsam_unstable.so.*
├── libmetis*.so
└── liblivox_sdk*.so
```

### 3.1 RPATH 检查

builder 镜像里的二进制 RPATH 写的是 `$ORIGIN/../lib:/opt/ros/humble/lib:/usr/local/lib`,目标机只要把库装在同样位置即可,不必改 RPATH:

```bash
chrpath -l ./_export/ros2_ws_install/lio_slam/lib/lio_slam/lio_slam_fastLioPGO
# RPATH=$ORIGIN/../lib:/opt/ros/humble/lib/x86_64-linux-gnu:...
```

如果 RPATH 缺 `/usr/local/lib`,后面 §4.3 用 `ldconfig` 解决。

---

## 4. 在目标机安装

### 4.1 安装路径建议

```
/opt/lio_slam/                           # 二进制根目录
├── ros2_ws_install/                     # 解压自 tar
└── usr_local_lib/                       # 解压自 tar(后面 ldconfig 进来)

/etc/lio_slam/                           # 外挂配置(由运维维护)
├── params_fast_lio_pgo.yaml
└── mower.yaml

/var/log/lio_slam/                       # 日志
```

### 4.2 解压

```bash
sudo mkdir -p /opt/lio_slam /etc/lio_slam /var/log/lio_slam
sudo tar xzf liosam-native-arm64-*.tar.gz -C /opt/lio_slam
sudo chown -R "$USER":"$USER" /opt/lio_slam /var/log/lio_slam
```

### 4.3 注册第三方共享库

```bash
sudo cp -a /opt/lio_slam/usr_local_lib/* /usr/local/lib/
sudo ldconfig
ldconfig -p | grep -E 'gtsam|livox'    # 确认能找到
```

> 不想往 `/usr/local/lib` 塞东西,可以放原地 + 写 ldconfig 配置:
> ```bash
> echo "/opt/lio_slam/usr_local_lib" | sudo tee /etc/ld.so.conf.d/lio_slam.conf
> sudo ldconfig
> ```

### 4.4 依赖完整性校验

```bash
ldd /opt/lio_slam/ros2_ws_install/lio_slam/lib/lio_slam/lio_slam_fastLioPGO | grep 'not found'
ldd /opt/lio_slam/ros2_ws_install/fast_lio/lib/fast_lio/fastlio_mapping     | grep 'not found'
```

任何 `not found` 都要先解决再往下。

常见缺失:

| 缺什么 | apt 包 |
|--------|-------|
| `libpcl_*.so.1.12` | `libpcl-dev` |
| `libtbb.so.12` | `libtbb12` |
| `libssl.so.3` | `libssl3` |
| `libyaml-cpp.so.0.7` | `libyaml-cpp0.7` |

---

## 5. 配置文件(完全外挂)

`/etc/lio_slam/` 下的两个 yaml 由运维准备,不进二进制包。

### 5.1 `params_fast_lio_pgo.yaml`(生产推荐值)

```yaml
fast_lio_pgo:
  ros__parameters:
    lidar_time_offset: 0.0          # 实时数据为 0.0,bag 回放才是 37.0
    use_twist_odom: true
    pgo_smooth_tau: 0.2
    loop_hessian_eigenvalue_ratio: 0.0
    enu_yaw_tau: 5.0

fast_lio_pgo_imu_preint:
  ros__parameters:
    imu_topic: "/fixposition/fpa/rawimu"
    imu_input_type: "fixposition_fpa"
    imu_acc_scale: 1.0
    imu_time_offset: 0.0

fast_lio_pgo_fusion:
  ros__parameters:
    use_relocalization: false

diagnostics:
  latency_enable: false
```

### 5.2 `mower.yaml`(FAST-LIO2 配置,生产推荐值)

```yaml
common:
  imu_topic: "/fixposition/fpa/rawimu"
  imu_input_type: "fixposition_fpa"
  imu_bias_topic: "/fixposition/fpa/imubias"
  imu_time_offset: 0.0

mapping:
  extrinsic_est_en: false              # 标定后关闭
  wheel_odom_en: true
  prior_map_pcd: ""
  extrinsic_T: [-0.13045, 0.01, -0.0149]   # 按车体标定填
  extrinsic_R: [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]

pcd_save:
  pcd_save_en: false                   # 生产关闭

runtime_pos_log_enable: false
```

> 这两个文件就是 §4 文档里的「外挂参数清单」,把测试脚本所有 sed 行落到文件里。

---

## 6. 启动脚本(原生版)

把下面这段保存为 `/usr/local/bin/lio_slam_native.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail

# ==== 可由 systemd 或环境覆盖 ====
INSTALL_ROOT="${INSTALL_ROOT:-/opt/lio_slam/ros2_ws_install}"
CONFIG_DIR="${CONFIG_DIR:-/etc/lio_slam}"
LOG_DIR="${LOG_DIR:-/var/log/lio_slam}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"

# ==== source ROS2 + 应用 install ====
source /opt/ros/humble/setup.bash
source "${INSTALL_ROOT}/setup.bash"

export ROS_DOMAIN_ID RMW_IMPLEMENTATION OMP_NUM_THREADS
export OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1

mkdir -p "${LOG_DIR}"
TS="$(date +%Y%m%d_%H%M%S)"

# ==== 优雅退出 ====
PIDS=()
cleanup() {
  echo "[INFO] Shutting down..."
  for pid in "${PIDS[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
  sleep 3
  for pid in "${PIDS[@]}"; do kill "${pid}" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM

# ==== 1. FAST-LIO2 ====
ros2 launch fast_lio mapping.launch.py \
    config_path:="${CONFIG_DIR}" \
    config_file:=mower.yaml \
    rviz:=false \
    >"${LOG_DIR}/fastlio_${TS}.log" 2>&1 &
PIDS+=($!)
sleep 5
kill -0 "${PIDS[0]}" 2>/dev/null || { echo "[ERROR] FAST-LIO2 failed"; exit 1; }

# ==== 2. FastLioPGO ====
ros2 run lio_slam lio_slam_fastLioPGO \
    --ros-args --params-file "${CONFIG_DIR}/params_fast_lio_pgo.yaml" \
    >"${LOG_DIR}/pgo_${TS}.log" 2>&1 &
PIDS+=($!)
sleep 2
kill -0 "${PIDS[1]}" 2>/dev/null || { echo "[ERROR] FastLioPGO failed"; exit 1; }

echo "[INFO] All nodes running. PIDs=${PIDS[*]}, logs in ${LOG_DIR}"
wait
```

```bash
sudo chmod +x /usr/local/bin/lio_slam_native.sh
```

### 6.1 手动测试启动

```bash
sudo /usr/local/bin/lio_slam_native.sh
# 另一个终端:
ros2 topic hz /odom            # ~10Hz
ros2 topic hz /Odometry        # ~10Hz
```

---

## 7. systemd 接管(生产推荐)

### 7.1 unit 文件

`/etc/systemd/system/lio_slam.service`:

```ini
[Unit]
Description=LIO-SLAM (FAST-LIO2 + PGO) native runtime
After=network-online.target livox_driver.service fixposition_driver.service
Wants=network-online.target

[Service]
Type=simple
User=mower
Group=mower
Environment=INSTALL_ROOT=/opt/lio_slam/ros2_ws_install
Environment=CONFIG_DIR=/etc/lio_slam
Environment=LOG_DIR=/var/log/lio_slam
Environment=ROS_DOMAIN_ID=0
Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
Environment=OMP_NUM_THREADS=4
ExecStart=/usr/local/bin/lio_slam_native.sh
Restart=on-failure
RestartSec=5
KillSignal=SIGINT
TimeoutStopSec=15

# CPU/IO 优先级(可选)
Nice=-5
CPUSchedulingPolicy=other
CPUAffinity=0 1 2 3

[Install]
WantedBy=multi-user.target
```

### 7.2 上线

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now lio_slam.service
journalctl -u lio_slam -f
```

### 7.3 升级

```bash
# 1) 推新 tar 到目标机
scp liosam-native-arm64-NEW.tar.gz mower@robot:/tmp/
# 2) 切换软链或就地替换
sudo systemctl stop lio_slam
sudo rm -rf /opt/lio_slam/ros2_ws_install
sudo tar xzf /tmp/liosam-native-arm64-NEW.tar.gz -C /opt/lio_slam
sudo cp -a /opt/lio_slam/usr_local_lib/* /usr/local/lib/
sudo ldconfig
sudo systemctl start lio_slam
```

可以做成「双版本 + 软链 atomic switch」:`/opt/lio_slam/current -> v20260505/` 切换,失败回滚只是改软链。

---

## 8. 兜底:在目标机从源码编译

只在目标机无法接收外部镜像/tar、必须本地编译时才走这条路径。

```bash
sudo apt install -y ros-humble-ros-base ros-humble-ament-cmake-auto \
                    libpcl-dev libapr1-dev libtbb-dev libeigen3-dev \
                    libyaml-cpp-dev nlohmann-json3-dev git cmake build-essential

# GTSAM 4.2
git clone --depth 1 -b 4.2.0 https://github.com/borglab/gtsam.git
cd gtsam && cmake -B build -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_TESTS=OFF
sudo cmake --build build --target install --parallel $(nproc)
sudo ldconfig
cd ..

# Livox-SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && cmake -B build -DCMAKE_BUILD_TYPE=Release
sudo cmake --build build --target install
sudo ldconfig
cd ..

# Fixposition SDK(amd64 可走 ghcr 镜像;arm64 必须源码编)
# 见 Dockerfile.arm64:75-83

# LIO-SLAM 主仓库
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --recursive <repo-url> LIO-SLAM
cp -r LIO-SLAM/third_party/FAST_LIO ./FAST_LIO

source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-up-to lio_slam fast_lio fixposition_driver_ros2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**这条路 ARM 上编 GTSAM + ROS2 节点动辄 30–60 分钟,且容易因 IO/内存 OOM 失败,生产升级流程不要依赖它。**

---

## 9. ARM/AMD 注意事项

| 关注点 | amd64 | arm64 |
|---|---|---|
| Builder 镜像 | `Dockerfile`(基于 fixposition-sdk 镜像) | `Dockerfile.arm64`(基于 ros:humble,自编 fpsdk)|
| 第三方库可移植性 | GTSAM `march=native` 关闭 ✓ | 同 ✓ |
| 制品 tar | `liosam-native-amd64-*.tar.gz` | `liosam-native-arm64-*.tar.gz` |
| 性能取舍 | OMP_NUM_THREADS=4 一般够 | Orin/AGX 8 核可调到 6–8 |
| 注意 | x86 builder 出来的 tar **不能**用于 ARM 机器 | 反之亦然 |

为避免拷错,tar 文件名一定带架构 suffix。systemd 启动脚本里也可加一行卫语句:

```bash
ARCH_EXPECTED="aarch64"   # 或 x86_64
[[ "$(uname -m)" == "${ARCH_EXPECTED}" ]] || { echo "[FATAL] arch mismatch"; exit 1; }
```

---

## 10. 与 Docker 部署的取舍

| 维度 | Docker (`run_fast_lio_pgo_prod.sh`) | 原生二进制(本文档) |
|---|---|---|
| 隔离性 | 强,系统包和 host 解耦 | 弱,与 host libstdc++/PCL 强耦合 |
| 启动耗时 | docker run 5–10s | 进程直接 exec,~1s |
| CPU/内存开销 | 容器层略高,DDS shm 需额外 `--ipc=host` | 与 host 同源,最低 |
| 升级 | `docker pull` + restart | tar 替换 + ldconfig + restart |
| 可调试性 | 需 `docker exec` 进容器 | 直接 gdb / strace 二进制 |
| 跨发行版 | OK(只要 docker 可用) | 不可,目标机必须 Jammy |
| systemd 集成 | 通过 `--restart` 或 systemd unit 包 docker run | 直接 systemd Type=simple |

**经验法则:**

- 生产车机系统稳定、和 builder 同发行版 → 走原生,启动更快、调试更直接
- 多车型 / 跨发行版 / 经常换基础系统 → 走 Docker,配置成本一次性付清

---

## 11. 故障排除速查

| 现象 | 排查 |
|---|---|
| `error while loading shared libraries: libgtsam.so.4` | §4.3 ldconfig 没跑,或 GTSAM .so 没拷到 `/usr/local/lib` |
| `libpcl_*.so.1.12 not found` | apt 缺 `libpcl-dev`(运行时也用它的 .so) |
| 启动后 `/Odometry` 无消息 | LiDAR 驱动没起,或 ROS_DOMAIN_ID 与驱动不一致 |
| `/odom` 跳变到 (0,0,0) | `lidar_time_offset` / `imu_time_offset` 仍是 37.0,与实时数据不匹配 |
| GTSAM `IndeterminantLinearSystem` | 通常是因子图退化,不是部署问题;查 pgo.log 上下文 |
| 服务起不来,journalctl 显示 `python3` 找不到 launch | `INSTALL_ROOT` 没 source,或 launch 文件不在 install/share 路径下 |
| arm64 上跑 amd64 二进制 → `Exec format error` | 拿错 tar,见 §9 |

---

## 12. 一句话结论

> 原生部署 = builder 镜像里的 `install/` + `/usr/local/lib/libgtsam* liblivox*` 两份文件直接 docker cp 出来打 tar,
> 目标机装 ROS2 Humble + libpcl + libtbb 等系统包后解压、`ldconfig`、source、systemd 拉起,
> **不要在目标机上 colcon build,也不要从 apt 装 GTSAM**;
> 配置依然全部走 `/etc/lio_slam/` 外挂,与 Docker 部署共用一份 yaml。
