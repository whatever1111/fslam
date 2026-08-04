# fslam — 云深处 M20 机器狗 SLAM 部署 | Deep Robotics M20 deployment

基于 LIO-SLAM CI/CD 发布镜像 `wanderer123/fslam-humble:{arm64,amd64}` 的生产部署仓库。
狗上只需要这个 checkout + docker,不需要源码/编译。

## 部署 | Deploy

```bash
# 首次 | first time
git clone https://github.com/whatever1111/fslam.git && cd fslam
docker pull wanderer123/fslam-humble:arm64

# fslam 模式(RTK + FAST-LIO-PGO)| fslam mode
./run_fast_lio_pgo_prod.sh

# fixposition-only 模式(纯 RTK,无 SLAM)| pure RTK, no SLAM
./run_fixposition_prod.sh

# M20 模式(纯 RTK,三话题由驱动直出,无宿主机 Python 节点)| driver-native
tools/build_m20_image.sh && ./run_m20_prod.sh

# 开机自启(fp-only)| boot-start
sudo cp systemd/rtk_loc.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now rtk_loc

# 更新 = git pull(配置/脚本) + docker pull(算法二进制)
```

## 数据流 | Data flow

两种模式对外接口一致(狗导航栈消费)| both modes expose the same dog-facing interface:

| 话题 | 类型 | 来源(fslam 模式) | 来源(fp-only 模式) |
|------|------|-------------------|---------------------|
| `/ODOM` | nav_msgs/Odometry | canonical 管线**直出**(fusion 节点 `odom_topic:=/ODOM`,无中继) | 宿主机 `fp_to_odom.py` 中继 `/fixposition/odometry_enu` |
| `/LOC_BODY_POINTS` | sensor_msgs/PointCloud2 | 宿主机 `fp_to_odom.py`:`/LIDAR/POINTS2` 去畸变到 base_link,时戳对齐最新 `/ODOM` | 同左 |
| `/LOCATION_STATUS` | drdds/LocationStatus | 宿主机 `fp_to_odom.py`(与点云同戳;total_status: 0 未初始化/1 正常/2 低质量/3 丢失) | 同左 |

fslam 模式:容器内 ① Fixposition driver(等 RTK/FPA 输出)→ ② canonical 管线
(`--profile m20`,fusion 直出 `/ODOM` @100Hz,配置树用本 checkout 的
`config/slam/` 只读覆盖)。宿主机自动拉起 `motion_info_to_twist.py`(轮速桥)
与 `fp_to_odom.py`(`relay_enable:=false`,只订阅 `/ODOM` 作位姿源,不中继)。

> 直出 `/ODOM` 需要镜像内 fusion 节点带 `odom_topic` 参数(旧镜像忽略之,
> `/ODOM` 无数据 → `--check` 可见,请 `docker pull` 更新镜像)。

常用 | common:

```bash
./run_fast_lio_pgo_prod.sh --foreground          # 前台调试
./run_fast_lio_pgo_prod.sh --fp-stream tcpcli://<ip>:21000
./run_fast_lio_pgo_prod.sh --check               # 链路体检:逐话题测频找断点
docker logs -f fslam-runtime                     # 看运行日志
docker stop fslam-runtime && docker rm fslam-runtime   # 停止(宿主机节点随看门狗回收)
```

## M20 模式(C++ 驱动版)| M20 mode (driver-native)

`fp_to_odom.py` 和 `motion_info_to_twist.py` 做的事已经全部移进 Fixposition
driver 进程:三话题、去畸变、2 Hz 状态、轮速回灌都在驱动里完成,宿主机不再跑
Python 节点。驱动源码在 `whatever1111/fixposition_driver` 分支 `m20`,设计说明见
该仓库的 `M20.md`。

Everything `fp_to_odom.py` and `motion_info_to_twist.py` did now happens inside
the Fixposition driver process — the three topics, the deskewing, the 2 Hz
status and the wheelspeed feedback — so no Python runs on the host any more.
The driver lives in `whatever1111/fixposition_driver` branch `m20`; see `M20.md`
there for the design.

**LIO-SLAM 仓库不受影响**:它是多产品共用的基座,M20 专属的东西只在本仓库叠加
(`container/Dockerfile.m20` 在发布镜像上加一层)。
**LIO-SLAM is untouched**: it is the shared base for other products, so the
M20-specific driver is layered on top of its published image from this
repository only.

```bash
tools/build_m20_image.sh                       # 联网机器:自动 clone 驱动源码
tools/build_m20_image.sh --source ./fixposition_driver   # 狗上:源码用 bundle 传过去
./run_m20_prod.sh                              # 起容器(默认镜像 fslam-m20:<arch>)
docker logs -f m20-runtime
```

改回 Python 版随时可以:`./run_fixposition_prod.sh`(它会停掉 M20 容器)。
两套不能同时跑 —— 都发 `/ODOM`,导航会看到两个打架的位姿源。
Going back is always possible with `./run_fixposition_prod.sh`. The two must
never run together: both publish `/ODOM`.

差异 | differences:

| | Python 版 | M20 版 |
|---|---|---|
| 进程 | 容器 driver + 宿主机 2 个 Python 节点 | 容器 driver 一个进程 |
| `/ODOM` | `/fixposition/odometry_enu` 中继一跳 | driver 解析线程内直发,无跳 |
| 点云耗时 | ~46 ms/帧(1 MB 云) | ~1 ms/帧,消息缓冲区原地变换 |
| 轮速 | `/MOTION_INFO` → Twist → driver converter | driver 直接吃 `/MOTION_INFO` |
| DDS profile | 容器白名单 + 宿主机默认 | 容器默认(全接口) |

## 目录 | Layout

```
run_fast_lio_pgo_prod.sh   fslam 模式启动入口 | fslam-mode launcher
run_fixposition_prod.sh    fp-only 模式启动入口(Python 版)| fp-only launcher
run_m20_prod.sh            M20 模式启动入口(驱动直出)| M20-mode launcher
tools/build_m20_image.sh   构建 M20 驱动镜像 | build the M20 driver image
lib/deploy_common.sh       两脚本共享的宿主机进程管理函数 | shared helpers
container/                 容器载荷(只读挂载)| container payloads (mounted ro)
  entrypoint_fslam.sh        · FP driver → canonical 管线
  entrypoint_fixposition.sh  · FP driver + robot_state_publisher
  fixposition_driver.launch  · fslam 模式 FP driver 启动文件(respawn)
  entrypoint_m20.sh          · M20 模式:带 M20 模块的 FP driver + robot_state_publisher
  Dockerfile.m20             · 在发布镜像上叠加 M20 驱动 | M20 driver layered on the base image
host/                      宿主机 ROS2 节点(狗侧 FastDDS + drdds)| host nodes
  fp_to_odom.py              · /ODOM(fp-only 中继)+ /LOC_BODY_POINTS + /LOCATION_STATUS
  motion_info_to_twist.py    · 轮速桥 /MOTION_INFO → FP 设备融合
config/
  dds/                       · cyclonedds.xml(容器)/ fastdds.xml(宿主机)
  fixposition/               · 驱动配置:config_fp_only.yaml(Python 版)/
                               config_m20.yaml(M20 版)/ node.launch / urdf
  slam/                      · canonical 三层配置树(base+profiles+modes,
                               源:LIO-SLAM 仓库,保持同步,勿在此直接改)
systemd/rtk_loc.service    开机自启单元(Python 版 fp-only)| boot unit
systemd/m20_loc.service    开机自启单元(M20 版,与上者互斥)| boot unit (exclusive)
logs/                      运行时日志(git 忽略)| runtime logs (git-ignored)
legacy/                    旧链存档(fp_imu_relay/字段重命名/别名中继时代),勿再使用
```

## 注意 | Notes

- 所有路径均由脚本从 checkout 位置推导,可用环境变量覆盖 —— 无硬编码路径。
  All paths derive from the checkout location (env-overridable) — no hard-coding.
- `config/slam/` 与镜像内烘焙配置两边不一致时以本 checkout 为准(挂载优先);
  配置改动请在 LIO-SLAM 仓库改并同步过来,保持单一事实源。
- `/LOCATION_STATUS` 子字段(exec/loss/input)语义仍待 OEM 规范确认:Python 版见
  `host/fp_to_odom.py` 顶部常量块,M20 版见驱动仓库 `status_monitor.hpp` 顶部,
  两处取值一致,规范到位时一起改。
