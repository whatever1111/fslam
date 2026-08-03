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

## 目录 | Layout

```
run_fast_lio_pgo_prod.sh   fslam 模式启动入口 | fslam-mode launcher
run_fixposition_prod.sh    fp-only 模式启动入口 | fp-only launcher
lib/deploy_common.sh       两脚本共享的宿主机进程管理函数 | shared helpers
container/                 容器载荷(只读挂载)| container payloads (mounted ro)
  entrypoint_fslam.sh        · FP driver → canonical 管线
  entrypoint_fixposition.sh  · FP driver + robot_state_publisher
  fixposition_driver.launch  · fslam 模式 FP driver 启动文件(respawn)
host/                      宿主机 ROS2 节点(狗侧 FastDDS + drdds)| host nodes
  fp_to_odom.py              · /ODOM(fp-only 中继)+ /LOC_BODY_POINTS + /LOCATION_STATUS
  motion_info_to_twist.py    · 轮速桥 /MOTION_INFO → FP 设备融合
config/
  dds/                       · cyclonedds.xml(容器)/ fastdds.xml(宿主机)
  fixposition/               · fp-only 模式驱动配置(node.launch/yaml/urdf)
  slam/                      · canonical 三层配置树(base+profiles+modes,
                               源:LIO-SLAM 仓库,保持同步,勿在此直接改)
systemd/rtk_loc.service    开机自启单元(fp-only)| boot unit
logs/                      运行时日志(git 忽略)| runtime logs (git-ignored)
legacy/                    旧链存档(fp_imu_relay/字段重命名/别名中继时代),勿再使用
```

## 注意 | Notes

- 所有路径均由脚本从 checkout 位置推导,可用环境变量覆盖 —— 无硬编码路径。
  All paths derive from the checkout location (env-overridable) — no hard-coding.
- `config/slam/` 与镜像内烘焙配置两边不一致时以本 checkout 为准(挂载优先);
  配置改动请在 LIO-SLAM 仓库改并同步过来,保持单一事实源。
- `/LOCATION_STATUS` 子字段(exec/loss/input)语义仍待 OEM 规范确认,
  见 `host/fp_to_odom.py` 顶部常量块。
