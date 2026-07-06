# fslam — 云深处 M20 机器狗 SLAM 部署 | Deep Robotics M20 deployment

基于 LIO-SLAM CI/CD 发布镜像 `wanderer123/fslam-humble:arm64` 的生产部署仓库。
狗上只需要这个 checkout + docker,不需要源码/编译。

## 部署 | Deploy

```bash
# 首次 | first time
git clone https://github.com/whatever1111/fslam.git && cd fslam
docker pull wanderer123/fslam-humble:arm64

# 启动(配置树自动从本 checkout 的 fslam/config 挂载)
./fslam/run_fast_lio_pgo_prod.sh

# 更新 = git pull(配置/脚本) + docker pull(算法二进制)
```

容器内启动顺序:① Fixposition driver(等 RTK/FPA 输出)→ ② canonical 管线
(`--profile m20`,`/odom` @100Hz)→ ③ `/odom→/ODOM` 别名中继。
宿主机轮速桥 `motion_info_to_twist.py` 若存在会自动拉起。

常用:

```bash
./fslam/run_fast_lio_pgo_prod.sh --foreground        # 前台调试
./fslam/run_fast_lio_pgo_prod.sh --fp-stream tcpcli://<ip>:21000
docker logs -f fslam-runtime                          # 看运行日志
docker stop fslam-runtime && docker rm fslam-runtime  # 停止
```

日志/录包落 `~/fslam/logs`(挂载为容器内 `/data`)。

## 目录 | Layout

- `fslam/run_fast_lio_pgo_prod.sh` — 唯一启动脚本(来源:LIO-SLAM 仓库 `tools/`,勿在此直接改)
- `fslam/config/` — canonical 三层配置(base + profiles/m20 + modes),同样来源于 LIO-SLAM 仓库
- `fixposition/` — fixposition-only 模式(`fslam/run_fixposition_prod.sh`,纯 RTK 无 SLAM)
- `motion_info_to_twist.py` — 宿主机轮速桥(狗 `/MOTION_INFO` → FP 设备内部融合)
- `legacy/` — 旧链(fp_imu_relay/字段重命名/ecef 桥时代)存档,**勿再使用**

## 注意 | Notes

- 镜像烘入 m20 profile 后,`fslam/config/` 仅作覆盖层;两边不一致时以本 checkout 为准(挂载优先)。
- 配置改动请在 LIO-SLAM 仓库改并同步过来,保持单一事实源。
