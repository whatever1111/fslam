# fslam 发行版分支:foxy | Distro branch: foxy

本分支是 fslam 的 **ROS 2 Foxy** 配对分支,与 fixposition driver 的同发行版分支/发布线绑定。
This is the **ROS 2 Foxy** pairing branch of fslam, tied to the same-distro branch and
release line of the fixposition driver fork.

| fslam 分支 | driver 分支 | driver 发布线 | 部署形态 |
|---|---|---|---|
| `main` | `m20` (trunk) | — | 可移植主干 | portable trunk |
| **`foxy`(本分支 this branch)** | `m20-foxy` | `foxy-v*` | 狗上原生 native on the robot |
| `humble`(未建 not yet) | `m20-humble` | `humble-v*` | 容器 container (`fslam-m20:<arch>`) |
| `jazzy`(未建 not yet) | `m20-jazzy` | `jazzy-v*` | 未开始 not started |

为什么 M20 必须 Foxy:OEM 雷达的 `/LIDAR/POINTS` writer 绑在 127.0.0.1;Humble 的
Fast DDS 2.6 不再为多网卡参与者通告 loopback locator,永远发现不了它;Foxy 的 2.0 可以。
详见 driver 仓库 `m20-foxy` 分支的 DISTRO.md。
Why Foxy: the OEM cloud writer is loopback-only; Fast DDS 2.6 (Humble) never discovers it,
2.0 (Foxy) does. Full story in DISTRO.md on the driver's `m20-foxy` branch.

## 两条部署路径 | Two deployment paths

**A. 狗上编译 | build on the robot**(现行 live 路径,`/home/user/m20_ws`):

```bash
tools/build_m20_foxy.sh --source /home/user/m20_src   # m20_src = driver 分支 m20-foxy
sudo systemctl enable --now m20_loc_foxy
```

**B. 二进制发布包 | binary release tarball**(免编译 no compile;狗无公网,包用 scp 送上去):

driver 仓库每个 `foxy-v*` release 附带的 `fixposition-driver-m20_<ver>_foxy_arm64.tar.gz`
解开后的目录布局(`install/` + `fpsdk/`)与 `run_m20_foxy.sh` 的 WS 约定完全一致,
可直接当工作区用 —— 已在 106 上实测(2026-08-08,含 rsdriver/hsLidar 重启鲁棒性演练):
The tarball attached to every `foxy-v*` driver release unpacks to exactly the WS layout
`run_m20_foxy.sh` expects (`install/` + `fpsdk/`) and works as a drop-in workspace —
validated live on 106 (2026-08-08, incl. restart-robustness drills):

```bash
tar -xzf fixposition-driver-m20_<ver>_foxy_arm64.tar.gz -C /home/user/
WS=/home/user/fixposition-driver-m20 ./run_m20_foxy.sh          # 手动 | by hand
# 或 systemd:drop-in 里加 Environment=WS=... | or a systemd drop-in with Environment=WS=...
```

注意 | notes:
- Foxy 发布包**不含 drdds**(狗上是系统包,再带一份会遮蔽它);容器镜像才自带。
  The foxy tarball ships **no drdds** (system package on the robot); only the docker image vendors it.
- 发布包无源码、无 python typesupport → fixposition 自定义消息不能 `ros2 topic echo`;
  OEM 的 drdds 话题不受影响。
  Binary-only: no python typesupport → no `ros2 topic echo` of fixposition custom msgs;
  OEM drdds topics unaffected.

## 本分支 Foxy 专属的坑 | Foxy-specific traps carried by this branch

- `node.launch` 是 Galactic+ 方言,Foxy `launch_xml` 直接拒载 → `run_m20_foxy.sh` 直接
  `ros2 run` 可执行文件,守护交给 systemd `Restart=`。
- 狗上 Foxy 无 xacro → 用预展开的 `config/fixposition/robot.urdf`。
- Foxy 的 `ros2 topic echo` 没有 `--once`/`--field`(Galactic+ 才有);探针还会阶段性
  "聋" —— 验证以 graph info + `/LOCATION_STATUS` 的 `meta.frame_id` 计数为准。
- distro→driver 分支映射:`lib/deploy_common.sh` 的 `fslam_driver_branch()`。

主干原则不变:能写成可移植的就进 `main`,发行版分支只留真正分叉的东西。
Trunk rule unchanged: portable things go to `main`; distro branches carry only true divergence.
