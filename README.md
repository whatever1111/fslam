# fslam — 机器人定位部署项目 | Robot localization deployment project

一个仓库承载**多个机器人目标**的定位部署。全局只有两个模式名:**`rtk_only`** 与 **`fslam`**;
每个模式给出 **native** 和 **docker** 两种部署形态(能给的都给)。`main` 只放所有目标共享的
通用机制;每个部署目标一条**产品主干分支**,其下按 ROS 发行版分**叶子分支**。

One repo hosting localization deployment for **multiple robot targets**. There are exactly two
mode names, globally: **`rtk_only`** and **`fslam`**; each mode ships in **native** and **docker**
form wherever possible. `main` carries only machinery shared by every target; each target gets a
**product trunk branch** with **distro leaf branches** under it.

## 模式 × 形态 | Mode × form

| 模式 mode | 定位来源 what localizes | native | docker |
|---|---|---|---|
| **`rtk_only`** | RTK/INS(fixposition 驱动直接交付输出契约 the driver delivers the output contract itself)| ✅ 驱动二进制 release 直跑 driver binary release | ✅ 驱动 release 镜像 + 烘焙配置 driver release image + baked config |
| **`fslam`** | SLAM(LIO-SLAM:FAST-LIO + PGO;容器内用上游原版 fixposition 驱动)| —(SLAM 按设计容器化 containerized by design)| ✅ LIO-SLAM 镜像 + 宿主机胶水 image + host glue |

同一目标上两模式互斥(都发同一里程话题),靠 systemd 单元切换 —— 模式是运行时维度,不是分支。
On one target the modes are mutually exclusive (same odometry topic) and switch by systemd unit —
mode is a runtime axis, never a branch.

## 命名规则 | Naming rules

全局统一用这两个 token,禁止别名(不再有 "m20 模式"、"fp-only"、"rtk" 等旧叫法):
The two tokens are used verbatim everywhere; no aliases (no more "m20 mode", "fp-only", "rtk"):

| 东西 thing | 规则 rule | 例 example |
|---|---|---|
| 启动脚本 launchers | `run_<mode>[_<form>].sh` | `run_rtk_only_native.sh` · `run_fslam.sh` |
| systemd 单元 units | `<mode>.service` | `rtk_only.service` · `fslam.service` |
| 发布资产 assets | `<mode>_<ver>_[<distro>_]arm64[...]` | `rtk_only_1.2.0_foxy_arm64.tar.gz` · `fslam_1.2.0_arm64.tar.gz` |
| 镜像 images | `<mode>` | `ghcr.io/whatever1111/rtk_only:foxy-<ver>` |
| 钉版文件 pins | 模式各一 one per mode | `DRIVER_RELEASE`(rtk_only)· `SLAM_IMAGE`(fslam)|

## 分支模型 | Branch model

```
main                                通用机制 | generic machinery (lib/, workflow template, this README)
└── <vendor>-<robot>                产品主干:载荷 | product trunk: launchers/config/units/docs
    └── <vendor>-<robot>-<distro>   发行版叶子:钉版 + DISTRO.md | distro leaf: pins only
```

| 分支 branch | 发布 tag |
|---|---|
| `main` | — |
| `deep-robotics-m20`(云深处 M20,rtk_only native 线上 live)| — |
| `deep-robotics-m20-foxy` | `deep-robotics-m20-foxy-v*` |

合并单向:`main` → 主干 → 叶子;**产品分支之间永不互并**;`main` 上禁止产品专属内容。
Merges flow one way: `main` → trunk → leaf; **product branches never merge into each other**;
nothing product-specific on `main`.

## 新增部署目标 | Adding a target

```bash
git checkout -b <vendor>-<robot> main      # 1. 主干:加载荷 + 该目标的 release 打包步骤
git checkout -b <vendor>-<robot>-<distro>  # 2. 叶子:加钉版文件 + DISTRO.md
git tag <vendor>-<robot>-<distro>-v0.1.0   # 3. 发布(流水线跑 tag 所在 commit 的 workflow)
```

## 历史 | History

2026-08-08 之前:单目标结构(M20 载荷在 `main`,分支 `foxy`,tag `foxy-v*`,资产名
`fslam-m20_*`/`fslam-rtk_*`)。旧 release 保留有效;新 tag/资产一律用新格式。
Pre-2026-08-08 this repo was single-target (payload on `main`, branch `foxy`, `foxy-v*` tags,
`fslam-m20_*`/`fslam-rtk_*` assets). Old releases stay valid; everything new uses the new names.
