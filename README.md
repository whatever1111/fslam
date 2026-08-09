# fslam — 机器人定位部署项目 | Robot localization deployment project

一个仓库承载**多个机器人目标**的定位部署。`main` 只放所有目标共享的通用机制;每个部署目标
一条**产品主干分支**(载荷:启动脚本、配置、systemd 单元、文档),其下按 ROS 发行版分**叶子
分支**(只放钉版文件)。发布 tag 打在叶子上。

One repo hosting localization deployment for **multiple robot targets**. `main` carries only the
machinery every target shares; each deployment target gets a **product trunk branch** (payload:
launchers, config, systemd units, docs) with **distro leaf branches** under it (pin files only).
Release tags go on leaves.

## 分支模型 | Branch model

```
main                                通用机制 | generic machinery (lib/, release workflow, this README)
└── <vendor>-<robot>                产品主干:该机器人的全部部署载荷 | product trunk: the payload
    └── <vendor>-<robot>-<distro>   发行版叶子:钉版文件 + DISTRO.md | distro leaf: pins only
```

| 分支 branch | 内容 contents | 发布 tag |
|---|---|---|
| `main` | `lib/deploy_common.sh`、release 流水线模板、本 README | — |
| `deep-robotics-m20` | 云深处 M20:双模式载荷(rtk + fslam)| — |
| `deep-robotics-m20-foxy` | 上者 + `DRIVER_RELEASE`/`SLAM_IMAGE` 钉版 + `DISTRO.md` | `deep-robotics-m20-foxy-v*` |

当前目标 | current targets:**deep-robotics-m20**(云深处 M20 机器狗,rtk 模式线上运行)。
文档入口在各产品主干的 `README.md` / `DEPLOYMENT.md`。
Docs live on each product trunk (`README.md` / `DEPLOYMENT.md`).

## 合并规则 | Merge rules

- `main` → 产品主干 → 发行版叶子,单向流;**产品分支之间永不互并**。
  One direction only: `main` → trunk → leaf; **product branches never merge into each other**.
- 通用改动(lib、流水线)改在 `main`,往下并;产品改动改在主干,往叶子并;钉版只改叶子。
  Generic changes land on `main` and flow down; product changes on the trunk; pins on the leaf.
- `main` 上**禁止出现产品专属内容** —— 否则每个新目标都会继承它。
  Nothing product-specific may live on `main` — every new target would inherit it.

## 新增部署目标 | Adding a deployment target

```bash
git checkout -b <vendor>-<robot> main          # 1. 产品主干 | product trunk
#    加载荷:启动脚本、config/、systemd/、DEPLOYMENT.md,
#    并按需在 .github/workflows/release.yml 里加该目标的打包步骤
git checkout -b <vendor>-<robot>-<distro>      # 2. 发行版叶子 | distro leaf
#    加钉版文件(如 DRIVER_RELEASE / SLAM_IMAGE)+ DISTRO.md
git tag <vendor>-<robot>-<distro>-v0.1.0       # 3. 发布 | release
```

tag 格式 `<branch>-v<X.Y.Z>`,流水线从 tag 反解 target/distro/version;运行的是**tag 所在
commit 上那份 workflow**,所以每个产品分支可以有自己的打包逻辑。
Tags are `<branch>-v<X.Y.Z>`; the pipeline parses target/distro/version from the tag and runs
**the workflow at the tagged commit**, so each product branch may carry its own packaging logic.

## 历史 | History

2026-08-08 之前本仓库是 M20 单目标结构(`main` 带 M20 载荷,分支 `foxy`,tag `foxy-v*`)。
旧 release(`foxy-v1.0.0`…`foxy-v1.1.0`)保留有效;新 tag 一律用新格式。
Before 2026-08-08 this repo was single-target (M20 payload on `main`, branch `foxy`, tags
`foxy-v*`). Old releases remain valid; new tags use the new format.
