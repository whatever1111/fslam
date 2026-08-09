# 从 LIO-SLAM 迁来的部署遗产 | Deployment legacy moved out of LIO-SLAM

2026-08-08 从 LIO-SLAM 仓库(da5247c)整体迁入:LIO-SLAM 是跨产品共享的算法仓库,
部署层内容不属于它。这里只做历史留存,**都已被取代,别再用**:

Moved wholesale out of the LIO-SLAM repo (da5247c) on 2026-08-08: LIO-SLAM is the
product-agnostic algorithm repo and deployment content does not belong there.
Kept for history only — **everything here is superseded, do not use**:

| 文件 file | 被什么取代 superseded by |
|---|---|
| `run_fast_lio_pgo_prod.sh` | 仓库根的同名脚本(现行 fslam 模式启动器)the repo-root launcher of the same name |
| `fixposition_odom_to_ODOM.py` | `host/fp_to_odom.py` |
| `deployment_guide.md` / `deployment_guide_native.md` | `DEPLOYMENT.md`(May 2026 一代,mower/giant 时期)May-2026-generation docs |
| `deploy/`(compose) | 启动脚本 + systemd 单元 the launchers + systemd units |
