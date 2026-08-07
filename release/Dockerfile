# ============================================================================
# fslam-m20 发布镜像:driver 发布镜像 + 烘焙好的 M20 产品配置。
# fslam-m20 release image: the driver's release image with the M20 product
# config baked in.
#
# BASE_IMAGE 由 release workflow 从 driver 仓库同发行版 release 的
# docker-image 资产 `docker load` 得来 —— 不经 GHCR,狗和 CI 都用公开的
# release 资产,权限简单。基础镜像的 ENTRYPOINT 会 source 驱动环境,CMD 已经
# 指向 /config/fixposition.yaml,所以这里只需要放配置,一行不多。
# BASE_IMAGE is `docker load`-ed by the release workflow from the driver
# repo's same-distro release asset (no GHCR dependency — public release assets
# everywhere). The base ENTRYPOINT sources the driver env and its CMD already
# points at /config/fixposition.yaml, so baking the config is all there is.
# ============================================================================
ARG BASE_IMAGE
FROM ${BASE_IMAGE}
LABEL org.opencontainers.image.source="https://github.com/whatever1111/fslam"
LABEL org.opencontainers.image.description="M20 localization (fslam) — driver release + baked config"

COPY config/fixposition/config_m20.yaml /config/fixposition.yaml
