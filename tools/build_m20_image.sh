#!/usr/bin/env bash
# ============================================================================
# build_m20_image.sh —— 构建 M20 定位驱动镜像 | build the M20 driver image
# ============================================================================
# 在能上网的机器上(s100 / 开发机)可直接跑,会自己 clone driver 源码;
# 狗上没有公网,先把源码用 git bundle 传过去,再用 --source 指过去。
#
# On a machine with internet this clones the driver itself. The robot has no
# public network, so ship the sources over as a git bundle first and point
# --source at the checkout.
#
# 用法 | usage:
#   tools/build_m20_image.sh                      # clone/update, then build
#   tools/build_m20_image.sh --source /path/to/fixposition_driver
#   BASE_IMAGE=wanderer123/fslam-humble:arm64 tools/build_m20_image.sh
#   BUILD_JOBS=0 tools/build_m20_image.sh          # 专用构建机:放开并行度
#   BUILD_CPUS=3 BUILD_JOBS=1 tools/build_m20_image.sh   # 狗上:钉单核,保住 ssh
#
# 产物 | result: 本地镜像 fslam-m20:<arch>  (不推 registry | not pushed anywhere)
# ============================================================================
set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=lib/deploy_common.sh
source "${BASE_DIR}/lib/deploy_common.sh"

DRIVER_REPO="${DRIVER_REPO:-https://github.com/whatever1111/fixposition_driver.git}"
DRIVER_BRANCH="${DRIVER_BRANCH:-m20}"
BASE_IMAGE="${BASE_IMAGE:-wanderer123/fslam-humble:$(fslam_arch_tag)}"
# 狗上是边跑定位边编译 —— 并行度留余量,别把 sshd 和定位饿死(见 Dockerfile.m20)。
# On the robot this compiles alongside live localization: keep parallelism low
# so sshd and the localization keep their share (see Dockerfile.m20).
BUILD_JOBS="${BUILD_JOBS:-2}"
# 把构建钉在指定核上,别和定位/sshd 抢 CPU。狗上必设(如 BUILD_CPUS=3):106 空载
# 就已经 load≈3.3/4 核,一旦 ssh 卡死,现场会以为机器死了直接断电重启,构建也就没了。
# --cpuset-cpus 只有传统构建器认,所以设了就关掉 BuildKit。
# Pin the build to specific cores so it does not fight the localization or sshd.
# Required on the robot (e.g. BUILD_CPUS=3): 106 idles at load ~3.3 on 4 cores,
# and once ssh stops answering the box looks dead and gets power-cycled, taking
# the build with it. --cpuset-cpus is honoured only by the legacy builder, so
# setting it turns BuildKit off.
BUILD_CPUS="${BUILD_CPUS:-}"
IMAGE_TAG="${IMAGE_TAG:-fslam-m20:$(fslam_arch_tag)}"
SOURCE_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --source) SOURCE_DIR="$2"; shift 2 ;;
    --tag)    IMAGE_TAG="$2"; shift 2 ;;
    -h|--help) sed -n '2,22p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "[ERROR] 未知参数 | unknown argument: $1" >&2; exit 1 ;;
  esac
done

command -v docker >/dev/null 2>&1 || { echo "[ERROR] docker 不可用 | docker not available" >&2; exit 1; }
docker image inspect "${BASE_IMAGE}" >/dev/null 2>&1 \
  || { echo "[ERROR] 基础镜像不存在 | base image not found: ${BASE_IMAGE}" >&2; exit 1; }

# ---- 取驱动源码 | obtain the driver sources ---------------------------------
WORK_DIR="$(mktemp -d)"
trap 'rm -rf "${WORK_DIR}"' EXIT

if [[ -n "${SOURCE_DIR}" ]]; then
  [[ -d "${SOURCE_DIR}" ]] || { echo "[ERROR] 源码目录不存在 | no such source dir: ${SOURCE_DIR}" >&2; exit 1; }
  echo "[INFO] 使用本地源码 | using local sources: ${SOURCE_DIR}"
  cp -r "${SOURCE_DIR}" "${WORK_DIR}/fixposition_driver"
else
  echo "[INFO] clone ${DRIVER_REPO} (${DRIVER_BRANCH})..."
  git clone --depth 1 --branch "${DRIVER_BRANCH}" "${DRIVER_REPO}" "${WORK_DIR}/fixposition_driver"
fi

# 记录构建出的 driver 版本,方便日后对账 | record what went in
DRIVER_REF="$(git -C "${WORK_DIR}/fixposition_driver" rev-parse --short HEAD 2>/dev/null || echo unknown)"
for pkg in drdds fixposition_driver_m20 fixposition_driver_ros2; do
  [[ -d "${WORK_DIR}/fixposition_driver/${pkg}" ]] \
    || { echo "[ERROR] 源码里没有 ${pkg} —— 分支对吗? | ${pkg} missing, wrong branch?" >&2; exit 1; }
done

cp "${BASE_DIR}/container/Dockerfile.m20" "${WORK_DIR}/Dockerfile"

# ---- 构建 | build -----------------------------------------------------------
echo "[INFO] 构建 ${IMAGE_TAG}(base ${BASE_IMAGE}, driver ${DRIVER_REF}, jobs ${BUILD_JOBS})..."
DOCKER_BUILD_ARGS=()
if [[ -n "${BUILD_CPUS}" ]]; then
  export DOCKER_BUILDKIT=0
  DOCKER_BUILD_ARGS+=(--cpuset-cpus "${BUILD_CPUS}")
  echo "[INFO] 限核 | pinned to CPUs ${BUILD_CPUS} (BuildKit off so --cpuset-cpus applies)"
fi
docker build "${DOCKER_BUILD_ARGS[@]}" \
  --build-arg "BASE_IMAGE=${BASE_IMAGE}" \
  --build-arg "BUILD_JOBS=${BUILD_JOBS}" \
  --label "fslam.m20.driver_ref=${DRIVER_REF}" \
  -t "${IMAGE_TAG}" \
  "${WORK_DIR}"

echo "[INFO] 完成 | done: ${IMAGE_TAG}  (driver ${DRIVER_REF})"
echo "[INFO] 部署 | deploy: DOCKER_IMAGE=${IMAGE_TAG} ./run_m20_prod.sh"
