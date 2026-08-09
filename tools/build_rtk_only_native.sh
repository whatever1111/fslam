#!/usr/bin/env bash
# ============================================================================
# build_rtk_only_native.sh —— 在狗上原生编译 M20 定位驱动(Foxy,无容器)
#                      build the M20 driver natively on the robot (Foxy, no container)
# ============================================================================
# 为什么原生而不是容器:OEM 的点云 writer 绑在 127.0.0.1-only 的 DDS participant 上。
# Foxy 的 FastDDS 2.0 会连同环回 locator 一起通告,宿主机进程直接就能收到 /LIDAR/POINTS;
# Humble 的 2.6 不会,容器里那一路永远收不到(实测 pubs=0)。详见驱动仓库 DISTRO.md。
#
# Why native rather than a container: the OEM lidar publishes /LIDAR/POINTS from a
# participant bound to 127.0.0.1 only. Foxy's Fast DDS 2.0 announces a loopback
# locator alongside real NICs, so a host process receives it directly; Humble's 2.6
# does not, and a container never sees that topic (measured: pubs=0). See DISTRO.md
# in the driver repo.
#
# 狗上没有公网 | the robot has no public internet:
#   先把驱动源码(含 fixposition-sdk 子模块)拷到狗上,再用 --source 指过去。
#   Copy the driver sources (including the fixposition-sdk submodule) over first
#   and point --source at the checkout.
#
# 用法 | usage:
#   tools/build_rtk_only_native.sh --source /home/user/m20_src
#   BUILD_CPUS=2,3 BUILD_JOBS=2 tools/build_rtk_only_native.sh --source /home/user/m20_src
#
# 产物 | result: ${WS}/install(默认 /home/user/m20_ws),run_rtk_only_native.sh 直接用
# ============================================================================
set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=lib/deploy_common.sh
source "${BASE_DIR}/lib/deploy_common.sh"

ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
WS="${WS:-/home/user/m20_ws}"
SDK_PREFIX="${SDK_PREFIX:-${WS}/fpsdk}"
SOURCE_DIR=""
# 狗是 4 核还要跑定位:钉核 + 限并行,别把 sshd 和 OEM 栈饿死(见 MEMORY: 106 负载)。
# The robot has 4 cores and is running localization: pin cores and cap parallelism
# so sshd and the OEM stack keep their share.
BUILD_CPUS="${BUILD_CPUS:-2,3}"
BUILD_JOBS="${BUILD_JOBS:-2}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --source) SOURCE_DIR="$2"; shift 2 ;;
    --ws)     WS="$2"; shift 2 ;;
    -h|--help) sed -n '2,26p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "[ERROR] 未知参数 | unknown argument: $1" >&2; exit 1 ;;
  esac
done

[[ -n "${SOURCE_DIR}" ]] || { echo "[ERROR] 需要 --source <驱动源码目录> | --source <driver checkout> required" >&2; exit 1; }
[[ -d "${SOURCE_DIR}" ]] || { echo "[ERROR] 源码目录不存在 | source not found: ${SOURCE_DIR}" >&2; exit 1; }
[[ -f "${ROS_SETUP}" ]] || { echo "[ERROR] 没找到 ROS2 环境 | ROS 2 not found: ${ROS_SETUP}" >&2; exit 1; }
[[ -d "${SOURCE_DIR}/fixposition-sdk/fpsdk_common" ]] \
  || { echo "[ERROR] fixposition-sdk 子模块没拉全 | SDK submodule not populated: ${SOURCE_DIR}/fixposition-sdk" >&2; exit 1; }

TASKSET=()
if [[ -n "${BUILD_CPUS}" ]] && command -v taskset >/dev/null 2>&1; then
  TASKSET=(taskset -c "${BUILD_CPUS}")
  echo "[INFO] 构建钉在 CPU ${BUILD_CPUS} | build pinned to CPUs ${BUILD_CPUS}"
fi

# ROS 的 setup.bash 会引用未定义变量(AMENT_TRACE_SETUP_FILES 等),在 set -u 下
# 直接把脚本打死,所以 source 前后要临时关掉 -u。
# ROS's setup.bash references unbound variables (AMENT_TRACE_SETUP_FILES and
# friends), which is fatal under set -u — relax it just around the sourcing.
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

# ---- nlohmann/json:狗上没装,SDK 自带一份 | vendored by the SDK, absent on the robot ----
if [[ ! -f /usr/local/include/nlohmann/json.hpp ]]; then
  echo "[INFO] 安装 SDK 自带的 nlohmann/json | installing the SDK's vendored nlohmann/json"
  mkdir -p /usr/local/include/nlohmann /usr/local/lib/cmake/nlohmann_json
  cp "${SOURCE_DIR}/fixposition-sdk/json.hpp" /usr/local/include/nlohmann/json.hpp
  printf '%s\n' \
    'add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED)' \
    'set_target_properties(nlohmann_json::nlohmann_json PROPERTIES' \
    '  INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include")' \
    'set(nlohmann_json_FOUND TRUE)' \
    > /usr/local/lib/cmake/nlohmann_json/nlohmann_jsonConfig.cmake
fi

# ---- Fixposition SDK ---------------------------------------------------------
# 在副本里构建:要删掉 bagwriter.cpp(它 include 的 rosbag2_storage/storage_options.hpp
# 在 Foxy 下叫 rosbag2_cpp/storage_options.hpp)。驱动根本不用 BagWriter,而 SDK 是
# submodule,所以这个删除放在部署仓库而不是驱动仓库。
# Build from a copy: bagwriter.cpp includes rosbag2_storage/storage_options.hpp, which
# Foxy places under rosbag2_cpp/. The driver never uses BagWriter, and the SDK is a
# submodule, so this deletion lives here rather than in the driver repo.
SDK_SRC="${WS}/sdk_src"
echo "[INFO] 准备 SDK 源码 | staging SDK sources -> ${SDK_SRC}"
rm -rf "${SDK_SRC}"
mkdir -p "$(dirname "${SDK_SRC}")"
cp -r "${SOURCE_DIR}/fixposition-sdk" "${SDK_SRC}"
# SDK 的暂存副本就在工作区里,colcon 会把它当成一个包去编(连 fpsdk_apps/fpltool
# 一起,而那个恰好 include 了我们刚删掉的 bagwriter.hpp)。明确让 colcon 别碰。
# The staged SDK copy lives inside the workspace, so colcon would discover it as a
# package and build fpsdk_apps/fpltool too — which includes the bagwriter.hpp we
# just removed. Keep colcon out of it.
touch "${SDK_SRC}/COLCON_IGNORE"
rm -f "${SDK_SRC}/fpsdk_ros2/src/bagwriter.cpp" \
      "${SDK_SRC}/fpsdk_ros2/include/fpsdk_ros2/bagwriter.hpp" \
      "${SDK_SRC}/fpsdk_ros2/include/fpsdk_ros2/ext/rosbag2_cpp_writer.hpp"
sed -i '/add_gtest(TARGET bagwriter_test/d' "${SDK_SRC}/fpsdk_ros2/CMakeLists.txt"

for comp in fpsdk_common fpsdk_ros2; do
  echo "[INFO] 构建 ${comp}..."
  "${TASKSET[@]}" cmake -S "${SDK_SRC}/${comp}" -B "${WS}/build_${comp}" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${SDK_PREFIX}" \
    -DCMAKE_PREFIX_PATH="${SDK_PREFIX}" -DFPSDK_USE_PROJ=OFF -DBUILD_TESTING=OFF > /dev/null
  "${TASKSET[@]}" cmake --build "${WS}/build_${comp}" --parallel "${BUILD_JOBS}" > /dev/null
  "${TASKSET[@]}" cmake --install "${WS}/build_${comp}" > /dev/null
done
test -f "${SDK_PREFIX}/lib/cmake/fpsdk_common/fpsdk_common-config.cmake"
test -f "${SDK_PREFIX}/lib/cmake/fpsdk_ros2/fpsdk_ros2-config.cmake"
echo "[INFO] SDK 就位 | SDK installed at ${SDK_PREFIX}"

# ---- 驱动包 | driver packages ------------------------------------------------
echo "[INFO] 准备驱动源码 | staging driver sources -> ${WS}/src"
rm -rf "${WS}/src" "${WS}/build" "${WS}/install" "${WS}/log"
mkdir -p "${WS}/src"
for pkg in rtcm_msgs fixposition_driver_lib fixposition_driver_msgs fixposition_driver_m20 fixposition_driver_ros2; do
  [[ -d "${SOURCE_DIR}/${pkg}" ]] || { echo "[ERROR] 缺包 | missing package: ${pkg}" >&2; exit 1; }
  cp -r "${SOURCE_DIR}/${pkg}" "${WS}/src/${pkg}"
done
# drdds 用狗上系统自带的那份(OEM 的接口包),不要编仓库里 vendored 的副本。
# Use the robot's system drdds (the OEM's own interface package), not the vendored copy.
if [[ ! -d /opt/ros/foxy/share/drdds ]]; then
  echo "[ERROR] 系统里没有 drdds —— 这台机器不是狗?| no system drdds; is this the robot?" >&2
  exit 1
fi

cd "${WS}"
# --base-paths src:只扫 src/,别把 sdk_src / build_* 也当成包
# --base-paths src: scan only src/, not sdk_src or the build_* trees
"${TASKSET[@]}" colcon build \
  --base-paths "${WS}/src" \
  --parallel-workers "${BUILD_JOBS}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="${SDK_PREFIX}" -DBUILD_TESTING=OFF

EXE="${WS}/install/fixposition_driver_ros2/lib/fixposition_driver_ros2/fixposition_driver_ros2_exec"
[[ -x "${EXE}" ]] || { echo "[ERROR] 没产出可执行文件 | no executable produced" >&2; exit 1; }

# 自检:确实链到了 Foxy,而且 M20 模块在里面。
# Smoke checks: really linked against Foxy, and the M20 module is in there.
set +u
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u
export LD_LIBRARY_PATH="${SDK_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
ldd "${EXE}" | grep -q "/opt/ros/foxy/" || { echo "[ERROR] 没链到 Foxy | not linked against Foxy" >&2; exit 1; }
if ldd "${EXE}" | grep -q "not found"; then
  echo "[ERROR] 有未解析的动态库 | unresolved shared libraries:" >&2
  ldd "${EXE}" | grep "not found" >&2
  exit 1
fi
ldd "${EXE}" | grep -q "libfixposition_driver_m20.so" || { echo "[ERROR] 镜像里没有 M20 模块 | M20 module not linked" >&2; exit 1; }

echo "[INFO] 完成 | done: ${EXE}"
echo "[INFO] 部署 | deploy: ${BASE_DIR}/run_rtk_only_native.sh"
