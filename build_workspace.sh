#!/usr/bin/env bash
set -eo pipefail
WS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ "${GO2_EXPLORE_CLEAN_BUILD:-}" != 1 ]]; then
  exec env -u CMAKE_PREFIX_PATH -u ROS_PACKAGE_PATH -u PYTHONPATH -u LD_LIBRARY_PATH \
    -u ROS_ROOT -u ROS_ETC_DIR -u ROS_DISTRO -u ROS_VERSION \
    GO2_EXPLORE_CLEAN_BUILD=1 /bin/bash --noprofile --norc "$0" "$@"
fi
source /opt/ros/noetic/setup.bash
set -u
export ROS_HOME="${WS_ROOT}/.ros"
export ROS_LOG_DIR="${WS_ROOT}/logs/build_ros"
mkdir -p "${WS_ROOT}/artifacts" "${ROS_HOME}" "${ROS_LOG_DIR}"
test "$(df -Pk "${WS_ROOT}" | awk 'NR==2 {print $4}')" -gt 5242880
cmake -S "${WS_ROOT}/src/third_party/Livox-SDK2" -B "${WS_ROOT}/build_livox_sdk" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${WS_ROOT}/vendor"
cmake --build "${WS_ROOT}/build_livox_sdk" --target install --parallel "${BUILD_JOBS:-2}"
cd "${WS_ROOT}"
catkin_make -j"${BUILD_JOBS:-2}" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="-I${WS_ROOT}/vendor/include" \
  -DLIVOX_LIDAR_SDK_LIBRARY="${WS_ROOT}/vendor/lib/liblivox_lidar_sdk_static.a" \
  -DUNITREE_SDK_ROOT="${WS_ROOT}/src/third_party/Unitree_SDK2" \
  -DCATKIN_TEST_RESULTS_DIR="${WS_ROOT}/artifacts/test_results"
