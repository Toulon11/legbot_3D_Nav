#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
export PATH=/opt/ros/jazzy/bin:/usr/bin:/bin
export CC=/usr/bin/gcc CXX=/usr/bin/g++
export ONNXRUNTIME_ROOT="${PWD}/third_party/onnxruntime"
export MAKEFLAGS=-j1
export CMAKE_BUILD_PARALLEL_LEVEL=1
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH="${PWD}/third_party/libtorch:${PWD}/third_party/unitree_sdk2/install:${PWD}/third_party/Livox-SDK2/install:${PWD}/third_party/pct/install:/opt/ros/jazzy"
# rosidl may leave a real generated Python directory where a later
# ament_cmake_python symlink must go. Only remove this build artifact.
scan_python_link="${PWD}/build/scan_planner/ament_cmake_python/scan_planner/scan_planner"
if [[ -d "$scan_python_link" && ! -L "$scan_python_link" ]]; then
  rm -rf -- "$scan_python_link"
fi
colcon build --base-paths src --executor sequential "$@" --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON \
  -DBUILD_SOPHUS_TESTS=OFF \
  -DBUILD_SOPHUS_EXAMPLES=OFF
