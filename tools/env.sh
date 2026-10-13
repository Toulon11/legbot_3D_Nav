#!/usr/bin/env bash
# Source this file from the workspace or any other directory.
legbot_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH=/usr/bin:/bin
source /opt/ros/jazzy/setup.bash
if [[ -f "${legbot_workspace}/install/setup.bash" ]]; then
  source "${legbot_workspace}/install/setup.bash"
else
  source "${legbot_workspace}/install/local_setup.sh"
fi
export PATH="${legbot_workspace}/.venv-jazzy/bin:${PATH}"
export LEGBOT_DATA_DIR="${legbot_workspace}/data"
export ONNXRUNTIME_ROOT="${legbot_workspace}/third_party/onnxruntime"
# Use the project-local compatibility library when the system package is older.
diagnostic_overlay="${legbot_workspace}/.deps/diagnostic_updater_4.2.7/opt/ros/jazzy/lib"
if [[ -f "${diagnostic_overlay}/libdiagnostic_updater.so" ]]; then
  export LD_LIBRARY_PATH="${diagnostic_overlay}:${LD_LIBRARY_PATH:-}"
else
  diagnostic_version="$(dpkg-query -W -f='${Version}' ros-jazzy-diagnostic-updater 2>/dev/null || true)"
  if [[ -z "${diagnostic_version}" ]] || ! dpkg --compare-versions "${diagnostic_version}" ge 4.2.7; then
    echo "diagnostic_updater 4.2.7 or newer is required; install it or provide ${diagnostic_overlay}" >&2
    return 1 2>/dev/null || exit 1
  fi
fi
export LD_LIBRARY_PATH="${legbot_workspace}/third_party/onnxruntime/lib:${legbot_workspace}/third_party/libtorch/lib:${legbot_workspace}/third_party/pct/install/lib:${legbot_workspace}/third_party/Livox-SDK2/install/lib:${legbot_workspace}/third_party/unitree_sdk2/install/lib:${LD_LIBRARY_PATH:-}"
# SDK2 talks to the robot directly; keep ROS processes on the Fast DDS RMW.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset diagnostic_overlay diagnostic_version legbot_workspace
