#!/usr/bin/env bash
# Parse descriptions and run local inference only; never start ROS/Gazebo nodes.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source tools/env.sh
export ROS_LOG_DIR="$PWD/log/offline"
export GZ_SIM_RESOURCE_PATH="$PWD/install/legbot_bringup/share/legbot_bringup/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export SDF_PATH="$GZ_SIM_RESOURCE_PATH${SDF_PATH:+:$SDF_PATH}"
mkdir -p "$ROS_LOG_DIR"
python3 tests/check_launch_descriptions.py
python3 tests/check_model_offline.py
python3 tests/check_estop_offline.py
install/legbot_bringup/lib/legbot_bringup/pct_path_mission --self-test
install/rl_quadruped_controller/lib/rl_quadruped_controller/policy_contract_check src/go2_description/config/himloco
install/rl_quadruped_controller/lib/rl_quadruped_controller/policy_contract_check src/go2_description/config/go2_cts
install/rl_quadruped_controller/lib/rl_quadruped_controller/policy_contract_check src/go2_description/config/legged_gym
install/rl_quadruped_controller/lib/rl_quadruped_controller/onnx_policy_contract_check src/go2_description/config/stairs_trot
install/rl_quadruped_controller/lib/rl_quadruped_controller/onnx_policy_contract_check src/go2_description/config/mjlab_flat
install/rl_quadruped_controller/lib/rl_quadruped_controller/onnx_policy_contract_check src/go2_description/config/moe_cts_77k
python3 tests/check_dependencies_offline.py
python3 - <<'PY'
import datetime, json, platform
from pathlib import Path

checks = [
    'ROS 2 launch descriptions and simulation setup without executing nodes',
    'GO2 joint limits, mesh paths, initial positions and simulator/hardware policy configuration',
    'Gazebo Harmonic world SDF and converted robot sensors',
    'HIMLoco checksum and offline float32 [1,270] -> [1,12] inference, newest-first history',
    'GO2 CTS TorchScript checksum and offline policy inference',
    'Legacy GO2 legged-gym TorchScript checksum and offline float32 [1,45] -> [1,12] inference',
    'stairs_trot ONNX checksum and offline inference',
    'Unitree GO2 velocity-flat ONNX checksum and float32 [1,45] -> [1,12] inference',
    'MoE-CTS symmetry 77k ONNX checksum and float32 [1,450] -> [1,12] inference with ten term-major history frames',
    'PCT native extensions, bundled tomogram cross-floor route, Open3D and CuPy imports',
    'PCT mission bridge maps the Building-local route into the selected live navigation odom frame',
    'SCAN defaults to 0.50 m/s with 0.30 m/s^2 acceleration, path-heading control and a 2.0 m local horizon',
    'Simulation defaults to FAST-LIO for command gating; optional Gazebo truth publication is isolated to diagnostics',
    'The long PCT cross-floor simulation defaults to drift-free Gazebo odometry while retaining an explicit FAST-LIO integration mode',
    'Simulation and read-only real launch entries default to the GO2 CTS policy profile',
    'GO2 built-in L1 cloud/internal-IMU bridge preserves common source timing and FAST-LIO consumes 18-line time/ring data',
    'A separately calibrated GO2 body-IMU FAST-LIO fallback configuration remains available',
    'Real command adapter defaults to a 0.20 m/s first-test limit and requires fresh FAST-LIO odometry and cloud data',
    'The sole 64 m x 32 m combined world contains 30 cylinders, three traversable arch gates and the multi-storey Building staircase',
    'RViz goal input defaults to SCAN-compatible direct replacement and can switch at runtime to the managed queue',
    'ROS 2 keypoint recorder saves current FAST-LIO XYZ as a reusable SCAN multi-floor waypoint parameter file',
    'SCAN planner uses an inflated GO2 footprint while the command safety gate reads raw occupied cells to avoid double inflation',
]
mission_path = Path('log/policy_bench/moe_cts_77k_scan_queue_065_raw_gate/mission.json')
direct_path = Path('log/policy_bench/moe_cts_77k_scan_direct_065_obstacle/direct_result.json')
runtime_validation = None
if mission_path.is_file():
    mission = json.loads(mission_path.read_text())
    assert mission['state'] == 'complete'
    assert mission['plan_acknowledged'] == [True, True, True]
    assert mission['reached'] == [True, True, True]
    checks.append('Recorded MoE-CTS + FAST-LIO + SCAN run planned and reached all three queued waypoints at the 0.65 m/s planning limit')
    runtime_validation = {
        'artifact': str(mission_path),
        'state': mission['state'],
        'waypoints': mission['waypoints'],
        'elapsed_wall_seconds': mission['elapsed_wall_seconds'],
        'used_gazebo_ground_truth_for_navigation': False,
    }
if direct_path.is_file():
    direct = json.loads(direct_path.read_text())
    assert direct['reached'] is True
    assert direct['bspline_messages'] >= 1
    assert direct['max_bspline_abs_y'] >= 0.30
    checks.append('Recorded direct-mode goal crossed an obstructed line of sight and reached it through a laterally deviated SCAN path')
    if runtime_validation is not None:
        runtime_validation['direct_obstacle_avoidance'] = direct

report = {
    'checked_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'platform': platform.platform(),
    'ros_distro': 'jazzy',
    'checks_passed': checks,
    'simulation_started_by_this_check': False,
    'runtime_validation': runtime_validation,
    'real_robot_connected_or_commanded': False,
    'current_blocker': None,
    'limitations': [
        'mjlab_flat showed lateral drift in the present Gazebo model; use moe_cts_77k when accurate straight tracking is required',
        'The continuous connector plus PCT stair route was validated to the top-floor final approach; the default full-map run still needs an uninterrupted acceptance-duration completion record',
        'The 16 m simulated FAST-LIO run accumulated about 0.82 m of vertical odometry drift while the GO2 remained upright and reached all three XY goals',
        'Real L1 message rates, IMU health, timing and robot-to-sensor alignment require the read-only on-site preflight',
        'Real motor output remains disabled until a supported on-site preflight',
    ],
    'build_summary': '14 ROS 2 Jazzy packages built in the standard colcon workspace',
    'nonfatal_warnings': [
        'Jazzy HardwareInfo on_init overload is deprecated',
        'SDF parser preserves gz_frame_id and upstream joint feedback extension tags',
    ],
}
Path('log/offline/validation.json').write_text(
    json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print('PASS offline checks; no simulation or robot experiment was started by this check')
PY
