#!/usr/bin/env bash
# Read-only GO2 EDU Ethernet and ros2_control state check. Never publishes LowCmd.
set -Eeo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
network_interface="${1:-enp131s0}"
robot_ip="${GO2_IP:-192.168.123.161}"
output_dir="${2:-${workspace_root}/data/real_preflight/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${output_dir}/ros_logs"

if [[ ! -d "/sys/class/net/${network_interface}" || "${network_interface}" == lo ]]; then
  echo "Invalid Ethernet interface: ${network_interface}" >&2
  exit 2
fi
if [[ "$(cat "/sys/class/net/${network_interface}/operstate")" != up ]]; then
  echo "Ethernet interface ${network_interface} is not UP." >&2
  exit 2
fi
if ! ip -4 -brief address show dev "${network_interface}" | grep -q '192\.168\.123\.'; then
  echo "${network_interface} has no 192.168.123.x IPv4 address." >&2
  exit 2
fi
if ! ping -I "${network_interface}" -c 3 -W 1 "${robot_ip}" >"${output_dir}/ping.txt"; then
  echo "GO2 ${robot_ip} is not reachable through ${network_interface}." >&2
  exit 2
fi

source "${workspace_root}/tools/env.sh"
export ROS_LOG_DIR="${output_dir}/ros_logs"

launch_pid=""
lidar_pid=""
cleanup() {
  for pid in "${lidar_pid}" "${launch_pid}"; do
    [[ -n "${pid}" ]] || continue
    if kill -0 "${pid}" 2>/dev/null; then
      kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${lidar_pid}" "${launch_pid}"; do
    [[ -n "${pid}" ]] || continue
    for _ in {1..20}; do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.25
    done
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM -- "-${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

setsid nice -n 10 taskset -c 0-2 stdbuf -oL -eL ros2 launch legbot_bringup real.launch.py \
  network_interface:="${network_interface}" enable_commands:=false policy_profile:=moe_cts_77k \
  >"${output_dir}/real_readonly.log" 2>&1 &
launch_pid=$!

setsid nice -n 10 taskset -c 3-4 stdbuf -oL -eL ros2 launch legbot_bringup unitree_lidar.launch.py \
  network_interface:="${network_interface}" \
  >"${output_dir}/unitree_lidar.log" 2>&1 &
lidar_pid=$!

wait_for_typed_topic() {
  local topic="$1"
  local timeout_seconds="$2"
  local log_file="$3"
  local deadline=$((SECONDS + timeout_seconds))
  local topic_type=""
  while (( SECONDS < deadline )); do
    topic_type="$(timeout 3 ros2 topic type "${topic}" 2>/dev/null || true)"
    if [[ -n "${topic_type}" ]]; then
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for typed topic ${topic}; see ${log_file}" >&2
  exit 3
}

wait_for_typed_topic /joint_states 35 "${output_dir}/real_readonly.log"
wait_for_typed_topic /imu_sensor_broadcaster/imu 35 "${output_dir}/real_readonly.log"
wait_for_typed_topic /unitree/lidar 20 "${output_dir}/unitree_lidar.log"
wait_for_typed_topic /unitree/lidar_imu 20 "${output_dir}/unitree_lidar.log"

capture_topic_once() {
  local topic="$1"
  local output_file="$2"
  shift 2
  if ! timeout 10 ros2 topic echo "${topic}" --once --no-lost-messages "$@" >"${output_file}"; then
    echo "Timed out waiting for a message on ${topic}; see ${output_dir}/real_readonly.log and ${output_dir}/unitree_lidar.log" >&2
    exit 3
  fi
  if [[ ! -s "${output_file}" ]]; then
    echo "Received no usable data from ${topic}; evidence file is empty: ${output_file}" >&2
    exit 3
  fi
}

capture_topic_once /joint_states "${output_dir}/joint_states.yaml"
capture_topic_once /imu_sensor_broadcaster/imu "${output_dir}/imu.yaml"
capture_topic_once /unitree/lidar_imu "${output_dir}/lidar_imu.yaml"
capture_topic_once /unitree/lidar "${output_dir}/lidar_header.yaml" --field header
capture_topic_once /unitree/lidar "${output_dir}/lidar_fields.yaml" --field fields
python3 - "${output_dir}/joint_states.yaml" "${output_dir}/imu.yaml" \
  "${output_dir}/lidar_imu.yaml" "${output_dir}/lidar_fields.yaml" <<'PY'
import math, re, sys, yaml
joint = next(yaml.safe_load_all(open(sys.argv[1], encoding='utf-8')))
imu = next(yaml.safe_load_all(open(sys.argv[2], encoding='utf-8')))
lidar_imu = next(yaml.safe_load_all(open(sys.argv[3], encoding='utf-8')))
names, positions = joint.get('name', []), joint.get('position', [])
expected = {f'{leg}_{joint}_joint' for leg in ('FR','FL','RR','RL')
            for joint in ('hip','thigh','calf')}
if set(names) != expected or len(positions) != 12 or not all(math.isfinite(x) for x in positions):
    raise SystemExit('Invalid GO2 joint state packet')
if max(map(abs, positions), default=0.0) < 0.05:
    raise SystemExit('Joint states are all near zero; LowState reception is not proven')
q = imu['orientation']
norm = math.sqrt(sum(float(q[k]) ** 2 for k in ('w','x','y','z')))
if not 0.8 <= norm <= 1.2:
    raise SystemExit(f'Invalid GO2 IMU quaternion norm: {norm}')
lq = lidar_imu['orientation']
lq_norm = math.sqrt(sum(float(lq[k]) ** 2 for k in ('w','x','y','z')))
la = lidar_imu['linear_acceleration']
lg = lidar_imu['angular_velocity']
acc_norm = math.sqrt(sum(float(la[k]) ** 2 for k in ('x','y','z')))
gyro = [float(lg[k]) for k in ('x','y','z')]
if not 0.8 <= lq_norm <= 1.2 or not 5.0 <= acc_norm <= 15.0 or not all(map(math.isfinite, gyro)):
    raise SystemExit(
        'Invalid L1 internal IMU; use fastlio_unitree_l1_body_imu.yaml only after checking time sync '
        f'(quaternion_norm={lq_norm:.4f}, acceleration_norm={acc_norm:.4f})')
field_text = open(sys.argv[4], encoding='utf-8').read()
# Jazzy ros2cli currently renders ``--field fields`` as either YAML mappings
# or Python PointField reprs depending on the installed rosidl runtime.
field_names = set(re.findall(
    r"\bname\s*(?:=|:)\s*['\"]?([A-Za-z_][A-Za-z0-9_]*)", field_text))
required = {'x', 'y', 'z', 'intensity', 'time', 'ring'}
if not required.issubset(field_names):
    raise SystemExit(f'Unitree L1 cloud is missing FAST-LIO fields: {sorted(required - field_names)}')
print('GO2_READONLY_PREFLIGHT=PASS '
      f'joints=12 body_imu_qnorm={norm:.4f} l1_imu_qnorm={lq_norm:.4f} '
      f'l1_acc_norm={acc_norm:.4f} lidar_fields=PASS')
PY

echo "No motor command was enabled or published. Evidence: ${output_dir}"
