#!/usr/bin/env bash
set -eu
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)

exec python3 - "$workspace" "$@" <<'PY'
import argparse
import os
from pathlib import Path
import re
import signal
import sys
import time
from collections import namedtuple

workspace = Path(sys.argv[1])
parser = argparse.ArgumentParser(prog="cleanup_cross_layer.sh",
                                 description="停止 cross_layer_test 仿真及可识别的残留进程")
parser.add_argument("--dry-run", action="store_true", help="只显示将被终止的进程")
parser.add_argument("--all-gazebo", action="store_true",
                    help="也清理无法识别归属的 Gazebo server/gui。仅在没有其他仿真时使用")
parser.add_argument("--force", action="store_true", help="仍未响应 SIGTERM 时使用 SIGKILL")
args = parser.parse_args(sys.argv[2:])
Process = namedtuple("Process", "pid parent started argv")
workspace_nodes = {
    "legbot_bringup": {
        "cloud_transform", "global_navigation_adapter", "go2_command_adapter",
        "go2_demo_sequencer", "scan_waypoint_mission",
    },
    "legbot_exploration": {"combined_exploration_cpp", "explorer_cpp", "online_layer_map_cpp"},
    "scan_planner": {"scan_planner_node", "scan_go2_cmd_adapter"},
}


def read_process(pid):
    directory = Path("/proc") / str(pid)
    try:
        if directory.stat().st_uid != os.getuid():
            return None
        fields = (directory / "stat").read_text().rsplit(") ", 1)[1].split()
        if fields[0] == "Z":
            return None
        argv = (directory / "cmdline").read_bytes().decode(errors="replace").rstrip("\0").split("\0")
        return Process(pid, int(fields[1]), fields[19], argv) if argv[0] else None
    except (OSError, IndexError):
        return None


def is_launch(process):
    argv = process.argv
    return any(Path(arg).name == "ros2" and argv[i + 1:i + 4] ==
               ["launch", "legbot_exploration", "cross_layer_test.launch.py"]
               for i, arg in enumerate(argv[:2]))


def is_residual(process):
    argv = process.argv
    executable = Path(argv[0]).name
    # Python ROS nodes use the installed entry point as their second argument.
    entry = Path(argv[1]) if executable.startswith("python") and len(argv) > 1 else Path(argv[0])
    install = workspace / "install"
    if entry.is_relative_to(install):
        parts = entry.relative_to(install).parts
        if (len(parts) == 4 and parts[1] == "lib" and
                parts[3] in workspace_nodes.get(parts[2], ())):
            return True
    if executable == "parameter_bridge":
        return any(arg.startswith(("/go2/odometry@", "/go2/pose_tf@")) for arg in argv[1:])
    if executable == "robot_state_publisher":
        return "robot_description:=/go2/robot_description" in argv
    if executable == "rviz2":
        return any(Path(arg).is_relative_to(workspace) and
                   Path(arg).name == "multi_layer_map.rviz" for arg in argv[1:])
    command = " ".join(argv)
    if not re.match(r"^(?:[^ ]*/)?gz sim(?: |$)", command):
        return False
    if "BuildingCrossLayerTest.sdf" in command and str(workspace) in command:
        return True
    return args.all_gazebo and bool(re.match(r"^gz sim (?:server|gui)(?: |$)", command))


def collect():
    processes = {}
    for directory in Path("/proc").iterdir():
        if directory.name.isdecimal() and (process := read_process(int(directory.name))):
            processes[process.pid] = process
    launches = {pid for pid, process in processes.items() if is_launch(process)}
    targets = launches | {pid for pid, process in processes.items() if is_residual(process)}
    # Gazebo may change its process title or detach after launch exits.
    descendants = launches.copy()
    while new := {pid for pid, process in processes.items()
                  if process.parent in descendants} - descendants:
        descendants.update(new)
    return processes, launches, targets | descendants


def still_running(process):
    current = read_process(process.pid)
    return current is not None and current.started == process.started


def stop(process, sig):
    if still_running(process):
        try:
            os.kill(process.pid, sig)
        except ProcessLookupError:
            pass


processes, launches, targets = collect()
if args.dry_run:
    for pid in sorted(targets):
        print(f"{pid:>7}  {' '.join(processes[pid].argv)}")
    if not targets:
        print("没有找到匹配的仿真进程。")
    sys.exit(0)

for pid in sorted(launches):
    stop(processes[pid], signal.SIGINT)
if launches:
    time.sleep(5)

tracked = {(processes[pid].pid, processes[pid].started): processes[pid] for pid in targets}
# A node may have started during shutdown. Check for recognizable leftovers again.
current, _, targets = collect()
for pid in targets:
    process = current[pid]
    tracked[(pid, process.started)] = process
remaining = [process for process in tracked.values() if still_running(process)]
for process in remaining:
    stop(process, signal.SIGTERM)
if remaining:
    time.sleep(2)
remaining = [process for process in remaining if still_running(process)]
if remaining and args.force:
    for process in remaining:
        stop(process, signal.SIGKILL)
    time.sleep(1)
    remaining = [process for process in remaining if still_running(process)]
if remaining:
    for process in remaining:
        print(f"仍在运行 {process.pid}: {' '.join(process.argv)}", file=sys.stderr)
    sys.exit(1)
print("匹配的仿真进程已清理。")
PY
