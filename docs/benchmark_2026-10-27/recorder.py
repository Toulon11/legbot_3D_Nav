#!/usr/bin/env python3
"""Record independent truth coverage and exploration status for one ROS run."""

import argparse
import json
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid, Path
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from legbot_exploration.truth_coverage import reachable_ground_cells


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=300.)
    parser.add_argument('--output', default='/tmp/go2-quant-current.jsonl')
    args = parser.parse_args()
    world = os.path.join(get_package_share_directory('legbot_bringup'),
                         'worlds', 'ExplorationFlat.sdf')
    rclpy.init()
    node = rclpy.create_node('go2_quant_recorder')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    started = time.monotonic()
    state = {'status': {}, 'truth': {}, 'path_m': 0., 'ready': False,
             'sim_s': None, 'truth_cells': None}
    next_print = started + 30.
    out = open(args.output, 'w', buffering=1)

    def record(event):
        now = time.monotonic()
        out.write(json.dumps({'wall_s': round(now - started, 3),
                              'event': event, 'sim_s': state['sim_s'],
                              'ready': state['ready'],
                              'status': state['status'],
                              'truth': state['truth'],
                              'path_m': round(state['path_m'], 3)},
                             ensure_ascii=False) + '\n')

    def on_status(msg):
        try:
            state['status'] = json.loads(msg.data)
        except ValueError:
            return
        record('status')

    def on_grid(msg):
        key = (msg.info.origin.position.x, msg.info.origin.position.y,
               msg.info.width, msg.info.height, msg.info.resolution)
        if state.get('grid_key') != key:
            try:
                state['truth_cells'] = reachable_ground_cells(
                    world, key[0], key[1], key[2], key[3], key[4],
                    (-27., 6.), .5)
                state['grid_key'] = key
            except Exception as exc:
                state['truth'] = {'error': str(exc)}
                record('grid_error')
                return
        truth = state['truth_cells']
        missing = sum(msg.data[i] == -1 for i in truth)
        state['truth'] = {'reachable_ground_cells': len(truth),
                          'unobserved_reachable_cells': missing,
                          'reachable_ground_coverage': 1 - missing / len(truth)}
        record('grid')

    def on_path(msg):
        if len(msg.poses) < 2:
            return
        points = [p.pose.position for p in msg.poses]
        state['path_m'] = sum(math.hypot(b.x - a.x, b.y - a.y)
                              for a, b in zip(points, points[1:]))

    def on_clock(msg):
        state['sim_s'] = round(msg.clock.sec + msg.clock.nanosec * 1e-9, 3)

    node.create_subscription(String, '/exploration/status', on_status, qos)
    node.create_subscription(OccupancyGrid, '/exploration/coverage_grid', on_grid, qos)
    node.create_subscription(Path, '/exploration/trajectory', on_path, qos)
    node.create_subscription(Bool, '/go2/demo_ready',
                             lambda msg: state.__setitem__('ready', msg.data), 10)
    node.create_subscription(Clock, '/clock', on_clock, 10)
    deadline = started + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.2)
        now = time.monotonic()
        if now >= next_print:
            record('tick')
            print(json.dumps({'wall_s': round(now - started, 1),
                              'ready': state['ready'],
                              'state': state['status'].get('state'),
                              'truth': state['truth'],
                              'path_m': round(state['path_m'], 2)},
                             ensure_ascii=False), flush=True)
            next_print += 30.
        if state['status'].get('state') == 'exploration_complete':
            record('complete')
            break
    record('end')
    out.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
