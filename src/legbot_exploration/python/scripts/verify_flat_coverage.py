#!/usr/bin/env python3
"""订阅探索验收网格，用 Gazebo 单层场景几何做独立核验。"""

import argparse
import json
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from legbot_exploration.truth_coverage import verify_grid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default=os.path.join(
        get_package_share_directory('legbot_bringup'), 'worlds', 'ExplorationFlat.sdf'))
    parser.add_argument('--spawn-x', type=float, default=-27.)
    parser.add_argument('--spawn-y', type=float, default=6.)
    parser.add_argument('--robot-radius', type=float, default=.5)
    parser.add_argument('--timeout', type=float, default=15.)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('flat_coverage_verifier')
    messages = {}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/exploration/coverage_grid',
                             lambda msg: messages.__setitem__('grid', msg), qos)
    node.create_subscription(String, '/exploration/status',
                             lambda msg: messages.__setitem__('status', msg), qos)
    deadline = time.monotonic() + args.timeout
    while 'grid' not in messages and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.2)
    if 'grid' not in messages:
        print('未收到 /exploration/coverage_grid', file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    grid = messages['grid']
    report = verify_grid(args.world, grid.info.origin.position.x,
                         grid.info.origin.position.y, grid.info.width,
                         grid.info.height, grid.info.resolution, grid.data,
                         (args.spawn_x, args.spawn_y), args.robot_radius)
    if 'status' in messages:
        status = json.loads(messages['status'].data)
        report['exploration_state'] = status.get('state')
        report['online_reachable_unknown_cells'] = status.get('reachable_unknown_cells')
        report['online_excluded_unknown_cells'] = status.get('excluded_unknown_cells')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    if report.get('exploration_state') == 'exploration_complete' and (
            report['unobserved_reachable_cells'] or
            report.get('online_reachable_unknown_cells', 0)):
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
