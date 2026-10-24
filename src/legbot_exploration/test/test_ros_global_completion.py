"""ROS 集成回归：可达未知格阻止完成，已覆盖的区域可以完成。"""

import json
import os
import signal
import subprocess
import sys
import time

from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String


def main():
    executable = sys.argv[1]
    complete_case = len(sys.argv) > 2 and sys.argv[2] == 'complete'
    config = os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
                                          'config', 'exploration.yaml'))
    overrides = ([
        '-p', 'exploration_min_x:=0.0', '-p', 'exploration_max_x:=0.25',
        '-p', 'exploration_min_y:=0.0', '-p', 'exploration_max_y:=0.25',
        '-p', 'robot_radius:=0.0',
    ] if complete_case else [
        '-p', 'exploration_min_x:=-1.0', '-p', 'exploration_max_x:=3.0',
        '-p', 'exploration_min_y:=-1.0', '-p', 'exploration_max_y:=3.0',
        '-p', 'robot_radius:=0.0',
    ])
    child = subprocess.Popen([
        executable, '--ros-args', '--params-file', config,
        '-p', 'use_sim_time:=false', '-p', 'observation_only:=true',
        '-p', 'global_search:=true', '-p', 'bounds_relative_to_start:=false',
        '-p', 'no_frontier_hold:=1.0',
        '-p', 'point_stride:=1', '-p', 'max_points:=100',
        '-p', 'require_planner_occupancy:=false',
    ] + overrides, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = rclpy.create_node('global_completion_regression')
    state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    statuses, grids = [], []
    node.create_subscription(String, '/exploration/status',
                             lambda msg: statuses.append(json.loads(msg.data)), state_qos)
    node.create_subscription(OccupancyGrid, '/exploration/coverage_grid',
                             lambda msg: grids.append(msg), state_qos)
    sensor_pub = node.create_publisher(Odometry, '/slam/odometry_lidar', state_qos)
    base_pub = node.create_publisher(Odometry, '/slam/global_odometry_base', state_qos)
    cloud_pub = node.create_publisher(PointCloud2,
                                      '/slam/global_cloud_registered', state_qos)
    deadline = time.monotonic() + 8.0
    passed = False
    try:
        while time.monotonic() < deadline:
            if child.poll() is not None:
                raise AssertionError('explorer process exited unexpectedly')
            stamp = node.get_clock().now().to_msg()
            sensor = Odometry()
            sensor.header.frame_id = 'map'
            sensor.header.stamp = stamp
            sensor.pose.pose.position.z = .4
            sensor.pose.pose.orientation.w = 1.
            base = Odometry()
            base.header.frame_id = 'map'
            base.header.stamp = stamp
            base.pose.pose.position.z = .3
            base.pose.pose.orientation.w = 1.
            cloud = point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id='map'),
                [(2., 0., -.2), (0., 2., -.2), (1., 1., .4)])
            sensor_pub.publish(sensor)
            base_pub.publish(base)
            cloud_pub.publish(cloud)
            rclpy.spin_once(node, timeout_sec=.1)
            if statuses and ((complete_case and statuses[-1].get('state') ==
                              'exploration_complete') or
                             (not complete_case and statuses[-1].get('reachability_valid') and
                              statuses[-1].get('reachable_unknown_cells', 0) > 0)):
                if not grids:
                    raise AssertionError('coverage grid missing')
                if complete_case and statuses[-1]['unknown_cells'] != 0:
                    raise AssertionError('completed with unknown cells')
                if not complete_case and statuses[-1]['unknown_cells'] <= 0:
                    raise AssertionError('unknown count missing')
                if not complete_case and statuses[-1]['state'] == 'exploration_complete':
                    raise AssertionError('completed with reachable unknown cells')
                if (statuses[-1]['reachable_unknown_cells'] +
                        statuses[-1]['excluded_unknown_cells'] != statuses[-1]['unknown_cells']):
                    raise AssertionError('reachable and excluded unknown counts disagree')
                if statuses[-1]['unknown_cells'] != sum(value == -1 for value in grids[-1].data):
                    raise AssertionError('status and coverage grid disagree')
                passed = True
                break
        if not passed:
            raise AssertionError('expected incomplete state; last statuses: %r' % statuses[-3:])
        if not complete_case and any(status['state'] == 'exploration_complete' for status in statuses):
            raise AssertionError('false exploration_complete')
        print('PASS', executable, statuses[-1])
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
        try:
            output, _ = child.communicate(timeout=4)
        except subprocess.TimeoutExpired:
            child.kill()
            output, _ = child.communicate()
        if not passed:
            print(output, file=sys.stderr)


if __name__ == '__main__':
    main()
