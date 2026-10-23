"""Regression: isolated residual frontiers must not trigger trajectory backtracking.

Run with a private ROS_DOMAIN_ID and the installed explorer executable path.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String


def main():
    executable = sys.argv[1]
    expect_residual_goal = '--expect-residual-goal' in sys.argv[2:]
    config = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config',
                                          'exploration.yaml'))
    child = subprocess.Popen([
        executable, '--ros-args', '--params-file', config,
        '-p', 'use_sim_time:=false', '-p', 'observation_only:=false',
        '-p', 'global_search:=true', '-p', 'min_frontier_cells:=100000',
        '-p', 'min_goal_distance:=1.0' if expect_residual_goal else
              'min_goal_distance:=20.0',
        '-p', 'exploration_min_x:=-5.0', '-p', 'exploration_max_x:=5.0',
        '-p', 'exploration_min_y:=-5.0', '-p', 'exploration_max_y:=5.0',
    ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = rclpy.create_node('residual_frontier_regression')
    status, targets, traveled = [], [], []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/exploration/status',
                             lambda m: status.append(json.loads(m.data)), qos)
    node.create_subscription(PoseStamped, '/scan/waypoint_input', targets.append, 10)
    node.create_subscription(Path, '/exploration/trajectory',
                             lambda m: traveled.append(max(
                                 (p.pose.position.x for p in m.poses), default=0.0)), qos)
    ready_pub = node.create_publisher(Bool, '/go2/demo_ready', qos)
    base_pub = node.create_publisher(Odometry, '/slam/global_odometry_base', qos_profile_sensor_data)
    sensor_pub = node.create_publisher(Odometry, '/slam/odometry_lidar', qos_profile_sensor_data)
    cloud_pub = node.create_publisher(PointCloud2, '/slam/global_cloud_registered', qos_profile_sensor_data)
    points = [(4.0 * math.cos(i * math.pi / 180),
               4.0 * math.sin(i * math.pi / 180), -0.2) for i in range(360)]
    succeeded = False
    try:
        start = time.monotonic()
        while time.monotonic() - start < 11.0:
            if child.poll() is not None:
                break
            elapsed = time.monotonic() - start
            x = 0.0 if elapsed < 1.5 or elapsed >= 3.0 else 2.5
            stamp = node.get_clock().now().to_msg()
            base = Odometry()
            base.header = Header(stamp=stamp, frame_id='map')
            base.pose.pose.position.x = x
            base.pose.pose.position.z = 0.3
            base.pose.pose.orientation.w = 1.0
            sensor = Odometry()
            sensor.header = base.header
            sensor.pose.pose.position.x = x
            sensor.pose.pose.position.z = 0.4
            sensor.pose.pose.orientation.w = 1.0
            ready_pub.publish(Bool(data=True))
            base_pub.publish(base)
            sensor_pub.publish(sensor)
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id='map'), points))
            rclpy.spin_once(node, timeout_sec=0.1)
        residual = [s for s in status if s.get('state') == 'frontiers_below_cluster_threshold'
                    and s.get('frontier_cells', 0) > 0 and s.get('frontier_clusters') == 0]
        assert child.poll() is None, 'explorer exited during test'
        assert max(traveled, default=0.0) >= 2.0, 'historical path was not built'
        assert residual, 'no residual-frontier state; latest status: %r' % status[-3:]
        assert not any(s.get('detail') == 'recovery_backtrack' for s in status), status[-3:]
        if expect_residual_goal:
            assert targets, 'single-cell residual fallback did not select a goal'
            assert any(s.get('detail') == 'residual_frontier' for s in status), status[-3:]
        else:
            assert not targets, 'unreachable residual frontiers triggered %d recovery target(s)' % len(targets)
        succeeded = True
        print('PASS', executable, 'residual_frontiers=', residual[-1]['frontier_cells'])
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
        if not succeeded:
            print('STATUS:', status[-5:], 'TARGETS:', len(targets), '\nEXPLORER:\n', output,
                  file=sys.stderr)


if __name__ == '__main__':
    main()
