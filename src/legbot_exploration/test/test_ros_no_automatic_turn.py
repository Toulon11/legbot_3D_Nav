"""Confirm that receiving initial sensor data does not command a startup turn."""
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, Int32, String
from sensor_msgs.msg import PointCloud2


def main():
    executable = sys.argv[1]
    config = os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
                                          'config', 'exploration.yaml'))
    child = subprocess.Popen([executable, '--ros-args', '--params-file', config,
                              '-p', 'use_sim_time:=false', '-p', 'observation_only:=false',
                              '-p', 'global_search:=false',
                              '-p', 'require_planner_occupancy:=false'],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = rclpy.create_node('exploration_no_automatic_turn_test')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sensor_pub = node.create_publisher(Odometry, '/slam/odometry_lidar', qos_profile_sensor_data)
    base_pub = node.create_publisher(Odometry, '/slam/global_odometry_base', qos_profile_sensor_data)
    cloud_pub = node.create_publisher(PointCloud2, '/slam/global_cloud_registered', qos_profile_sensor_data)
    ready_pub = node.create_publisher(Bool, '/go2/demo_ready', latched)
    turns, modes, statuses = [], [], []
    node.create_subscription(Twist, '/exploration/bootstrap_turn_cmd', turns.append, 10)
    node.create_subscription(Int32, '/go2/control_mode', modes.append, 10)
    node.create_subscription(String, '/exploration/status',
                             lambda msg: statuses.append(json.loads(msg.data)), latched)
    try:
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline and child.poll() is None:
            stamp = node.get_clock().now().to_msg()
            sensor = Odometry()
            sensor.header = Header(stamp=stamp, frame_id='map')
            sensor.pose.pose.orientation.w = 1.0
            base = Odometry()
            base.header = Header(stamp=stamp, frame_id='map')
            base.pose.pose.orientation.w = 1.0
            ready_pub.publish(Bool(data=True))
            sensor_pub.publish(sensor)
            base_pub.publish(base)
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id='map'), [(2.0, 0.0, 0.0)]))
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        assert child.poll() is None, 'explorer exited'
        assert statuses, 'no explorer status'
        assert not any(t.angular.z != 0.0 for t in turns), 'unexpected automatic turn'
        assert not any(m.data == 2 for m in modes), 'GO2 crouch command must not repeat'
        assert not any(s['state'].startswith('bootstrap_') for s in statuses)
        print('PASS initial sensor data causes no automatic turn')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
        try:
            child.communicate(timeout=4)
        except subprocess.TimeoutExpired:
            child.kill()
            child.communicate()


if __name__ == '__main__':
    main()
