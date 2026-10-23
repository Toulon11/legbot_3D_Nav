"""Isolated ROS smoke test: feed stamped cloud/odometry and observe a map update.

Usage: python3 test/test_ros_observation.py /absolute/path/to/explorer_cpp
       python3 test/test_ros_observation.py /absolute/path/to/explorer_py
Set a private ROS_DOMAIN_ID and source the matching install before running.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String


def main():
    executable = sys.argv[1]
    config = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config', 'exploration.yaml'))
    child = subprocess.Popen(
        [executable, '--ros-args', '--params-file', config, '-p', 'use_sim_time:=false',
         '-p', 'observation_only:=true'], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = rclpy.create_node('exploration_smoke_test')
    status = []
    sensor_pub = node.create_publisher(Odometry, '/slam/odometry_lidar', qos_profile_sensor_data)
    base_pub = node.create_publisher(Odometry, '/slam/global_odometry_base', qos_profile_sensor_data)
    cloud_pub = node.create_publisher(PointCloud2, '/slam/global_cloud_registered', qos_profile_sensor_data)
    state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/exploration/status',
                             lambda msg: status.append(json.loads(msg.data)), state_qos)
    points = [(4.0 * math.cos(i * math.pi / 100 - math.pi / 2),
               4.0 * math.sin(i * math.pi / 100 - math.pi / 2), -0.2)
              for i in range(101)]
    succeeded = False
    try:
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            if child.poll() is not None:
                break
            stamp = node.get_clock().now().to_msg()
            sensor = Odometry()
            sensor.header.frame_id = 'map'
            sensor.header.stamp = stamp
            sensor.pose.pose.position.z = 0.4
            sensor.pose.pose.orientation.w = 1.0
            base = Odometry()
            base.header.frame_id = 'map'
            base.header.stamp = stamp
            base.pose.pose.position.z = 0.3
            base.pose.pose.orientation.w = 1.0
            cloud = point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id='map'), points)
            # Deliberately allow transport to reorder these three topics.
            cloud_pub.publish(cloud)
            sensor_pub.publish(sensor)
            base_pub.publish(base)
            rclpy.spin_once(node, timeout_sec=0.1)
            if any(item.get('map_version', 0) > 0 and item.get('voxel_count', 0) > 100
                   for item in status):
                succeeded = True
                break
        if not succeeded:
            raise AssertionError('No cumulative map update; last status: %r' % (status[-3:],))
        print('PASS', executable, status[-1])
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
            print(output, file=sys.stderr)


if __name__ == '__main__':
    main()
