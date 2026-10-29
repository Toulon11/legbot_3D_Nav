"""在独立 ROS_DOMAIN_ID 中检查起步转向、停车与安全目标交接。"""
import json
import math
import os
import signal
import struct
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, Int32, String


def layer_message(stamp):
    # 在起点约一米外提供一个观测到的平坦支撑格；起点附近没有支撑格。
    fields = [PointField(name=name, offset=offset, datatype=kind, count=1)
              for name, offset, kind in (
                  ('x', 0, PointField.FLOAT32), ('y', 4, PointField.FLOAT32),
                  ('z', 8, PointField.FLOAT32), ('surface_kind', 20, PointField.UINT8),
                  ('clearance_kind', 21, PointField.UINT8),
                  ('support_columns', 22, PointField.UINT16),
                  ('hit_observations', 28, PointField.UINT32))]
    data = bytearray(40)
    struct.pack_into('<fff', data, 0, 1.15, 0.15, -0.43)
    struct.pack_into('<BBH', data, 20, 1, 1, 10)
    struct.pack_into('<I', data, 28, 10)
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id='map')
    msg.height, msg.width, msg.point_step, msg.row_step = 1, 1, 40, 40
    msg.fields, msg.data, msg.is_dense = fields, bytes(data), True
    return msg


def main():
    executable = sys.argv[1]
    config = os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
                                          'config', 'exploration.yaml'))
    child = subprocess.Popen([executable, '--ros-args', '--params-file', config,
                              '-p', 'use_sim_time:=false', '-p', 'observation_only:=false',
                              '-p', 'global_search:=false',
                              '-p', 'require_planner_occupancy:=false',
                              '-p', 'startup_turn_timeout:=10.0'],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = rclpy.create_node('exploration_bootstrap_test')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sensor_pub = node.create_publisher(Odometry, '/slam/odometry_lidar', qos_profile_sensor_data)
    base_pub = node.create_publisher(Odometry, '/slam/global_odometry_base', qos_profile_sensor_data)
    cloud_pub = node.create_publisher(PointCloud2, '/slam/global_cloud_registered', qos_profile_sensor_data)
    layer_pub = node.create_publisher(PointCloud2, '/exploration/layers/surfaces', latched)
    ready_pub = node.create_publisher(Bool, '/go2/demo_ready', latched)
    turns, modes, goals, statuses = [], [], [], []
    node.create_subscription(Twist, '/exploration/bootstrap_turn_cmd', turns.append, 10)
    node.create_subscription(Int32, '/go2/control_mode', modes.append, 10)
    node.create_subscription(PoseStamped, '/scan/waypoint_input', goals.append, 10)
    node.create_subscription(String, '/exploration/status',
                             lambda msg: statuses.append(json.loads(msg.data)), latched)
    # 原点向四周的射线使远处空闲格得到观测；层析图仍保留起点的支撑盲区。
    points = [(4 * math.cos(i * math.tau / 3600),
               4 * math.sin(i * math.tau / 3600), -0.61)
              for i in range(3600)]
    yaw = 0.0
    succeeded = False
    try:
        deadline = time.monotonic() + 16.0
        while time.monotonic() < deadline:
            if child.poll() is not None:
                break
            if turns and turns[-1].angular.z > 0.0:
                yaw += 0.20
            stamp = node.get_clock().now().to_msg()
            sensor = Odometry()
            sensor.header = Header(stamp=stamp, frame_id='map')
            sensor.pose.pose.position.z = -0.01
            sensor.pose.pose.orientation.w = 1.0
            base = Odometry()
            base.header = Header(stamp=stamp, frame_id='map')
            base.pose.pose.position.z = -0.11
            base.pose.pose.orientation.z = math.sin(yaw / 2)
            base.pose.pose.orientation.w = math.cos(yaw / 2)
            ready_pub.publish(Bool(data=True))
            sensor_pub.publish(sensor)
            base_pub.publish(base)
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id='map'), points))
            layer_pub.publish(layer_message(stamp))
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
            if goals:
                assert any(msg.angular.z > 0.1 for msg in turns)
                assert turns[-1].angular.z == 0.0, 'Turn command must stop before goal'
                assert any(msg.data == 3 for msg in modes)
                assert all(msg.data != 2 for msg in modes), 'Repeated mode 2 makes GO2 crouch'
                assert any(item['state'] == 'bootstrap_turning' for item in statuses)
                assert 0.65 <= math.hypot(goals[-1].pose.position.x,
                                          goals[-1].pose.position.y) <= 2.0
                succeeded = True
                break
        if not succeeded:
            raise AssertionError('No bootstrap goal; statuses=%s' %
                                 [(s['state'], s.get('detail')) for s in statuses[-12:]])
        print('PASS bootstrap turn, zero stop, no crouch command, safe goal')
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
