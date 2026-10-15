"""Check explorer -> existing waypoint coordinator -> SCAN goal -> stop ACK.

Run only with an isolated ROS_DOMAIN_ID. This test never starts SCAN or a robot.
Pass the installed explorer executable path as the sole argument.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String


def launch_process(args):
    return subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def stop_process(child):
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
    try:
        output, _ = child.communicate(timeout=4)
    except subprocess.TimeoutExpired:
        child.kill()
        output, _ = child.communicate()
    return output


def main():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    config = os.path.join(root, 'config', 'exploration.yaml')
    coordinator_script = '/home/likai/legbot_3D_Nav/src/legbot_bringup/scripts/scan_waypoint_mission'
    coordinator = launch_process([
        sys.executable, coordinator_script, '--mode', 'direct',
        '--input-topic', '/scan/waypoint_input', '--ready-topic', '/go2/demo_ready',
        '--odom-topic', '/fast_lio/odometry_base', '--output',
        '/tmp/legbot_exploration_handoff_mission.json', '--dwell-seconds', '0.1'])
    explorer = launch_process([
        sys.argv[1], '--ros-args', '--params-file', config,
        '-p', 'use_sim_time:=false', '-p', 'observation_only:=false'])
    rclpy.init()
    node = rclpy.create_node('exploration_handoff_test')
    sensor_pub = node.create_publisher(Odometry, '/fast_lio/odometry_lidar', qos_profile_sensor_data)
    base_pub = node.create_publisher(Odometry, '/fast_lio/odometry_base', qos_profile_sensor_data)
    cloud_pub = node.create_publisher(PointCloud2, '/fast_lio/cloud_registered', qos_profile_sensor_data)
    ack_pub = node.create_publisher(PoseStamped, '/scan/goal_stop_ack', 10)
    state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    ready_pub = node.create_publisher(Bool, '/go2/demo_ready', state_qos)
    goals, statuses = [], []
    node.create_subscription(PoseStamped, '/scan/goal', goals.append, 10)
    node.create_subscription(String, '/exploration/status',
                             lambda msg: statuses.append(json.loads(msg.data)), state_qos)
    points = [(4.0 * math.cos(i * math.pi / 100 - math.pi / 2),
               4.0 * math.sin(i * math.pi / 100 - math.pi / 2), -0.61)
              for i in range(101)]
    succeeded = False
    ack_start = None
    try:
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            if coordinator.poll() is not None or explorer.poll() is not None:
                break
            stamp = node.get_clock().now().to_msg()
            sensor = Odometry()
            sensor.header = Header(stamp=stamp, frame_id='odom')
            sensor.pose.pose.position.z = -0.01
            sensor.pose.pose.orientation.w = 1.0
            base = Odometry()
            base.header = Header(stamp=stamp, frame_id='odom')
            base.pose.pose.position.z = -0.11
            base.pose.pose.orientation.w = 1.0
            cloud = point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id='odom'), points)
            ready_pub.publish(Bool(data=True))
            sensor_pub.publish(sensor)
            base_pub.publish(base)
            cloud_pub.publish(cloud)
            if goals:
                assert goals[0].pose.position.z == 0.0, 'SCAN same-floor goal must have z=0'
                if ack_start is None:
                    ack_start = time.monotonic()
                if time.monotonic() - ack_start < 2.0:
                    ack_pub.publish(goals[0])
            rclpy.spin_once(node, timeout_sec=0.1)
            if any(item['state'] == 'goal_reached' for item in statuses):
                succeeded = True
                break
        if not succeeded:
            raise AssertionError('Goal handoff did not complete; goals=%d, states=%s' %
                                 (len(goals), [item['state'] for item in statuses[-12:]]))
        print('PASS', sys.argv[1], 'forwarded_goal=',
              (goals[0].pose.position.x, goals[0].pose.position.y))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        explorer_output = stop_process(explorer)
        coordinator_output = stop_process(coordinator)
        if not succeeded:
            print('EXPLORER:\n', explorer_output, '\nCOORDINATOR:\n', coordinator_output,
                  file=sys.stderr)


if __name__ == '__main__':
    main()
