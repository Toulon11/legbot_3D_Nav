#!/usr/bin/env python3
"""Report GO2 LiDAR extrinsics from its model and live camera intrinsics."""

import argparse
import json
import math
import os
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory


def _joint_matrix(joint):
    origin = joint.find('origin')
    xyz = [0.0, 0.0, 0.0] if origin is None else [
        float(value) for value in origin.get('xyz', '0 0 0').split()]
    roll, pitch, yaw = ([0.0, 0.0, 0.0] if origin is None else [
        float(value) for value in origin.get('rpy', '0 0 0').split()])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    matrix = np.eye(4)
    matrix[:3, :3] = rz @ ry @ rx
    matrix[:3, 3] = xyz
    return matrix


def _model_transforms():
    description_share = get_package_share_directory('go2_description')
    controller_config = os.path.join(description_share, 'config', 'gazebo.yaml')
    robot = ET.fromstring(xacro.process_file(
        os.path.join(description_share, 'xacro', 'robot.xacro'),
        mappings={'GAZEBO': 'true', 'CLASSIC': 'false',
                  'ENABLE_LIDAR': 'true', 'controller_config': controller_config},
    ).toxml())
    joints = {}
    for joint in robot.findall('joint'):
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is not None and child is not None:
            joints[child.get('link')] = (parent.get('link'), _joint_matrix(joint))
    def to_root(frame):
        matrix = np.eye(4)
        seen = set()
        while frame in joints:
            if frame in seen:
                raise ValueError('Joint cycle at ' + frame)
            seen.add(frame)
            parent, transform = joints[frame]
            matrix = transform @ matrix
            frame = parent
        return matrix

    lidar = to_root('livox_link')
    imu = to_root('livox_imu_link')
    camera = to_root('front_camera_optical')
    return np.linalg.inv(imu) @ lidar, np.linalg.inv(camera) @ lidar


def _camera_info(topic, timeout):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo

    rclpy.init()
    node = rclpy.create_node('fastlivo_calibration_report')
    received = []
    subscription = node.create_subscription(
        CameraInfo, topic, received.append, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + timeout
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.25, deadline - time.monotonic())))
        return received[0] if received else None
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def _values(array):
    return [round(float(value), 12) for value in np.asarray(array).reshape(-1)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-info-topic', default='/camera/camera_info')
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='Seconds to wait for CameraInfo (default: 10)')
    parser.add_argument('--extrinsics-only', action='store_true',
                        help='Do not wait for the simulated camera')
    args = parser.parse_args()
    if args.timeout < 0:
        parser.error('--timeout must be nonnegative')

    imu_from_lidar, camera_from_lidar = _model_transforms()
    print('Transforms from the installed GO2 model (p_target = R * p_lidar + P):')
    print('  LiDAR -> IMU (livox_imu_link):')
    print('    extrinsic_R: ' + json.dumps(_values(imu_from_lidar[:3, :3])))
    print('    extrinsic_T: ' + json.dumps(_values(imu_from_lidar[:3, 3])))
    print('  LiDAR -> camera optical frame (front_camera_optical):')
    print('    Rcl: ' + json.dumps(_values(camera_from_lidar[:3, :3])))
    print('    Pcl: ' + json.dumps(_values(camera_from_lidar[:3, 3])))
    if args.extrinsics_only:
        return 0

    info = _camera_info(args.camera_info_topic, args.timeout)
    if info is None:
        print('No CameraInfo received on ' + args.camera_info_topic +
              '; start the camera-enabled simulation and retry.', file=sys.stderr)
        return 1
    print('CameraInfo from ' + args.camera_info_topic + ':')
    print('  frame_id: ' + info.header.frame_id)
    print('  width: ' + str(info.width))
    print('  height: ' + str(info.height))
    print('  distortion_model: ' + info.distortion_model)
    print('  K: ' + json.dumps([float(value) for value in info.k]))
    print('  D: ' + json.dumps([float(value) for value in info.d]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
