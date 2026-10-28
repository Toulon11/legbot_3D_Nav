"""Headless, isolated flat exploration benchmark using Gazebo truth pose."""

import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def inc(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share(package), 'launch', launch_file)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    world = os.path.join(share('legbot_bringup'), 'worlds', 'ExplorationFlat.sdf')
    odom = '/slam/global_odometry_base'
    cloud = '/slam/global_cloud_registered'
    return LaunchDescription([
        inc('legbot_bringup', 'simulation.launch.py', {
            'gui': 'false', 'world': world,
            'navigation_source': 'ground_truth',
            'navigation_interface': 'global',
            'publish_ground_truth': 'true',
        }),
        TimerAction(period=12.0, actions=[inc('legbot_bringup', 'go2_demo_control.launch.py', {
            'use_sim_time': 'true', 'odom_topic': odom, 'cloud_topic': cloud,
            'localization_label': 'Gazebo ground truth',
        })]),
        TimerAction(period=13.0, actions=[inc('legbot_bringup', 'scan.launch.py', {
            'use_sim_time': 'true', 'navi_mode': '1', 'frame_id': 'map',
            'odom_topic': odom, 'cloud_topic': cloud,
            'sensor_pose_topic': '/slam/odometry_lidar',
            'manual_goal_use_message_z': 'false',
        })]),
        TimerAction(period=15.0, actions=[inc('legbot_exploration', 'exploration.launch.py', {
            'localization_source': 'gazebo', 'impl': 'cpp', 'use_sim_time': 'true',
            'observation_only': 'false', 'start_waypoint_coordinator': 'true',
            'flat_safety': 'true',
        })]),
    ])
