"""Same headless stack with the exploratory node restricted to local frontiers."""

import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


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
        TimerAction(period=15.0, actions=[
            inc('legbot_bringup', 'scan_waypoints.launch.py', {
                'use_sim_time': 'true', 'mode': 'direct',
                'input_topic': '/scan/waypoint_input',
                'odom_topic': odom, 'frame_id': 'map',
                'ready_topic': '/go2/demo_ready',
            }),
            Node(package='legbot_exploration', executable='explorer_cpp',
                 name='legbot_explorer', output='screen', parameters=[
                     os.path.join(share('legbot_exploration'), 'config', 'exploration.yaml'),
                     {'use_sim_time': True, 'observation_only': False,
                      'base_odom_topic': odom,
                      'sensor_odom_topic': '/slam/odometry_lidar',
                      'cloud_topic': cloud, 'frame_id': 'map',
                      'robot_radius': .50, 'max_goal_distance': 4.0,
                      'global_search': False,
                      'bounds_relative_to_start': False,
                      'exploration_min_x': -31.5, 'exploration_max_x': -0.5,
                      'exploration_min_y': -15.5, 'exploration_max_y': 15.5,
                      'require_planner_occupancy': True}
                 ]),
        ]),
    ])
