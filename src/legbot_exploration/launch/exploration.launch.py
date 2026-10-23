"""Run one explorer and an optional SCAN direct-waypoint coordinator."""

import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    source = LaunchConfiguration('localization_source').perform(context)
    impl = LaunchConfiguration('impl').perform(context)
    odom = '/slam/global_odometry_base'
    cloud = '/slam/global_cloud_registered'
    sensor_pose = '/slam/odometry_lidar'
    use_sim_time = LaunchConfiguration('use_sim_time')
    flat_safety = LaunchConfiguration('flat_safety').perform(context) == 'true'

    actions = []
    if LaunchConfiguration('start_waypoint_coordinator').perform(context) == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share('legbot_bringup'), 'launch', 'scan_waypoints.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'mode': 'direct',
                'input_topic': '/scan/waypoint_input',
                'odom_topic': odom,
                'frame_id': 'map',
                'ready_topic': '/go2/demo_ready',
            }.items()))
    actions.append(Node(
        package='legbot_exploration',
        executable='explorer_cpp' if impl == 'cpp' else 'explorer_py',
        name='legbot_explorer',
        output='screen',
        parameters=[
            os.path.join(share('legbot_exploration'), 'config', 'exploration.yaml'),
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'observation_only': ParameterValue(
                    LaunchConfiguration('observation_only'), value_type=bool),
                'base_odom_topic': odom,
                'sensor_odom_topic': sensor_pose,
                'cloud_topic': cloud,
                'frame_id': 'map',
                **({'robot_radius': 0.50,
                    'max_goal_distance': 4.0,
                    'global_search': True,
                    'bounds_relative_to_start': True,
                    'exploration_min_x': -4.5,
                    'exploration_max_x': 26.5,
                    'exploration_min_y': -21.5,
                    'exploration_max_y': 9.5,
                    'require_planner_occupancy': True} if flat_safety else {}),
            },
        ],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('localization_source', default_value='fastlio',
                              choices=['gazebo', 'fastlio', 'fastlivo']),
        DeclareLaunchArgument('impl', default_value='cpp', choices=['cpp', 'python']),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('observation_only', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_waypoint_coordinator', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('flat_safety', default_value='false',
                              choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
