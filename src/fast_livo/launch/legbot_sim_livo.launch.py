"""Start FAST-LIVO2 using sensor topics provided by another launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    livo_share = get_package_share_directory('fast_livo')
    bringup_share = get_package_share_directory('legbot_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=os.path.join(livo_share, 'config', 'legbot_sim.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('sensor_frame', default_value='livox_imu_link'),
        DeclareLaunchArgument('lidar_frame', default_value='livox_link'),
        DeclareLaunchArgument('publish_global', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('alignment_mode', default_value='gazebo_truth',
                              choices=['gazebo_truth', 'spawn_pose']),
        DeclareLaunchArgument('spawn_x', default_value='-27.0'),
        DeclareLaunchArgument('spawn_y', default_value='6.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.50'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        Node(
            package='fast_livo', executable='fastlivo_mapping', name='laserMapping',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
        ),
        Node(
            package='legbot_bringup', executable='fastlio_odom_adapter',
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'sensor_frame': LaunchConfiguration('sensor_frame'),
                'lidar_frame': LaunchConfiguration('lidar_frame'),
                'base_frame': 'base',
            }],
            remappings=[
                ('input', '/fast_livo/odometry_sensor'),
                ('output', '/fast_livo/odometry_base'),
                ('lidar_output', '/fast_livo/odometry_lidar'),
            ],
        ),
        Node(
            package='legbot_bringup', executable='global_navigation_adapter',
            condition=IfCondition(LaunchConfiguration('publish_global')),
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'source': 'fastlivo',
                'alignment_mode': LaunchConfiguration('alignment_mode'),
                'spawn_x': ParameterValue(LaunchConfiguration('spawn_x'), value_type=float),
                'spawn_y': ParameterValue(LaunchConfiguration('spawn_y'), value_type=float),
                'spawn_z': ParameterValue(LaunchConfiguration('spawn_z'), value_type=float),
                'spawn_yaw': ParameterValue(LaunchConfiguration('spawn_yaw'), value_type=float),
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', os.path.join(bringup_share, 'rviz', 'scan_fastlio_flat.rviz')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
