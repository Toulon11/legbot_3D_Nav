"""Start exploration and the online multi-layer map with one Gazebo and one RViz."""

import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = share('legbot_bringup')
    exploration = share('legbot_exploration')
    exploration_demo = os.path.join(exploration, 'launch', 'exploration_demo.launch.py')
    rviz_config = os.path.join(exploration, 'rviz', 'multi_layer_map.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('mission_return_home',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('record_metrics',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('result_directory',default_value=''),
        DeclareLaunchArgument('metrics_floors',default_value='[]'),
        DeclareLaunchArgument('metrics_target_floor',default_value=''),
        DeclareLaunchArgument('metrics_single_floor',default_value=''),
        DeclareLaunchArgument('metrics_reference_file',default_value=''),
        DeclareLaunchArgument('metrics_spawn_ground',default_value='[-27.0,6.0,0.0]'),
        DeclareLaunchArgument('metrics_max_sim_seconds',default_value='0.0'),
        DeclareLaunchArgument('metrics_capture_wall_seconds',default_value='0.0'),
        DeclareLaunchArgument('metrics_partial_run',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('metrics_method',default_value='legbot_online_pct'),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(bringup, 'worlds', 'Building.sdf'),
            description='Gazebo world used by the exploration demo.'),
        DeclareLaunchArgument('localization_source', default_value='gazebo',
                              choices=['gazebo', 'fastlio', 'fastlivo']),
        DeclareLaunchArgument('control_start_delay', default_value='12.0'),
        DeclareLaunchArgument('scan_robot_radius', default_value='0.35',
                              description='SCAN horizontal obstacle inflation radius in metres.'),
        DeclareLaunchArgument('show_layer_rviz', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('gazebo_gui', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('enable_ground_lidar', default_value='true', choices=['true', 'false'],
                              description='Start the downward LiDAR and fuse its registered cloud into the online PCT map.'),
        DeclareLaunchArgument('skip_start_floor_exploration', default_value='', choices=['', 'true', 'false'],
                              description='Skip initial-floor ordinary tasks and select ascending frontiers or measured supports. Empty uses exploration.yaml.'),
        # The exploration entry owns Gazebo, localization, GO2 control and SCAN.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(exploration_demo),
            launch_arguments={
                **{name:LaunchConfiguration(name) for name in ('record_metrics','result_directory',
                    'metrics_floors','metrics_target_floor','metrics_single_floor','metrics_reference_file',
                    'metrics_spawn_ground','metrics_max_sim_seconds','metrics_method',
                    'metrics_capture_wall_seconds','metrics_partial_run')},
                'world': LaunchConfiguration('world'),
                'localization_source': LaunchConfiguration('localization_source'),
                'control_start_delay': LaunchConfiguration('control_start_delay'),
                'start_rviz': 'false',
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                'enable_ground_lidar': LaunchConfiguration('enable_ground_lidar'),
                'mission_return_home': LaunchConfiguration('mission_return_home'),
                'skip_start_floor_exploration': LaunchConfiguration('skip_start_floor_exploration'),
                'planner_safety': 'true',
                'scan_robot_radius': LaunchConfiguration('scan_robot_radius'),
                'layer_map_in_process': 'true',
            }.items()),
        # The combined executable owns both explorer and map nodes. RViz
        # still receives the unchanged complete map topics from the map node.
        Node(package='rviz2', executable='rviz2',
             name='multi_layer_map_rviz', output='screen',
             arguments=['-d', rviz_config],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('show_layer_rviz'))),
    ])
