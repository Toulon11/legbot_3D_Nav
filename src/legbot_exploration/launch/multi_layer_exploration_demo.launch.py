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
    layer_config = os.path.join(exploration, 'config', 'multi_layer_map.yaml')
    rviz_config = os.path.join(exploration, 'rviz', 'multi_layer_map.rviz')

    return LaunchDescription([
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
        # The exploration entry owns Gazebo, localization, GO2 control and SCAN.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(exploration_demo),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'localization_source': LaunchConfiguration('localization_source'),
                'control_start_delay': LaunchConfiguration('control_start_delay'),
                'start_rviz': 'false',
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                'planner_safety': 'true',
                'scan_robot_radius': LaunchConfiguration('scan_robot_radius'),
            }.items()),
        # Keep these nodes in this launch file. In particular, the RViz
        # condition has a distinct name from exploration_demo's start_rviz,
        # which is set to false above to avoid opening two RViz windows.
        Node(package='legbot_exploration', executable='online_layer_map_cpp',
             name='online_layer_map', output='screen',
             parameters=[layer_config, {'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2',
             name='multi_layer_map_rviz', output='screen',
             arguments=['-d', rviz_config],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('show_layer_rviz'))),
    ])
