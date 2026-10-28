"""Start Gazebo, its global truth adapter, the online layer map, and RViz."""

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
    simulation = os.path.join(bringup, 'launch', 'simulation.launch.py')
    config = os.path.join(exploration, 'config', 'multi_layer_map.yaml')
    rviz_config = os.path.join(exploration, 'rviz', 'multi_layer_map.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=os.path.join(bringup, 'worlds', 'Building.sdf'),
            description='Gazebo world to observe with the online layer map.'),
        DeclareLaunchArgument('start_simulation', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('gazebo_gui', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('start_rviz', default_value='true',
                              choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation),
            condition=IfCondition(LaunchConfiguration('start_simulation')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'gui': LaunchConfiguration('gazebo_gui'),
                'navigation_source': 'ground_truth',
                'navigation_interface': 'global',
                'publish_ground_truth': 'true',
            }.items()),
        Node(
            package='legbot_exploration',
            executable='online_layer_map_cpp',
            name='online_layer_map',
            output='screen',
            parameters=[config, {'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='multi_layer_map_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
    ])
