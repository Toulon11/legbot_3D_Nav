"""Run the exploration workflow in the single-level flat Gazebo arena."""

import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    flat_world = os.path.join(share('legbot_bringup'), 'worlds', 'ExplorationFlat.sdf')
    exploration_demo = os.path.join(
        share('legbot_exploration'), 'launch', 'exploration_demo.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument(
            'localization_source', default_value='gazebo',
            choices=['gazebo', 'fastlio', 'fastlivo']),
        DeclareLaunchArgument('control_start_delay', default_value='12.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(exploration_demo),
            launch_arguments={
                'world': flat_world,
                'localization_source': LaunchConfiguration('localization_source'),
                'control_start_delay': LaunchConfiguration('control_start_delay'),
            }.items()),
    ])
