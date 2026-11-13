"""Use the compact Building world with the normal online exploration stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    exploration = get_package_share_directory('legbot_exploration')
    bringup = get_package_share_directory('legbot_bringup')
    world = os.path.join(bringup, 'worlds', 'BuildingCrossLayerTest.sdf')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world, description='Compact Building test world'),
        DeclareLaunchArgument('skip_start_floor_exploration', default_value='', choices=['', 'true', 'false'],
                              description='Use shared exploration to recognize stairs, then leave the initial floor early. Empty uses exploration.yaml.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                exploration, 'launch', 'multi_layer_exploration_demo.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'skip_start_floor_exploration': LaunchConfiguration('skip_start_floor_exploration'),
                # Start on open ground with the first flight in front of GO2.
                'x': '-6.75',
                'y': '9.60',
                'yaw': '-1.5707963',
            }.items()),
    ])
