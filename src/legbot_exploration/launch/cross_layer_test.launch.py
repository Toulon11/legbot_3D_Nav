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
        DeclareLaunchArgument('record_metrics',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('result_directory',default_value=''),
        DeclareLaunchArgument('metrics_method',default_value='legbot_online_pct'),
        DeclareLaunchArgument('metrics_max_sim_seconds',default_value='0.0'),
        DeclareLaunchArgument('metrics_capture_wall_seconds',default_value='0.0'),
        DeclareLaunchArgument('metrics_partial_run',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('world', default_value=world, description='Compact Building test world'),
        DeclareLaunchArgument('skip_start_floor_exploration', default_value='', choices=['', 'true', 'false'],
                              description='Skip initial-floor ordinary tasks and select ascending frontiers or measured supports. Empty uses exploration.yaml.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                exploration, 'launch', 'multi_layer_exploration_demo.launch.py')),
            launch_arguments={
                'record_metrics':LaunchConfiguration('record_metrics'),
                'result_directory':LaunchConfiguration('result_directory'),
                **{name:LaunchConfiguration(name) for name in ('metrics_method','metrics_max_sim_seconds',
                    'metrics_capture_wall_seconds','metrics_partial_run')},
                'metrics_floors':'[{"name":"floor_1","height":0.0},{"name":"floor_2","height":1.46},'
                                 '{"name":"floor_3","height":2.96},{"name":"floor_4","height":4.43}]',
                'metrics_target_floor':'floor_4',
                'metrics_single_floor':'',
                'mission_return_home':'true',
                'metrics_spawn_ground':'[-6.75,9.60,0.0]',
                'world': LaunchConfiguration('world'),
                'skip_start_floor_exploration': LaunchConfiguration('skip_start_floor_exploration'),
                # Start on open ground with the first flight in front of GO2.
                'x': '-6.75',
                'y': '9.60',
                'yaw': '-1.5707963',
            }.items()),
    ])
