"""Run the upstream Avia bag profile with its RViz view and image republisher."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_nodes(context):
    config = LaunchConfiguration('avia_params_file').perform(context)
    with open(config, encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)['/**']['ros__parameters']
    image_topic = parameters['common']['img_topic']
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    rviz_config = os.path.join(
        get_package_share_directory('fast_livo'), 'rviz_cfg', 'fast_livo2.rviz')

    return [
        # The upstream bag stores compressed images. Publish their raw form
        # on the image topic selected in avia.yaml for FAST-LIVO2.
        Node(
            package='image_transport', executable='republish', name='republish',
            remappings=[('in/compressed', image_topic + '/compressed'),
                        ('out', image_topic)],
            parameters=[{'use_sim_time': sim_time,
                         'in_transport': 'compressed',
                         'out_transport': 'raw'}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('republish_compressed')),
            respawn=LaunchConfiguration('use_respawn'),
        ),
        Node(
            package='fast_livo', executable='fastlivo_mapping', name='laserMapping',
            parameters=[config, {'use_sim_time': sim_time}], output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': sim_time}], output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ]


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('fast_livo'), 'config', 'avia.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('avia_params_file', default_value=config),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('republish_compressed', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('use_respawn', default_value='true',
                              choices=['true', 'false']),
        OpaqueFunction(function=launch_nodes),
    ])
