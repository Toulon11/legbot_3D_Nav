"""Run one exploration implementation and one existing SCAN waypoint coordinator."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('legbot_exploration')
    bringup = get_package_share_directory('legbot_bringup')
    params = os.path.join(share, 'config', 'exploration.yaml')
    common = [params, {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'observation_only': ParameterValue(LaunchConfiguration('observation_only'), value_type=bool),
    }]
    return LaunchDescription([
        DeclareLaunchArgument('impl', default_value='cpp', choices=['cpp', 'python']),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('observation_only', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_waypoint_coordinator', default_value='true', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'scan_waypoints.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_waypoint_coordinator')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'mode': 'direct',
                'input_topic': '/scan/waypoint_input',
                'ready_topic': '/go2/demo_ready',
            }.items()),
        Node(package='legbot_exploration', executable='explorer_cpp', name='legbot_explorer',
             output='screen', parameters=common,
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('impl'), "' == 'cpp'"]))),
        Node(package='legbot_exploration', executable='explorer_py', name='legbot_explorer',
             output='screen', parameters=common,
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('impl'), "' == 'python'"]))),
    ])
