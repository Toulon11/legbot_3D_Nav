import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    config = LaunchConfiguration('config').perform(context)
    sensor_frame = LaunchConfiguration('sensor_frame').perform(context)
    return [
        Node(package='fast_lio', executable='fastlio_mapping', parameters=[config, {'use_sim_time': use_sim_time}],
             remappings=[('/Odometry','/fast_lio/odometry_sensor'),
                         ('/cloud_registered','/fast_lio/cloud_registered'),
                         ('/cloud_registered_body','/fast_lio/cloud_registered_body'),
                         ('/Laser_map','/fast_lio/map'),
                         ('/path','/fast_lio/path')]),
        Node(package='legbot_bringup', executable='fastlio_odom_adapter',
             parameters=[{'use_sim_time': use_sim_time, 'sensor_frame': sensor_frame,
                          'lidar_frame': LaunchConfiguration('lidar_frame').perform(context),
                          'base_frame': 'base'}],
             remappings=[('input','/fast_lio/odometry_sensor'),
                         ('output','/fast_lio/odometry_base'),
                         ('lidar_output','/fast_lio/odometry_lidar')]),
        Node(package='legbot_bringup', executable='global_navigation_adapter',
             condition=IfCondition(LaunchConfiguration('publish_global')),
             parameters=[{'use_sim_time': use_sim_time,
                          'source': 'fastlio',
                          'alignment_mode': LaunchConfiguration('alignment_mode'),
                          'spawn_x': ParameterValue(LaunchConfiguration('spawn_x'), value_type=float),
                          'spawn_y': ParameterValue(LaunchConfiguration('spawn_y'), value_type=float),
                          'spawn_z': ParameterValue(LaunchConfiguration('spawn_z'), value_type=float),
                          'spawn_yaw': ParameterValue(LaunchConfiguration('spawn_yaw'), value_type=float)}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config', default_value=os.path.join(share('legbot_bringup'),'config/fastlio_real.yaml')),
        DeclareLaunchArgument('sensor_frame', default_value='livox_imu_link'),
        DeclareLaunchArgument('lidar_frame', default_value='livox_link'),
        DeclareLaunchArgument('publish_global', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('alignment_mode', default_value='gazebo_truth',
                              choices=['gazebo_truth', 'spawn_pose']),
        DeclareLaunchArgument('spawn_x', default_value='-27.0'),
        DeclareLaunchArgument('spawn_y', default_value='6.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.50'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        OpaqueFunction(function=setup),
    ])
