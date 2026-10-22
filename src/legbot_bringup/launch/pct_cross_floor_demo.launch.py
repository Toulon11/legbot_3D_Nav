"""One-command Gazebo + PCT + SCAN cross-floor mission."""

import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def include(package, launch_file, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share(package), 'launch', launch_file)),
        launch_arguments=(arguments or {}).items(),
        condition=condition)


def setup(context):
    navigation_source = LaunchConfiguration('navigation_source').perform(context)
    use_ground_truth = navigation_source == 'ground_truth'
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')

    if use_ground_truth:
        odom_topic = '/Odometry_gazebo'
        cloud_topic = '/livox/points_world'
        sensor_pose_topic = '/go2/lidar_pose'
        localization_label = 'Gazebo ground truth'
        rviz_config = os.path.join(
            share('legbot_bringup'), 'rviz', 'scan_stairs_demo.rviz')
    else:
        odom_topic = '/fast_lio/odometry_base'
        cloud_topic = '/fast_lio/cloud_registered'
        sensor_pose_topic = '/fast_lio/odometry_lidar'
        localization_label = 'FAST-LIO'
        rviz_config = os.path.join(
            share('legbot_bringup'), 'rviz', 'scan_fastlio_flat.rviz')

    actions = [
        include('legbot_bringup', 'simulation.launch.py', {
            'gui': gui,
            'x': spawn_x,
            'y': spawn_y,
            'z': spawn_z,
            'yaw': spawn_yaw,
            'navigation_source': navigation_source,
            'publish_ground_truth': 'true' if use_ground_truth else 'false',
        }),
    ]
    if not use_ground_truth:
        actions.append(include('legbot_bringup', 'fastlio.launch.py', {
            'use_sim_time': use_sim_time,
            'config': os.path.join(
                share('legbot_bringup'), 'config', 'fastlio_sim.yaml'),
        }))

    # The normal six-terminal workflow starts this coordinator only after all
    # ros2_control spawners finish. Preserve that ordering in the all-in-one
    # launch so its first mode pulse is never lost.
    actions.extend([
        TimerAction(period=8.0, actions=[include(
            'legbot_bringup', 'go2_demo_control.launch.py', {
                'use_sim_time': use_sim_time,
                'odom_topic': odom_topic,
                'cloud_topic': cloud_topic,
                'localization_label': localization_label,
            })]),
        include('legbot_bringup', 'pct_plan.launch.py', {
            'use_sim_time': use_sim_time,
            'tomogram': 'building2_9',
            'path_topic': '/pct_path_raw',
            'frame_id': 'building_pct',
        }),
        include('legbot_bringup', 'scan.launch.py', {
            'use_sim_time': use_sim_time,
            'navi_mode': '3',
            'global_path_topic': '/pct_path',
            # pct_plan also declares a generic frame_id argument.  Pass this
            # explicitly so the include cannot leak building_pct into SCAN's
            # grid-map and marker headers.
            'frame_id': 'odom',
            'manual_goal_use_message_z': 'true',
            # The learned gait can settle about 0.54 m from the final goal
            # after the terminal slowdown. Keep the mission's arrival circle
            # beyond that repeatable stall while still requiring the top
            # floor via the separate Z gate.
            'finish_dist': '0.65',
            'odom_topic': odom_topic,
            'cloud_topic': cloud_topic,
            'sensor_pose_topic': sensor_pose_topic,
        }),
        Node(
            package='legbot_bringup',
            executable='pct_path_mission',
            name='pct_path_mission',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'input_topic': '/pct_path_raw',
                'output_topic': '/pct_path',
                'goal_topic': '/scan/goal',
                'odom_topic': odom_topic,
                'ready_topic': '/go2/demo_ready',
                'frame_id': 'odom',
                'pct_frame_id': 'building_pct',
                # Ground-truth odom is the Gazebo world frame, so its map TF
                # is published immediately below.  FAST-LIO still needs the
                # mission bridge's start-pose calibration.
                'publish_map_tf': not use_ground_truth,
                'building_x': 13.0,
                'building_y': 0.0,
                'building_z': 0.0,
                'building_yaw': 0.0,
                'spawn_x': ParameterValue(spawn_x, value_type=float),
                'spawn_y': ParameterValue(spawn_y, value_type=float),
                'spawn_z': ParameterValue(spawn_z, value_type=float),
                'spawn_yaw': ParameterValue(spawn_yaw, value_type=float),
            }],
        ),
        *([Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='building_pct_ground_truth_tf',
            output='screen',
            arguments=[
                '--x', '13.0', '--y', '0.0', '--z', '0.0',
                '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', 'odom', '--child-frame-id', 'building_pct',
            ],
        )] if use_ground_truth else []),
        include(
            'legbot_bringup', 'scan_rviz.launch.py',
            {'use_sim_time': use_sim_time, 'config': rviz_config},
            condition=IfCondition(rviz)),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'navigation_source', default_value='ground_truth',
            choices=['ground_truth', 'fastlio'],
            description=(
                'Ground truth is the reliable default for the long simulated cross-floor run; '
                'fastlio remains available for localization integration tests.')),
        DeclareLaunchArgument('spawn_x', default_value='-27.0'),
        DeclareLaunchArgument('spawn_y', default_value='6.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.50'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        OpaqueFunction(function=setup),
    ])
