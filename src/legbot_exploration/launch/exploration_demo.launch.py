"""Start the simulation exploration workflow in guide order."""

import math
import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share(package), 'launch', filename)),
        launch_arguments=(arguments or {}).items(),
    )


def setup(context):
    source = LaunchConfiguration('localization_source').perform(context)
    delay = float(LaunchConfiguration('control_start_delay').perform(context))
    if not math.isfinite(delay) or delay < 0.0:
        raise RuntimeError('control_start_delay must be a finite, nonnegative number')

    truth = source == 'gazebo'
    odom = '/slam/global_odometry_base'
    cloud = '/slam/global_cloud_registered'
    sensor_pose = '/slam/odometry_lidar'
    label = {
        'gazebo': 'Gazebo ground truth',
        'fastlio': 'FAST-LIO2',
        'fastlivo': 'FAST-LIVO2',
    }[source]
    bringup = share('legbot_bringup')
    world = LaunchConfiguration('world', default=os.path.join(
        bringup, 'worlds', 'Building.sdf')).perform(context)
    flat_world = os.path.basename(
        world) == 'ExplorationFlat.sdf'

    # Terminal 1: Gazebo, sensors, controller manager and the robot.  In global
    # interface mode the non-truth branch does not select a SLAM algorithm; it
    # only disables Gazebo's odom -> base TF so the estimator owns that frame.
    actions = [include('legbot_bringup', 'simulation.launch.py', {
        'gui': 'true',
        'world': world,
        'navigation_source': 'ground_truth' if truth else 'fastlio',
        'navigation_interface': 'global',
        'publish_ground_truth': 'true',
    })]

    # Terminal 2: one localization/registered-cloud source.  Gazebo truth is
    # already provided by simulation.launch.py and needs no second process.
    if source == 'fastlio':
        actions.append(include('legbot_bringup', 'fastlio.launch.py', {
            'use_sim_time': 'true',
            'publish_global': 'true',
            'config': os.path.join(bringup, 'config', 'fastlio_sim.yaml'),
        }))
    elif source == 'fastlivo':
        actions.append(include('fast_livo', 'legbot_sim_livo.launch.py', {
            'use_sim_time': 'true',
            'publish_global': 'true',
            'use_rviz': 'false',
            'config': os.path.join(share('fast_livo'), 'config', 'legbot_sim.yaml'),
        }))

    # Terminals 3-6.  The fixed spacing keeps launch order readable; the GO2
    # sequencer and explorer still wait for live data and /go2/demo_ready.
    actions.extend([
        TimerAction(period=delay, actions=[include(
            'legbot_bringup', 'go2_demo_control.launch.py', {
                'use_sim_time': 'true',
                'odom_topic': odom,
                'cloud_topic': cloud,
                'localization_label': label,
            })]),
        TimerAction(period=delay + 1.0, actions=[include(
            'legbot_bringup', 'scan.launch.py', {
                'use_sim_time': 'true',
                'navi_mode': '1',
                'frame_id': 'map',
                'odom_topic': odom,
                'cloud_topic': cloud,
                'sensor_pose_topic': sensor_pose,
                'manual_goal_use_message_z': 'false',
            })]),
        TimerAction(period=delay + 2.0, actions=[include(
            'legbot_bringup', 'scan_rviz.launch.py', {
                'use_sim_time': 'true',
                'config': os.path.join(bringup, 'rviz', 'scan_exploration_global.rviz'),
            })]),
        TimerAction(period=delay + 3.0, actions=[include(
            'legbot_exploration', 'exploration.launch.py', {
                'localization_source': source,
                'impl': 'cpp',
                'use_sim_time': 'true',
                'observation_only': 'false',
                'start_waypoint_coordinator': 'true',
                'flat_safety': 'true' if flat_world else 'false',
            })]),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'localization_source', default_value='gazebo',
            choices=['gazebo', 'fastlio', 'fastlivo'],
            description='Gazebo truth, FAST-LIO2, or FAST-LIVO2.'),
        DeclareLaunchArgument(
            'control_start_delay', default_value='12.0',
            description='Seconds from launch start until GO2 high-level control starts.'),
        OpaqueFunction(function=setup),
    ])
