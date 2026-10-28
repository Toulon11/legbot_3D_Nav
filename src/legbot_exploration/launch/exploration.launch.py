"""启动一个探索节点，并可选同时启动 SCAN 直达航点协调器。"""

# 导入 拼接安装后的启动文件路径。
import os

# 导入 ament_index_python.packages 提供的类型或函数。
from ament_index_python.packages import get_package_share_directory as share
# 导入 launch 提供的类型或函数。
from launch import LaunchDescription
# 导入 launch.actions 提供的类型或函数。
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
# 导入 launch.launch_description_sources 提供的类型或函数。
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 导入 launch.substitutions 提供的类型或函数。
from launch.substitutions import LaunchConfiguration
# 导入 launch_ros.actions 提供的类型或函数。
from launch_ros.actions import Node
# 导入 launch_ros.parameter_descriptions 提供的类型或函数。
from launch_ros.parameter_descriptions import ParameterValue


# 根据启动参数安排本次要启动的节点。
def setup(context):
    source = LaunchConfiguration('localization_source').perform(context)
    impl = LaunchConfiguration('impl').perform(context)
    odom = '/slam/global_odometry_base'
    cloud = '/slam/global_cloud_registered'
    sensor_pose = '/slam/odometry_lidar'
    use_sim_time = LaunchConfiguration('use_sim_time')
    flat_safety = LaunchConfiguration('flat_safety').perform(context) == 'true'
    planner_safety = LaunchConfiguration('planner_safety').perform(context) == 'true'

    # 将计算结果保存为 actions（启动文件返回的启动动作列表）。
    actions = []
    if LaunchConfiguration('start_waypoint_coordinator').perform(context) == 'true':
        # 把本次结果放入容器，供下一轮处理。
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share('legbot_bringup'), 'launch', 'scan_waypoints.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'mode': 'direct',
                'input_topic': '/scan/waypoint_input',
                'odom_topic': odom,
                'frame_id': 'map',
                'ready_topic': '/go2/demo_ready',
            }.items()))
    # 把本次结果放入容器，供下一轮处理。
    actions.append(Node(
        package='legbot_exploration',
        executable='explorer_cpp' if impl == 'cpp' else 'explorer_py',
        name='legbot_explorer',
        output='screen',
        parameters=[
            os.path.join(share('legbot_exploration'), 'config', 'exploration.yaml'),
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'observation_only': ParameterValue(
                    LaunchConfiguration('observation_only'), value_type=bool),
                'base_odom_topic': odom,
                'sensor_odom_topic': sensor_pose,
                'cloud_topic': cloud,
                'frame_id': 'map',
                'planner_occupancy_radius': float(
                    LaunchConfiguration('planner_occupancy_radius').perform(context)),
                **({'robot_radius': 0.50,
                    'max_goal_distance': 4.0,
                    'global_search': True,
                    'bounds_relative_to_start': True,
                    'exploration_min_x': -4.5,
                    'exploration_max_x': 26.5,
                    'exploration_min_y': -21.5,
                    'exploration_max_y': 9.5,
                    'require_planner_occupancy': True} if flat_safety else {}),
                **({'robot_radius': 0.50,
                    'max_goal_distance': 4.0,
                    'require_planner_occupancy': True}
                   if planner_safety and not flat_safety else {}),
            },
        ],
    ))
    # 返回本函数计算得到的结果。
    return actions


# 声明 ros2 launch 可传入的参数与启动动作。
def generate_launch_description():
    # 返回本启动文件的参数及启动动作。
    return LaunchDescription([
        DeclareLaunchArgument('localization_source', default_value='fastlio',
                              choices=['gazebo', 'fastlio', 'fastlivo']),
        DeclareLaunchArgument('impl', default_value='cpp', choices=['cpp', 'python']),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('observation_only', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_waypoint_coordinator', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('flat_safety', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('planner_safety', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('planner_occupancy_radius', default_value='0.0'),
        # 等参数解析后才运行 setup，选择要启动的节点。
        OpaqueFunction(function=setup),
    ])
