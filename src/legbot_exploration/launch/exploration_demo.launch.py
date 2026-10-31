"""按启动指南中的顺序启动仿真、自主探索相关节点。"""

# 导入 数学函数，如开方、向下取整和距离计算。
import math
# 导入 拼接安装后的启动文件路径。
import os

# 导入 ament_index_python.packages 提供的类型或函数。
from ament_index_python.packages import get_package_share_directory as share
# 导入 launch 提供的类型或函数。
from launch import LaunchDescription
# 导入 launch.actions 提供的类型或函数。
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
# 导入 launch.launch_description_sources 提供的类型或函数。
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 导入 launch.substitutions 提供的类型或函数。
from launch.substitutions import LaunchConfiguration


def include(package, filename, arguments=None):
    # 返回本函数计算得到的结果。
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share(package), 'launch', filename)),
        launch_arguments=(arguments or {}).items(),
    )


# 根据启动参数安排本次要启动的节点。
def setup(context):
    source = LaunchConfiguration('localization_source').perform(context)
    delay = float(LaunchConfiguration('control_start_delay').perform(context))
    # 延迟时间不能是 NaN、无穷大或负数，否则定时启动顺序失去意义。
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

    # 第一阶段：启动 Gazebo、传感器、控制器管理器和机器人。
    # 估计定位模式会关闭 Gazebo 的 odom→base TF，由定位器负责发布该坐标关系。
    actions = [include('legbot_bringup', 'simulation.launch.py', {
        'gui': LaunchConfiguration('gazebo_gui').perform(context),
        'world': world,
        'navigation_source': 'ground_truth' if truth else 'fastlio',
        'navigation_interface': 'global',
        'publish_ground_truth': 'true',
    })]

    # 第二阶段：只选一种定位和已配准点云来源；Gazebo 真值已由 simulation 启动。
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

    # 后续阶段：依次启动 GO2 控制、SCAN、可选 RViz、探索节点。
    # 固定间隔仅用于排列顺序；控制与探索仍会等实时数据和就绪信号。
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
                'robot_radius': LaunchConfiguration('scan_robot_radius').perform(context),
                # 探索路径含有近距离转折点；0.80 m 减速圈会让约 0.45 m 的路点
                # 从下发起就只有极小速度。仅覆盖本探索演示的启动参数。
                'goal_slowdown_distance': '0.55',
            })]),
    ])
    if LaunchConfiguration('start_rviz').perform(context) == 'true':
        actions.append(TimerAction(period=delay + 2.0, actions=[include(
            'legbot_bringup', 'scan_rviz.launch.py', {
                'use_sim_time': 'true',
                'config': os.path.join(bringup, 'rviz', 'scan_exploration_global.rviz'),
            })]))
    actions.append(TimerAction(period=delay + 3.0, actions=[include(
            'legbot_exploration', 'exploration.launch.py', {
                'use_sim_time': 'true',
                'observation_only': 'false',
                'start_waypoint_coordinator': 'true',
                'flat_safety': 'true' if flat_world else 'false',
                'planner_safety': LaunchConfiguration('planner_safety').perform(context),
                'planner_occupancy_radius': (
                    LaunchConfiguration('scan_robot_radius').perform(context)
                    if flat_world or LaunchConfiguration('planner_safety').perform(context) == 'true'
                    else '0.0'),
                'layer_map_in_process': LaunchConfiguration(
                    'layer_map_in_process').perform(context),
            })]))
    # 返回本函数计算得到的结果。
    return actions


# 声明 ros2 launch 可传入的参数与启动动作。
def generate_launch_description():
    # 返回本启动文件的参数及启动动作。
    return LaunchDescription([
        # 声明一个可由 ros2 launch 命令覆盖的启动参数。
        DeclareLaunchArgument(
            'localization_source', default_value='gazebo',
            choices=['gazebo', 'fastlio', 'fastlivo'],
            description='Gazebo truth, FAST-LIO2, or FAST-LIVO2.'),
        # 声明一个可由 ros2 launch 命令覆盖的启动参数。
        DeclareLaunchArgument(
            'control_start_delay', default_value='12.0',
            description='Seconds from launch start until GO2 high-level control starts.'),
        DeclareLaunchArgument('start_rviz', default_value='true',
                              choices=['true', 'false'],
                              description='Start the original SCAN RViz window.'),
        DeclareLaunchArgument('gazebo_gui', default_value='true',
                              choices=['true', 'false'],
                              description='Show the Gazebo GUI.'),
        DeclareLaunchArgument('planner_safety', default_value='false',
                              choices=['true', 'false'],
                              description='Require current SCAN occupancy and use wider clearance for exploration goals.'),
        DeclareLaunchArgument('layer_map_in_process', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('scan_robot_radius', default_value='0.20',
                              description='SCAN local obstacle inflation radius in metres.'),
        # 等参数解析后才运行 setup，选择要启动的节点。
        OpaqueFunction(function=setup),
    ])
