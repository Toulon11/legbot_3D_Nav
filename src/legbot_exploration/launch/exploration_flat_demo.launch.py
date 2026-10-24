"""在单层平面 Gazebo 场景中启动整套自主探索流程。"""

# 导入 拼接安装后的启动文件路径。
import os

# 导入 ament_index_python.packages 提供的类型或函数。
from ament_index_python.packages import get_package_share_directory as share
# 导入 launch 提供的类型或函数。
from launch import LaunchDescription
# 导入 launch.actions 提供的类型或函数。
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# 导入 launch.launch_description_sources 提供的类型或函数。
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 导入 launch.substitutions 提供的类型或函数。
from launch.substitutions import LaunchConfiguration


# 声明 ros2 launch 可传入的参数与启动动作。
def generate_launch_description():
    flat_world = os.path.join(share('legbot_bringup'), 'worlds', 'ExplorationFlat.sdf')
    exploration_demo = os.path.join(
        share('legbot_exploration'), 'launch', 'exploration_demo.launch.py')
    # 返回本函数计算得到的结果。
    return LaunchDescription([
        # 声明一个可由 ros2 launch 命令覆盖的启动参数。
        DeclareLaunchArgument(
            'localization_source', default_value='gazebo',
            choices=['gazebo', 'fastlio', 'fastlivo']),
        DeclareLaunchArgument('control_start_delay', default_value='12.0'),
        # 把另一个功能包的启动文件纳入本次启动。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(exploration_demo),
            launch_arguments={
                'world': flat_world,
                'localization_source': LaunchConfiguration('localization_source'),
                'control_start_delay': LaunchConfiguration('control_start_delay'),
            }.items()),
    ])
