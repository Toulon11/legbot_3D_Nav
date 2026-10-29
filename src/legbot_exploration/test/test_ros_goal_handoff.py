"""检查探索器 → 航点协调器 → SCAN 目标 → 停车回执的交接流程。

只能在独立的 ROS_DOMAIN_ID 下运行；测试不会启动 SCAN 或真实机器人。
唯一命令行参数是已经安装的探索器可执行文件的绝对路径。
"""
# 导入 把状态字典编码成 JSON 文本。
import json
# 导入 数学函数，如开方、向下取整和距离计算。
import math
# 导入 拼接安装后的启动文件路径。
import os
# 导入 处理 Ctrl+C 退出时的信号。
import signal
# 导入 subprocess 提供的类型或函数。
import subprocess
# 导入 sys 提供的类型或函数。
import sys
# 导入 取得不会因系统校时跳变的单调时间。
import time

# 导入 geometry_msgs.msg 提供的类型或函数。
from geometry_msgs.msg import PoseStamped
# 导入 nav_msgs.msg 提供的类型或函数。
from nav_msgs.msg import Odometry, Path
# 导入测试客户端使用的 ROS 2 Python 接口。
import rclpy
# 导入 rclpy.qos 提供的类型或函数。
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
# 导入 sensor_msgs.msg 提供的类型或函数。
from sensor_msgs.msg import PointCloud2
# 导入 sensor_msgs_py 提供的类型或函数。
from sensor_msgs_py import point_cloud2
# 导入 std_msgs.msg 提供的类型或函数。
from std_msgs.msg import Bool, Header, String


# 启动被测试的 ROS 子进程。
def launch_process(args):
    # 返回本函数计算得到的结果。
    return subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


# 结束测试中启动的 ROS 子进程。
def stop_process(child):
    # 条件检查：对应数据还没有取得或初始化；成立才进入下面的分支。
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
    # 先执行可能抛出异常的 ROS 数据读取。
    try:
        output, _ = child.communicate(timeout=4)
    # 捕获读取或服务调用失败，避免节点直接退出。
    except subprocess.TimeoutExpired:
        child.kill()
        output, _ = child.communicate()
    # 返回本函数计算得到的结果。
    return output


# 程序入口：初始化 ROS 或执行本文件的测试。
def main():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    config = os.path.join(root, 'config', 'exploration.yaml')
    coordinator_script = '/home/likai/legbot_3D_Nav/src/legbot_bringup/scripts/scan_waypoint_mission'
    coordinator = launch_process([
        sys.executable, coordinator_script, '--mode', 'direct',
        '--input-topic', '/scan/waypoint_input', '--ready-topic', '/go2/demo_ready',
        '--odom-topic', '/slam/global_odometry_base', '--frame-id', 'map', '--output',
        '/tmp/legbot_exploration_handoff_mission.json', '--dwell-seconds', '0.1'])
    explorer = launch_process([
        sys.argv[1], '--ros-args', '--params-file', config,
        '-p', 'use_sim_time:=false', '-p', 'observation_only:=false',
        '-p', 'min_goal_distance:=2.5', '-p', 'global_search:=false',
        '-p', 'startup_scan_enabled:=false'])
    rclpy.init()
    node = rclpy.create_node('exploration_handoff_test')
    sensor_pub = node.create_publisher(Odometry, '/slam/odometry_lidar', qos_profile_sensor_data)
    base_pub = node.create_publisher(Odometry, '/slam/global_odometry_base', qos_profile_sensor_data)
    cloud_pub = node.create_publisher(PointCloud2, '/slam/global_cloud_registered', qos_profile_sensor_data)
    ack_pub = node.create_publisher(PoseStamped, '/scan/goal_stop_ack', 10)
    # 将计算结果保存为 state_qos（让晚启动的订阅者也能收到最新状态的通信配置）。
    state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    ready_pub = node.create_publisher(Bool, '/go2/demo_ready', state_qos)
    goals, statuses, planned_paths = [], [], []
    node.create_subscription(PoseStamped, '/scan/goal', goals.append, 10)
    node.create_subscription(Path, '/exploration/planned_path', planned_paths.append,
                             state_qos)
    node.create_subscription(String, '/exploration/status',
                             # 把本次结果放入容器，供下一轮处理。
                             lambda msg: statuses.append(json.loads(msg.data)), state_qos)
    points = [(4.0 * math.cos(i * 2 * math.pi / 3600),
               4.0 * math.sin(i * 2 * math.pi / 3600), -0.61)
              # 依次处理集合中的每一项。
              for i in range(3600)]
    succeeded = False
    ack_start = None
    # 先执行可能抛出异常的 ROS 数据读取。
    try:
        deadline = time.monotonic() + 12.0
        # 反复处理待处理项目，直到循环条件不再成立。
        while time.monotonic() < deadline:
            if coordinator.poll() is not None or explorer.poll() is not None:
                # 找到满足条件的结果，退出当前循环。
                break
            stamp = node.get_clock().now().to_msg()
            sensor = Odometry()
            sensor.header = Header(stamp=stamp, frame_id='map')
            sensor.pose.pose.position.z = -0.01
            sensor.pose.pose.orientation.w = 1.0
            base = Odometry()
            base.header = Header(stamp=stamp, frame_id='map')
            base.pose.pose.position.z = -0.11
            base.pose.pose.orientation.w = 1.0
            cloud = point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id='map'), points)
            # 把更新后的 ROS 消息发布到相应话题。
            ready_pub.publish(Bool(data=True))
            # 把更新后的 ROS 消息发布到相应话题。
            sensor_pub.publish(sensor)
            # 把更新后的 ROS 消息发布到相应话题。
            base_pub.publish(base)
            # 把更新后的 ROS 消息发布到相应话题。
            cloud_pub.publish(cloud)
            if goals:
                assert goals[0].pose.position.z == 0.0, 'SCAN same-floor goal must have z=0'
                # 条件检查：对应数据还没有取得或初始化；成立才进入下面的分支。
                if ack_start is None or ack_start[0] != len(goals):
                    ack_start = (len(goals), time.monotonic())
                if time.monotonic() - ack_start[1] < 2.0:
                    # 把更新后的 ROS 消息发布到相应话题。
                    ack_pub.publish(goals[-1])
            rclpy.spin_once(node, timeout_sec=0.1)
            if (len(goals) >= 2 and
                    math.hypot(goals[0].pose.position.x - goals[1].pose.position.x,
                               goals[0].pose.position.y - goals[1].pose.position.y) > 0.3 and
                    any(len(path.poses) >= 3 for path in planned_paths) and
                    any(item.get('route_stage_index', 0) >= 1 for item in statuses)):
                succeeded = True
                # 找到满足条件的结果，退出当前循环。
                break
        if not succeeded:
            raise AssertionError('Staged route handoff did not advance; goals=%d, states=%s' %
                                 (len(goals), [(item['state'], item.get('detail'))
                                               for item in statuses[-12:]]))
        print('PASS', sys.argv[1], 'forwarded_goal=',
              (goals[0].pose.position.x, goals[0].pose.position.y))
    # 无论是否发生异常，最后都清理 ROS 资源。
    finally:
        node.destroy_node()
        rclpy.shutdown()
        explorer_output = stop_process(explorer)
        coordinator_output = stop_process(coordinator)
        if not succeeded:
            print('EXPLORER:\n', explorer_output, '\nCOORDINATOR:\n', coordinator_output,
                  file=sys.stderr)


if __name__ == '__main__':
    main()
