"""回归测试：零散前沿格不能错误地触发轨迹回退。

运行时需要独立的 ROS_DOMAIN_ID 和已安装探索器的可执行文件路径。
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


# 程序入口：初始化 ROS 或执行本文件的测试。
def main():
    executable = sys.argv[1]
    expect_residual_goal = '--expect-residual-goal' in sys.argv[2:]
    config = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config',
                                          'exploration.yaml'))
    child = subprocess.Popen([
        executable, '--ros-args', '--params-file', config,
        '-p', 'use_sim_time:=false', '-p', 'observation_only:=false',
        '-p', 'global_search:=true', '-p', 'min_frontier_cells:=100000',
        '-p', 'min_goal_distance:=1.0' if expect_residual_goal else
              'min_goal_distance:=20.0',
        '-p', 'exploration_min_x:=-5.0', '-p', 'exploration_max_x:=5.0',
        '-p', 'exploration_min_y:=-5.0', '-p', 'exploration_max_y:=5.0',
    ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = rclpy.create_node('residual_frontier_regression')
    status, targets, traveled = [], [], []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/exploration/status',
                             # 把本次结果放入容器，供下一轮处理。
                             lambda m: status.append(json.loads(m.data)), qos)
    node.create_subscription(PoseStamped, '/scan/waypoint_input', targets.append, 10)
    node.create_subscription(Path, '/exploration/trajectory',
                             # 把本次结果放入容器，供下一轮处理。
                             lambda m: traveled.append(max(
                                 (p.pose.position.x for p in m.poses), default=0.0)), qos)
    ready_pub = node.create_publisher(Bool, '/go2/demo_ready', qos)
    base_pub = node.create_publisher(Odometry, '/slam/global_odometry_base', qos_profile_sensor_data)
    sensor_pub = node.create_publisher(Odometry, '/slam/odometry_lidar', qos_profile_sensor_data)
    cloud_pub = node.create_publisher(PointCloud2, '/slam/global_cloud_registered', qos_profile_sensor_data)
    points = [(4.0 * math.cos(i * math.pi / 180),
               4.0 * math.sin(i * math.pi / 180), -0.2) for i in range(360)]
    succeeded = False
    # 先执行可能抛出异常的 ROS 数据读取。
    try:
        start = time.monotonic()
        # 反复处理待处理项目，直到循环条件不再成立。
        while time.monotonic() - start < 11.0:
            if child.poll() is not None:
                # 找到满足条件的结果，退出当前循环。
                break
            elapsed = time.monotonic() - start
            x = 0.0 if elapsed < 1.5 or elapsed >= 3.0 else 2.5
            stamp = node.get_clock().now().to_msg()
            base = Odometry()
            base.header = Header(stamp=stamp, frame_id='map')
            base.pose.pose.position.x = x
            base.pose.pose.position.z = 0.3
            base.pose.pose.orientation.w = 1.0
            sensor = Odometry()
            sensor.header = base.header
            sensor.pose.pose.position.x = x
            sensor.pose.pose.position.z = 0.4
            sensor.pose.pose.orientation.w = 1.0
            # 把更新后的 ROS 消息发布到相应话题。
            ready_pub.publish(Bool(data=True))
            # 把更新后的 ROS 消息发布到相应话题。
            base_pub.publish(base)
            # 把更新后的 ROS 消息发布到相应话题。
            sensor_pub.publish(sensor)
            # 把更新后的 ROS 消息发布到相应话题。
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id='map'), points))
            rclpy.spin_once(node, timeout_sec=0.1)
        residual = [s for s in status if s.get('state') == 'frontiers_below_cluster_threshold'
                    and s.get('frontier_cells', 0) > 0 and s.get('frontier_clusters') == 0]
        assert child.poll() is None, 'explorer exited during test'
        assert max(traveled, default=0.0) >= 2.0, 'historical path was not built'
        assert residual, 'no residual-frontier state; latest status: %r' % status[-3:]
        assert not any(s.get('state') == 'exploration_complete' for s in status), (
            'filtered frontiers must not be treated as completed exploration')
        assert not any(s.get('detail') == 'recovery_backtrack' for s in status), status[-3:]
        if expect_residual_goal:
            assert targets, 'single-cell residual fallback did not select a goal'
            assert any(s.get('detail') == 'residual_frontier' for s in status), status[-3:]
        # 上述条件不成立时，改走这个分支。
        else:
            assert not targets, 'unreachable residual frontiers triggered %d recovery target(s)' % len(targets)
        succeeded = True
        print('PASS', executable, 'residual_frontiers=', residual[-1]['frontier_cells'])
    # 无论是否发生异常，最后都清理 ROS 资源。
    finally:
        node.destroy_node()
        rclpy.shutdown()
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
        if not succeeded:
            print('STATUS:', status[-5:], 'TARGETS:', len(targets), '\nEXPLORER:\n', output,
                  file=sys.stderr)


if __name__ == '__main__':
    main()
