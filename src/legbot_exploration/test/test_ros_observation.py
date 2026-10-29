"""ROS 独立冒烟测试：输入带时间戳的点云和里程计，检查地图是否更新。

用法：python3 test/test_ros_observation.py /探索器可执行文件的绝对路径
运行前要设置独立的 ROS_DOMAIN_ID，并 source 对应的 install 环境。
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

# 导入 nav_msgs.msg 提供的类型或函数。
from nav_msgs.msg import Odometry
# 导入测试客户端使用的 ROS 2 Python 接口。
import rclpy
# 导入 rclpy.qos 提供的类型或函数。
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
# 导入 sensor_msgs.msg 提供的类型或函数。
from sensor_msgs.msg import PointCloud2
# 导入 sensor_msgs_py 提供的类型或函数。
from sensor_msgs_py import point_cloud2
# 导入 std_msgs.msg 提供的类型或函数。
from std_msgs.msg import Header, String


# 程序入口：初始化 ROS 或执行本文件的测试。
def main():
    executable = sys.argv[1]
    config = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config', 'exploration.yaml'))
    child = subprocess.Popen(
        [executable, '--ros-args', '--params-file', config, '-p', 'use_sim_time:=false',
         '-p', 'observation_only:=true'], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = rclpy.create_node('exploration_smoke_test')
    status = []
    sensor_pub = node.create_publisher(Odometry, '/slam/odometry_lidar', qos_profile_sensor_data)
    base_pub = node.create_publisher(Odometry, '/slam/global_odometry_base', qos_profile_sensor_data)
    cloud_pub = node.create_publisher(PointCloud2, '/slam/global_cloud_registered', qos_profile_sensor_data)
    # 将计算结果保存为 state_qos（让晚启动的订阅者也能收到最新状态的通信配置）。
    state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/exploration/status',
                             # 把本次结果放入容器，供下一轮处理。
                             lambda msg: status.append(json.loads(msg.data)), state_qos)
    points = [(4.0 * math.cos(i * math.pi / 100 - math.pi / 2),
               4.0 * math.sin(i * math.pi / 100 - math.pi / 2), -0.2)
              # 依次处理集合中的每一项。
              for i in range(101)]
    succeeded = False
    # 先执行可能抛出异常的 ROS 数据读取。
    try:
        deadline = time.monotonic() + 12.0
        # 反复处理待处理项目，直到循环条件不再成立。
        while time.monotonic() < deadline:
            if child.poll() is not None:
                # 找到满足条件的结果，退出当前循环。
                break
            stamp = node.get_clock().now().to_msg()
            sensor = Odometry()
            sensor.header.frame_id = 'map'
            sensor.header.stamp = stamp
            sensor.pose.pose.position.z = 0.4
            sensor.pose.pose.orientation.w = 1.0
            base = Odometry()
            base.header.frame_id = 'map'
            base.header.stamp = stamp
            base.pose.pose.position.z = 0.3
            base.pose.pose.orientation.w = 1.0
            cloud = point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id='map'), points)
            # 故意允许通信层改变这三个话题的到达顺序，验证节点的时间同步逻辑。
            # 把更新后的 ROS 消息发布到相应话题。
            cloud_pub.publish(cloud)
            # 把更新后的 ROS 消息发布到相应话题。
            sensor_pub.publish(sensor)
            # 把更新后的 ROS 消息发布到相应话题。
            base_pub.publish(base)
            rclpy.spin_once(node, timeout_sec=0.1)
            if any(item.get('map_version', 0) > 0 and item.get('voxel_count', 0) > 100
                   # 依次处理集合中的每一项。
                   for item in status):
                succeeded = True
                # 找到满足条件的结果，退出当前循环。
                break
        if not succeeded:
            raise AssertionError('No cumulative map update; last status: %r' % (status[-3:],))
        print('PASS', executable, status[-1])
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
            # 向终端显示本次运行的信息。
            print(output, file=sys.stderr)


if __name__ == '__main__':
    main()
