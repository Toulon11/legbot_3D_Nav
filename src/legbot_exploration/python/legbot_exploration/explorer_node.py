"""Python 版 ROS 2 探索节点：把核心选点算法接到点云、位姿与 SCAN。"""
# 导入 collections 提供的类型或函数。
from collections import deque
# 导入 replace，用于复制配置并临时调整最小前沿簇规模。
from dataclasses import replace
# 导入 把状态字典编码成 JSON 文本。
import json
# 导入 数学函数，如开方、向下取整和距离计算。
import math
# 导入 处理 Ctrl+C 退出时的信号。
import signal
# 导入 取得不会因系统校时跳变的单调时间。
import time

# 导入 geometry_msgs.msg 提供的类型或函数。
from geometry_msgs.msg import PoseStamped
# 导入 nav_msgs.msg 提供的类型或函数。
from nav_msgs.msg import OccupancyGrid, Odometry, Path
# 导入 Python 版 ROS 2 节点接口。
import rclpy
# 导入 rclpy.node 提供的类型或函数。
from rclpy.node import Node
# 导入 rclpy.qos 提供的类型或函数。
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
# 导入 sensor_msgs.msg 提供的类型或函数。
from sensor_msgs.msg import PointCloud2
# 导入 sensor_msgs_py 提供的类型或函数。
from sensor_msgs_py import point_cloud2
# 导入 std_msgs.msg 提供的类型或函数。
from std_msgs.msg import Bool, String
# 导入 std_srvs.srv 提供的类型或函数。
from std_srvs.srv import SetBool, Trigger
# 导入 visualization_msgs.msg 提供的类型或函数。
from visualization_msgs.msg import Marker, MarkerArray

# 导入 .exploration_core 提供的类型或函数。
from .exploration_core import (GoalProgressGate, RecoveryGate, Settings, VoxelMap,
                               assess_reachable_coverage, select_frontier, select_observation_goal)


# 把 ROS 秒和纳秒组合成一个整数时间戳。
def stamp_ns(stamp):
    # 返回本函数计算得到的结果。
    return stamp.sec * 1_000_000_000 + stamp.nanosec


# 定义 ROS 2 探索节点。
class Explorer(Node):
    # 创建对象并为后续运行准备数据。
    def __init__(self):
        # 先初始化 ROS 2 父类 Node，并把节点命名为 legbot_explorer。
        super().__init__('legbot_explorer')
        # 先建立一份算法默认参数，后面再用 YAML/启动参数覆盖。
        cfg = Settings()
        # 依次处理算法配置。
        for name in cfg.__dataclass_fields__:
            # 按字段名把 ROS 参数写回 cfg，使核心算法用到实际配置。
            setattr(cfg, name, self.declare_parameter(name, getattr(cfg, name)).value)
        # 更新节点状态 cfg，供后续回调或主循环读取。
        self.cfg = cfg
        # 更新节点状态 map，供后续回调或主循环读取。
        self.map = VoxelMap(cfg)
        # 依次处理集合中的每一项。
        for name, default in (
            ('frame_id', 'map'), ('cloud_topic', '/slam/global_cloud_registered'),
            ('sensor_odom_topic', '/slam/odometry_lidar'),
            ('base_odom_topic', '/slam/global_odometry_base'),
            ('planner_occupancy_topic', '/local_planner/grid_map/occupancy_inflate'),
            ('waypoint_topic', '/scan/waypoint_input'), ('scan_goal_topic', '/scan/goal'),
            ('ready_topic', '/go2/demo_ready'), ('point_stride', 8),
            ('max_points_per_cloud', 1000), ('sensor_sync_tolerance', 0.03),
            ('odom_timeout', 1.0), ('cloud_timeout', 2.0),
            ('goal_reach_xy', 0.55), ('progress_distance', 0.35),
            ('progress_timeout', 12.0), ('goal_timeout', 90.0),
            ('goal_forward_timeout', 3.0), ('failed_frontier_cooldown', 45.0),
            ('no_frontier_hold', 8.0), ('recovery_min_new_cells', 8),
            ('planner_occupancy_timeout', 2.0),
            ('planner_occupancy_height_tolerance', 0.5),
            ('require_planner_occupancy', False),
            ('near_goal_dwell', 1.5), ('blocked_goal_dwell', 2.0),
            ('observation_only', True), ('bounds_relative_to_start', False)):
            # 把运行时参数保存到节点对象的同名字段。
            setattr(self, name, self.declare_parameter(name, default).value)
        # 条件检查：按间隔对点云抽样；成立才进入下面的分支。
        if (cfg.resolution <= 0 or cfg.local_search_radius <= 0 or self.point_stride < 1
                or self.max_points_per_cloud < 1 or cfg.max_voxels < 1 or
                self.recovery_min_new_cells < 1):
            # 关键数值若非正数，就在启动阶段报错，避免之后除零或空循环。
            raise ValueError('resolution, search radius, stride, max points, max voxels and recovery gain must be positive')
        # 条件检查：当前启用全图搜索；成立才进入下面的分支。
        if (cfg.global_search and (cfg.exploration_min_x >= cfg.exploration_max_x or
                                   cfg.exploration_min_y >= cfg.exploration_max_y)):
            # 全局边界的最小值必须小于最大值，否则区域面积无效。
            raise ValueError('global exploration bounds must have positive area')
        # 更新节点状态 sensor_poses，供后续回调或主循环读取。
        self.sensor_poses = deque(maxlen=100)
        # 更新节点状态 base_poses，供后续回调或主循环读取。
        self.base_poses = deque(maxlen=100)
        # 更新节点状态 pending_clouds，供后续回调或主循环读取。
        self.pending_clouds = deque(maxlen=5)
        # 更新节点状态 external_obstacles，供后续回调或主循环读取。
        self.external_obstacles = []
        # 更新节点状态 occupancy_wall，供后续回调或主循环读取。
        self.occupancy_wall = 0.0
        # 更新节点状态 base，供后续回调或主循环读取。
        self.base = None
        # 更新节点状态 base_wall，供后续回调或主循环读取。
        self.base_wall = 0.0
        # 更新节点状态 cloud_wall，供后续回调或主循环读取。
        self.cloud_wall = 0.0
        # 更新节点状态 ready，供后续回调或主循环读取。
        self.ready = False
        # 更新节点状态 paused，供后续回调或主循环读取。
        self.paused = False
        # 更新节点状态 active，供后续回调或主循环读取。
        self.active = None
        # 更新节点状态 clearing，供后续回调或主循环读取。
        self.clearing = False
        # 空字符串表示目前没有正在记录的清除原因。
        self.clearing_reason = ''
        # 更新节点状态 last_clear_wall，供后续回调或主循环读取。
        self.last_clear_wall = 0.0
        # 更新节点状态 clear_future，供后续回调或主循环读取。
        self.clear_future = None
        # 更新节点状态 suppressed，供后续回调或主循环读取。
        self.suppressed = []
        # 更新节点状态 recovery_gate，供后续回调或主循环读取。
        self.recovery_gate = RecoveryGate(self.recovery_min_new_cells)
        # 更新节点状态 goal_progress_gate，供后续回调或主循环读取。
        self.goal_progress_gate = GoalProgressGate(self.recovery_min_new_cells)
        # 更新节点状态 empty_since，供后续回调或主循环读取。
        self.empty_since = None
        # 更新节点状态 residual_since，供后续回调或主循环读取。
        self.residual_since = None
        # 保存上次 JSON 文本，状态没变化时不重复发布。
        self.last_status = ''
        # 更新节点状态 last_marker_version，供后续回调或主循环读取。
        self.last_marker_version = -1
        # 更新节点状态 last_frontier_cells，供后续回调或主循环读取。
        self.last_frontier_cells = 0
        # 更新节点状态 last_frontier_clusters，供后续回调或主循环读取。
        self.last_frontier_clusters = 0
        # 更新节点状态 last_selection_diagnostics，供后续回调或主循环读取。
        self.last_selection_diagnostics = {}
        self.coverage = None
        self.unresolved_unknown_cells = 0
        # 更新节点状态 bounds_initialized，供后续回调或主循环读取。
        self.bounds_initialized = False
        # 更新节点状态 trajectory，供后续回调或主循环读取。
        self.trajectory = Path()
        # 给行走轨迹指定与定位消息相同的地图坐标系。
        self.trajectory.header.frame_id = self.frame_id
        # 将计算结果保存为 state_qos（让晚启动的订阅者也能收到最新状态的通信配置）。
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 建立 ROS 发布者，用来向指定话题发送消息。
        self.waypoint_pub = self.create_publisher(PoseStamped, self.waypoint_topic, 10)
        # 建立 RViz 前沿可视化标记的发布者。
        self.marker_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', state_qos)
        # 建立探索状态 JSON 字符串的发布者。
        self.status_pub = self.create_publisher(String, '/exploration/status', state_qos)
        self.coverage_pub = self.create_publisher(
            OccupancyGrid, '/exploration/coverage_grid', state_qos)
        # 建立机器狗历史路径的发布者。
        self.trajectory_pub = self.create_publisher(Path, '/exploration/trajectory', state_qos)
        # 订阅一个 ROS 话题，收到数据时调用对应回调。
        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, qos_profile_sensor_data)
        # 订阅一个 ROS 话题，收到数据时调用对应回调。
        self.create_subscription(Odometry, self.sensor_odom_topic, self.on_sensor, qos_profile_sensor_data)
        # 订阅一个 ROS 话题，收到数据时调用对应回调。
        self.create_subscription(Odometry, self.base_odom_topic, self.on_base, qos_profile_sensor_data)
        # 只有配置了规划器障碍话题才订阅它。
        if self.planner_occupancy_topic:
            # 订阅一个 ROS 话题，收到数据时调用对应回调。
            self.create_subscription(PointCloud2, self.planner_occupancy_topic,
                                     self.on_occupancy, qos_profile_sensor_data)
        # 订阅一个 ROS 话题，收到数据时调用对应回调。
        self.create_subscription(Bool, self.ready_topic, self.on_ready, state_qos)
        # 订阅一个 ROS 话题，收到数据时调用对应回调。
        self.create_subscription(PoseStamped, self.waypoint_topic, self.on_input, 10)
        # 订阅一个 ROS 话题，收到数据时调用对应回调。
        self.create_subscription(PoseStamped, self.scan_goal_topic, self.on_goal, 10)
        # 订阅 SCAN 停车回执，用来确认目标确实停止。
        self.create_subscription(PoseStamped, '/scan/goal_stop_ack', self.on_ack, 10)
        # 订阅 SCAN 完成信号，与停车回执一起判定目标结束。
        self.create_subscription(Bool, '/scan/mission_complete', self.on_complete, state_qos)
        # 对外提供暂停或恢复探索的 SetBool 服务。
        self.pause_srv = self.create_service(SetBool, '/exploration/set_paused', self.on_pause)
        # 准备调用 SCAN 的清空航点服务。
        self.clear_client = self.create_client(Trigger, '/scan/waypoints/clear')
        # 建立周期计时器，定期执行探索状态机。
        self.create_timer(1.0, self.tick)
        # 只观察模式只报告选出的点，不让机器狗移动。
        self.report('observation_only' if self.observation_only else 'waiting_for_data')

    # 把当前探索状态发布为 JSON 文本。
    def report(self, state, **extra):
        # 把状态名称、地图版本、前沿计数等组成一个字典。
        data = {'state': state, 'implementation': 'python', 'map_version': self.map.version,
                'voxel_count': len(self.map.voxels), 'paused': self.paused,
                'observation_only': self.observation_only,
                'global_search': self.cfg.global_search,
                'frontier_cells': self.last_frontier_cells,
                'frontier_clusters': self.last_frontier_clusters}
        # 把最近一次选点的诊断计数并入状态字典。
        data.update(self.last_selection_diagnostics)
        if self.coverage is not None:
            data.update(self.coverage)
        data['coverage_valid'] = self.coverage is not None
        data['unresolved_unknown_cells'] = self.unresolved_unknown_cells
        # 补入本次调用传来的目标坐标或失败原因等额外信息。
        data.update(extra)
        # 将计算结果保存为 encoded（序列化成 JSON 的状态消息）。
        encoded = json.dumps(data, sort_keys=True)
        # 只有状态内容变化才发布，避免持续刷屏。
        if encoded != self.last_status:
            # 把更新后的 ROS 消息发布到相应话题。
            self.status_pub.publish(String(data=encoded))
            # 更新节点状态 last_status，供后续回调或主循环读取。
            self.last_status = encoded

    def publish_coverage_grid(self, grid):
        """发布与完成判定完全相同的全局网格，供 RViz 与独立验收读取。"""
        if self.coverage is None:
            return
        r = self.cfg.resolution
        x0 = math.ceil(self.cfg.exploration_min_x / r - .5)
        x1 = math.floor(self.cfg.exploration_max_x / r - .5)
        y0 = math.ceil(self.cfg.exploration_min_y / r - .5)
        y1 = math.floor(self.cfg.exploration_max_y / r - .5)
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = r
        msg.info.width = x1 - x0 + 1
        msg.info.height = y1 - y0 + 1
        msg.info.origin.position.x = x0 * r
        msg.info.origin.position.y = y0 * r
        msg.info.origin.orientation.w = 1.0
        msg.data = [(-1 if (x, y) not in grid else 0 if grid[(x, y)] == 0 else 100)
                    for y in range(y0, y1 + 1) for x in range(x0, x1 + 1)]
        self.coverage_pub.publish(msg)

    # 接收雷达位姿并尝试处理排队的点云。
    def on_sensor(self, msg):
        # 条件检查：消息坐标系是否等于地图坐标系；成立才进入下面的分支。
        if msg.header.frame_id != self.frame_id:
            # 结束当前函数，不再执行后面的语句。
            return
        # 将计算结果保存为 p（消息中的三维位置）。
        p = msg.pose.pose.position
        # 条件检查：数据必须是有效有限数；成立才进入下面的分支。
        if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            # 把本次结果放入容器，供下一轮处理。
            self.sensor_poses.append((stamp_ns(msg.header.stamp), (p.x, p.y, p.z)))
            # 新位姿或点云一到，就尝试处理此前等待时间同步的点云。
            self.process_clouds()

    # 接收机体位姿、更新轨迹并尝试处理点云。
    def on_base(self, msg):
        # 条件检查：消息坐标系是否等于地图坐标系；成立才进入下面的分支。
        if msg.header.frame_id != self.frame_id:
            # 结束当前函数，不再执行后面的语句。
            return
        # 将计算结果保存为 p（消息中的三维位置）。
        p = msg.pose.pose.position
        # 条件检查：数据必须是有效有限数；成立才进入下面的分支。
        if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            # 条件检查：当前启用全图搜索；成立才进入下面的分支。
            if self.cfg.global_search and self.bounds_relative_to_start and not self.bounds_initialized:
                # 将 X 下界从相对起点偏移换算成全局 X 坐标。
                self.cfg.exploration_min_x += p.x
                # 将 X 上界从相对起点偏移换算成全局 X 坐标。
                self.cfg.exploration_max_x += p.x
                # 将 Y 下界从相对起点偏移换算成全局 Y 坐标。
                self.cfg.exploration_min_y += p.y
                # 将 Y 上界从相对起点偏移换算成全局 Y 坐标。
                self.cfg.exploration_max_y += p.y
                # 更新节点状态 bounds_initialized，供后续回调或主循环读取。
                self.bounds_initialized = True
            # 更新节点状态 base，供后续回调或主循环读取。
            self.base = (p.x, p.y, p.z)
            # 把本次结果放入容器，供下一轮处理。
            self.base_poses.append((stamp_ns(msg.header.stamp), self.base))
            # 更新节点状态 base_wall，供后续回调或主循环读取。
            self.base_wall = time.monotonic()
            # 轨迹为空或机体至少移动 0.1 米，才记录新轨迹点。
            if (not self.trajectory.poses or
                    math.dist(self.base[:2], (
                        self.trajectory.poses[-1].pose.position.x,
                        self.trajectory.poses[-1].pose.position.y)) >= 0.1):
                # 将计算结果保存为 pose（一个带位姿的消息）。
                pose = PoseStamped()
                # 给新轨迹点复制本次里程计时间戳和坐标系。
                pose.header = msg.header
                # 给新轨迹点复制本次机体位姿。
                pose.pose = msg.pose.pose
                # 把本次结果放入容器，供下一轮处理。
                self.trajectory.poses.append(pose)
                # 限制轨迹点数，避免运行太久后内存无限增长。
                if len(self.trajectory.poses) > 20000:
                    # 从容器中取走已处理或失效的数据。
                    self.trajectory.poses.pop(0)
            # 新位姿或点云一到，就尝试处理此前等待时间同步的点云。
            self.process_clouds()

    # 接收 SCAN 规划器提供的障碍点。
    def on_occupancy(self, msg):
        # 条件检查：对应数据还没有取得或初始化；成立才进入下面的分支。
        if msg.header.frame_id != self.frame_id or self.base is None:
            # 结束当前函数，不再执行后面的语句。
            return
        # 点云字段或数据格式可能有误；出错时记录告警，不让节点崩溃。
        try:
            # 更新节点状态 external_obstacles，供后续回调或主循环读取。
            self.external_obstacles = [
                (float(p[0]), float(p[1]))
                for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
                if abs(float(p[2]) - self.base[2]) <= self.planner_occupancy_height_tolerance]
        # 捕获读取或服务调用失败，避免节点直接退出。
        except (ValueError, KeyError) as exc:
            # SCAN 障碍点云格式异常时记录告警并忽略这一帧。
            self.get_logger().warning('Invalid SCAN occupancy cloud: %s' % exc)
            # 结束当前函数，不再执行后面的语句。
            return
        # 更新节点状态 occupancy_wall，供后续回调或主循环读取。
        self.occupancy_wall = time.monotonic()

    # 判断 SCAN 障碍点云是否仍在有效时限内。
    def occupancy_fresh(self, now):
        # 返回SCAN 障碍数据是否仍有效。
        return self.occupancy_wall > 0 and now - self.occupancy_wall <= self.planner_occupancy_timeout

    # 判断给定目标附近是否存在规划器障碍。
    def planner_blocks(self, xy):
        # 返回目标是否被规划器障碍占据。
        return any(math.dist(xy, p) <= self.cfg.robot_radius for p in self.external_obstacles)

    # 接收一帧已配准点云并排队处理。
    def on_cloud(self, msg):
        # 条件检查：消息坐标系是否等于地图坐标系；成立才进入下面的分支。
        if msg.header.frame_id != self.frame_id:
            # 结束当前函数，不再执行后面的语句。
            return
        # 把本次结果放入容器，供下一轮处理。
        self.pending_clouds.append(msg)
        # 新位姿或点云一到，就尝试处理此前等待时间同步的点云。
        self.process_clouds()

    # 用时间对齐的里程计处理排队的点云。
    def process_clouds(self):
        # 条件检查：对应数据还没有取得或初始化；成立才进入下面的分支。
        if self.base is None or not self.sensor_poses or not self.base_poses:
            # 结束当前函数，不再执行后面的语句。
            return
        # 将计算结果保存为 now（当前本机单调时间）。
        now = time.monotonic()
        # 条件检查：机体里程计是否仍然新鲜；成立才进入下面的分支。
        if now - self.base_wall > self.odom_timeout:
            # 结束当前函数，不再执行后面的语句。
            return
        # 将计算结果保存为 remaining（尚未处理的点云或目标剩余距离）。
        remaining = deque(maxlen=5)
        # 依次处理收到或待发送的 ROS 消息。
        for msg in self.pending_clouds:
            # 当前帧暂时无法同步位姿，判断是否仍值得留在等待队列。
            if not self.integrate_cloud(msg, now):
                # 将点云 ROS 时间戳转为纳秒整数，便于和位姿比较。
                cloud_stamp = stamp_ns(msg.header.stamp)
                # 若位姿流还有机会追上点云时间，就保留这帧稍后重试。
                if (self.sensor_poses[-1][0] - cloud_stamp <= self.sensor_sync_tolerance * 1e9 or
                        self.base_poses[-1][0] - cloud_stamp <= self.sensor_sync_tolerance * 1e9):
                    # 把本次结果放入容器，供下一轮处理。
                    remaining.append(msg)
        # 更新节点状态 pending_clouds，供后续回调或主循环读取。
        self.pending_clouds = remaining

    # 将一帧点云融合到稀疏体素地图。
    def integrate_cloud(self, msg, now):
        # 将点云 ROS 时间戳转为纳秒整数，便于和位姿比较。
        cloud_stamp = stamp_ns(msg.header.stamp)
        # 从缓存里找时间最接近该点云的雷达位姿。
        sensor_stamp, sensor = min(self.sensor_poses, key=lambda item: abs(item[0] - cloud_stamp))
        if abs(sensor_stamp - cloud_stamp) > self.sensor_sync_tolerance * 1e9:
            # 雷达位姿与点云不同步，暂不处理这帧。
            return False
        # 从缓存里找时间最接近该点云的机体位姿。
        base_stamp, base_at_cloud = min(self.base_poses, key=lambda item: abs(item[0] - cloud_stamp))
        if abs(base_stamp - cloud_stamp) > self.sensor_sync_tolerance * 1e9:
            # 机体位姿与点云不同步，暂不处理这帧。
            return False
        # 建立抽样后的点列表，避免处理整帧海量原始点。
        points = []
        # 先执行可能抛出异常的 ROS 数据读取。
        try:
            # 依次处理当前点云点。
            for index, point in enumerate(point_cloud2.read_points(
                    msg, field_names=('x', 'y', 'z'), skip_nans=True)):
                # 条件检查：按间隔对点云抽样；成立才进入下面的分支。
                if index % self.point_stride == 0:
                    # 把本次结果放入容器，供下一轮处理。
                    points.append((float(point[0]), float(point[1]), float(point[2])))
                    # 条件检查：本帧点数是否达到处理上限；成立才进入下面的分支。
                    if len(points) >= self.max_points_per_cloud:
                        # 找到满足条件的结果，退出当前循环。
                        break
        # 捕获读取或服务调用失败，避免节点直接退出。
        except (ValueError, KeyError) as exc:
            # 点云字段缺失或格式错误时记录告警。
            self.get_logger().warning('Invalid point cloud: %s' % exc)
            # 格式错误的点云直接丢弃，避免同一坏消息在队列里反复重试。
            return True
        # 只有确实融合了有效点，才更新点云的新鲜时间。
        if self.map.integrate(sensor, points, base_at_cloud[2]):
            # 更新节点状态 cloud_wall，供后续回调或主循环读取。
            self.cloud_wall = now
        # 点云已经融合，返回 True 表示可从待处理队列移除。
        return True

    # 记录 GO2 控制器是否准备完毕。
    def on_ready(self, msg):
        # 更新节点状态 ready，供后续回调或主循环读取。
        self.ready = bool(msg.data)

    # 识别手动输入目标，必要时暂停自动探索。
    def on_input(self, msg):
        # RViz 手动目标优先；若只是本节点自己发出的目标回流，则忽略。
        # 将计算结果保存为 xy（地图坐标系下的目标水平坐标）。
        xy = (msg.pose.position.x, msg.pose.position.y)
        # 若这是本节点自身目标回流，按坐标和时间戳识别并忽略。
        if (self.active and math.dist(xy, self.active['xy']) < 0.05
                and stamp_ns(msg.header.stamp) == self.active['input_stamp']):
            # 结束当前函数，不再执行后面的语句。
            return
        # 正在清除旧目标时不要再把回流目标当作手动目标。
        if not self.clearing:
            # 更新节点状态 active，供后续回调或主循环读取。
            self.active = None
            # 更新节点状态 paused，供后续回调或主循环读取。
            self.paused = True
            # 发布已因手动目标暂停自动探索的状态。
            self.report('paused_for_manual_goal')

    # 确认 SCAN 已接收或转发当前目标。
    def on_goal(self, msg):
        # 条件检查：消息坐标系是否等于地图坐标系；成立才进入下面的分支。
        if self.clearing or msg.header.frame_id != self.frame_id:
            # 结束当前函数，不再执行后面的语句。
            return
        # 将计算结果保存为 xy（地图坐标系下的目标水平坐标）。
        xy = (msg.pose.position.x, msg.pose.position.y)
        # 清除后短时间内的近身旧目标属于残留转发，忽略它。
        if (self.base is not None and time.monotonic() - self.last_clear_wall < 2.0
                and math.dist(xy, self.base[:2]) < 0.25):
            # 结束当前函数，不再执行后面的语句。
            return
        # 条件检查：对应数据还没有取得或初始化；成立才进入下面的分支。
        if self.active is None:
            # 更新节点状态 paused，供后续回调或主循环读取。
            self.paused = True
            # 发布已因手动目标暂停自动探索的状态。
            self.report('paused_for_manual_goal')
            # 结束当前函数，不再执行后面的语句。
            return
        # 只把坐标与当前活动目标一致的 SCAN 消息视为转发确认。
        if math.dist(xy, self.active['xy']) < 0.05:
            # 旧时间戳不能覆盖本轮目标的确认时间。
            if stamp_ns(msg.header.stamp) < self.active['input_stamp']:
                # 结束当前函数，不再执行后面的语句。
                return
            # 记住 SCAN 转发目标的时间戳，后续回执必须与它相同。
            self.active['stamp'] = stamp_ns(msg.header.stamp)
            # 记录目标真正转发给 SCAN 的本机时刻。
            self.active['forwarded'] = time.monotonic()
        # 上述条件不成立时，改走这个分支。
        else:
            # 更新节点状态 active，供后续回调或主循环读取。
            self.active = None
            # 更新节点状态 paused，供后续回调或主循环读取。
            self.paused = True
            # 发布已因手动目标暂停自动探索的状态。
            self.report('paused_for_manual_goal')

    # 确认 SCAN 已停车并回执目标。
    def on_ack(self, msg):
        # 只有已经转发的活动目标才可能匹配停车回执。
        if self.active and self.active['stamp'] is not None:
            # 将计算结果保存为 xy（地图坐标系下的目标水平坐标）。
            xy = (msg.pose.position.x, msg.pose.position.y)
            # 条件检查：消息坐标系是否等于地图坐标系；成立才进入下面的分支。
            if (msg.header.frame_id == self.frame_id and
                    stamp_ns(msg.header.stamp) == self.active['stamp']
                    and math.dist(xy, self.active['xy']) <= self.goal_reach_xy):
                # 记下已收到与当前目标匹配的停车回执。
                self.active['ack'] = True
                # 尝试检查停车回执和完成信号是否已经齐全。
                self.finish_if_complete()

    # 接收 SCAN 的任务完成信号。
    def on_complete(self, msg):
        # 只有存在正在执行的目标才处理这条消息。
        if self.active:
            # 先收到 false，表示任务周期已复位并可以接纳新的 true。
            if not msg.data:
                # 标记已观察到本轮任务的复位状态。
                self.active['armed'] = True
            # 先复位后到来的 true 才算本轮任务完成。
            elif self.active['armed']:
                # 记下 SCAN 对当前目标的完成信号。
                self.active['complete'] = True
                # 尝试检查停车回执和完成信号是否已经齐全。
                self.finish_if_complete()

    # 在回执和完成信号齐备时结算目标。
    def finish_if_complete(self):
        # 停车回执和任务完成信号都齐了，才结算这次目标。
        if self.active and self.active['ack'] and self.active['complete']:
            # 把已到达的前沿坐标写进日志。
            self.get_logger().info('Frontier goal reached: %s' % (self.active['xy'],))
            # 目标结束时检查地图是否增加新格，决定是否暂时屏蔽目标。
            self.finish_active_goal()
            # 更新节点状态 active，供后续回调或主循环读取。
            self.active = None
            # 更新节点状态 empty_since，供后续回调或主循环读取。
            self.empty_since = None
            # 向状态话题报告目标已到达。
            self.report('goal_reached')

    # 处理手动暂停或继续探索的服务请求。
    def on_pause(self, request, response):
        # 暂停期间若仍有活动目标，先要求 SCAN 清掉它。
        if request.data and self.active and not self.clearing:
            # 按暂停原因启动清除目标的异步流程。
            self.start_clear('paused')
        # 更新节点状态 paused，供后续回调或主循环读取。
        self.paused = bool(request.data)
        # 给暂停服务的调用者返回成功标记。
        response.success = True
        # 给暂停服务的调用者返回可读的暂停/恢复说明。
        response.message = 'exploration paused' if self.paused else 'exploration resumed'
        # 返回本函数计算得到的结果。
        return response

    # 进入清除当前目标的流程。
    def start_clear(self, reason):
        # 条件检查：对应数据还没有取得或初始化；成立才进入下面的分支。
        if self.active is None or self.clearing:
            # 结束当前函数，不再执行后面的语句。
            return
        # 超时或失败目标需要冷却；人工暂停与到达目标不算失败。
        if reason not in ('paused', 'proximity_reached'):
            # 把失败目标及解锁时间放入冷却列表。
            self.suppressed.append((self.active['xy'], time.monotonic() + self.failed_frontier_cooldown))
        # 更新节点状态 clearing，供后续回调或主循环读取。
        self.clearing = True
        # 更新节点状态 clearing_reason，供后续回调或主循环读取。
        self.clearing_reason = reason
        # 告诉外部目前正在清除旧目标及其原因。
        self.report('clearing_goal', reason=reason)
        # 向 SCAN 发出清空航点请求。
        self.request_clear()

    # 请求 SCAN 清空当前航点。
    def request_clear(self):
        # SCAN 服务可用时才发起异步清空请求。
        if self.clear_client.service_is_ready():
            # 更新节点状态 clear_future，供后续回调或主循环读取。
            self.clear_future = self.clear_client.call_async(Trigger.Request())

    # 检查清空服务是否完成。
    def tick_clear(self):
        # 正在清除旧目标时不要再把回流目标当作手动目标。
        if not self.clearing:
            # 没有清除流程，tick 可以继续检查目标或搜索前沿。
            return False
        # 条件检查：对应数据还没有取得或初始化；成立才进入下面的分支。
        if self.clear_future is None:
            # 向 SCAN 发出清空航点请求。
            self.request_clear()
        # 上一条件不满足时，再检查这个分支。
        elif self.clear_future.done():
            # 异步服务结果可能抛出异常，例如服务进程已退出。
            try:
                # 取得异步清空服务的最终结果。
                response = self.clear_future.result()
            except Exception as exc:  # ROS service can disappear during shutdown.
                # 清空服务调用失败时记录告警，下一轮还可继续处理。
                self.get_logger().warning('Waypoint clear failed: %s' % exc)
                # 失败时用空值表示本轮没有有效服务响应。
                response = None
            # 更新节点状态 clear_future，供后续回调或主循环读取。
            self.clear_future = None
            # 只有 SCAN 明确回复成功，才认为旧航点已清除。
            if response and response.success:
                # 更新节点状态 clearing，供后续回调或主循环读取。
                self.clearing = False
                # 更新节点状态 last_clear_wall，供后续回调或主循环读取。
                self.last_clear_wall = time.monotonic()
                # 目标结束时检查地图是否增加新格，决定是否暂时屏蔽目标。
                self.finish_active_goal()
                # 更新节点状态 active，供后续回调或主循环读取。
                self.active = None
                # 近距离到达兜底属于成功目标，而不是失败目标。
                if self.clearing_reason == 'proximity_reached':
                    # 更新节点状态 empty_since，供后续回调或主循环读取。
                    self.empty_since = None
                    # 上报近距离到达兜底的成功状态。
                    self.report('goal_reached', detail='proximity_fallback')
                # 上述条件不成立时，改走这个分支。
                else:
                    # 上报旧目标已清除，下一轮可重选。
                    self.report('goal_cleared')
                # 空字符串表示目前没有正在记录的清除原因。
                self.clearing_reason = ''
        # 清除流程仍占用本轮 tick，暂不下发新探索目标。
        return True

    # 把选中的探索目标发送给 SCAN 航点协调器。
    def publish_target(self, xy, detail=None):
        # 将计算结果保存为 msg（当前 ROS 消息）。
        msg = PoseStamped()
        # 给将发送的目标填写当前 ROS 时间。
        msg.header.stamp = self.get_clock().now().to_msg()
        # 把目标坐标声明为统一地图坐标系。
        msg.header.frame_id = self.frame_id
        # 把选出的水平坐标写入目标消息。
        msg.pose.position.x, msg.pose.position.y = xy
        # SCAN 同层手动目标接口只用 XY；Z 小于 -0.1 会被拒绝，所以这里设为 0。
        msg.pose.position.z = 0.0
        # 给目标设置合法的单位朝向四元数。
        msg.pose.orientation.w = 1.0
        # 将计算结果保存为 now（当前本机单调时间）。
        now = time.monotonic()
        # 将计算结果保存为 distance（机器人到目标的水平距离）。
        distance = math.dist(xy, self.base[:2])
        # 建立活动目标记录：发送时间、最短距离、回执及计时状态都在这里。
        self.active = {'xy': xy, 'sent': now, 'input_stamp': stamp_ns(msg.header.stamp),
                       'stamp': None, 'forwarded': None,
                       'best_distance': distance, 'progress_wall': now,
                       'ack': False, 'armed': False, 'complete': False,
                       'near_since': None, 'blocked_since': None,
                       'recovery': detail == 'recovery_backtrack',
                       'observed_cells_at_send': len(self.map.project(self.base[:2]))}
        # 沿轨迹回退目标要单独统计其地图收益。
        if self.active['recovery']:
            # 记录回退开始时的已观察格数。
            self.recovery_gate.start(self.active['observed_cells_at_send'])
        # 把更新后的 ROS 消息发布到相应话题。
        self.waypoint_pub.publish(msg)
        # 上报刚刚发出的目标坐标。
        self.report('goal_sent', goal=xy, **({'detail': detail} if detail else {}))

    # 记录目标结束时地图是否增长。
    def finish_active_goal(self):
        # 条件检查：对应数据还没有取得或初始化；成立才进入下面的分支。
        if not self.active or self.base is None:
            # 结束当前函数，不再执行后面的语句。
            return
        # 重新数当前已观察的二维格，与发送目标时的数量比较。
        observed = len(self.map.project(self.base[:2]))
        # 沿轨迹回退目标要单独统计其地图收益。
        if self.active['recovery']:
            # 根据回退后新增网格数决定能否再次回退。
            self.recovery_gate.finish(observed)
        # 普通到达目标才使用无收益目标屏蔽规则。
        elif self.clearing_reason in ('', 'proximity_reached'):
            # 把本次目标的地图增长量交给屏蔽门控判断。
            self.goal_progress_gate.finish(self.active['xy'],
                                           self.active['observed_cells_at_send'], observed)

    # 从旧轨迹中寻找合适的回退目标。
    def backtrack_goal(self):
        # 条件检查：对应数据还没有取得或初始化；成立才进入下面的分支。
        if self.base is None or not self.trajectory.poses:
            # 返回旧轨迹中找到的可用回退点。
            return None
        # 将计算结果保存为 grid（高度切片后的二维占用表）。
        grid = self.map.project(self.base[:2])
        # 依次处理已记录的机体行走轨迹。
        for pose in reversed(self.trajectory.poses):
            # 将计算结果保存为 xy（地图坐标系下的目标水平坐标）。
            xy = (pose.pose.position.x, pose.pose.position.y)
            # 将计算结果保存为 distance（机器人到目标的水平距离）。
            distance = math.dist(xy, self.base[:2])
            # 条件检查：目标是否离机器人太近；成立才进入下面的分支。
            if not max(2.0, self.cfg.min_goal_distance) <= distance <= self.cfg.max_goal_distance:
                # 跳过本次循环，检查下一个元素。
                continue
            # 把旧轨迹上的米制坐标换成二维格号。
            cell = (math.floor(xy[0] / self.cfg.resolution),
                    math.floor(xy[1] / self.cfg.resolution))
            # 回退点必须是已观察空闲格，并且不能被 SCAN 判为障碍。
            if grid.get(cell) != 0 or self.planner_blocks(xy):
                # 跳过本次循环，检查下一个元素。
                continue
            # 返回本函数计算得到的结果。
            return xy
        # 返回本函数计算得到的结果。
        return None

    # 更新 RViz 中的前沿目标标记。
    def publish_marker(self, xy, clusters):
        # 建立一条装有多个 RViz 标记的消息。
        array = MarkerArray()
        # 建立本轮要显示或删除的单个目标标记。
        marker = Marker()
        # 标记也使用与地图一致的坐标系。
        marker.header.frame_id = self.frame_id
        # 给 RViz 标记填写当前时间戳。
        marker.header.stamp = self.get_clock().now().to_msg()
        # 设置标记命名空间，避免与其他 RViz 标记冲突。
        marker.ns = 'selected_frontier'
        # 给同一标记固定编号，新发布会更新旧标记。
        marker.id = 0
        # 用球体显示被选中的前沿目标。
        marker.type = Marker.SPHERE
        # 有目标就显示球体，无目标则删除旧球体。
        marker.action = Marker.ADD if xy else Marker.DELETE
        # 设置合法的单位朝向四元数。
        marker.pose.orientation.w = 1.0
        # 只有选中了目标，才填写球体的位置、大小和颜色。
        if xy:
            # 标记的 X/Y 放在选出的前沿目标位置。
            marker.pose.position.x, marker.pose.position.y = xy
            # 标记高度取当前机体高度，便于在 RViz 中看见。
            marker.pose.position.z = self.base[2]
            # 球体三个方向直径均设为 0.35 米。
            marker.scale.x = marker.scale.y = marker.scale.z = 0.35
            # 设置标记的 RGBA 颜色和透明度。
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.9, 1.0, 0.9
        # 把本次结果放入容器，供下一轮处理。
        array.markers.append(marker)
        # 把更新后的 ROS 消息发布到相应话题。
        self.marker_pub.publish(array)

    # 每秒执行一次探索状态机。
    def tick(self):
        # 将计算结果保存为 now（当前本机单调时间）。
        now = time.monotonic()
        # 有历史轨迹时，定期发布完整 Path 供 RViz 显示。
        if self.trajectory.poses:
            # 给整条轨迹更新最新显示时间。
            self.trajectory.header.stamp = self.get_clock().now().to_msg()
            # 把更新后的 ROS 消息发布到相应话题。
            self.trajectory_pub.publish(self.trajectory)
        # 旧目标尚在清除时，本轮不再下发新目标。
        if self.tick_clear():
            self.empty_since = None
            # 结束当前函数，不再执行后面的语句。
            return
        # 条件检查：对应数据还没有取得或初始化；成立才进入下面的分支。
        if (self.base is None or now - self.base_wall > self.odom_timeout or
                not self.cloud_wall or now - self.cloud_wall > self.cloud_timeout):
            # 更新节点状态 empty_since，供后续回调或主循环读取。
            self.empty_since = None
            # 缺少新鲜点云或位姿时对外报告等待数据。
            self.report('waiting_for_fresh_data')
            # 结束当前函数，不再执行后面的语句。
            return
        # 更新节点状态 suppressed，供后续回调或主循环读取。
        self.suppressed = [(xy, expiry) for xy, expiry in self.suppressed if expiry > now]
        projected = self.map.project(self.base[:2])
        self.coverage = assess_reachable_coverage(projected, self.cfg, self.base[:2])
        self.publish_coverage_grid(projected)
        # 只有存在正在执行的目标才处理这条消息。
        if self.active:
            self.empty_since = None
            # 将计算结果保存为 target（当前活动目标）。
            target = self.active
            # 发了航点，但尚未收到 SCAN 转发确认。
            if target['stamp'] is None:
                # 超过转发时限后放弃这次未生效的目标。
                if now - target['sent'] > self.goal_forward_timeout:
                    if target['recovery']:
                        # 根据回退后新增网格数决定能否再次回退。
                        self.recovery_gate.finish(len(self.map.project(self.base[:2])))
                    # 更新节点状态 active，供后续回调或主循环读取。
                    self.active = None
                    # 状态中标明目标没有被 SCAN 转发。
                    self.report('goal_not_forwarded')
                # 结束当前函数，不再执行后面的语句。
                return
            # 计算机体到当前目标的水平距离。
            distance = math.dist(target['xy'], self.base[:2])
            # 条件检查：机体已足够靠近目标；成立才进入下面的分支。
            if distance <= self.goal_reach_xy:
                # 目标不再被障碍阻挡，清掉阻挡计时。
                target['blocked_since'] = None
                # 首次进入目标附近时开始停留计时。
                if target['near_since'] is None:
                    # 记录刚开始接近目标的时刻。
                    target['near_since'] = now
                # 在目标附近持续停留足够久时启用到达兜底。
                elif now - target['near_since'] >= self.near_goal_dwell:
                    # 先清旧航点，清除成功后才视为近距离到达。
                    self.start_clear('proximity_reached')
                    # 结束当前函数，不再执行后面的语句。
                    return
            # 上述条件不成立时，改走这个分支。
            else:
                # 离开目标附近后重置停留计时。
                target['near_since'] = None
                # 仅用新鲜 SCAN 障碍图检查目标是否被占据。
                if self.occupancy_fresh(now) and self.planner_blocks(target['xy']):
                    # 首次发现目标被挡住时开始计时。
                    if target['blocked_since'] is None:
                        # 记录目标开始被挡住的时刻。
                        target['blocked_since'] = now
                    # 阻挡持续达到门槛时要求清除这个目标。
                    elif now - target['blocked_since'] >= self.blocked_goal_dwell:
                        # 以目标被 SCAN 判为障碍为由清除航点。
                        self.start_clear('target_occupied_in_scan')
                        # 结束当前函数，不再执行后面的语句。
                        return
                # 上述条件不成立时，改走这个分支。
                else:
                    # 目标不再被障碍阻挡，清掉阻挡计时。
                    target['blocked_since'] = None
            # 比历史最近距离再前进一定量，才算有效进度。
            if distance < target['best_distance'] - self.progress_distance:
                # 更新至今最短的目标距离。
                target['best_distance'] = distance
                # 记录最近一次有效进展的本机时刻。
                target['progress_wall'] = now
            # 超过转发时限后放弃这次未生效的目标。
            if now - target['sent'] > self.goal_timeout:
                # 按目标总超时原因清除目标。
                self.start_clear('goal_timeout')
            # 长时间没有有效进展，主动放弃当前目标。
            elif now - target['progress_wall'] > self.progress_timeout:
                # 按无进度原因清除目标并准备重选。
                self.start_clear('no_progress')
            # 结束当前函数，不再执行后面的语句。
            return
        # 条件检查：体素地图已达到容量上限；成立才进入下面的分支。
        if len(self.map.voxels) >= self.cfg.max_voxels:
            self.empty_since = None
            # 体素表到达上限，报告地图容量已满。
            self.report('map_capacity_reached')
            # 结束当前函数，不再执行后面的语句。
            return
        # 将计算结果保存为 projected（以当前机体高度投影的二维地图）。
        if self.cfg.global_search and (self.coverage is None or
                (self.bounds_relative_to_start and not self.bounds_initialized)):
            self.empty_since = None
            self.report('invalid_exploration_bounds')
            return
        self.unresolved_unknown_cells = 0
        # 没有一个已观察空闲格时，暂时无法从机器人位置找前沿。
        if not any(value == 0 for value in projected.values()):
            self.empty_since = None
            # 状态中提示正在等待形成可搜索的空闲地图。
            self.report('waiting_for_free_space')
            # 结束当前函数，不再执行后面的语句。
            return
        # 平面安全配置要求 SCAN 障碍图必须新鲜。
        if self.require_planner_occupancy and not self.occupancy_fresh(now):
            self.empty_since = None
            # 状态中提示正在等待 SCAN 障碍图。
            self.report('waiting_for_planner_occupancy')
            # 结束当前函数，不再执行后面的语句。
            return
        if self.cfg.global_search and not self.coverage['reachability_valid']:
            self.empty_since = None
            self.report('waiting_for_reachability_seed')
            return
        if self.cfg.global_search and self.coverage['reachable_unknown_cells'] == 0:
            self.unresolved_unknown_cells = 0
            self.last_frontier_cells = 0
            self.last_frontier_clusters = 0
            self.last_selection_diagnostics = {'reachable_frontier_cells': 0,
                                               'candidate_goals': 0,
                                               'suppressed_goals': 0,
                                               'open_start_neighbors': 0}
            self.publish_marker(None, 0)
            if self.empty_since is None:
                self.empty_since = now
            self.report('exploration_complete' if now - self.empty_since >= self.no_frontier_hold
                        else 'waiting_for_coverage_confirmation')
            return
        # 将计算结果保存为 suppressed_xy（本轮选点要排除的坐标）。
        suppressed_xy = [xy for xy, _ in self.suppressed]
        # 再加入到达但无地图收益的目标，一并从本轮选点排除。
        suppressed_xy.extend(self.goal_progress_gate.blocked(len(projected)))
        # 将计算结果保存为 obstacles（仍在有效期内的 SCAN 障碍）。
        obstacles = self.external_obstacles if self.occupancy_fresh(now) else ()
        # 将计算结果保存为 diagnostics（供状态话题输出的选点计数）。
        goal, score, clusters, frontier_cells, diagnostics = select_frontier(
            self.map, self.base, self.cfg, suppressed_xy, obstacles)
        # 将计算结果保存为 residual_goal（本轮是否选中了零散前沿）。
        residual_goal = False
        # 条件检查：当前启用全图搜索；成立才进入下面的分支。
        if self.cfg.global_search and frontier_cells > 0 and clusters == 0:
            # 条件检查：对应数据还没有取得或初始化；成立才进入下面的分支。
            if self.residual_since is None:
                # 更新节点状态 residual_since，供后续回调或主循环读取。
                self.residual_since = now
            # 条件检查：无前沿状态持续达到确认时间；成立才进入下面的分支。
            if now - self.residual_since >= self.no_frontier_hold:
                # 将计算结果保存为 residual_cfg（把最小簇规模暂时调为一格后的参数）。
                residual_cfg = replace(self.cfg, min_frontier_cells=1)
                # 以单格最小簇规模重新搜索剩余零散前沿。
                residual, residual_score, _, _, _ = select_frontier(
                    self.map, self.base, residual_cfg, suppressed_xy, obstacles)
                # 零散前沿确实找到目标，才覆盖普通选点结果。
                if residual is not None:
                    # 将计算结果保存为 score（目标收益减去路径代价后的评分）。
                    goal, score = residual, residual_score
                    # 将计算结果保存为 residual_goal（本轮是否选中了零散前沿）。
                    residual_goal = True
        # 上述条件不成立时，改走这个分支。
        else:
            # 更新节点状态 residual_since，供后续回调或主循环读取。
            self.residual_since = None
        # 更新节点状态 last_frontier_cells，供后续回调或主循环读取。
        self.last_frontier_cells = frontier_cells
        # 更新节点状态 last_frontier_clusters，供后续回调或主循环读取。
        self.last_frontier_clusters = clusters
        # 更新节点状态 last_selection_diagnostics，供后续回调或主循环读取。
        self.last_selection_diagnostics = diagnostics
        observation_goal = False
        if (self.cfg.global_search and goal is None and frontier_cells == 0 and
                self.coverage['reachable_unknown_cells'] > 0):
            goal = select_observation_goal(self.map, self.base, self.cfg, projected,
                                           suppressed_xy, obstacles)
            observation_goal = goal is not None
            if observation_goal:
                score = 0.0
        # 把本轮选点结果显示在 RViz；无目标时删旧标记。
        self.publish_marker(goal, clusters)
        # 条件检查：对应数据还没有取得或初始化；成立才进入下面的分支。
        if goal is None:
            # 条件检查：当前启用全图搜索；成立才进入下面的分支。
            if (self.cfg.global_search and self.recovery_gate.eligible(
                    clusters, diagnostics['reachable_frontier_cells'],
                    diagnostics['candidate_goals'], len(projected)) and
                    not self.observation_only and
                    not self.paused and self.ready and
                    self.waypoint_pub.get_subscription_count() > 0):
                # 将计算结果保存为 recovery（沿旧轨迹找到的回退落点）。
                recovery = self.backtrack_goal()
                # 旧轨迹中找到了可用回退点，才允许发回退目标。
                if recovery is not None:
                    # 把本次结果放入容器，供下一轮处理。
                    self.suppressed.append((self.base[:2], now + self.failed_frontier_cooldown))
                    # 把沿旧轨迹的回退点发给航点协调器。
                    self.publish_target(recovery, detail='recovery_backtrack')
                    # 结束当前函数，不再执行后面的语句。
                    return
            if self.cfg.global_search:
                self.unresolved_unknown_cells = self.coverage['reachable_unknown_cells']
            # 条件检查：当前仍然存在原始未知边界格；成立才进入下面的分支。
            if frontier_cells > 0:
                # 更新节点状态 empty_since，供后续回调或主循环读取。
                self.empty_since = None
                # 仍有前沿时按诊断结果说明不可达、被屏蔽或簇过小。
                self.report('frontiers_unreachable_or_suppressed' if clusters > 0 else
                            'frontiers_below_cluster_threshold',
                            **({'detail': 'recovery_waiting_for_new_observations'}
                               # 条件检查：仍然存在达到规模门槛的前沿簇；成立才进入下面的分支。
                               if clusters > 0 and self.recovery_gate.waiting_for_new_cells(
                                   len(projected)) else {}))
            # 上述条件不成立时，改走这个分支。
            else:
                if self.cfg.global_search and self.coverage['reachable_unknown_cells'] > 0:
                    self.empty_since = None
                    self.report('unknown_regions_unobservable_or_unreachable', clusters=0)
                else:
                    if self.empty_since is None:
                        self.empty_since = now
                    state = (('exploration_complete' if self.cfg.global_search else
                              'current_region_complete') if now - self.empty_since >= self.no_frontier_hold
                             else 'waiting_for_frontier')
                    self.report(state, clusters=0)
            # 结束当前函数，不再执行后面的语句。
            return
        # 更新节点状态 empty_since，供后续回调或主循环读取。
        self.empty_since = None
        # 条件检查：当前处于只观察模式；成立才进入下面的分支。
        if self.observation_only:
            # 只观察模式只报告选出的点，不让机器狗移动。
            self.report('observation_only', goal=goal, clusters=clusters, score=round(score, 3))
        # 上一条件不满足时，再检查这个分支。
        elif self.paused:
            # 手动暂停期间保留选点结果，但不发导航命令。
            self.report('paused', goal=goal)
        # 上一条件不满足时，再检查这个分支。
        elif not self.ready or self.waypoint_pub.get_subscription_count() == 0:
            # GO2 或航点协调器未就绪时等待，不贸然发送目标。
            self.report('waiting_for_waypoint_coordinator', goal=goal)
        # 上述条件不成立时，改走这个分支。
        else:
            # 全部运行条件满足后发送本轮最佳探索目标。
            self.publish_target(goal, detail=('supplemental_observation' if observation_goal else
                                              'residual_frontier' if residual_goal else None))


# 程序入口：初始化 ROS 或执行本文件的测试。
def main():
    # 先初始化 Python 的 ROS 2 客户端库。
    rclpy.init()
    # 创建上面定义的探索节点对象。
    node = Explorer()
    # spin 持续处理 ROS 回调；用户按 Ctrl+C 时进入异常分支并清理资源。
    try:
        # 进入 ROS 事件循环，持续处理订阅、服务和定时器回调。
        rclpy.spin(node)
    # 捕获读取或服务调用失败，避免节点直接退出。
    except KeyboardInterrupt:
        # 这里有意不执行额外操作。
        pass
    # 无论是否发生异常，最后都清理 ROS 资源。
    finally:
        # ros2 launch 退出时可能再发送一次 SIGINT；清理期间暂时忽略它。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        # 退出时销毁节点，释放 ROS 资源。
        node.destroy_node()
        # 只有 ROS 客户端仍处于运行状态时才调用关闭。
        if rclpy.ok():
            # 关闭 ROS 2 客户端库，结束进程。
            rclpy.shutdown()
