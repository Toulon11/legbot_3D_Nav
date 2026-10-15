"""ROS2 orchestration for the Python frontier implementation."""
from collections import deque
import json
import math
import signal
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .exploration_core import Settings, VoxelMap, select_frontier


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Explorer(Node):
    def __init__(self):
        super().__init__('legbot_explorer')
        cfg = Settings()
        for name in cfg.__dataclass_fields__:
            setattr(cfg, name, self.declare_parameter(name, getattr(cfg, name)).value)
        self.cfg = cfg
        self.map = VoxelMap(cfg)
        for name, default in (
            ('frame_id', 'odom'), ('cloud_topic', '/fast_lio/cloud_registered'),
            ('sensor_odom_topic', '/fast_lio/odometry_lidar'),
            ('base_odom_topic', '/fast_lio/odometry_base'),
            ('waypoint_topic', '/scan/waypoint_input'), ('scan_goal_topic', '/scan/goal'),
            ('ready_topic', '/go2/demo_ready'), ('point_stride', 8),
            ('max_points_per_cloud', 1000), ('sensor_sync_tolerance', 0.03),
            ('odom_timeout', 1.0), ('cloud_timeout', 2.0),
            ('goal_reach_xy', 0.55), ('progress_distance', 0.35),
            ('progress_timeout', 12.0), ('goal_timeout', 90.0),
            ('goal_forward_timeout', 3.0), ('failed_frontier_cooldown', 45.0),
            ('no_frontier_hold', 8.0), ('observation_only', True)):
            setattr(self, name, self.declare_parameter(name, default).value)
        if (cfg.resolution <= 0 or cfg.local_search_radius <= 0 or self.point_stride < 1
                or self.max_points_per_cloud < 1 or cfg.max_voxels < 1):
            raise ValueError('resolution, search radius, stride, max points and max voxels must be positive')
        self.sensor_poses = deque(maxlen=100)
        self.pending_clouds = deque(maxlen=5)
        self.base = None
        self.base_wall = 0.0
        self.cloud_wall = 0.0
        self.ready = False
        self.paused = False
        self.active = None
        self.clearing = False
        self.last_clear_wall = 0.0
        self.clear_future = None
        self.suppressed = []
        self.empty_since = None
        self.last_status = ''
        self.last_marker_version = -1
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.waypoint_pub = self.create_publisher(PoseStamped, self.waypoint_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', state_qos)
        self.status_pub = self.create_publisher(String, '/exploration/status', state_qos)
        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.sensor_odom_topic, self.on_sensor, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.base_odom_topic, self.on_base, qos_profile_sensor_data)
        self.create_subscription(Bool, self.ready_topic, self.on_ready, state_qos)
        self.create_subscription(PoseStamped, self.waypoint_topic, self.on_input, 10)
        self.create_subscription(PoseStamped, self.scan_goal_topic, self.on_goal, 10)
        self.create_subscription(PoseStamped, '/scan/goal_stop_ack', self.on_ack, 10)
        self.create_subscription(Bool, '/scan/mission_complete', self.on_complete, state_qos)
        self.pause_srv = self.create_service(SetBool, '/exploration/set_paused', self.on_pause)
        self.clear_client = self.create_client(Trigger, '/scan/waypoints/clear')
        self.create_timer(1.0, self.tick)
        self.report('observation_only' if self.observation_only else 'waiting_for_data')

    def report(self, state, **extra):
        data = {'state': state, 'implementation': 'python', 'map_version': self.map.version,
                'voxel_count': len(self.map.voxels), 'paused': self.paused,
                'observation_only': self.observation_only}
        data.update(extra)
        encoded = json.dumps(data, sort_keys=True)
        if encoded != self.last_status:
            self.status_pub.publish(String(data=encoded))
            self.last_status = encoded

    def on_sensor(self, msg):
        if msg.header.frame_id != self.frame_id:
            return
        p = msg.pose.pose.position
        if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.sensor_poses.append((stamp_ns(msg.header.stamp), (p.x, p.y, p.z)))
            self.process_clouds()

    def on_base(self, msg):
        if msg.header.frame_id != self.frame_id:
            return
        p = msg.pose.pose.position
        if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.base = (p.x, p.y, p.z)
            self.base_wall = time.monotonic()
            self.process_clouds()

    def on_cloud(self, msg):
        if msg.header.frame_id != self.frame_id:
            return
        self.pending_clouds.append(msg)
        self.process_clouds()

    def process_clouds(self):
        if self.base is None or not self.sensor_poses:
            return
        now = time.monotonic()
        if now - self.base_wall > self.odom_timeout:
            return
        remaining = deque(maxlen=5)
        for msg in self.pending_clouds:
            if not self.integrate_cloud(msg, now):
                cloud_stamp = stamp_ns(msg.header.stamp)
                if self.sensor_poses[-1][0] - cloud_stamp <= self.sensor_sync_tolerance * 1e9:
                    remaining.append(msg)
        self.pending_clouds = remaining

    def integrate_cloud(self, msg, now):
        cloud_stamp = stamp_ns(msg.header.stamp)
        sensor_stamp, sensor = min(self.sensor_poses, key=lambda item: abs(item[0] - cloud_stamp))
        if abs(sensor_stamp - cloud_stamp) > self.sensor_sync_tolerance * 1e9:
            return False
        points = []
        try:
            for index, point in enumerate(point_cloud2.read_points(
                    msg, field_names=('x', 'y', 'z'), skip_nans=True)):
                if index % self.point_stride == 0:
                    points.append((float(point[0]), float(point[1]), float(point[2])))
                    if len(points) >= self.max_points_per_cloud:
                        break
        except (ValueError, KeyError) as exc:
            self.get_logger().warning('Invalid point cloud: %s' % exc)
            return True
        if self.map.integrate(sensor, points, self.base[2]):
            self.cloud_wall = now
        return True

    def on_ready(self, msg):
        self.ready = bool(msg.data)

    def on_input(self, msg):
        # A manual RViz goal takes priority. A copy of our own publication is ignored.
        xy = (msg.pose.position.x, msg.pose.position.y)
        if (self.active and math.dist(xy, self.active['xy']) < 0.05
                and stamp_ns(msg.header.stamp) == self.active['input_stamp']):
            return
        if not self.clearing:
            self.active = None
            self.paused = True
            self.report('paused_for_manual_goal')

    def on_goal(self, msg):
        if self.clearing or msg.header.frame_id != self.frame_id:
            return
        xy = (msg.pose.position.x, msg.pose.position.y)
        if (self.base is not None and time.monotonic() - self.last_clear_wall < 2.0
                and math.dist(xy, self.base[:2]) < 0.25):
            return
        if self.active is None:
            self.paused = True
            self.report('paused_for_manual_goal')
            return
        if math.dist(xy, self.active['xy']) < 0.05:
            if stamp_ns(msg.header.stamp) < self.active['input_stamp']:
                return
            self.active['stamp'] = stamp_ns(msg.header.stamp)
            self.active['forwarded'] = time.monotonic()
        else:
            self.active = None
            self.paused = True
            self.report('paused_for_manual_goal')

    def on_ack(self, msg):
        if self.active and self.active['stamp'] is not None:
            xy = (msg.pose.position.x, msg.pose.position.y)
            if (stamp_ns(msg.header.stamp) == self.active['stamp']
                    and math.dist(xy, self.active['xy']) <= self.goal_reach_xy):
                self.active['ack'] = True
                self.finish_if_complete()

    def on_complete(self, msg):
        if self.active:
            if not msg.data:
                self.active['armed'] = True
            elif self.active['armed']:
                self.active['complete'] = True
                self.finish_if_complete()

    def finish_if_complete(self):
        if self.active and self.active['ack'] and self.active['complete']:
            self.get_logger().info('Frontier goal reached: %s' % (self.active['xy'],))
            self.active = None
            self.empty_since = None
            self.report('goal_reached')

    def on_pause(self, request, response):
        if request.data and self.active and not self.clearing:
            self.start_clear('paused')
        self.paused = bool(request.data)
        response.success = True
        response.message = 'exploration paused' if self.paused else 'exploration resumed'
        return response

    def start_clear(self, reason):
        if self.active is None or self.clearing:
            return
        if reason != 'paused':
            self.suppressed.append((self.active['xy'], time.monotonic() + self.failed_frontier_cooldown))
        self.clearing = True
        self.report('clearing_goal', reason=reason)
        self.request_clear()

    def request_clear(self):
        if self.clear_client.service_is_ready():
            self.clear_future = self.clear_client.call_async(Trigger.Request())

    def tick_clear(self):
        if not self.clearing:
            return False
        if self.clear_future is None:
            self.request_clear()
        elif self.clear_future.done():
            try:
                response = self.clear_future.result()
            except Exception as exc:  # ROS service can disappear during shutdown.
                self.get_logger().warning('Waypoint clear failed: %s' % exc)
                response = None
            self.clear_future = None
            if response and response.success:
                self.clearing = False
                self.last_clear_wall = time.monotonic()
                self.active = None
                self.report('goal_cleared')
        return True

    def publish_target(self, xy):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y = xy
        # SCAN's same-floor manual-goal interface uses XY and rejects z < -0.1.
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        now = time.monotonic()
        distance = math.dist(xy, self.base[:2])
        self.active = {'xy': xy, 'sent': now, 'input_stamp': stamp_ns(msg.header.stamp),
                       'stamp': None, 'forwarded': None,
                       'best_distance': distance, 'progress_wall': now,
                       'ack': False, 'armed': False, 'complete': False}
        self.waypoint_pub.publish(msg)
        self.report('goal_sent', goal=xy)

    def publish_marker(self, xy, clusters):
        array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'selected_frontier'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD if xy else Marker.DELETE
        marker.pose.orientation.w = 1.0
        if xy:
            marker.pose.position.x, marker.pose.position.y = xy
            marker.pose.position.z = self.base[2]
            marker.scale.x = marker.scale.y = marker.scale.z = 0.35
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.9, 1.0, 0.9
        array.markers.append(marker)
        self.marker_pub.publish(array)

    def tick(self):
        now = time.monotonic()
        if self.tick_clear():
            return
        if (self.base is None or now - self.base_wall > self.odom_timeout or
                not self.cloud_wall or now - self.cloud_wall > self.cloud_timeout):
            self.empty_since = None
            self.report('waiting_for_fresh_data')
            return
        self.suppressed = [(xy, expiry) for xy, expiry in self.suppressed if expiry > now]
        if self.active:
            target = self.active
            if target['stamp'] is None:
                if now - target['sent'] > self.goal_forward_timeout:
                    self.active = None
                    self.report('goal_not_forwarded')
                return
            distance = math.dist(target['xy'], self.base[:2])
            if distance < target['best_distance'] - self.progress_distance:
                target['best_distance'] = distance
                target['progress_wall'] = now
            if now - target['sent'] > self.goal_timeout:
                self.start_clear('goal_timeout')
            elif now - target['progress_wall'] > self.progress_timeout:
                self.start_clear('no_progress')
            return
        if len(self.map.voxels) >= self.cfg.max_voxels:
            self.report('map_capacity_reached')
            return
        if not any(value == 0 for value in self.map.project(self.base[2], self.base[:2]).values()):
            self.report('waiting_for_free_space')
            return
        suppressed_xy = [xy for xy, _ in self.suppressed]
        goal, score, clusters = select_frontier(self.map, self.base, self.cfg, suppressed_xy)
        self.publish_marker(goal, clusters)
        if goal is None:
            if clusters > 0:
                self.empty_since = None
                self.report('frontiers_unreachable_or_suppressed', clusters=clusters)
            else:
                if self.empty_since is None:
                    self.empty_since = now
                state = ('current_region_complete' if now - self.empty_since >= self.no_frontier_hold
                         else 'waiting_for_frontier')
                self.report(state, clusters=0)
            return
        self.empty_since = None
        if self.observation_only:
            self.report('observation_only', goal=goal, clusters=clusters, score=round(score, 3))
        elif self.paused:
            self.report('paused', goal=goal)
        elif not self.ready or self.waypoint_pub.get_subscription_count() == 0:
            self.report('waiting_for_waypoint_coordinator', goal=goal)
        else:
            self.publish_target(goal)


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ros2 launch may forward a second SIGINT while the node is tearing down.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
