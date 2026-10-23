"""ROS2 orchestration for the Python frontier implementation."""
from collections import deque
from dataclasses import replace
import json
import math
import signal
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .exploration_core import GoalProgressGate, RecoveryGate, Settings, VoxelMap, select_frontier


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
            setattr(self, name, self.declare_parameter(name, default).value)
        if (cfg.resolution <= 0 or cfg.local_search_radius <= 0 or self.point_stride < 1
                or self.max_points_per_cloud < 1 or cfg.max_voxels < 1 or
                self.recovery_min_new_cells < 1):
            raise ValueError('resolution, search radius, stride, max points, max voxels and recovery gain must be positive')
        if (cfg.global_search and (cfg.exploration_min_x >= cfg.exploration_max_x or
                                   cfg.exploration_min_y >= cfg.exploration_max_y)):
            raise ValueError('global exploration bounds must have positive area')
        self.sensor_poses = deque(maxlen=100)
        self.base_poses = deque(maxlen=100)
        self.pending_clouds = deque(maxlen=5)
        self.external_obstacles = []
        self.occupancy_wall = 0.0
        self.base = None
        self.base_wall = 0.0
        self.cloud_wall = 0.0
        self.ready = False
        self.paused = False
        self.active = None
        self.clearing = False
        self.clearing_reason = ''
        self.last_clear_wall = 0.0
        self.clear_future = None
        self.suppressed = []
        self.recovery_gate = RecoveryGate(self.recovery_min_new_cells)
        self.goal_progress_gate = GoalProgressGate(self.recovery_min_new_cells)
        self.empty_since = None
        self.residual_since = None
        self.last_status = ''
        self.last_marker_version = -1
        self.last_frontier_cells = 0
        self.last_frontier_clusters = 0
        self.last_selection_diagnostics = {}
        self.bounds_initialized = False
        self.trajectory = Path()
        self.trajectory.header.frame_id = self.frame_id
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.waypoint_pub = self.create_publisher(PoseStamped, self.waypoint_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', state_qos)
        self.status_pub = self.create_publisher(String, '/exploration/status', state_qos)
        self.trajectory_pub = self.create_publisher(Path, '/exploration/trajectory', state_qos)
        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.sensor_odom_topic, self.on_sensor, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.base_odom_topic, self.on_base, qos_profile_sensor_data)
        if self.planner_occupancy_topic:
            self.create_subscription(PointCloud2, self.planner_occupancy_topic,
                                     self.on_occupancy, qos_profile_sensor_data)
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
                'observation_only': self.observation_only,
                'global_search': self.cfg.global_search,
                'frontier_cells': self.last_frontier_cells,
                'frontier_clusters': self.last_frontier_clusters}
        data.update(self.last_selection_diagnostics)
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
            if self.cfg.global_search and self.bounds_relative_to_start and not self.bounds_initialized:
                self.cfg.exploration_min_x += p.x
                self.cfg.exploration_max_x += p.x
                self.cfg.exploration_min_y += p.y
                self.cfg.exploration_max_y += p.y
                self.bounds_initialized = True
            self.base = (p.x, p.y, p.z)
            self.base_poses.append((stamp_ns(msg.header.stamp), self.base))
            self.base_wall = time.monotonic()
            if (not self.trajectory.poses or
                    math.dist(self.base[:2], (
                        self.trajectory.poses[-1].pose.position.x,
                        self.trajectory.poses[-1].pose.position.y)) >= 0.1):
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose = msg.pose.pose
                self.trajectory.poses.append(pose)
                if len(self.trajectory.poses) > 20000:
                    self.trajectory.poses.pop(0)
            self.process_clouds()

    def on_occupancy(self, msg):
        if msg.header.frame_id != self.frame_id or self.base is None:
            return
        try:
            self.external_obstacles = [
                (float(p[0]), float(p[1]))
                for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
                if abs(float(p[2]) - self.base[2]) <= self.planner_occupancy_height_tolerance]
        except (ValueError, KeyError) as exc:
            self.get_logger().warning('Invalid SCAN occupancy cloud: %s' % exc)
            return
        self.occupancy_wall = time.monotonic()

    def occupancy_fresh(self, now):
        return self.occupancy_wall > 0 and now - self.occupancy_wall <= self.planner_occupancy_timeout

    def planner_blocks(self, xy):
        return any(math.dist(xy, p) <= self.cfg.robot_radius for p in self.external_obstacles)

    def on_cloud(self, msg):
        if msg.header.frame_id != self.frame_id:
            return
        self.pending_clouds.append(msg)
        self.process_clouds()

    def process_clouds(self):
        if self.base is None or not self.sensor_poses or not self.base_poses:
            return
        now = time.monotonic()
        if now - self.base_wall > self.odom_timeout:
            return
        remaining = deque(maxlen=5)
        for msg in self.pending_clouds:
            if not self.integrate_cloud(msg, now):
                cloud_stamp = stamp_ns(msg.header.stamp)
                if (self.sensor_poses[-1][0] - cloud_stamp <= self.sensor_sync_tolerance * 1e9 or
                        self.base_poses[-1][0] - cloud_stamp <= self.sensor_sync_tolerance * 1e9):
                    remaining.append(msg)
        self.pending_clouds = remaining

    def integrate_cloud(self, msg, now):
        cloud_stamp = stamp_ns(msg.header.stamp)
        sensor_stamp, sensor = min(self.sensor_poses, key=lambda item: abs(item[0] - cloud_stamp))
        if abs(sensor_stamp - cloud_stamp) > self.sensor_sync_tolerance * 1e9:
            return False
        base_stamp, base_at_cloud = min(self.base_poses, key=lambda item: abs(item[0] - cloud_stamp))
        if abs(base_stamp - cloud_stamp) > self.sensor_sync_tolerance * 1e9:
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
        if self.map.integrate(sensor, points, base_at_cloud[2]):
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
            if (msg.header.frame_id == self.frame_id and
                    stamp_ns(msg.header.stamp) == self.active['stamp']
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
            self.finish_active_goal()
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
        if reason not in ('paused', 'proximity_reached'):
            self.suppressed.append((self.active['xy'], time.monotonic() + self.failed_frontier_cooldown))
        self.clearing = True
        self.clearing_reason = reason
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
                self.finish_active_goal()
                self.active = None
                if self.clearing_reason == 'proximity_reached':
                    self.empty_since = None
                    self.report('goal_reached', detail='proximity_fallback')
                else:
                    self.report('goal_cleared')
                self.clearing_reason = ''
        return True

    def publish_target(self, xy, detail=None):
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
                       'ack': False, 'armed': False, 'complete': False,
                       'near_since': None, 'blocked_since': None,
                       'recovery': detail == 'recovery_backtrack',
                       'observed_cells_at_send': len(self.map.project(self.base[:2]))}
        if self.active['recovery']:
            self.recovery_gate.start(self.active['observed_cells_at_send'])
        self.waypoint_pub.publish(msg)
        self.report('goal_sent', goal=xy, **({'detail': detail} if detail else {}))

    def finish_active_goal(self):
        if not self.active or self.base is None:
            return
        observed = len(self.map.project(self.base[:2]))
        if self.active['recovery']:
            self.recovery_gate.finish(observed)
        elif self.clearing_reason in ('', 'proximity_reached'):
            self.goal_progress_gate.finish(self.active['xy'],
                                           self.active['observed_cells_at_send'], observed)

    def backtrack_goal(self):
        if self.base is None or not self.trajectory.poses:
            return None
        grid = self.map.project(self.base[:2])
        for pose in reversed(self.trajectory.poses):
            xy = (pose.pose.position.x, pose.pose.position.y)
            distance = math.dist(xy, self.base[:2])
            if not max(2.0, self.cfg.min_goal_distance) <= distance <= self.cfg.max_goal_distance:
                continue
            cell = (math.floor(xy[0] / self.cfg.resolution),
                    math.floor(xy[1] / self.cfg.resolution))
            if grid.get(cell) != 0 or self.planner_blocks(xy):
                continue
            return xy
        return None

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
        if self.trajectory.poses:
            self.trajectory.header.stamp = self.get_clock().now().to_msg()
            self.trajectory_pub.publish(self.trajectory)
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
                    if target['recovery']:
                        self.recovery_gate.finish(len(self.map.project(self.base[:2])))
                    self.active = None
                    self.report('goal_not_forwarded')
                return
            distance = math.dist(target['xy'], self.base[:2])
            if distance <= self.goal_reach_xy:
                target['blocked_since'] = None
                if target['near_since'] is None:
                    target['near_since'] = now
                elif now - target['near_since'] >= self.near_goal_dwell:
                    self.start_clear('proximity_reached')
                    return
            else:
                target['near_since'] = None
                if self.occupancy_fresh(now) and self.planner_blocks(target['xy']):
                    if target['blocked_since'] is None:
                        target['blocked_since'] = now
                    elif now - target['blocked_since'] >= self.blocked_goal_dwell:
                        self.start_clear('target_occupied_in_scan')
                        return
                else:
                    target['blocked_since'] = None
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
        projected = self.map.project(self.base[:2])
        if not any(value == 0 for value in projected.values()):
            self.report('waiting_for_free_space')
            return
        if self.require_planner_occupancy and not self.occupancy_fresh(now):
            self.report('waiting_for_planner_occupancy')
            return
        suppressed_xy = [xy for xy, _ in self.suppressed]
        suppressed_xy.extend(self.goal_progress_gate.blocked(len(projected)))
        obstacles = self.external_obstacles if self.occupancy_fresh(now) else ()
        goal, score, clusters, frontier_cells, diagnostics = select_frontier(
            self.map, self.base, self.cfg, suppressed_xy, obstacles)
        residual_goal = False
        if self.cfg.global_search and frontier_cells > 0 and clusters == 0:
            if self.residual_since is None:
                self.residual_since = now
            if now - self.residual_since >= self.no_frontier_hold:
                residual_cfg = replace(self.cfg, min_frontier_cells=1)
                residual, residual_score, _, _, _ = select_frontier(
                    self.map, self.base, residual_cfg, suppressed_xy, obstacles)
                if residual is not None:
                    goal, score = residual, residual_score
                    residual_goal = True
        else:
            self.residual_since = None
        self.last_frontier_cells = frontier_cells
        self.last_frontier_clusters = clusters
        self.last_selection_diagnostics = diagnostics
        self.publish_marker(goal, clusters)
        if goal is None:
            if (self.cfg.global_search and self.recovery_gate.eligible(
                    clusters, diagnostics['reachable_frontier_cells'],
                    diagnostics['candidate_goals'], len(projected)) and
                    not self.observation_only and
                    not self.paused and self.ready and
                    self.waypoint_pub.get_subscription_count() > 0):
                recovery = self.backtrack_goal()
                if recovery is not None:
                    self.suppressed.append((self.base[:2], now + self.failed_frontier_cooldown))
                    self.publish_target(recovery, detail='recovery_backtrack')
                    return
            if frontier_cells > 0:
                self.empty_since = None
                self.report('frontiers_unreachable_or_suppressed' if clusters > 0 else
                            'frontiers_below_cluster_threshold',
                            **({'detail': 'recovery_waiting_for_new_observations'}
                               if clusters > 0 and self.recovery_gate.waiting_for_new_cells(
                                   len(projected)) else {}))
            else:
                if self.empty_since is None:
                    self.empty_since = now
                state = (('exploration_complete' if self.cfg.global_search else
                          'current_region_complete') if now - self.empty_since >= self.no_frontier_hold
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
            self.publish_target(goal, detail='residual_frontier' if residual_goal else None)


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
