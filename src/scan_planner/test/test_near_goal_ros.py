#!/usr/bin/python3
"""Real FSM/executor/coordinator: reached short goals must never fail later.

The coordinator intentionally settles longer than the old 20*10ms failure
window. No Gazebo, external motion commands or production ROS domain is used.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = str(95+os.getpid()%30)
os.environ['ROS_LOG_DIR'] = tempfile.mkdtemp(prefix='scan_near_goal_')
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from scan_planner.msg import GoalRequest
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String
from visualization_msgs.msg import Marker


def main():
    rclpy.init()
    node = rclpy.create_node('near_goal_regression')
    statuses, failures, acks, commands, forwarded = [], [], [], [], []
    local_targets=[];global_paths=[]
    node.create_subscription(Marker,'/scan_planner_node/global_list',global_paths.append,50)
    node.create_subscription(Marker,'/scan_planner_node/local_target',local_targets.append,50)
    for kind, topic, storage in (
        (String, '/scan/goal_tolerance_status', statuses),
        (String, '/scan/goal_plan_failed', failures),
        (PoseStamped, '/scan/goal_stop_ack', acks),
        (Twist, '/cmd_vel', commands),
        (GoalRequest, '/scan/goal_request', forwarded)):
        node.create_subscription(kind, topic, storage.append, 100)
    odom = node.create_publisher(Odometry, '/near_goal_test/odom', 20)
    request = node.create_publisher(GoalRequest, '/scan/waypoint_request', 10)
    cloud = node.create_publisher(PointCloud2, '/grid_map/cloud', 10)
    done = []
    node.create_subscription(Bool, '/scan/mission_complete', lambda m: done.append(m.data),
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    processes, logs = [], []
    position = [0., 0., .32]
    next_odom = 0.
    next_cloud = 0.
    obstacle_points = None

    def start(command):
        log = tempfile.TemporaryFile(mode='w+')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        processes.append(process)

    def pump(seconds):
        nonlocal next_odom,next_cloud
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            assert all(p.poll() is None for p in processes), 'test process exited'
            if time.monotonic() >= next_odom:
                m = Odometry()
                m.header.frame_id = 'map'
                m.header.stamp = node.get_clock().now().to_msg()
                m.pose.pose.position = Point(x=position[0], y=position[1], z=position[2])
                m.pose.pose.orientation.w = 1.
                odom.publish(m)
                next_odom = time.monotonic()+.01
                if obstacle_points is not None and time.monotonic()>=next_cloud:
                    cloud.publish(point_cloud2.create_cloud_xyz32(
                        Header(frame_id='map',stamp=m.header.stamp),obstacle_points))
                    next_cloud=time.monotonic()+.05
            rclpy.spin_once(node, timeout_sec=.005)

    def wait(condition, seconds=8.):
        until = time.monotonic()+seconds
        while time.monotonic() < until and not condition():
            pump(.02)
        assert condition(), 'timeout: failures='+str([m.data for m in failures[-4:]])

    def ns(stamp):
        return stamp.sec*1_000_000_000+stamp.nanosec

    def records(stamp, source, event):
        return [d for m in statuses for d in [json.loads(m.data)]
                if d.get('stamp_ns') == stamp and d.get('source') == source and d.get('event') == event]

    def send(distance, tolerance=None, height=.32, reference_points=()):
        m = GoalRequest()
        m.target.header.frame_id = 'map'
        m.target.header.stamp = node.get_clock().now().to_msg()
        m.target.pose.position = Point(x=position[0]+distance, y=0., z=height)
        if reference_points:
            m.reference_points=list(reference_points)
            m.target.pose.position=reference_points[-1]
        m.target.pose.orientation.w = 1.
        m.goal_id = len(forwarded)+1
        m.attempt_id = m.goal_id+100
        m.has_xy_tolerance = tolerance is not None
        m.xy_tolerance = tolerance or 0.
        previous = len(forwarded)
        request.publish(m)
        wait(lambda: len(forwarded) > previous)
        stamp = ns(forwarded[-1].target.header.stamp)
        wait(lambda: bool(records(stamp, 'planner', 'accepted')))
        return stamp

    try:
        bringup = Path(get_package_share_directory('legbot_bringup'))
        coordinator = bringup.parent.parent/'lib/legbot_bringup/scan_waypoint_mission'
        statefile = Path(os.environ['ROS_LOG_DIR'])/'mission.json'
        start([sys.argv[1], '--ros-args', '-p', 'body_pose_topic:=/near_goal_test/odom',
               '-p', 'finish_dist:=0.4', '-p', 'use_goal_z:=true', '-p', 'finish_dist_z:=0.25'])
        start([sys.argv[2], '--ros-args', '--params-file', str(bringup/'config/scan.yaml'),
               '-p', 'fsm.navi_mode:=1', '-p', 'body_pose_topic:=/near_goal_test/odom',
               '-p', 'fsm.planning_horizon:=3.0',
               '-p', 'fsm.finish_dist:=0.4', '-p', 'fsm.finish_dist_z:=0.25',
               '-p', 'fsm.manual_goal_use_message_z:=true',
               '-r', '/grid_map/body_pose:=/near_goal_test/odom',
               '-r', '/grid_map/sensor_pose:=/near_goal_test/odom',
               '-r', '/planning/bspline:=/scan/planning/bspline'])
        start([str(coordinator), '--mode', 'direct', '--input-topic', '/scan/waypoint_input',
               '--frame-id', 'map', '--odom-topic', '/near_goal_test/odom',
               '--output', str(statefile), '--reach-xy', '0.4', '--reach-z', '0.25',
               '--dwell-seconds', '0.30'])
        wait(lambda: odom.get_subscription_count() >= 3 and request.get_subscription_count() > 0)
        pump(1.2)  # FSM startup sleeps for one second before spinning callbacks.
        for distance, tolerance in ((.095, None), (.158, None), (0., None),
                                    (.399, None), (.149, .15), (0., .15)):
            stamp = send(distance, tolerance)
            wait(lambda: bool(records(stamp, 'executor', 'arrived')))
            wait(lambda: statefile.exists() and json.loads(statefile.read_text())['state'] == 'direct_goal_complete')
            pump(.4)  # Observe beyond the original false-failure window.
            assert any(ns(m.header.stamp) == stamp for m in acks)
            assert not any(m.data.startswith(str(stamp)+'|') for m in failures), 'failure after arrival'
            assert done and done[-1]
            assert commands and abs(commands[-1].linear.x)+abs(commands[-1].linear.y)+abs(commands[-1].angular.z) < 1e-8
            print('PASS reached distance=%s tolerance=%s stamp=%s: arrived -> mission_complete, no failure' %
                  (distance, tolerance or .4, stamp), flush=True)
        # Accept outside the radius, then enter the arrival tolerance while the
        # first route is being prepared. Exercise timer-stage arrival.
        entering = send(.16, .15)
        position[0] = .05
        wait(lambda: bool(records(entering, 'executor', 'arrived')))
        pump(.7)
        assert not any(m.data.startswith(str(entering)+'|') for m in failures)
        assert done and done[-1]
        print('PASS newly reached goal during initial replanning completes without failure', flush=True)
        position[0] = 0.
        pump(.1)
        # The old <0.20m rejection must not strand unreached 0.15m goals.
        # Hold odometry fixed: successful planning must not imply arrival.
        for distance in (.16,.179):
            outside = send(distance,.15)
            pump(.7)
            assert not records(outside,'executor','arrived')
            assert not any(ns(m.header.stamp)==outside for m in acks)
            assert not any(m.data.startswith(str(outside)+'|') for m in failures)
            assert any(abs(m.linear.x)+abs(m.linear.y)+abs(m.angular.z)>1e-4 for m in commands[-20:]), 'no short-goal execution'
            position[0]=distance-.14  # Just inside the 0.15 m arrival tolerance.
            pump(.2)
            assert not records(outside,'executor','arrived'), 'acknowledged before settling'
            wait(lambda:bool(records(outside,'executor','arrived')))
            pump(.4)
            assert not any(m.data.startswith(str(outside)+'|') for m in failures)
            position[0]=0.;pump(.1)
            print('PASS unreached short goal %sm plans and completes after settling inside tolerance'%distance,flush=True)
        # A coarse floor mismatch cannot be mistaken for a reached XY point.
        wrong_floor = send(.095, None, height=1.0)
        pump(.4)
        assert not records(wrong_floor, 'executor', 'arrived')
        assert not any(ns(m.header.stamp) == wrong_floor for m in acks)
        # Neither failure nor a prior custom tolerance poisons the next goal.
        reset = send(.158)
        wait(lambda: bool(records(reset, 'executor', 'arrived')))
        pump(.6)
        assert not any(m.data.startswith(str(reset)+'|') for m in failures)
        assert records(reset, 'planner', 'accepted')[-1]['effective_xy_tolerance'] == .4
        print('PASS floor gate preserved; default tolerance and arrival recover on next goal', flush=True)
        # One identified goal carries the complete shallow-bend reference.
        # Passing its interior vertices must keep executing the same mission.
        before=len(global_paths)
        route=[Point(x=.6,y=0.,z=.32),Point(x=1.2,y=.2,z=.32),Point(x=1.8,y=.4,z=.32)]
        continuous=send(1.8,.15,reference_points=route)
        wait(lambda:len(global_paths)>before)
        assert forwarded[-1].reference_points==route,'coordinator lost route geometry'
        samples=[p for marker in global_paths[before:] for p in marker.points]
        assert any(abs(p.y)<.005 and .3<p.x<.55 for p in samples),'reference was replaced by a line to final goal'
        for waypoint in route[:-1]:
            position[:]=[waypoint.x,waypoint.y,waypoint.z]
            pump(.4)
            assert not records(continuous,'executor','arrived')
            assert not any(m.data.startswith(str(continuous)+'|') for m in failures)
        position[:]=[1.72,.4,.32]
        wait(lambda:bool(records(continuous,'executor','arrived')))
        print('PASS continuous reference preserves bends, passes interior points, and stops only at its endpoint',flush=True)
        position[:]=[0.,0.,.32];pump(.3)
        # Real SCAN occupancy must still reject a short endpoint whose body
        # footprint intersects a newly observed wall, outside the initial body.
        obstacle_points=[(.35,y*.05,z*.05) for y in range(-8,9) for z in range(2,14)]
        wait(lambda:cloud.get_subscription_count()>0)
        pump(1.0)
        previous_targets=len(local_targets)
        blocked=send(.179,.15)
        wait(lambda:any(m.data.startswith(str(blocked)+'|') for m in failures))
        blocked_reasons=[m.data.split('|',1)[1] for m in failures if m.data.startswith(str(blocked)+'|')]
        # Existing SCAN policy may plan a free prefix before an occupied final
        # endpoint, then report executor no-progress. It must not plan through
        # the wall or accept that prefix as arrival at the actual goal.
        restricted=local_targets[previous_targets:]
        assert restricted and all(m.pose.position.x<.178 for m in restricted), 'occupied endpoint was not restricted'
        logs[1].seek(0)
        assert 'is currently occupied' in logs[1].read(), 'synthetic wall did not enter SCAN occupancy'
        assert not records(blocked,'executor','arrived')
        assert not any(ns(m.header.stamp)==blocked for m in acks)
        pump(.1)
        assert abs(commands[-1].linear.x)+abs(commands[-1].linear.y)+abs(commands[-1].angular.z)<1e-8
        print('PASS occupied short endpoint restricted to free prefix; no false arrival; failure latches zero: '+str(blocked_reasons),flush=True)
    except Exception:
        for log in logs:
            log.seek(0)
            print(log.read()[-8000:])
        raise
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in reversed(processes):
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
