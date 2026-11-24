#!/usr/bin/python3
"""Exercise the installed coordinator, with no motors, planner or Gazebo."""
import os
import math
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    os.environ['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 70)
    os.environ['ROS_LOG_DIR'] = tempfile.mkdtemp(prefix='go2_idle_')
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from scan_planner.msg import GoalRequest
    from sensor_msgs.msg import JointState, PointCloud2
    from std_msgs.msg import Bool, Int32, String
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from controller_manager_msgs.srv import ListControllers
    from controller_manager_msgs.msg import ControllerState

    rclpy.init()
    node = rclpy.create_node('execution_idle_test')
    def controllers(_request, response):
        response.controller = [ControllerState(name='rl_quadruped_controller', state='active')]
        return response
    node.create_service(ListControllers, '/controller_manager/list_controllers', controllers)
    modes, ready, enables, states = [], [], [], []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(String, '/go2/execution_state',
            lambda m: states.append(json.loads(m.data)), latched),
        node.create_subscription(Int32, '/go2/control_mode',
            lambda m: modes.append((time.monotonic(), m.data)), 100),
        node.create_subscription(Bool, '/go2/demo_ready', lambda m: ready.append(m.data), latched),
        node.create_subscription(String, '/scan/recovery_action_enabled', lambda m: enables.append(m.data), 100)]
    pubs = {key: node.create_publisher(kind, topic,
        latched if key in ('plan', 'complete', 'task') else 20) for key, kind, topic in (
        ('task', Bool, '/exploration/task_active'),
        ('goal', PoseStamped, '/scan/goal'), ('request', GoalRequest, '/scan/goal_request'), ('ack', PoseStamped, '/scan/goal_stop_ack'),
        ('failure', String, '/scan/goal_plan_failed'), ('plan', Bool, '/scan/plan_ready'),
        ('complete', Bool, '/scan/mission_complete'), ('action', String, '/scan/recovery_action_state'),
        ('odom', Odometry, '/test/body'), ('cloud', PointCloud2, '/test/cloud'),
        ('joints', JointState, '/joint_states'), ('cmd', Twist, '/cmd_vel'))}
    coordinator = Path(get_package_prefix('legbot_bringup'))/'lib/legbot_bringup/go2_demo_sequencer'
    log = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen([str(coordinator), '--ros-args',
        '-p', 'odom_topic:=/test/body', '-p', 'cloud_topic:=/test/cloud',
        '-p', 'crouch_wait_seconds:=0.02', '-p', 'stand_wait_seconds:=0.02',
        '-p', 'mode_pulse_seconds:=0.10'], stdout=log, stderr=log, start_new_session=True)

    motion = {'speed': 0., 'yaw_rate': 0., 'tilt': 0., 'command': 0., 'poses': True,
              'commands': True, 'repeat_stamp': False, 'x': 0., 'yaw': 0.}
    last_pose_wall = time.monotonic()
    saved_stamp = None

    def pump(duration=0., until=None, timeout=5.):
        nonlocal last_pose_wall, saved_stamp
        end = time.monotonic() + (timeout if until else duration)
        while time.monotonic() < end:
            assert process.poll() is None, 'coordinator exited'
            odom = Odometry()
            odom.header.frame_id = 'map'
            current = time.monotonic()
            dt = current - last_pose_wall
            last_pose_wall = current
            motion['x'] += motion['speed'] * dt
            motion['yaw'] += motion['yaw_rate'] * dt
            odom.pose.pose.position.x = motion['x']
            yaw, pitch = motion['yaw'] / 2, motion['tilt'] / 2
            q = odom.pose.pose.orientation
            q.x = -math.sin(pitch) * math.sin(yaw)
            q.y = math.sin(pitch) * math.cos(yaw)
            q.z = math.cos(pitch) * math.sin(yaw)
            q.w = math.cos(pitch) * math.cos(yaw)
            if saved_stamp is None or not motion['repeat_stamp']:
                saved_stamp = node.get_clock().now().to_msg()
            odom.header.stamp = saved_stamp
            # Twist is deliberately zero even when poses show actual motion.
            if motion['poses']:
                pubs['odom'].publish(odom)
            pubs['joints'].publish(JointState(name=['j']*12, position=[0.]*12))
            cloud = PointCloud2()
            cloud.header.frame_id = 'map'
            cloud.width = cloud.height = 1
            pubs['cloud'].publish(cloud)
            if motion['commands']:
                cmd = Twist(); cmd.linear.x = motion['command']
                pubs['cmd'].publish(cmd)
            rclpy.spin_once(node, timeout_sec=.01)
            if until and until(): return
        if until: raise AssertionError('timeout waiting for coordinator output')

    def pose(stamp):
        m = PoseStamped()
        m.header.stamp.sec = stamp
        m.header.frame_id = 'map'
        m.pose.orientation.w = 1.
        return m

    def goal(stamp, extended=False):
        if extended:
            request=GoalRequest();request.target=pose(stamp);request.goal_id=stamp
            request.has_xy_tolerance=True;request.xy_tolerance=.15
            pubs['request'].publish(request)
        else:
            pubs['goal'].publish(pose(stamp))
        pump(.10)

    def ack(stamp):
        sent = time.monotonic()
        pubs['ack'].publish(pose(stamp))
        pump(.05)
        return sent

    def no_stand(since):
        assert not any(t >= since and mode == 2 for t,mode in modes), modes[-12:]

    try:
        pump(until=lambda: any(ready) and pubs['goal'].get_subscription_count() > 0)
        goal(1)
        pubs['plan'].publish(Bool(data=True))
        pump(until=lambda: any(mode == 3 for _,mode in modes))
        pump(.2)
        active = time.monotonic()
        pump(1.65)
        no_stand(active)  # An active task may temporarily command zero.
        first = ack(1)
        pump(.35)
        goal(2, extended=True)  # Cancel idle before 1.5 seconds; planning may take longer.
        pubs['plan'].publish(Bool(data=False))
        pubs['ack'].publish(pose(1))
        pubs['complete'].publish(Bool(data=True))  # Unstamped old completion.
        pump(1.65)
        no_stand(first)
        pubs['plan'].publish(Bool(data=True))
        pump(.15)
        second = ack(2)
        # Physical stopping and fresh sensor feedback are independent of ACK
        # and zero command. Each obstruction must hold mode 3 past 1.5s.
        motion['speed'] = .16
        pump(1.65)
        no_stand(second)
        assert states[-1]['stand_gate'] == 'body_still_moving', states[-1]
        motion['speed'], motion['yaw_rate'] = 0., .3
        pump(.8)
        no_stand(second)
        assert states[-1]['stand_gate'] == 'body_still_rotating', states[-1]
        motion['yaw_rate'], motion['tilt'] = 0., math.radians(15)
        pump(.8)
        no_stand(second)
        assert states[-1]['stand_gate'] == 'body_tilt_exceeds_limit', states[-1]
        motion['tilt'], motion['repeat_stamp'] = 0., True
        pump(.8)
        no_stand(second)
        assert states[-1]['stand_gate'] in ('body_pose_stale', 'body_pose_window_incomplete'), states[-1]
        motion['repeat_stamp'], motion['command'] = False, .1
        pump(.8)
        no_stand(second)
        assert states[-1]['stand_gate'] == 'command_not_zero', states[-1]
        motion['command'], motion['commands'] = 0., False
        pump(.5)
        no_stand(second)
        assert states[-1]['stand_gate'] == 'command_stale', states[-1]
        motion['commands'] = True
        # Duplicate acknowledgments must not restart the inactivity timer.
        for _ in range(3):
            pubs['ack'].publish(pose(2))
            pubs['complete'].publish(Bool(data=False))
            pump(.2)
        pump(until=lambda: any(t >= second and mode == 2 for t,mode in modes))
        stood = next(t for t,mode in modes if t >= second and mode == 2)
        assert stood-second > 5., stood-second
        pump(.2)
        already_standing = time.monotonic()
        goal(3)
        ack(3)  # Goal already in the arrival circle: stay standing.
        pump(1.65)
        assert not any(t >= already_standing for t,_ in modes), 'unnecessary stand/walk toggle'
        pubs['action'].publish(String(data='4000000000|accepted'))
        pump(until=lambda: '4000000000' in enables)
        pubs['task'].publish(Bool(data=True))
        finished = time.monotonic()
        pubs['action'].publish(String(data='4000000000|finished'))
        pump(.35)
        goal(5, extended=True)
        pubs['plan'].publish(Bool(data=True))
        pubs['action'].publish(String(data='4000000000|finished'))
        pubs['failure'].publish(String(data='3000000000|old_failure'))
        pump(1.65)
        no_stand(finished)
        failed = time.monotonic()
        pubs['failure'].publish(String(data='5000000000|execution_no_progress'))
        pump(1.65)
        no_stand(failed)
        assert states[-1]['stand_gate'] == 'exploration_active', states[-1]
        goal(6, extended=True)
        pubs['plan'].publish(Bool(data=True))
        pump(.2)
        arrived = ack(6)
        pump(1.65)
        no_stand(arrived)
        assert states[-1]['stand_gate'] == 'exploration_active', states[-1]
        ended = time.monotonic()
        pubs['task'].publish(Bool(data=False))
        pump(until=lambda: any(t >= failed and mode == 2 for t,mode in modes))
        assert next(t for t,mode in modes if t >= failed and mode == 2)-ended >= 1.45
        print('PASS: stamped results/preemption, actual linear/angular pose motion, tilt, stale/replayed poses, nonzero/stale commands, exploration hold and stable task-end stand')
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5)
        log.seek(0)
        print(log.read())
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
