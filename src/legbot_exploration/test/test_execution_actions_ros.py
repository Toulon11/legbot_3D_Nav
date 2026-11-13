#!/usr/bin/python3
"""Exercise real execution/coordinator processes, without motors or Gazebo."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    os.environ['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 70)
    os.environ['ROS_LOG_DIR'] = tempfile.mkdtemp(prefix='execution_actions_')
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState, PointCloud2
    from std_msgs.msg import Bool, Int32, String
    from rclpy.qos import QoSProfile, DurabilityPolicy

    adapter = Path(get_package_prefix('scan_planner'))/'lib/scan_planner/scan_go2_cmd_adapter'
    coordinator = Path(get_package_prefix('legbot_bringup'))/'lib/legbot_bringup/go2_demo_sequencer'
    rclpy.init()
    node = rclpy.create_node('execution_action_test')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    outputs = {k: [] for k in ('cmd', 'mode', 'state', 'enable', 'turn', 'reverse', 'failure', 'ready')}
    subscriptions = []
    for key, kind, topic in (
        ('cmd', Twist, '/cmd_vel'), ('mode', Int32, '/go2/control_mode'),
        ('state', String, '/scan/recovery_action_state'),
        ('enable', String, '/scan/recovery_action_enabled'),
        ('turn', String, '/scan/recovery_turn_result'),
        ('reverse', String, '/scan/recovery_reverse_result'),
        ('failure', String, '/scan/goal_plan_failed'), ('ready', Bool, '/go2/demo_ready')):
        subscriptions.append(node.create_subscription(kind, topic,
            lambda m, key=key: outputs[key].append((time.monotonic(), m)),
            latched if key == 'ready' else 20))
    pubs = {}
    for key, kind, topic in (
        ('goal', PoseStamped, '/scan/goal'),
        ('ack', PoseStamped, '/scan/goal_stop_ack'),
        ('odom', Odometry, '/test/body'), ('joint', JointState, '/joint_states'),
        ('cloud', PointCloud2, '/test/cloud'),
        ('turn', String, '/scan/recovery_turn_request'),
        ('reverse', String, '/scan/recovery_reverse_request'),
        ('cancel', String, '/scan/recovery_action_cancel'),
        ('plan', Bool, '/scan/plan_ready'), ('complete', Bool, '/scan/mission_complete')):
        pubs[key] = node.create_publisher(kind, topic, latched if key in ('plan', 'complete') else 20)
    processes = []
    logs = []
    x = y = yaw = 0.0
    simulate = False
    last_step = time.monotonic()

    def pump(duration=0.0, until=None, timeout=6):
        nonlocal x, y, yaw, last_step
        end = time.monotonic() + (timeout if until else duration)
        while time.monotonic() < end:
            for process in processes:
                assert process.poll() is None, f'process exited: {process.args}'
            now = time.monotonic()
            dt = min(now-last_step, .05)
            last_step = now
            if simulate and outputs['cmd']:
                command = outputs['cmd'][-1][1]
                x += dt*(math.cos(yaw)*command.linear.x - math.sin(yaw)*command.linear.y)
                y += dt*(math.sin(yaw)*command.linear.x + math.cos(yaw)*command.linear.y)
                yaw += dt*command.angular.z
                yaw = math.remainder(yaw, 2*math.pi)
            msg = Odometry()
            msg.header.frame_id = 'map'
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = .32
            msg.pose.pose.orientation.z = math.sin(yaw/2)
            msg.pose.pose.orientation.w = math.cos(yaw/2)
            pubs['odom'].publish(msg)
            pubs['joint'].publish(JointState(name=[f'j{i}' for i in range(12)], position=[0.]*12))
            cloud = PointCloud2()
            cloud.header = msg.header
            cloud.width = cloud.height = 1
            pubs['cloud'].publish(cloud)
            rclpy.spin_once(node, timeout_sec=.01)
            if until and until():
                return
        if until:
            raise AssertionError('timeout; last outputs: '+str({k: [getattr(m, 'data', None) for _,m in v[-3:]] for k,v in outputs.items() if k!='cmd'}))

    def start(command):
        log = tempfile.TemporaryFile(mode='w+')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        processes.append(process)
        return process

    def has(key, value):
        return any(m.data == value for _, m in outputs[key])

    def stopped_since(when):
        return all(abs(m.linear.x)+abs(m.linear.y)+abs(m.angular.z)<1e-8
                   for t,m in outputs['cmd'] if t >= when)

    try:
        start([str(adapter), '--ros-args', '-p', 'use_sim_time:=false',
               '-p', 'body_pose_topic:=/test/body', '-p', 'recovery_turn_timeout:=2.0'])
        pump(until=lambda: pubs['turn'].get_subscription_count()>0 and bool(outputs['cmd']))
        pump(.3)  # allow odometry DDS discovery before accepting an action
        # No execution coordinator: the action must not move and must fail.
        began = time.monotonic()
        pubs['turn'].publish(String(data='100|0.3'))
        pump(until=lambda: has('failure', '100|turn_activation_timeout'))
        assert stopped_since(began), 'movement without gait activation'
        start([str(coordinator), '--ros-args', '-p', 'use_sim_time:=false',
               '-p', 'odom_topic:=/test/body', '-p', 'cloud_topic:=/test/cloud',
               '-p', 'crouch_wait_seconds:=0.05', '-p', 'stand_wait_seconds:=0.05',
               '-p', 'mode_pulse_seconds:=0.10', '-p', 'goal_settle_seconds:=0.20'])
        pump(until=lambda: has('ready', True))
        # Ordinary navigation retains the existing gait activation/stand cycle.
        pubs['complete'].publish(Bool(data=False))
        pubs['plan'].publish(Bool(data=True))
        pump(until=lambda: has('mode', 3))
        pump(.20)
        finished = time.monotonic()
        pubs['complete'].publish(Bool(data=True))
        pump(until=lambda: any(t>finished and m.data==2 for t,m in outputs['mode']))
        assert next(t for t,m in outputs['mode'] if t>finished and m.data==2)-finished >= .18
        pump(.20)
        # Turn wraps over +/-pi, activates gait once, preserves stamped result.
        yaw = 3.05
        pump(.1)
        simulate = True
        began = time.monotonic()
        pubs['turn'].publish(String(data='200|0.30'))
        pump(until=lambda: has('turn', '200|succeeded'))
        enabled_at = next(t for t,m in outputs['enable'] if m.data=='200')
        assert all(abs(m.angular.z)<1e-8 for t,m in outputs['cmd'] if began<=t<enabled_at)
        assert abs(x)+abs(y)<1e-6
        assert yaw < -2.8, yaw
        first_result_count = len(outputs['turn'])
        pubs['turn'].publish(String(data='200|0.30'))
        pubs['cancel'].publish(String(data='199'))
        pump(.3)
        assert len(outputs['turn'])==first_result_count
        # Reverse is another execution action, using the same mode owner.
        sx, sy, syaw = x, y, yaw
        tx, ty = sx-.22*math.cos(syaw), sy-.22*math.sin(syaw)
        pubs['reverse'].publish(String(data=f'300|{sx}|{sy}|{tx}|{ty}|{syaw}'))
        pump(until=lambda: has('reverse', '300|succeeded'))
        assert (sx-x)*math.cos(syaw)+(sy-y)*math.sin(syaw) >= .17
        assert abs(math.remainder(yaw-syaw,2*math.pi))<.03
        # A shift follows the validated world segment while holding body yaw.
        sx, sy, syaw = x, y, yaw
        tx, ty = sx-.25/math.sqrt(2), sy+.25/math.sqrt(2)
        began=time.monotonic()
        pubs['reverse'].publish(String(data=f'350|{sx}|{sy}|{tx}|{ty}|{syaw}|shift'))
        pump(until=lambda: has('reverse', '350|succeeded'))
        assert math.hypot(tx-x,ty-y)<.08
        assert abs(math.remainder(yaw-syaw,2*math.pi))<.03
        assert any(t>began and abs(m.linear.y)>.025 for t,m in outputs['cmd'])
        # Correct-stamp cancellation stops; an old cancellation is ignored.
        simulate = False
        pubs['turn'].publish(String(data='400|0.5'))
        pump(until=lambda: has('enable', '400'))
        pubs['cancel'].publish(String(data='300'))
        pump(.15)
        assert not has('failure', '400|turn_cancelled')
        pubs['cancel'].publish(String(data='400'))
        pump(until=lambda: has('failure', '400|turn_cancelled'))
        pump(.1)
        assert abs(outputs['cmd'][-1][1].angular.z)<1e-8
        # Finite turn timeout lives in the adapter, never in exploration.
        pubs['turn'].publish(String(data='500|0.5'))
        pump(until=lambda: has('failure', '500|turn_timeout'))
        # Stop coordinator callbacks while the action is active: lease expires.
        pubs['turn'].publish(String(data='600|0.5'))
        pump(until=lambda: has('enable', '600'))
        os.kill(processes[1].pid, signal.SIGSTOP)
        pump(until=lambda: has('failure', '600|turn_activation_lost'))
        os.kill(processes[1].pid, signal.SIGCONT)
        pump(.15)
        assert abs(outputs['cmd'][-1][1].angular.z)<1e-8
        # A new normal goal releases recovery gating even if the old action's
        # finished event and normal Bool feedback arrive in another order.
        pubs['turn'].publish(String(data='700|0.5'))
        pump(until=lambda: has('enable', '700'))
        old_goal = PoseStamped()
        old_goal.header.stamp.nanosec = 650
        old_goal.pose.orientation.w = 1.
        pubs['goal'].publish(old_goal)
        pump(.1)
        assert not has('failure', '700|turn_preempted')
        new_goal = PoseStamped()
        new_goal.header.stamp.nanosec = 701
        new_goal.pose.orientation.w = 1.
        new_goal.pose.position.x = x+2.
        new_goal.pose.position.y = y
        pubs['goal'].publish(new_goal)
        pump(until=lambda: has('failure', '700|turn_preempted'))
        normal_started = time.monotonic()
        pubs['complete'].publish(Bool(data=False))
        pubs['plan'].publish(Bool(data=True))
        pump(.3)
        # Recovery -> navigation keeps the already-active gait; no redundant
        # stand/walk pulse is needed between these adjacent tasks.
        assert outputs['mode'][-1][1].data == 3
        assert not any(t>normal_started and m.data==2 for t,m in outputs['mode'])
        pubs['complete'].publish(Bool(data=True))
        pump(.1)
        assert not any(t>normal_started and m.data==2 for t,m in outputs['mode'])
        normal_finished = time.monotonic()
        pubs['ack'].publish(new_goal)
        pump(until=lambda: any(t>normal_finished and m.data==2 for t,m in outputs['mode']))
        mode_publishers = node.get_publishers_info_by_topic('/go2/control_mode')
        assert len(mode_publishers)==1 and mode_publishers[0].node_name=='go2_demo_sequencer', mode_publishers
        print('PASS: gait owner, activation gate, turn/wrap, reverse, diagonal shift, cancellation, timeout, lease, navigation preemption, normal stand policy')
    except Exception:
        for log in logs:
            log.seek(0)
            print(log.read()[-8000:])
        raise
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.kill(process.pid, signal.SIGCONT)
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        node.destroy_node()
        rclpy.shutdown()
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
