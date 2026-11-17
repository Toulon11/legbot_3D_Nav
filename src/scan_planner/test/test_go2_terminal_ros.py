#!/usr/bin/python3
"""Run the real velocity adapter with synthetic splines, odometry and ROS time."""
import math
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def main():
    os.environ['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 70)
    os.environ['ROS_LOG_DIR'] = tempfile.mkdtemp(prefix='go2_terminal_test_')
    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import Point, PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from scan_planner.msg import Bspline, GoalRequest
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, DurabilityPolicy

    rclpy.init()
    node = rclpy.create_node('go2_terminal_test')
    clock_pub = node.create_publisher(Clock, '/clock', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    odom_pub = node.create_publisher(Odometry, '/test/odom', 20)
    goal_pub = node.create_publisher(PoseStamped, '/scan/goal', 10)
    request_pub = node.create_publisher(GoalRequest, '/scan/goal_request', 10)
    spline_pub = node.create_publisher(Bspline, '/scan/planning/bspline', 10)
    commands, acks, failures, stops = [], [], [], []
    node.create_subscription(Twist, '/cmd_vel', commands.append, 100)
    node.create_subscription(String,'/scan/execution_stop_status',lambda m:stops.append(json.loads(m.data)),100)
    node.create_subscription(PoseStamped, '/scan/goal_stop_ack', acks.append, 10)
    node.create_subscription(String, '/scan/goal_plan_failed', lambda m: failures.append(m.data), 10)
    sim = 10.0
    x = y = yaw = pitch = 0.0
    z = .32
    simulate = False
    process = None
    log = None

    def stamp(value):
        ns = round(value * 1_000_000_000)
        return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

    def ns(value):
        return value.sec*1_000_000_000+value.nanosec

    def pump(seconds):
        nonlocal sim, x, y, yaw
        for _ in range(max(1, math.ceil(seconds/.02))):
            assert process.poll() is None, f'adapter exited {process.returncode}'
            dt = .02
            if simulate and commands:
                cmd = commands[-1]
                # Model the measured policy dead zone: tiny commands do not step.
                if math.hypot(cmd.linear.x,cmd.linear.y) >= .075:
                    x += dt*(math.cos(yaw)*cmd.linear.x-math.sin(yaw)*cmd.linear.y)
                    y += dt*(math.sin(yaw)*cmd.linear.x+math.cos(yaw)*cmd.linear.y)
                yaw = math.remainder(yaw+dt*cmd.angular.z,2*math.pi)
            sim += dt
            clock_pub.publish(Clock(clock=stamp(sim)))
            msg = Odometry()
            msg.header.frame_id = 'map'
            msg.header.stamp = stamp(sim)
            msg.pose.pose.position = Point(x=x,y=y,z=z)
            msg.pose.pose.orientation.x = -math.sin(pitch/2)*math.sin(yaw/2)
            msg.pose.pose.orientation.y = math.sin(pitch/2)*math.cos(yaw/2)
            msg.pose.pose.orientation.z = math.cos(pitch/2)*math.sin(yaw/2)
            msg.pose.pose.orientation.w = math.cos(pitch/2)*math.cos(yaw/2)
            odom_pub.publish(msg)
            until = time.monotonic()+.006
            while time.monotonic()<until:
                rclpy.spin_once(node,timeout_sec=.001)

    def wait(condition, seconds=3):
        until = sim+seconds
        while sim < until:
            if condition(): return
            pump(.02)
        assert condition(), f'timeout: failures={failures[-3:]}, cmd={commands[-1] if commands else None}'

    def start(max_speed=.5):
        nonlocal process, log
        log = tempfile.TemporaryFile(mode='w+')
        process = subprocess.Popen([sys.argv[1],'--ros-args',
            '-p','use_sim_time:=true','-p','body_pose_topic:=/test/odom',
            '-p','finish_dist:=0.40','-p','finish_dist_z:=0.2','-p','use_goal_z:=true',
            '-p','goal_slowdown_distance:=0.55',
            '-p','min_walk_speed:=0.08',
            '-p','terminal_failure_seconds:=2.0','-p','execution_timeout:=6.0',
            '-p',f'max_vx:={max_speed}'], stdout=log,stderr=log,start_new_session=True)
        wall_end = time.monotonic()+8
        while time.monotonic()<wall_end and not goal_pub.get_subscription_count():
            pump(.04)
        assert goal_pub.get_subscription_count(), 'no adapter discovery'
        pump(.3)

    def stop():
        nonlocal process, log
        if process and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
        process = None
        if log:log.close();log=None

    def publish_spline(goal, endpoint=None, constant=False, duration=4.):
        endpoint = endpoint or goal.pose.position
        msg = Bspline()
        msg.order = 3
        msg.traj_id = ns(goal.header.stamp)
        msg.start_time = goal.header.stamp
        msg.knots = [0.]*4+[duration]*4
        for i in range(4):
            f = 1. if constant else i/3
            msg.pos_pts.append(Point(x=x+(endpoint.x-x)*f,
                y=y+(endpoint.y-y)*f,z=z+(endpoint.z-z)*f))
        spline_pub.publish(msg)

    def goal(distance=.41, height=.32, constant=False, endpoint=None, tolerance=None, duration=4.):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = stamp(sim)
        msg.pose.position = Point(x=x+distance,y=y,z=height)
        msg.pose.orientation.w = 1.
        if tolerance is None:
            goal_pub.publish(msg)
        else:
            request_pub.publish(GoalRequest(target=msg,has_xy_tolerance=True,xy_tolerance=tolerance))
        pump(.08)
        publish_spline(msg,endpoint,constant,duration)
        pump(.12)
        return msg

    def failed(g, reason):
        return f'{ns(g.header.stamp)}|{reason}' in failures

    def arrived(g):
        return any(ns(m.header.stamp)==ns(g.header.stamp) for m in acks)

    try:
        start()
        yaw = .8
        pump(.1)
        first = goal()
        pump(.35)
        assert abs(commands[-1].linear.x)+abs(commands[-1].linear.y)<1e-8
        assert commands[-1].angular.z < -.5, 'distance still scales angular velocity'
        yaw = .4
        pump(.18)
        assert abs(commands[-1].linear.x)+abs(commands[-1].linear.y)<1e-8, 'hysteresis exited too soon'
        yaw = .29
        pump(.35)
        speed = math.hypot(commands[-1].linear.x,commands[-1].linear.y)
        # The spline's tracking command remains above the old .111 m/s
        # distance cap here, even though the body is in the terminal zone.
        assert .12<speed<.17,speed
        pump(.9)
        yaw = .4  # Between exit/enter: keep translating after leaving rotation.
        pump(.15)
        assert abs(commands[-1].angular.z)<.6, 'yaw hysteresis unexpectedly re-entered rotation'
        x += .05
        pump(.12)
        assert not arrived(first), 'arrival acknowledged before physical settling'
        wait(lambda:arrived(first),2.)
        count = len(acks)
        x -= .04  # Arrival stays latched despite inertia/noise after the receipt.
        pump(.2)
        assert len(acks)==count and math.hypot(commands[-1].linear.x,commands[-1].linear.y)==0

        # Physical turning progress lasts longer than the no-progress budget;
        # each phase has its own clock, so translation has time after aligning.
        x=y=0.;yaw=1.2
        pump(.1)
        turning = goal()
        for angle in (1.1,1.0,.9,.8):
            pump(.7);yaw=angle;pump(.08)
        assert not any(f.startswith(str(ns(turning.header.stamp))+'|') for f in failures)
        yaw=.2;pump(.3)
        wait(lambda:math.hypot(commands[-1].linear.x,commands[-1].linear.y)>.04,1.)
        x=.05;pump(.1);assert not arrived(turning)
        wait(lambda:arrived(turning),2.)

        # A slow spline falls below the modelled gait dead zone. Keep moving
        # until the 15 cm tolerance, then release the floor and settle.
        x=y=yaw=0.;pump(.3)
        walking = goal(distance=.21,tolerance=.15,duration=12.)
        stop_count=len(stops)
        x=walking.pose.position.x-.155
        pump(.36)
        assert not arrived(walking)
        assert len(stops)==stop_count, 'stop began outside the arrival tolerance'
        assert .079<=commands[-1].linear.x<=.081, 'spline fell below the walking floor'
        simulate=True
        wait(lambda:arrived(walking),2.)
        simulate=False
        assert .12<math.hypot(walking.pose.position.x-x,walking.pose.position.y-y)<=.15
        assert not failed(walking,'execution_no_progress')
        assert any(r['phase']=='settled' for r in stops[stop_count:])
        assert math.hypot(commands[-1].linear.x,commands[-1].linear.y)==0

        # Enter the arrival circle with an active command, then keep the body
        # physically drifting even after cmd_vel becomes zero. A receipt is
        # valid only after both command and actual motion settle.
        x=y=yaw=0.;pump(.3)
        braking=goal(distance=.70)
        wait(lambda:commands[-1].linear.x>.20)
        before_stop=commands[-1].linear.x
        stop_count=len(stops)
        x=.31;pump(.08)  # 0.39 m: begin braking on entry into the 0.40 m tolerance.
        assert len(stops)>stop_count, 'did not begin stopping at the arrival tolerance'
        assert not arrived(braking)
        assert 0.<commands[-1].linear.x<before_stop, 'normal stop jumped directly to zero'
        stop_count=len(stops)
        for _ in range(110):
            x+=.001;pump(.02)
        assert math.hypot(commands[-1].linear.x,commands[-1].linear.y)<1e-8
        assert not arrived(braking), 'zero cmd_vel was mistaken for zero actual velocity'
        recent=stops[stop_count:]
        assert any(r.get('actual_speed',0)>.03 and r['command_speed']<.001 for r in recent)
        wait(lambda:arrived(braking),2.)
        assert any(r['phase']=='settled' and r.get('actual_speed',1)<=.03 for r in stops[stop_count:])
        settled=next(r for r in reversed(stops) if r['phase']=='settled')
        assert settled['stable_duration']>=settled['required_stable_duration']
        assert len(settled['actual_velocity_xyz'])==len(settled['actual_rotation_rate_xyz'])==3
        assert settled['stop_timeout']==5., 'diagnostics must not extend the stop budget'

        # A fixed XY position is insufficient while the body heaves/pitches.
        x=y=yaw=0.;z=.32;pump(.3)
        rocking=goal(distance=.70)
        wait(lambda:commands[-1].linear.x>.20)
        stop_count=len(stops);x=.31
        for _ in range(100):
            z=.32+.012*math.sin(sim*8.)
            pitch=.08*math.sin(sim*8.)
            pump(.02)
        assert not arrived(rocking), 'body height/pitch motion was ignored during settling'
        recent=stops[stop_count:]
        assert any(abs(r.get('actual_velocity_xyz',[0,0,0])[2])>.03 for r in recent)
        assert any(abs(r.get('actual_rotation_rate_xyz',[0,0,0])[1])>.1 for r in recent)
        pitch=0.;z=.32
        wait(lambda:arrived(rocking),2.)

        # A frozen yaw must produce one specific failure, never eternal spinning.
        x=y=0.;yaw=1.;pump(.1)
        stuck_turn = goal()
        wait(lambda:failed(stuck_turn,'execution_heading_no_progress'))
        publish_spline(stuck_turn);pump(.3)
        assert failures.count(f'{ns(stuck_turn.header.stamp)}|execution_heading_no_progress')==1
        assert abs(commands[-1].angular.z)<1e-8

        # New splines for the same goal do not reset its physical-progress clock.
        x=y=yaw=0.;pump(.1)
        stuck_walk = goal()
        for _ in range(8):
            publish_spline(stuck_walk);pump(.3)
        assert failed(stuck_walk,'execution_no_progress')
        assert not arrived(stuck_walk)

        # A constant or exhausted local spline cannot authorize a fabricated
        # straight-line motion, or acknowledge a still-distant final target.
        no_path = goal(constant=True)
        pump(.3)
        assert math.hypot(commands[-1].linear.x,commands[-1].linear.y)==0
        wait(lambda:failed(no_path,'execution_no_progress'))
        exhausted = goal(distance=1.,endpoint=Point(x=x,y=y,z=z))
        wait(lambda:failed(exhausted,'execution_no_progress'))
        assert not arrived(exhausted)

        # Preserve the Z arrival gate and the old-goal spline filter.
        wrong_height = goal(distance=.39,height=1.0)
        pump(.3);assert not arrived(wrong_height)
        replacement = PoseStamped()
        replacement.header.frame_id='map';replacement.header.stamp=stamp(sim)
        replacement.pose.position=Point(x=x+.41,y=y,z=z);replacement.pose.orientation.w=1.
        goal_pub.publish(replacement);pump(.1)
        publish_spline(wrong_height);pump(.15)
        assert math.hypot(commands[-1].linear.x,commands[-1].linear.y)==0, 'old spline drove new goal'
        publish_spline(replacement)
        wait(lambda:math.hypot(commands[-1].linear.x,commands[-1].linear.y)>.04,1.5)

        # Phase switches and continuing progress cannot evade the total budget.
        yaw=.8;pump(.1)
        bounded = goal()
        for i in range(24):
            yaw=.8 if i%2==0 else .2
            x=0. if i%2==0 else -.02
            publish_spline(bounded);pump(.3)
            if failed(bounded,'execution_timeout'):break
        assert failed(bounded,'execution_timeout'), failures
        print('PASS: terminal yaw hysteresis, spline slowdown, walking floor, controlled stop, '
              'phase progress, stamped failures, trajectory guards, Z gate, total timeout')
        stop()
        # The configured maximum applies before the terminal approach.
        start(.10)
        x=y=yaw=0.;pump(.1)
        goal(distance=.70);pump(.4)
        wait(lambda:math.hypot(commands[-1].linear.x,commands[-1].linear.y)>=.099,1.5)
        speed=math.hypot(commands[-1].linear.x,commands[-1].linear.y)
        assert .099<=speed<=.101,speed
        print('PASS: configured .10 m/s speed limit')
    except Exception:
        if log:
            log.seek(0);print(log.read()[-12000:])
        raise
    finally:
        stop();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
