#!/usr/bin/python3
"""Exercise the independent executor when the planner emits no valid spline."""
import os
import signal
import subprocess
import sys
import tempfile
import time


def main():
    os.environ['ROS_DOMAIN_ID']=str(170+os.getpid()%50)
    os.environ['ROS_LOG_DIR']=tempfile.mkdtemp(prefix='scan_plan_wait_')
    import rclpy
    from geometry_msgs.msg import Point, PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from scan_planner.msg import Bspline
    from builtin_interfaces.msg import Time
    from std_msgs.msg import String

    rclpy.init();node=rclpy.create_node('plan_wait_test')
    goals=node.create_publisher(PoseStamped,'/scan/goal',10)
    splines=node.create_publisher(Bspline,'/scan/planning/bspline',10)
    odometry=node.create_publisher(Odometry,'/test/odom',10)
    failures=[];commands=[]
    node.create_subscription(String,'/scan/goal_plan_failed',lambda m:failures.append(m.data),10)
    node.create_subscription(Twist,'/cmd_vel',commands.append,100)
    log=tempfile.TemporaryFile(mode='w+')
    process=subprocess.Popen([sys.argv[1],'--ros-args','-p','body_pose_topic:=/test/odom',
        '-p','first_trajectory_timeout:=0.4','-p','terminal_failure_seconds:=8.0'],
        stdout=log,stderr=log,start_new_session=True)

    def pump(seconds, extra=None):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            odom=Odometry();odom.header.stamp=node.get_clock().now().to_msg()
            odom.pose.pose.position.z=.32;odom.pose.pose.orientation.w=1.
            odometry.publish(odom)
            if extra:extra()
            rclpy.spin_once(node,timeout_sec=.01)

    def goal():
        g=PoseStamped();g.header.frame_id='map';g.header.stamp=node.get_clock().now().to_msg()
        g.pose.position=Point(x=2.,z=.32);g.pose.orientation.w=1.;goals.publish(g)
        return g

    def spline(g):
        b=Bspline(order=3,traj_id=1,start_time=Time(sec=g.header.stamp.sec,
            nanosec=g.header.stamp.nanosec),knots=[0.]*4+[4.]*4)
        b.pos_pts=[Point(x=i*2./3,z=.32) for i in range(4)]
        return b

    def failure(g):
        stamp=g.header.stamp.sec*10**9+g.header.stamp.nanosec
        return f'{stamp}|planning_wait_timeout'

    try:
        end=time.monotonic()+8.
        while not goals.get_subscription_count() and time.monotonic()<end:pump(.03)
        assert goals.get_subscription_count(),'executor not discovered'
        pump(.15)
        first=goal();started=time.monotonic()
        old=spline(first);old.start_time.sec-=1
        malformed=spline(first);malformed.knots[4]=float('nan')
        def noise():
            goals.publish(first)  # Repeated goal stamp cannot postpone timeout.
            splines.publish(old);splines.publish(malformed)
        while failure(first) not in failures and time.monotonic()-started<1.3:pump(.02,noise)
        assert failure(first) in failures,failures
        assert time.monotonic()-started<1.3
        pump(.15,lambda:splines.publish(spline(first)))
        assert sum(abs(v) for v in (commands[-1].linear.x,commands[-1].linear.y,commands[-1].angular.z))<1e-8
        assert failures.count(failure(first))==1
        second=goal();pump(.08);splines.publish(spline(second));pump(.7)
        assert failure(second) not in failures,'valid current trajectory did not cancel watchdog'
        assert any(abs(c.linear.x)+abs(c.angular.z)>1e-5 for c in commands[-20:])
        third=goal();pump(.65)
        assert failure(third) in failures,failures
        print('Planner wait: timeout, stale/malformed rejection, late spline latch, fresh-goal recovery passed')
    except Exception:
        log.seek(0);print(log.read()[-6000:]);raise
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
