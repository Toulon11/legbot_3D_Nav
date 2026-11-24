#!/usr/bin/python3
"""Paused/slow simulation time must not consume actuator startup waits."""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    os.environ['ROS_DOMAIN_ID']=str(150+os.getpid()%70)
    os.environ['ROS_LOG_DIR']=tempfile.mkdtemp(prefix='go2_startup_clock_')
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState,PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String,Bool,Int32
    from controller_manager_msgs.srv import ListControllers
    from controller_manager_msgs.msg import ControllerState
    rclpy.init();node=rclpy.create_node('startup_clock_test')
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    states=[];ready=[];modes=[];controller_active=False
    def controllers(_request,response):
        response.controller=[ControllerState(name='rl_quadruped_controller',
                                             state='active' if controller_active else 'inactive')]
        return response
    node.create_service(ListControllers,'/controller_manager/list_controllers',controllers)
    node.create_subscription(Int32,'/go2/control_mode',lambda m:modes.append(m.data),20)
    node.create_subscription(String,'/go2/execution_state',lambda m:states.append(json.loads(m.data)),retained)
    node.create_subscription(Bool,'/go2/demo_ready',lambda m:ready.append(m.data),retained)
    clock=node.create_publisher(Clock,'/clock',20)
    joints=node.create_publisher(JointState,'/joint_states',20)
    odom=node.create_publisher(Odometry,'/test/body',20)
    cloud=node.create_publisher(PointCloud2,'/test/cloud',20)
    executable=Path(get_package_prefix('legbot_bringup'))/'lib/legbot_bringup/go2_demo_sequencer'
    log=tempfile.TemporaryFile(mode='w+')
    process=subprocess.Popen([str(executable),'--ros-args','-p','use_sim_time:=true',
        '-p','odom_topic:=/test/body','-p','cloud_topic:=/test/cloud',
        '-p','mode_pulse_seconds:=0.1','-p','crouch_wait_seconds:=0.2',
        '-p','stand_wait_seconds:=0.2'],stdout=log,stderr=log,start_new_session=True)

    def pump(seconds,stamp):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            assert process.poll() is None,'coordinator exited'
            c=Clock();c.clock.sec=int(stamp);c.clock.nanosec=int((stamp-int(stamp))*1e9);clock.publish(c)
            joints.publish(JointState(name=['j']*12,position=[0.]*12))
            o=Odometry();o.header.frame_id='map';o.header.stamp=c.clock;o.pose.pose.orientation.w=1.;odom.publish(o)
            p=PointCloud2();p.header.frame_id='map';p.width=p.height=1;cloud.publish(p)
            rclpy.spin_once(node,timeout_sec=.01)

    try:
        # Discover subscriptions while ROS time is paused at zero. Timers
        # themselves also use ROS time, so advance small ticks to observe
        # each transition instead of expecting callbacks on a frozen clock.
        until=time.monotonic()+5.
        while (not joints.get_subscription_count() or not clock.get_subscription_count()) and time.monotonic()<until:
            pump(.1,0.)
        assert joints.get_subscription_count() and clock.get_subscription_count()
        pump(.3,0.)
        pump(.5,10.)
        assert not any(ready),'paused clock released navigation'
        assert not modes,'joint broadcaster alone released startup pulses before actuator activation'
        assert states[-1]['state']=='waiting_for_joint_states',states[-1:]
        controller_active=True
        for i in range(1,19):
            pump(.25,10.+i*.06)
            if i==5:
                assert states[-1]['state']=='crouch_wait' and not any(ready),states[-1:]
        assert any(ready) and states[-1]['state']=='waiting_for_scan_plan',states[-1:]
        print('Startup waits for actuator activation; pulses follow simulation time and pause safely')
    except Exception:
        log.seek(0);print(log.read());raise
    finally:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=4)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown();log.close()


if __name__=='__main__':main()
