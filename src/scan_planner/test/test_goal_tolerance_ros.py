#!/usr/bin/python3
"""Real coordinator and executor: atomic overrides, arrival, reset and rejection.

The second phase also starts the real SCAN FSM to verify the same command is
accepted with the same effective tolerance. Odometry/trajectory are synthetic;
this is an interface and controller test, not a locomotion-policy validation.
"""
import copy,json,math,os,signal,subprocess,sys,tempfile,time
from pathlib import Path
os.environ['ROS_DOMAIN_ID']=str(225+os.getpid()%5)
os.environ['ROS_LOG_DIR']=tempfile.mkdtemp(prefix='scan_goal_tolerance_')
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point,PoseStamped,Twist
from nav_msgs.msg import Odometry
from scan_planner.msg import Bspline,GoalRequest
from std_msgs.msg import String

rclpy.init();node=rclpy.create_node('goal_tolerance_integration_test')
statuses=[];forwarded=[];legacy=[];acks=[];failures=[];commands=[];processes=[];logs=[]
for kind,topic,storage in ((String,'/scan/goal_tolerance_status',statuses),
        (GoalRequest,'/scan/goal_request',forwarded),(PoseStamped,'/scan/goal',legacy),
        (PoseStamped,'/scan/goal_stop_ack',acks),(String,'/scan/goal_plan_failed',failures),
        (Twist,'/cmd_vel',commands)):
    node.create_subscription(kind,topic,storage.append,50)
requests=node.create_publisher(GoalRequest,'/scan/waypoint_request',10)
old_inputs=node.create_publisher(PoseStamped,'/scan/waypoint_input',10)
direct=node.create_publisher(GoalRequest,'/scan/goal_request',10)
splines=node.create_publisher(Bspline,'/scan/planning/bspline',10)
odometry=node.create_publisher(Odometry,'/tolerance_test/odom',20)
x=0.;next_odom=0.
ns=lambda s:s.sec*1_000_000_000+s.nanosec

def start(command):
    log=tempfile.TemporaryFile(mode='w+');logs.append(log)
    p=subprocess.Popen(command,stdout=log,stderr=log,start_new_session=True);processes.append(p);return p

def pump(seconds=.1):
    global next_odom
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        assert all(p.poll() is None for p in processes),'child process exited'
        if time.monotonic()>=next_odom:
            m=Odometry();m.header.frame_id='map';m.header.stamp=node.get_clock().now().to_msg()
            m.pose.pose.position=Point(x=x,y=0.,z=.32);m.pose.pose.orientation.w=1.
            odometry.publish(m);next_odom=time.monotonic()+.02
        rclpy.spin_once(node,timeout_sec=.005)

def wait(condition,timeout=6):
    until=time.monotonic()+timeout
    while time.monotonic()<until and not condition():pump(.03)
    assert condition(),'timeout; statuses='+str([s.data for s in statuses[-5:]])+' failures='+str([s.data for s in failures[-3:]])

def records(stamp,source='executor',event='accepted'):
    return [d for m in statuses for d in [json.loads(m.data)]
            if d.get('stamp_ns')==ns(stamp) and d.get('source')==source and d.get('event')==event]

def arrived(target):return any(ns(a.header.stamp)==ns(target.header.stamp) for a in acks)

def submit(tolerance=None):
    m=GoalRequest();m.target.header.frame_id='map';m.target.header.stamp=node.get_clock().now().to_msg()
    m.target.pose.position=Point(x=x+.3,y=0.,z=.32);m.target.pose.orientation.w=1.
    m.goal_id=len(forwarded)+10;m.attempt_id=m.goal_id+100;m.waypoint_index=2
    m.has_xy_tolerance=tolerance is not None;m.xy_tolerance=0. if tolerance is None else tolerance
    previous=len(forwarded);requests.publish(m);wait(lambda:len(forwarded)>previous)
    return forwarded[-1]

def spline(target):
    b=Bspline();b.order=3;b.traj_id=1;b.start_time=target.header.stamp;b.knots=[0.,0.,0.,0.,4.,4.,4.,4.]
    for i in range(4):b.pos_pts.append(Point(x=x+(target.pose.position.x-x)*i/3,y=0.,z=.32))
    splines.publish(b)

def stop_all():
    for p in reversed(processes):
        if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
    for p in reversed(processes):
        try:p.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    processes.clear()

try:
    bringup=Path(get_package_share_directory('legbot_bringup'))
    coordinator=bringup.parent.parent/'lib/legbot_bringup/scan_waypoint_mission'
    statefile=str(Path(os.environ['ROS_LOG_DIR'])/'mission.json')
    start([sys.argv[1],'--ros-args','-p','body_pose_topic:=/tolerance_test/odom',
           '-p','finish_dist:=0.4','-p','goal_slowdown_distance:=0.55'])
    start([str(coordinator),'--mode','direct','--input-topic','/scan/waypoint_input',
           '--frame-id','map','--odom-topic','/tolerance_test/odom','--output',statefile,
           '--reach-xy','0.4','--dwell-seconds','0.05'])
    wait(lambda:requests.get_subscription_count()>0 and direct.get_subscription_count()>1);pump(.2)
    first=submit(.15);wait(lambda:bool(records(first.target.header.stamp)))
    assert first.goal_id>=10 and first.attempt_id==first.goal_id+100 and first.waypoint_index==2
    assert first.has_xy_tolerance and first.xy_tolerance==.15
    assert not legacy,'extended goal was also published as a default-tolerance command'
    spline(first.target);pump(.35)
    assert not arrived(first.target),'stopped at default .4 rather than requested .15'
    assert math.hypot(commands[-1].linear.x,commands[-1].linear.y)>.05
    x=.19;wait(lambda:arrived(first.target));wait(lambda:bool(records(first.target.header.stamp,event='arrived')))
    report=records(first.target.header.stamp,event='arrived')[-1]
    assert report['effective_xy_tolerance']==.15 and report['goal_distance']<=.15
    assert report['actual_position'][0]==x
    pump(.15);assert json.loads(Path(statefile).read_text())['state']=='direct_goal_complete'
    second=submit();wait(lambda:arrived(second.target))
    wait(lambda:bool(records(second.target.header.stamp,event='arrived')))
    assert records(second.target.header.stamp)[-1]['effective_xy_tolerance']==.4
    count=len(statuses);direct.publish(first);pump(.15)
    assert len(statuses)==count,'stale custom goal changed current policy'
    for value in (0.,-.1,.049,float('nan'),float('inf'),1.01):
        bad=submit(value);stamp=bad.target.header.stamp
        wait(lambda:any(m.data==f'{ns(stamp)}|invalid_goal_tolerance' for m in failures))
        assert records(stamp,event='rejected') and not arrived(bad.target)
        pump(.05);assert abs(commands[-1].linear.x)+abs(commands[-1].linear.y)+abs(commands[-1].angular.z)<1e-8
    normal=PoseStamped();normal.header.frame_id='map';normal.header.stamp=node.get_clock().now().to_msg()
    normal.pose.position=Point(x=x+.3,y=0.,z=.32);normal.pose.orientation.w=1.;old_inputs.publish(normal)
    wait(lambda:bool(legacy));wait(lambda:arrived(legacy[-1]))
    assert records(legacy[-1].header.stamp)[-1]['effective_xy_tolerance']==.4
    print('PASS real coordinator/executor: atomic custom arrival, actual pose, default reset, stale and invalid rejection, legacy input')
    stop_all();pump(.1)
    # No odometry connection: the FSM must still apply/validate options before
    # reporting its separate no-body-pose planning failure.
    start([sys.argv[2],'--ros-args','--params-file',str(bringup/'config/scan.yaml'),
           '-p','fsm.navi_mode:=1','-p','fsm.finish_dist:=0.4'])
    wait(lambda:direct.get_subscription_count()>1)
    for number,value in enumerate((.15,None,float('nan'))):
        m=GoalRequest();m.target.header.frame_id='map';m.target.header.stamp=node.get_clock().now().to_msg()
        m.target.pose.position=Point(x=1.,y=0.,z=.32);m.target.pose.orientation.w=1.
        m.goal_id=number+1000;m.has_xy_tolerance=value is not None;m.xy_tolerance=value or 0.
        direct.publish(m);event='rejected' if number==2 else 'accepted'
        wait(lambda:bool(records(m.target.header.stamp,'planner',event)))
        d=records(m.target.header.stamp,'planner',event)[-1]
        assert d['goal_id']==m.goal_id
        if number<2:assert d['effective_xy_tolerance']==(.15 if number==0 else .4)
    print('PASS real SCAN FSM: same per-goal tolerance, omitted default and invalid rejection')
except Exception:
    for log in logs:
        log.seek(0);print(log.read()[-8000:])
    raise
finally:
    stop_all()
    for log in logs:log.close()
    node.destroy_node();rclpy.try_shutdown()
