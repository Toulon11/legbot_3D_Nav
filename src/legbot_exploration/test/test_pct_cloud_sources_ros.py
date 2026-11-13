"""Near-only PCT evidence uses each sensor at the exact cloud stamp."""
import json
import copy
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

os.environ['ROS_DOMAIN_ID']=str(180+os.getpid()%20)
os.environ['ROS_LOG_DIR']=tempfile.mkdtemp(prefix='pct_cloud_sources_')
source=Path(__file__).resolve().parents[1]
with tempfile.TemporaryFile(mode='w+') as log:
    process=subprocess.Popen([sys.argv[1],'--exploration-mode=map','--ros-args',
        '--params-file',str(source/'config/multi_layer_map.yaml'),
        '--params-file',str(source.parent/'legbot_bringup/config/robot_geometry.yaml'),
        '-p','use_sim_time:=false','-p','publish_period:=0.1',
        '-p','graph_preview_enabled:=false',
        '-p','cloud_sensor_odom_topic:=/test/main_pose',
        '-p','auxiliary_sensor_odom_topic:=/test/down_pose',
        '-p','auxiliary_cloud_topic:=/test/down_cloud'],stdout=log,stderr=log)
    rclpy.init();node=rclpy.create_node('cloud_sources_test')
    states=[];surfaces=[]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String,'/exploration/layers/status',lambda m:states.append(json.loads(m.data)),qos)
    node.create_subscription(PointCloud2,'/exploration/layers/planning_surfaces',surfaces.append,qos)
    cloud=node.create_publisher(PointCloud2,'/slam/global_cloud_registered',10)
    down=node.create_publisher(PointCloud2,'/test/down_cloud',10)
    main_pose=node.create_publisher(Odometry,'/test/main_pose',10)
    down_pose=node.create_publisher(Odometry,'/test/down_pose',10)
    points=[(x*.1,y*.1,0.) for x in range(-10,11) for y in range(-10,11) if (x,y)!=(0,0)]*3
    def patch(x, y, z):
        return [(x+dx,y+dy,z) for dx in (-.02,0.,.02) for dy in (-.02,0.,.02)]
    def spin_until(condition,timeout=8):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            assert process.poll() is None,'map process exited'
            rclpy.spin_once(node,timeout_sec=.02)
            if condition():return
        raise AssertionError('timeout; states='+str(states[-1:]))
    def pose(publisher,header,origin):
        msg=Odometry();msg.header=header;msg.pose.pose.orientation.w=1.
        msg.pose.pose.position.x=origin[0];msg.pose.pose.position.y=origin[1];msg.pose.pose.position.z=origin[2]
        publisher.publish(msg)
    def send(publisher,xyz,frame='map',origin=(0.,0.,.5),with_pose=True):
        header=Header(stamp=node.get_clock().now().to_msg(),frame_id=frame)
        if with_pose:pose(main_pose if publisher is cloud else down_pose,header,origin)
        publisher.publish(point_cloud2.create_cloud_xyz32(header,xyz))
        return header
    def at(x,y,z=0.):
        a=point_cloud2.read_points(surfaces[-1],skip_nans=False)
        return a[(abs(a['x']-x)<1e-6)&(abs(a['y']-y)<1e-6)&(abs(a['z']-z)<1e-6)]
    def center():
        a=point_cloud2.read_points(surfaces[-1],skip_nans=False)
        assert (a['source']==1).all(),'non-measured surface published'
        return a[(abs(a['x'])<1e-6)&(abs(a['y'])<1e-6)&(abs(a['z'])<1e-6)]
    def auxiliary():
        return next(s for s in states[-1]['cloud_sources'] if s['topic']=='/test/down_cloud')
    try:
        spin_until(lambda:cloud.get_subscription_count()>0 and down.get_subscription_count()>0 and main_pose.get_subscription_count()>0 and down_pose.get_subscription_count()>0)
        send(cloud,points)
        spin_until(lambda:states and states[-1]['clouds_received']>=1 and surfaces)
        assert not len(center()),'enclosed hole was filled without real hits'
        # A registered second sensor can complete the observed floor; no
        # synthetic pose publisher, shared ray origin, or interpolation needed.
        send(down,patch(0.,0.,0.))
        spin_until(lambda:auxiliary()['integrated']>=1 and len(center())>0)
        assert ((center()['flags']&32)!=0).all(),'real G did not mature'
        assert abs(center()['z']).max()<1e-6
        send(down,[(.3,.3,1.)]*3,frame='wrong_frame')
        spin_until(lambda:states[-1]['wrong_frame']>=1)
        assert auxiliary()['integrated']==1,'wrong-frame auxiliary cloud was fused'
        assert not node.get_publishers_info_by_topic('/cmd_vel')
        assert not node.get_publishers_info_by_topic('/go2/control_mode')
        before=states[-1]['pct_surface_records']
        send(cloud,[(8.,0.,.07)]*3)
        spin_until(lambda:states[-1]['cloud_sources'][0]['filtered_far_points']>=3)
        assert states[-1]['pct_surface_records']==before and not len(at(8.,0.,.07)), 'far hits initialized/matured a surface'
        # The world point is far from the primary sensor, but near this
        # auxiliary sensor. Cloud-before-pose and a mismatched stamp must wait.
        integrated=auxiliary()['integrated']
        header=send(down,patch(8.,0.,0.),with_pose=False)
        spin_until(lambda:auxiliary()['waiting_pose_frames']>=1)
        assert auxiliary()['integrated']==integrated
        wrong=copy.deepcopy(header)
        wrong.stamp.nanosec+=1
        pose(down_pose,wrong,(8.,0.,.5))
        until=time.monotonic()+.2
        spin_until(lambda:time.monotonic()>=until)
        assert auxiliary()['integrated']==integrated, 'used a different timestamp or primary sensor pose'
        pose(down_pose,header,(8.,0.,.5))
        spin_until(lambda:auxiliary()['integrated']>integrated and len(at(8.,0.))>0)
        assert ((at(8.,0.)['flags']&32)!=0).all()
        assert not len(at(8.,0.,.07)), 'far noise leaked into a second height group'
        print('PASS near-only, exact-stamp multi-sensor evidence; far points never mature, approaching sensor can mature the same world cell')
        print('PASS real-only multi-cloud fusion: enclosed hole stays unknown, real hits fill it, wrong frame rejected')
    except Exception:
        log.seek(0);print(log.read()[-5000:]);raise
    finally:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown()
