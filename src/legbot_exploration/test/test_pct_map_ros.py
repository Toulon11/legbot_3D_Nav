"""Offline map import (range gate explicitly disabled); map-only mode fuses real XYZ without pose and never issues motion."""
import json,os,signal,subprocess,sys,tempfile,time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header,String

os.environ['ROS_DOMAIN_ID']=str(225+os.getpid()%5)
os.environ['ROS_LOG_DIR']=tempfile.mkdtemp(prefix='pct_map_only_')
source=Path(__file__).resolve().parents[1]
with tempfile.TemporaryFile(mode='w+') as log:
    process=subprocess.Popen([sys.argv[1],'--exploration-mode=map','--ros-args',
        '--params-file',str(source/'config/multi_layer_map.yaml'),
        '-p','pct_reliable_observation_range:=0.0',
        '-p','use_sim_time:=false','-p','publish_period:=0.2'],stdout=log,stderr=log)
    rclpy.init();node=rclpy.create_node('map_without_odometry_test')
    states=[];surfaces=[];returns=[];qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String,'/exploration/layers/status',lambda m:states.append(json.loads(m.data)),qos)
    node.create_subscription(PointCloud2,'/exploration/layers/planning_surfaces',surfaces.append,qos)
    node.create_subscription(PointCloud2,'/exploration/layers/measured_returns',returns.append,qos)
    cloud=node.create_publisher(PointCloud2,'/slam/global_cloud_registered',10)
    # Offline import supplies finite room bounds. A lone ground return no
    # longer establishes infinite, unobserved clearance.
    points=[(i*.1,j*.1,z) for i in range(-15,16) for j in range(-15,16) for z in (0.,1.5)]
    try:
        deadline=time.monotonic()+15;next_send=0
        while time.monotonic()<deadline:
            assert process.poll() is None,'map-only process exited'
            if time.monotonic()>=next_send:
                cloud.publish(point_cloud2.create_cloud_xyz32(Header(stamp=node.get_clock().now().to_msg(),frame_id='map'),points))
                next_send=time.monotonic()+.25
            rclpy.spin_once(node,timeout_sec=.03)
            if states and states[-1]['pct_ready'] and states[-1]['pct_map']['safe_cells']>0 and surfaces and returns:break
        assert states and states[-1]['pct_ready'],states[-1:]
        assert states[-1]['inspection_valid'],states[-1]
        assert states[-1]['pct_map']['safe_cells']>0,states[-1]
        assert surfaces[-1].point_step==64 and returns[-1].point_step==16
        assert {'source','flags','cost','ceiling_z','node_z','C_clear'}<={f.name for f in surfaces[-1].fields}
        assert states[-1]['pct_map']['mapping_mechanism']=='xy_passage_intervals'
        assert not node.get_publishers_info_by_topic('/scan/waypoint_input')
        assert not node.get_publishers_info_by_topic('/scan/waypoint_request')
        assert not node.get_publishers_info_by_topic('/cmd_vel')
        assert (point_cloud2.read_points(surfaces[-1])['source']==1).all()
        print('PASS map-only PCT: registered XYZ without odometry -> actual G/C display, no motion')
    except Exception:
        log.seek(0);print(log.read()[-6000:]);raise
    finally:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown()
