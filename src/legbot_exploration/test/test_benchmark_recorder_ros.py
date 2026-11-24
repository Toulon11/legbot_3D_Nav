"""Isolated synthetic ROS trial: quantification, files and SIGINT, no robot commands."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import numpy as np


def main():
    os.environ['ROS_DOMAIN_ID']=str(150+os.getpid()%70)
    os.environ['ROS_LOG_DIR']=tempfile.mkdtemp(prefix='benchmark_ros_logs_')
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2,PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Bool,String,Header
    from std_srvs.srv import Trigger
    from rclpy.qos import QoSProfile,DurabilityPolicy
    rclpy.init();node=rclpy.create_node('benchmark_fixture')
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    odom=node.create_publisher(Odometry,'/test/benchmark_odom',10)
    ready=node.create_publisher(Bool,'/go2/demo_ready',retained)
    status=node.create_publisher(String,'/exploration/status',retained)
    surface=node.create_publisher(PointCloud2,'/test/benchmark_surfaces',retained)
    plans=node.create_publisher(String,'/exploration/planning_results',10)
    feedback=node.create_publisher(String,'/exploration/execution_feedback',10)
    process=None
    with tempfile.TemporaryDirectory(prefix='benchmark_trial_') as temporary:
        root=Path(temporary);reference=root/'ref.npz';out=root/'result';log=root/'node.log'
        np.savez(reference,xyz=np.asarray([[0.,0.,0.],[.1,0.,.5],[.2,0.,.5]]),resolution=.1)
        script=Path(__file__).resolve().parents[1]/'python/scripts/record_exploration_metrics'
        floors=json.dumps([dict(name='first',height=0.),dict(name='second',height=.5)])
        args=[sys.executable,str(script),'--ros-args','-p','use_sim_time:=false',
            '-p','auto_reference:=false','-p','result_directory:='+str(out),
            '-p','reference_file:='+str(reference),'-p',"floor_definitions_json:='"+floors+"'",
            '-p','target_floor:=second','-p','single_floor:=second','-p','floor_dwell_seconds:=0.15',
            '-p','odometry_topic:=/test/benchmark_odom','-p','surfaces_topic:=/test/benchmark_surfaces']
        floor_begin=[None,None];floor_end=[None,None];late_surface=[False]
        def publish(height):
            msg=Odometry();msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='map'
            msg.pose.pose.position.x=.1 if height>.4 else 0.;msg.pose.pose.position.z=height+.32
            msg.pose.pose.orientation.w=1.;odom.publish(msg)
            ready.publish(Bool(data=True))
            if height>.4 and floor_begin[0] is None:
                floor_begin[:]=[node.get_clock().now().nanoseconds*1e-9,time.monotonic()]
            status.publish(String(data=json.dumps(dict(state='waiting_for_scan_completion',task_active=True,
                paused=False,observation_only=False,pct_floor=dict(number=99,height=height,
                    ordinary_started_sim=floor_begin[0],ordinary_started_wall=floor_begin[1],
                    ordinary_ended_sim=floor_end[0],ordinary_ended_wall=floor_end[1]),
                pct_low_yield_deferred_clusters=2,pct_low_yield_deferred_cells=3))))
            fields=[PointField(name=name,offset=offset,datatype=kind,count=1) for name,offset,kind in
                    [('x',0,PointField.FLOAT32),('y',4,PointField.FLOAT32),('z',8,PointField.FLOAT32),('flags',12,PointField.UINT32)]]
            surface.publish(point_cloud2.create_cloud(Header(frame_id='map'),fields,
                [(0.,0.,0.,32),(.1,0.,.5,32),(0.,0.,0.,32)]+
                ([(.2,0.,.5,32)] if late_surface[0] else [])))
            plans.publish(String(data=json.dumps(dict(goal_id=1,attempt_id=1,planner='layer',usable=True,planning_ms=4.))))
            feedback.publish(String(data=json.dumps(dict(goal_id=1,attempt_id=1,stage_index=0,scan_goal_stamp=5,status='succeeded'))))
        def spin_for(seconds,height):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                assert process.poll() is None,log.read_text()
                publish(height);rclpy.spin_once(node,timeout_sec=.02)
        try:
            with log.open('w') as handle:
                process=subprocess.Popen(args,stdout=handle,stderr=handle)
            client=node.create_client(Trigger,'/exploration/benchmark/finish')
            assert client.wait_for_service(timeout_sec=8),log.read_text()
            spin_for(.8,0.);spin_for(.3,.25);spin_for(.6,.5)
            floor_end[:]=[node.get_clock().now().nanoseconds*1e-9,time.monotonic()]
            spin_for(.2,.5)
            late_surface[0]=True;spin_for(.3,.5)
            future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=5)
            assert future.result().success,log.read_text()
            assert process.wait(timeout=5)==0,log.read_text()
            result=json.loads(next(out.glob('*.json')).read_text())
            values=result['metrics']
            assert values['coverage_percent']==50.,result
            assert result['single_floor']['state']=='settled',result
            assert values['exploration_wall_seconds']<1.,result
            assert values['upward_attempt_count']==values['upward_success_count']==1,result
            assert values['upward_success_percent']==100.,result
            assert values['upward_mean_sim_seconds']>0.,result
            assert result['mission_success'] is True,result
            assert values['planning_attempt_count']==1,result
            assert values['planning_success_count']==1,result
            assert values['planning_mean_ms']==values['planning_p95_ms']==4.,result
            assert values['exploration_sim_seconds']>0.,result
            assert values['exploration_wall_seconds']>0.,result
            assert 'floors' not in result,result
            assert result['termination']=='manual_finish' and result['finalized'],result
            assert len(list(out.iterdir()))==1,list(out.iterdir())
            first=set(out.glob('*.json'))
            floor_begin[:]=[None,None];floor_end[:]=[None,None];late_surface[0]=False
            with log.open('a') as handle:process=subprocess.Popen(args,stdout=handle,stderr=handle)
            end=time.monotonic()+5
            while len(list(out.glob('*.json')))<2 and time.monotonic()<end:
                publish(0.);rclpy.spin_once(node,timeout_sec=.02)
            spin_for(.5,0.);process.send_signal(signal.SIGINT);process.wait(timeout=5)
            second=set(out.glob('*.json'))-first
            assert len(second)==1,log.read_text()
            result=json.loads(second.pop().read_text())
            assert result['finalized'] and result['termination']=='interrupted',log.read_text()
            assert not list(out.glob('*.json.tmp'))
            print('PASS scoped single-floor window freezes before late scans; cross-layer outcome, one JSON and SIGINT finalization')
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:process.kill();process.wait()
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
