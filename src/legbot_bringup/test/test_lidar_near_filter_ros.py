"""Two real cloud transformers: independent near range and shared geometry fallback."""
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID']=str(110+os.getpid()%30)
os.environ['ROS_LOG_DIR']=tempfile.mkdtemp(prefix='lidar_near_filter_')
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2,PointField
from tf2_ros import StaticTransformBroadcaster

source=Path(__file__).resolve().parents[1]
geometry=source/'config/robot_geometry.yaml'
POINT_STEP=32


def cloud(points):
    m=PointCloud2();m.header.frame_id='unused';m.height=2;m.width=len(points)//2
    m.point_step=POINT_STEP;m.row_step=m.width*POINT_STEP+16
    m.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1)
              for i,k in enumerate(('x','y','z','intensity'))]
    m.fields += [PointField(name='ring',offset=16,datatype=PointField.UINT16,count=1),
                 PointField(name='time',offset=24,datatype=PointField.FLOAT64,count=1)]
    m.data=bytes(m.row_step*m.height);data=bytearray(m.data)
    for i,xyz in enumerate(points):
        at=(i//m.width)*m.row_step+(i%m.width)*POINT_STEP
        data[at:at+POINT_STEP]=bytes([0xa5])*POINT_STEP
        struct.pack_into('<ffffH',data,at,*xyz,100.+i,1000+i)
        struct.pack_into('<d',data,at+24,1.25+i)
    m.data=bytes(data);return m


def main():
    rclpy.init();node=rclpy.create_node('lidar_near_filter_test');broadcaster=StaticTransformBroadcaster(node)
    samples={k:[] for k in ('main','down')};poses={k:[] for k in samples};global_clouds=[]
    pubs={};subscriptions=[];transforms=[]
    from nav_msgs.msg import Odometry
    for key in samples:
        pubs[key]=node.create_publisher(PointCloud2,'/near_test/'+key+'/raw',10)
        subscriptions.append(node.create_subscription(PointCloud2,'/near_test/'+key+'/registered',lambda m,k=key:samples[k].append(m),qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(Odometry,'/near_test/'+key+'/pose',lambda m,k=key:poses[k].append(m),qos_profile_sensor_data))
        t=TransformStamped();t.header.frame_id='map';t.child_frame_id=key+'_sensor'
        t.transform.translation.x=10. if key=='main' else 10.35
        t.transform.translation.y=2.;t.transform.translation.z=.2 if key=='main' else .44
        t.transform.rotation.w=1.
        if key=='down':t.transform.rotation.y=math.sqrt(.5);t.transform.rotation.w=math.sqrt(.5)
        transforms.append(t)
    broadcaster.sendTransform(transforms)
    subscriptions.append(node.create_subscription(PointCloud2,'/slam/global_cloud_registered',global_clouds.append,10))
    processes=[]
    with tempfile.TemporaryFile(mode='w+') as log, tempfile.TemporaryDirectory() as tmp:
        def start(config):
            for key in samples:
                args=[sys.argv[1],'--ros-args','-r','__node:=near_'+key,
                      '--params-file',str(config),'-p','target_frame:=map',
                      '-p','input_topic:=/near_test/'+key+'/raw',
                      '-p','output_topic:=/near_test/'+key+'/registered',
                      '-p','pose_topic:=/near_test/'+key+'/pose']
                processes.append(subprocess.Popen(args,stdout=log,stderr=log))
            processes.append(subprocess.Popen([sys.argv[2],'--ros-args','-r','__node:=near_global',
                '-p','source:=gazebo','-p','local_frame:=map','-p','global_frame:=world',
                '-p','cloud_topic:=/near_test/main/registered'],stdout=log,stderr=log))
        def stop():
            for p in processes:
                if p.poll() is None:p.send_signal(signal.SIGINT)
            for p in processes:
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:p.kill();p.wait()
            processes.clear()
        def spin(condition,timeout=8):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                assert all(p.poll() is None for p in processes),'transformer exited'
                rclpy.spin_once(node,timeout_sec=.02)
                if condition():return
            raise AssertionError('timed out waiting for filter output')
        def send(points):
            messages={}
            for key,pub in pubs.items():
                m=cloud(points);m.header.frame_id=key+'_sensor';m.header.stamp=node.get_clock().now().to_msg()
                messages[key]=m
            # Output is best effort. Repeat the same stamped input until both
            # subscribers and the static-TF listeners have finished discovery.
            next_send=0.
            def received():
                nonlocal next_send
                if time.monotonic()>=next_send:
                    for key,pub in pubs.items():pub.publish(messages[key])
                    next_send=time.monotonic()+.15
                return (all(any(m.header.stamp==messages[k].header.stamp for m in samples[k]) for k in samples)
                        and any(m.header.stamp==messages['main'].header.stamp for m in global_clouds))
            spin(received)
            return messages
        def verify(messages,points,retained):
            for key in samples:
                original=messages[key]
                m=next(m for m in reversed(samples[key]) if m.header.stamp==original.header.stamp)
                assert m.header.frame_id=='map' and m.header.stamp==original.header.stamp
                assert m.height==1 and m.width==len(retained) and m.row_step==len(retained)*POINT_STEP
                assert m.point_step==POINT_STEP and m.fields==original.fields
                for i,index in enumerate(retained):
                    offset=i*POINT_STEP;src=(index//original.width)*original.row_step+(index%original.width)*POINT_STEP
                    assert bytes(m.data[offset+12:offset+POINT_STEP])==bytes(original.data[src+12:src+POINT_STEP]),'extra fields or point padding corrupted'
                    x,y,z=points[index]
                    expected=(10.+x,2.+y,.2+z) if key=='main' else (10.35+z,2.+y,.44-x)
                    actual=struct.unpack_from('<fff',m.data,offset)
                    assert max(abs(a-b) for a,b in zip(actual,expected))<2e-5,(key,actual,expected)
                spin(lambda:any(p.header.stamp==original.header.stamp for p in poses[key]))
                assert next(p for p in poses[key] if p.header.stamp==original.header.stamp).child_frame_id==key+'_sensor'
            forwarded=next(m for m in global_clouds if m.header.stamp==messages['main'].header.stamp)
            assert forwarded.header.frame_id=='world' and forwarded.width==len(retained)
            assert len(forwarded.data)==len(retained)*POINT_STEP
        try:
            # Fix the test cutoff so user tuning of the installed YAML does
            # not change the boundary/field-preservation fixture below.
            import yaml
            config=yaml.safe_load(geometry.read_text());params=config['/**']['ros__parameters']
            params['lidar_min_range']=.5
            modified=Path(tmp)/'geometry.yaml';modified.write_text(yaml.safe_dump(config))
            start(modified);spin(lambda:all(p.get_subscription_count()>0 for p in pubs.values()))
            points=[(0.,0.,0.),(.2,0.,0.),(.3,.3,0.),(0.,0.,-.32),
                    (0.,0.,.50),(.6,0.,0.),(math.nan,0.,0.),(math.inf,0.,0.)]
            verify(send(points),points,[4,5])
            # All-near scans remain valid empty messages with timestamped pose.
            near=[(.1,0.,0.)]*8;verify(send(near),near,[])
            # Invalid layout must be rejected without killing the transformer.
            key='main';bad=cloud(points);bad.header.frame_id=key+'_sensor'
            bad.header.stamp=node.get_clock().now().to_msg();bad.row_step=1;pubs[key].publish(bad)
            until=time.monotonic()+.2;spin(lambda:time.monotonic()>=until)
            assert not any(m.header.stamp==bad.header.stamp for m in samples[key])
            verify(send(points),points,[4,5])
            stop()
            # Without an override, both sensors still follow shared geometry.
            params.pop('lidar_min_range');params['robot_radius']=.35
            modified.write_text(yaml.safe_dump(config))
            start(modified);spin(lambda:all(p.get_subscription_count()>0 for p in pubs.values()))
            points=[(.3,.3,0.),(.4,0.,0.),(.3,0.,0.),(0.,0.,.1)]*2
            verify(send(points),points,[0,1,4,5])
            stop()
            # Explicit filtering is independent of a different planning radius.
            params['robot_radius']=.6;params['lidar_min_range']=.5
            modified.write_text(yaml.safe_dump(config))
            start(modified);spin(lambda:all(p.get_subscription_count()>0 for p in pubs.values()))
            points=[(.4,0.,0.),(.5,0.,0.),(.6,0.,0.),(0.,0.,.1)]*2
            verify(send(points),points,[1,2,5,6])
            stop()
            # Both horizontal edges of each organized row are discarded;
            # retained point fields/stamps still follow the regular TF path.
            params['lidar_edge_columns']=1
            modified.write_text(yaml.safe_dump(config))
            start(modified);spin(lambda:all(p.get_subscription_count()>0 for p in pubs.values()))
            points=[(1.,0.,0.)]*8
            verify(send(points),points,[1,2,5,6])
            print('PASS two sensor origins, range override/fallback, independent planning radius, organized edge columns, padded clouds, fields/stamps, empty and malformed input')
        except Exception:
            log.seek(0);print(log.read()[-7000:]);raise
        finally:
            stop();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
