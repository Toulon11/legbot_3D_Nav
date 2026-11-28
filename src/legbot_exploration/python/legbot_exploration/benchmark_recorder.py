"""Read-only ROS recorder for repeatable exploration experiment result files."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import math
import subprocess
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .benchmark_metrics import ExperimentMetrics, SurfaceCoverage, ResultWriter, distribution, default_result_root
from .surface_update_stream import SurfaceUpdateStream
from .benchmark_reference import build_reference, save_reference, REFERENCE_VERSION


def asset_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class BenchmarkRecorder(Node):
    def __init__(self):
        super().__init__('exploration_benchmark')
        def parameter(name,default):return self.declare_parameter(name,default).value
        root=Path(parameter('result_directory',str(default_result_root()))).expanduser()
        self.wall_start=time.monotonic();self.ready=False;self.pose=None;self.status={};self.closed=False
        self.map_versions=set()
        self.partial=bool(parameter('partial_run',False))
        self.immediate=bool(parameter('start_immediately',False))
        self.reference_height=float(parameter('robot_reference_height',.32))
        self.limit=float(parameter('max_sim_seconds',0.))
        self.capture=float(parameter('capture_wall_seconds',0.))
        if any(not math.isfinite(v) or v<0 for v in (self.limit,self.capture)):
            raise ValueError('recording limits must be finite and nonnegative')
        self.world=str(parameter('world',''))
        method=str(parameter('method_label','legbot_online_pct'))
        start_floor_skip=str(parameter('start_floor_skip_option',''))
        source=str(parameter('localization_source','gazebo'))
        reference=str(parameter('reference_file',''))
        auto=bool(parameter('auto_reference',True))
        resolution=float(parameter('metric_resolution',.1))
        tolerance=float(parameter('surface_height_tolerance',.05))
        spawn=list(parameter('spawn_ground',[-6.75,9.6,0.]))
        floors=json.loads(str(parameter('floor_definitions_json','[]')))
        target=str(parameter('target_floor',''))
        single_floor=str(parameter('single_floor',''))
        per_floor=bool(parameter('per_floor',bool(floors) and not single_floor))
        reference_issue=None
        if not reference and auto and self.world:
            try:
                from ament_index_python.packages import get_package_share_directory
                model_root=Path(get_package_share_directory('legbot_bringup'))/'models'
                cache=root/'.reference_cache'/('world_'+asset_hash(self.world)[:16]+'.npz')
                valid=False
                if cache.exists():
                    with np.load(cache,allow_pickle=False) as f:
                        meta=json.loads(str(f['metadata']))
                    valid=(meta.get('builder_version')==REFERENCE_VERSION and meta['spawn']==spawn and meta['resolution']==resolution and
                           all(Path(p).exists() and asset_hash(p)==h for p,h in meta['assets'].items()))
                if not valid:
                    xyz,meta=build_reference(self.world,[model_root],spawn,resolution)
                    save_reference(cache,xyz,meta)
                reference=str(cache)
            except (OSError,ValueError,KeyError) as error:
                reference_issue=str(error)
                self.get_logger().warning('Coverage percentage unavailable: '+reference_issue)
        coverage=SurfaceCoverage(resolution,tolerance,reference or None)
        floor_dwell=float(parameter('floor_dwell_seconds',2.))
        transition_timeout=float(parameter('transition_timeout_seconds',600.))
        self.metrics=ExperimentMetrics(coverage,floors,target,floor_dwell,transition_timeout,
            partial_run=self.partial,single_floor=single_floor,per_floor=per_floor)
        def topic(name,default):return str(parameter(name,default))
        odom=topic('odometry_topic','/Odometry_gazebo')
        self.cloud_topic=topic('surfaces_topic','/exploration/layers/planning_surfaces')
        self.coverage_source=str(parameter('coverage_source','pct_surfaces'))
        try:
            commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=default_result_root().parent,
                text=True,stderr=subprocess.DEVNULL,timeout=2).strip()
        except (OSError,subprocess.SubprocessError):commit=None
        paths=[]
        package=default_result_root().parent
        for folder in ['python','cpp','config','launch']:
            paths.extend(p for p in (package/folder).rglob('*') if p.is_file() and
                         p.suffix in ('.py','.cpp','.hpp','.h','.yaml') and '__pycache__' not in p.parts)
        self.writer=ResultWriter(root,dict(created_at_utc=datetime.now(timezone.utc).isoformat(),
            method=method,localization_source=source,world=self.world,
            world_sha256=asset_hash(self.world) if self.world else None,
            git_head=commit,source_hashes={str(p.relative_to(package)):asset_hash(p) for p in sorted(paths)},
            platform=platform.platform(),cpu=platform.processor(),logical_cpus=os.cpu_count(),
            reference_file=reference or None,reference_metadata=coverage.reference_meta,
            reference_builder_version=coverage.reference_meta.get('builder_version'),
            reference_issue=reference_issue,odometry_topic=odom,surfaces_topic=self.cloud_topic,
            coverage_source=self.coverage_source,
            floors=floors,target_floor=target,single_floor=single_floor,per_floor=per_floor,partial_run=self.partial,
            start_floor_skip_option=start_floor_skip,max_sim_seconds=self.limit,
            capture_wall_seconds=self.capture,
            metric_resolution=resolution,height_tolerance=tolerance,
            trial_seed=int(parameter('trial_seed',-1)),robot_reference_height=self.reference_height,
            floor_dwell_seconds=floor_dwell,transition_timeout_seconds=transition_timeout,
            reference_sha256=asset_hash(reference) if reference else None,
            source_sha256=hashlib.sha256(json.dumps(
                {str(p.relative_to(package)):asset_hash(p) for p in sorted(paths)},sort_keys=True).encode()).hexdigest()))
        self.updates_topic=str(parameter('surface_updates_topic',''))
        self.surface_stream=SurfaceUpdateStream(coverage) if self.updates_topic and self.coverage_source=='pct_surfaces' else None
        self.pending_status=[]
        self.pid=int(parameter('explorer_pid',0)) or self._explorer_pid()
        self.resource_last=None;self.cpu=[];self.rss=[];self.resource={}
        self.frontier_deferral_peaks={}
        retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        events=QoSProfile(depth=100)
        self.subscriptions_owned=[
            self.create_subscription(Odometry,odom,self.on_odometry,qos_profile_sensor_data),
            self.create_subscription(Bool,'/go2/demo_ready',self.on_ready,retained),
            self.create_subscription(String,'/exploration/status',self.on_status,retained),
            self.create_subscription(String,'/exploration/layers/status',self.on_map_status,retained),
            self.create_subscription(PointCloud2,self.cloud_topic,self.on_surfaces,retained),
            self.create_subscription(OccupancyGrid,'/exploration/coverage_grid',self.on_grid,retained),
            self.create_subscription(String,'/exploration/planning_results',lambda m:self.event('planning',m),events),
            self.create_subscription(String,'/exploration/execution_feedback',lambda m:self.event('execution',m),events)]
        if self.surface_stream is not None:
            self.subscriptions_owned.append(self.create_subscription(String,self.updates_topic,
                self.on_surface_update,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        self.summary_pub=self.create_publisher(String,'/exploration/benchmark',retained)
        self.create_service(Trigger,'/exploration/benchmark/finish',self.finish_service)
        self.timer=self.create_timer(1.,self.sample,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.writer.snapshot(self.metrics.summary())
        self.get_logger().info('Quantitative result: '+str(self.writer.prefix.with_suffix('.json')))

    def now_sim(self):return self.get_clock().now().nanoseconds*1e-9

    def maybe_start(self):
        if self.pose is not None and (self.immediate or (self.ready and self.status.get('task_active') and
                not self.status.get('paused') and not self.status.get('observation_only'))):
            self.metrics.start(time.monotonic(),self.now_sim())

    def on_ready(self,message):self.ready=message.data;self.maybe_start()

    def on_status(self,message):
        value=json.loads(message.data)
        if value.get('state')!=self.status.get('state'):
            self.writer.event('state',self.now_sim(),dict(state=value.get('state'),paused=value.get('paused')))
        self.status=value;self.maybe_start()
        # Ordinary-window closure must see coverage through the controller's
        # committed version; RViz full-snapshot throttling cannot freeze stale area.
        end=value.get('pct_floor',{}).get('ordinary_ended_sim')
        terminal=value.get('state') in ('mission_complete','exploration_complete','pct_region_complete','current_region_complete')
        closing=end is not None or value.get('mission',{}).get('returning') or terminal
        if self.surface_stream is not None and (self.pending_status or
                closing and not self.surface_stream.covers(value.get('layer_map_version',0))):
            signature=(value.get('pct_floor',{}).get('number'),end,
                       value.get('mission',{}).get('returning'),terminal)
            if self.pending_status and self.pending_status[-1][0]==signature:
                # Keep the FIRST closing version; chasing every later status
                # would postpone closure indefinitely while mapping continues.
                if not closing:self.pending_status[-1]=(signature,value)
            else:self.pending_status.append((signature,value))
            self.metrics.defer_floor_closure=True
        else:self.metrics.update_status(value,time.monotonic(),self.now_sim())
        for name in ['low_yield_deferred_clusters','low_yield_deferred_cells']:
            count=value.get('pct_'+name)
            if isinstance(count,int):
                self.frontier_deferral_peaks[name]=max(count,self.frontier_deferral_peaks.get(name,0))
        if value.get('state') in ('mission_complete','exploration_complete','pct_region_complete','current_region_complete'):
            if not self.pending_status:self.finalize('controller_complete')

    def on_map_status(self,message):
        value=json.loads(message.data);self.metrics.map_status=value
        version=value.get('version')
        if version in self.map_versions:return
        self.map_versions.add(version)
        for key,field in [('map_publish_ms','update_ms'),('map_lock_held_ms','lock_held_ms')]:
            if isinstance(value.get(field),(int,float)):self.metrics.timings[key].append(value[field])
        for field in ['build_ms','frontier_update_ms']:
            v=value.get('pct_map',{}).get(field)
            if isinstance(v,(int,float)):self.metrics.timings['map_'+field].append(v)

    def on_odometry(self,message):
        p=message.pose.pose.position;stamp=message.header.stamp
        self.pose=(p.x,p.y,p.z);self.maybe_start()
        seconds=stamp.sec+stamp.nanosec*1e-9
        before=self.metrics.path_last_sample
        self.metrics.odometry(self.pose,seconds,time.monotonic(),self.reference_height)

    def on_surfaces(self,message):
        if self.coverage_source!='pct_surfaces' or self.surface_stream is not None:return
        fields={field.name for field in message.fields}
        if not {'x','y','z','flags'}<=fields:
            self.metrics.odom_anomalies['invalid_surface_schema']+=1;return
        data=point_cloud2.read_points(message,field_names=['x','y','z','flags'],skip_nans=False)
        xyz=np.column_stack([data[name].reshape(-1) for name in ['x','y','z']])
        self.metrics.coverage.observe(xyz,data['flags'].reshape(-1))

    def on_surface_update(self,message):
        if not self.surface_stream.apply(json.loads(message.data)):return
        while self.pending_status and self.surface_stream.covers(self.pending_status[0][1].get('layer_map_version',0)):
            _,value=self.pending_status.pop(0)
            self.metrics.defer_floor_closure=False
            self.metrics.update_status(value,time.monotonic(),self.now_sim())
            self.metrics.defer_floor_closure=bool(self.pending_status)
            if value.get('state') in ('mission_complete','exploration_complete','pct_region_complete','current_region_complete'):
                self.finalize('controller_complete');break

    def on_grid(self,message):
        if self.coverage_source!='occupancy_grid':return
        info=message.info;p=info.origin.position;q=info.origin.orientation
        if abs(q.x)+abs(q.y)+abs(q.z)>1e-6:
            self.metrics.odom_anomalies['rotated_coverage_grid']+=1;return
        try:
            self.metrics.coverage.observe_grid((p.x,p.y),info.resolution,info.width,info.height,message.data)
        except ValueError:
            self.metrics.odom_anomalies['coverage_grid_geometry_changed']+=1

    def event(self,kind,message):
        if self.metrics.finalized or self.metrics.started_wall is None:return
        value=json.loads(message.data)
        if kind=='planning':self.metrics.planning_result(value)
        else:self.metrics.execution_result(value)
        self.writer.event(kind,self.now_sim(),value)

    @staticmethod
    def _explorer_pid():
        found=[]
        for path in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                args=path.read_bytes().split(b'\0')
                if args and args[0].endswith(b'/combined_exploration_cpp'):found.append(int(path.parent.name))
            except OSError:pass
        return found[0] if len(found)==1 else None

    def resources(self):
        if not self.pid:self.pid=self._explorer_pid()
        if not self.pid:return
        try:
            text=Path(f'/proc/{self.pid}/stat').read_text().split(') ',1)[1].split()
            ticks=int(text[11])+int(text[12]);now=time.monotonic()
            rss=int(text[21])*os.sysconf('SC_PAGE_SIZE')/1024**2
            if self.resource_last:
                elapsed=now-self.resource_last[0]
                cpu=100*(ticks-self.resource_last[1])/os.sysconf('SC_CLK_TCK')/elapsed
                if cpu>=0:self.cpu.append(cpu)
            self.rss.append(rss);self.resource_last=(now,ticks)
            self.resource=dict(explorer_pid=self.pid,cpu_one_core_percent=distribution(self.cpu),
                               resident_memory_mib=distribution(self.rss))
        except OSError:self.pid=None;self.resource_last=None

    def sample(self):
        if self.closed:return
        self.metrics.tick(time.monotonic(),self.now_sim());self.resources()
        summary=self.metrics.summary();m=self.metrics.map_status
        row={name:summary[name] for name in ['elapsed_sim_seconds','elapsed_wall_seconds',
            'path_length_xy_m','mapped_surface_area_m2','covered_reference_area_m2','reference_area_m2',
            'coverage_ratio','exploration_speed_m2_per_sim_s','recent_exploration_speed_m2_per_sim_s',
            'current_physical_floor']}
        row.update(state=self.status.get('state'),frontier_cells=self.status.get('pct_frontier_cells'),
                   low_yield_deferred_clusters=self.status.get('pct_low_yield_deferred_clusters'),
                   low_yield_deferred_cells=self.status.get('pct_low_yield_deferred_cells'),
                   dropped_planning=m.get('dropped_planning'),cloud_messages_seen=m.get('cloud_messages_seen'),
                   map_publish_ms=m.get('update_ms'))
        self.writer.row(row)
        self.writer.snapshot(summary,self.extras())
        if self.context.ok():self.summary_pub.publish(String(data=json.dumps(summary,allow_nan=False)))
        if self.limit>0 and summary['elapsed_sim_seconds'] is not None and summary['elapsed_sim_seconds']>=self.limit:
            self.finalize('time_limit')
        elif self.capture>0 and time.monotonic()-self.wall_start>=self.capture:self.finalize('capture_window_ended')
        elif self.metrics.finalized:self.finalize(self.metrics.termination)

    def finish_service(self,request,response):
        self.finalize('manual_finish');response.success=True
        response.message=str(self.writer.prefix.with_suffix('.json'));return response

    def extras(self):
        return dict(coverage_export=(None if self.surface_stream is None else dict(
            version=self.surface_stream.version,epoch=self.surface_stream.epoch,
            synchronized=self.surface_stream.synchronized,gaps=self.surface_stream.gaps,
            pending_closures=len(self.pending_status))),resources=self.resource,frontier_deferrals={k:self.status.get('pct_'+k) for k in
            ['low_yield_deferred_clusters','low_yield_deferred_cells']},
            frontier_deferral_peak_per_query=self.frontier_deferral_peaks,
            map_counters={k:self.metrics.map_status.get(k) for k in
            ['cloud_messages_seen','clouds_received','dropped_planning','rejected_frames']})

    def finalize(self,reason):
        if self.closed:return
        self.metrics.finish(reason,time.monotonic(),self.now_sim())
        self.writer.event('finished',self.now_sim(),dict(reason=reason))
        self.writer.snapshot(self.metrics.summary(),self.extras())
        self.writer.close();self.closed=True
        self.get_logger().info('Saved quantitative result: '+str(self.writer.prefix.with_suffix('.json')))


def main():
    rclpy.init();node=None;reason='interrupted'
    try:
        node=BenchmarkRecorder()
        while rclpy.ok() and not node.closed:rclpy.spin_once(node,timeout_sec=.2)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    except Exception:
        reason='recorder_error'
        raise
    finally:
        if node is not None:
            node.finalize(reason);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
