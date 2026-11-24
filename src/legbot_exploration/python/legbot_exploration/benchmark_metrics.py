"""ROS-independent experiment accounting. Never reads a map as its own truth."""
from bisect import bisect_left
from collections import Counter, defaultdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import uuid

import numpy as np


def default_result_root():
    """Keep run files in the source package, including symlink installations."""
    for parent in Path(__file__).resolve().parents:
        if parent.name=='legbot_exploration' and (parent/'package.xml').exists():return parent/'result'
        candidate=parent/'src/legbot_exploration'
        if (candidate/'package.xml').exists():return candidate/'result'
    return Path.cwd()/'result'


def distribution(values):
    a=np.asarray(list(values),dtype=float)
    a=a[np.isfinite(a)]
    if not len(a):return dict(count=0,mean=None,std_sample=None,median=None,p95=None,maximum=None)
    return dict(count=len(a),mean=float(a.mean()),std_sample=None if len(a)<2 else float(a.std(ddof=1)),median=float(np.median(a)),
                p95=float(np.percentile(a,95)),maximum=float(a.max()))


class SurfaceCoverage:
    """Unique physical XY/height observations, independent of slice multiplicity."""
    def __init__(self,resolution=.1,height_tolerance=.05,reference=None):
        if not math.isfinite(resolution) or resolution<=0 or not math.isfinite(height_tolerance) or height_tolerance<0:
            raise ValueError('metric resolution must be positive and height tolerance nonnegative')
        self.resolution=resolution;self.tolerance=height_tolerance
        self.heights=defaultdict(list);self.count=0
        self.grid_seen=set();self.grid_signature=None;self.grid_area=None
        self.reference=None;self.reference_meta={};self.lookup=defaultdict(list)
        self.seen=None
        if reference:
            with np.load(reference,allow_pickle=False) as f:
                self.reference=np.asarray(f['xyz'],dtype=float)
                r=float(f['resolution'])
                if abs(r-resolution)>1e-8:raise ValueError('reference resolution differs from metric resolution')
                if 'metadata' in f:self.reference_meta=json.loads(str(f['metadata']))
            if self.reference.ndim!=2 or self.reference.shape[1]!=3 or not len(self.reference):
                raise ValueError('reference must contain nonempty Nx3 xyz')
            if not np.isfinite(self.reference).all():raise ValueError('nonfinite reference')
            self.seen=np.zeros(len(self.reference),dtype=bool)
            for i,(x,y,z) in enumerate(self.reference):
                self.lookup[tuple(np.rint(np.asarray((x,y))/resolution).astype(int))].append((float(z),i))
            for records in self.lookup.values():records.sort()

    def observe(self,points,flags=None):
        points=np.asarray(points,dtype=float).reshape(-1,3)
        valid=np.isfinite(points).all(axis=1)
        if flags is not None:
            flags=np.asarray(flags,dtype=np.uint32)
            # Export bit 1: unknown. Bit 5: mature measured G.
            valid &= ((flags&2)==0)&((flags&32)!=0)
        points=points[valid]
        if not len(points):return
        xy=np.rint(points[:,:2]/self.resolution).astype(np.int64)
        # Remove exact slice aliases before the per-column tolerance match.
        unique=np.unique(np.column_stack((xy,points[:,2])),axis=0)
        for x,y,z in unique:
            key=int(x),int(y);known=self.heights[key];at=bisect_left(known,z)
            near=known[max(0,at-1):at+1]
            if not any(abs(v-z)<=self.tolerance+1e-7 for v in near):
                known.insert(at,float(z));self.count+=1
            records=self.lookup.get(key)
            if records:
                height,index=min(records,key=lambda record:abs(record[0]-z))
                if abs(height-z)<=self.tolerance+1e-7:self.seen[index]=True

    @property
    def area(self):return self.count*self.resolution**2 if self.grid_area is None else self.grid_area

    def observe_grid(self,origin,resolution,width,height,data,ground_height=0.):
        """Single-floor OccupancyGrid counterpart; never credit independent floors."""
        if len(data)!=width*height or resolution<=0:raise ValueError('invalid coverage grid')
        signature=(*origin,resolution,width,height)
        if self.grid_signature is not None and self.grid_signature!=signature:
            raise ValueError('coverage grid geometry changed during trial')
        self.grid_signature=signature
        states=np.asarray(data)
        self.grid_seen.update(map(int,np.flatnonzero(states!=-1)))
        self.grid_area=len(self.grid_seen)*resolution**2
        if self.reference is None:return
        indices=np.floor((self.reference[:,:2]-np.asarray(origin))/resolution).astype(int)
        valid=(indices[:,0]>=0)&(indices[:,0]<width)&(indices[:,1]>=0)&(indices[:,1]<height)
        valid &= abs(self.reference[:,2]-ground_height)<=self.tolerance+1e-7
        at=np.flatnonzero(valid)
        self.seen[at] |= states[indices[at,1]*width+indices[at,0]]!=-1

    def summary(self):
        known=int(self.seen.sum()) if self.seen is not None else None
        return dict(mapped_surface_area_m2=self.area,
                    reference_coverage_valid=self.seen is not None,
                    covered_reference_area_m2=None if known is None else known*self.resolution**2,
                    reference_area_m2=None if self.seen is None else len(self.seen)*self.resolution**2,
                    coverage_ratio=None if self.seen is None else known/len(self.seen),
                    coverage_percent=None if self.seen is None else 100.*known/len(self.seen),
                    coverage_definition='unique observed reachable support XY/height samples; projected area',
                    explored_volume_m3=None)

    def floor_at(self,position,floors,pose_tolerance=.10,plane_tolerance=.08):
        x,y,ground=map(float,position)
        for floor in floors:
            if abs(ground-floor['height'])>pose_tolerance:continue
            region=floor.get('region')
            if region and not (region[0]<=x<=region[1] and region[2]<=y<=region[3]):continue
            if self.reference is not None:
                key=tuple(np.rint(np.asarray((x,y))/self.resolution).astype(int))
                if not any(abs(z-floor['height'])<=plane_tolerance and abs(z-ground)<=pose_tolerance
                           for z,_ in self.lookup.get(key,())):continue
            return floor['name']
        return None


class SingleFloorTrial:
    """Freeze one physical floor at its first ordinary-exploration handoff."""
    def __init__(self,coverage,floor):
        self.coverage=coverage;self.floor=dict(floor)
        self.started_sim=self.started_wall=self.ended_sim=self.ended_wall=None
        self.outcome='not_started';self.frozen=None
        self.mask=(None if coverage.reference is None else
                   abs(coverage.reference[:,2]-floor['height'])<=.08+1e-7)

    def area(self):
        if self.mask is None or not np.any(self.mask):
            return dict(reference_area_m2=None,observed_area_m2=None,coverage_percent=None)
        n=int(self.mask.sum());seen=int(self.coverage.seen[self.mask].sum())
        return dict(reference_area_m2=n*self.coverage.resolution**2,
                    observed_area_m2=seen*self.coverage.resolution**2,coverage_percent=100.*seen/n)

    def status(self,value,physical_floor,wall,sim):
        if self.ended_sim is not None:return
        band=value.get('pct_floor',{});height=band.get('height')
        if self.started_sim is not None and (value.get('mission',{}).get('returning') or
                (isinstance(height,(int,float)) and abs(height-self.floor['height'])>.08+1e-7
                 and physical_floor!=self.floor['name'])):
            self.close('incomplete_left_floor',wall,sim)
            return
        if not isinstance(height,(int,float)) or abs(height-self.floor['height'])>.08+1e-7:return
        start=band.get('ordinary_started_sim');start_wall=band.get('ordinary_started_wall')
        if not all(isinstance(t,(int,float)) and not isinstance(t,bool) and math.isfinite(t)
                   for t in (start,start_wall)):return
        if self.started_sim is None:
            # The height-band ordinal cannot establish arrival. Require actual
            # odometry at the floor's independently referenced XY/support.
            if physical_floor!=self.floor['name'] or value.get('mission',{}).get('returning'):return
            self.started_sim=float(start);self.started_wall=float(start_wall);self.outcome='active'
        end=band.get('ordinary_ended_sim');end_wall=band.get('ordinary_ended_wall')
        if all(isinstance(t,(int,float)) and not isinstance(t,bool) and math.isfinite(t)
               for t in (end,end_wall)) and end>=self.started_sim and end_wall>=self.started_wall:
            self.close('settled',end_wall,end)

    def close(self,outcome,wall,sim):
        if self.started_sim is None or self.ended_sim is not None:return
        self.ended_sim=max(self.started_sim,float(sim));self.ended_wall=max(self.started_wall,float(wall))
        self.outcome=outcome;self.frozen=self.area()

    def summary(self,wall,sim):
        area=self.frozen or self.area()
        if self.started_sim is None:
            area=dict(area,observed_area_m2=None,coverage_percent=None)
        return dict(name=self.floor['name'],ground_height_m=self.floor['height'],state=self.outcome,
            completed=self.outcome=='settled',started_sim=self.started_sim,ended_sim=self.ended_sim,
            exploration_sim_seconds=None if self.started_sim is None else
                max(0.,(self.ended_sim if self.ended_sim is not None else sim)-self.started_sim),
            exploration_wall_seconds=None if self.started_wall is None else
                max(0.,(self.ended_wall if self.ended_wall is not None else wall)-self.started_wall),**area)


class ExperimentMetrics:
    def __init__(self,coverage=None,floors=(),target_floor='',floor_dwell=2.,
                 transition_timeout=600.,departure_height=.16,partial_run=False,single_floor=''):
        self.coverage=coverage or SurfaceCoverage()
        self.floors=sorted((dict(f) for f in floors),key=lambda f:f['height'])
        names=[f['name'] for f in self.floors]
        if any(not math.isfinite(f['height']) for f in self.floors):raise ValueError('nonfinite floor height')
        if len(set(names))!=len(names):raise ValueError('duplicate floor names')
        if target_floor and target_floor not in names:raise ValueError('unknown target floor')
        if single_floor and single_floor not in names:raise ValueError('unknown single metric floor')
        self.single_floor=(SingleFloorTrial(self.coverage,next(f for f in self.floors if f['name']==single_floor))
                           if single_floor else None)
        self.target_floor=target_floor;self.floor_dwell=floor_dwell
        self.transition_timeout=transition_timeout;self.departure_height=departure_height
        self.partial_run=partial_run
        self.departure_latched=False
        self.started_wall=self.started_sim=None;self.last_wall=self.last_sim=None
        self.path_xy=self.path_xyz=0.;self.path_anchor=None;self.path_stamp=None
        self.last_received_stamp=None;self.path_last_sample=None
        self.odom_anomalies=Counter();self.current_floor=None;self.floor_candidate=None
        self.arrivals={};self.transition=None;self.transitions=[]
        self.instant_floor=None;self.floor_seconds=Counter();self.floor_wall_seconds=Counter()
        self.plans={};self.feedback={};self.timings=defaultdict(list)
        self.status={};self.map_status={};self.state_seconds=Counter()
        self.coverage_threshold_times={};self.area_history=[]
        self.baseline_area=None;self.finalized=False;self.termination='running'
        self.baseline_reference_area=None

    def start(self,wall,sim):
        if self.started_wall is not None:return
        self.started_wall=self.last_wall=float(wall)
        self.started_sim=self.last_sim=float(sim)
        self.baseline_area=self.coverage.area
        self.baseline_reference_area=self.coverage.summary()['covered_reference_area_m2']

    def update_status(self,value,wall,sim):
        self.tick(wall,sim)
        self.status=value
        if self.single_floor and self.started_wall is not None and not self.finalized:
            self.single_floor.status(value,self.instant_floor,wall,sim)

    def tick(self,wall,sim):
        if self.started_wall is None or self.finalized:return
        if sim<self.last_sim-1e-6:
            self.odom_anomalies['simulation_clock_reset']+=1
            self.finish('clock_reset',wall,self.last_sim)
            return
        dt=max(0.,sim-self.last_sim)
        self.state_seconds[self.status.get('state','unreported')]+=dt
        if self.instant_floor and not self.status.get('mission',{}).get('returning',False):
            self.floor_seconds[self.instant_floor]+=dt
            self.floor_wall_seconds[self.instant_floor]+=max(0.,wall-self.last_wall)
        self.last_wall=float(wall);self.last_sim=float(sim)
        if self.transition and sim-self.transition['started_sim']>=self.transition_timeout:
            self._end_transition('timeout',sim)
        summary=self.coverage.summary()
        for level in (.5,.8,.9,.95,.99):
            if summary['coverage_ratio'] is not None and summary['coverage_ratio']>=level:
                self.coverage_threshold_times.setdefault(str(level),sim-self.started_sim)
        self.area_history.append((sim,summary['covered_reference_area_m2']
                                  if self.coverage.seen is not None else self.coverage.area))
        while len(self.area_history)>2 and self.area_history[1][0]<sim-10.:
            self.area_history.pop(0)

    def odometry(self,position,stamp,wall,reference_height=.32):
        if self.started_wall is None or self.finalized:return
        if self.last_received_stamp is not None and stamp<=self.last_received_stamp:
            self.odom_anomalies['out_of_order_or_duplicate']+=1;return
        self.last_received_stamp=stamp
        p=np.asarray(position,dtype=float)
        if not np.isfinite(p).all():self.odom_anomalies['nonfinite']+=1;return
        if self.path_last_sample is None or stamp-self.path_last_sample>=.1-1e-8:
            if self.path_anchor is not None:
                dt=stamp-self.path_stamp;delta=p-self.path_anchor
                if stamp-self.path_last_sample>2.:self.odom_anomalies['gap_over_2s']+=1
                elif np.linalg.norm(delta)>3.*dt+.5:self.odom_anomalies['position_jump']+=1
                elif np.linalg.norm(delta[:2])>=.01:
                    self.path_xy+=float(np.linalg.norm(delta[:2]));self.path_xyz+=float(np.linalg.norm(delta))
                else:
                    # Keep the last distance anchor while rejecting body jitter.
                    self.path_last_sample=stamp
                    self._floor_progress(p-[0.,0.,reference_height],stamp)
                    if self.single_floor:self.single_floor.status(self.status,self.instant_floor,wall,stamp)
                    return
            self.path_anchor=p;self.path_stamp=stamp;self.path_last_sample=stamp
        self._floor_progress(p-[0.,0.,reference_height],stamp)
        if self.single_floor:
            self.single_floor.status(self.status,self.instant_floor,wall,stamp)

    def _floor_progress(self,ground,sim):
        floor=self.coverage.floor_at(ground,self.floors)
        self.instant_floor=floor
        if self.current_floor and self.transition is None and not self.departure_latched:
            current=next(f for f in self.floors if f['name']==self.current_floor)
            dz=ground[2]-current['height']
            if abs(dz)>self.departure_height:
                possible=[f for f in self.floors if (f['height']-current['height'])*dz>0]
                if possible:
                    target=min(possible,key=lambda f:abs(f['height']-current['height']))
                    self.transition=dict(from_floor=self.current_floor,to_floor=target['name'],
                        started_sim=sim,direction='up' if dz>0 else 'down')
                    self.departure_latched=True
        if floor is None:self.floor_candidate=None;return
        if self.floor_candidate is None or self.floor_candidate[0]!=floor:
            self.floor_candidate=(floor,sim)
        if sim-self.floor_candidate[1]<self.floor_dwell:return
        self.arrivals.setdefault(floor,sim-self.started_sim)
        if self.transition:
            if floor==self.transition['to_floor']:self._end_transition('succeeded',sim)
            elif floor==self.transition['from_floor']:self._end_transition('returned_to_origin',sim)
        self.current_floor=floor
        self.departure_latched=False

    def _end_transition(self,outcome,sim):
        result=dict(self.transition,outcome=outcome,ended_sim=sim,
                    duration_sim_seconds=sim-self.transition['started_sim'])
        self.transitions.append(result);self.transition=None

    def planning_result(self,event):
        key=tuple(event.get(k) for k in ('goal_id','attempt_id','planner','occupancy_version','layer_version'))
        if key in self.plans:return
        self.plans[key]=dict(event)
        elapsed=event.get('planning_ms')
        if isinstance(elapsed,(int,float)) and not isinstance(elapsed,bool) and math.isfinite(elapsed) and elapsed>=0:
            self.timings['planning_ms'].append(elapsed)

    def execution_result(self,event):
        key=tuple(event.get(k) for k in ('goal_id','attempt_id','stage_index','scan_goal_stamp'))
        self.feedback.setdefault(key,dict(event))

    def finish(self,reason,wall,sim):
        if self.finalized:return
        if self.started_wall is not None:
            self.tick(wall,sim)
            if self.finalized:return
            if self.transition:self._end_transition('interrupted' if reason!='time_limit' else 'timeout',sim)
        if self.single_floor:self.single_floor.close('timeout' if reason=='time_limit' else 'interrupted',wall,sim)
        self.termination=reason;self.finalized=True

    def summary(self):
        elapsed=None if self.started_sim is None else max(0.,self.last_sim-self.started_sim)
        wall=None if self.started_wall is None else max(0.,self.last_wall-self.started_wall)
        attempts=len(self.transitions)+(self.transition is not None)
        successes=sum(t['outcome']=='succeeded' for t in self.transitions)
        failed=sum(t['outcome'] not in ('succeeded','interrupted') for t in self.transitions)
        completed=successes+failed
        plans=list(self.plans.values());feedback=list(self.feedback.values())
        upward=[t for t in self.transitions if t['direction']=='up']
        up_success=[t for t in upward if t['outcome']=='succeeded']
        up_pending=int(self.transition is not None and self.transition['direction']=='up')
        up_attempts=len(upward)+up_pending
        plan_times=distribution(self.timings['planning_ms'])
        recent=None
        if len(self.area_history)>1:
            dt=self.area_history[-1][0]-self.area_history[0][0]
            if dt>0:recent=(self.area_history[-1][1]-self.area_history[0][1])/dt
        mission_status=self.status.get('mission',{})
        return_required=mission_status.get('return_enabled',False)
        criteria=('target_exploration_settled_and_returned_home' if return_required else
                  'target_floor_arrival' if self.target_floor else 'reference_coverage_95_percent')
        mission=(self.target_floor in self.arrivals if self.target_floor else
                 (self.coverage.summary()['coverage_ratio']>=.95 if self.coverage.seen is not None else None))
        if return_required:
            mission=bool(mission_status.get('exploration_finished') and mission_status.get('returned_home') and
                         (not self.target_floor or self.target_floor in self.arrivals))
        floors=[]
        for floor in self.floors:
            area=seen=percent=None
            if self.coverage.reference is not None:
                mask=abs(self.coverage.reference[:,2]-floor['height'])<=.08+1e-7
                n=int(mask.sum());v=int(self.coverage.seen[mask].sum())
                area=n*self.coverage.resolution**2;seen=v*self.coverage.resolution**2
                percent=100.*v/n if n else None
            floors.append(dict(name=floor['name'],ground_height_m=floor['height'],
                coverage_percent=percent,observed_area_m2=seen,reference_area_m2=area,
                exploration_sim_seconds=self.floor_seconds[floor['name']],
                exploration_wall_seconds=self.floor_wall_seconds[floor['name']],
                arrival_sim_seconds=self.arrivals.get(floor['name'])))
        covered=self.coverage.summary()['covered_reference_area_m2']
        new_reference=None if self.baseline_reference_area is None else covered-self.baseline_reference_area
        area_gain=(new_reference if new_reference is not None else
                   (self.coverage.area-self.baseline_area if self.baseline_area is not None else None))
        return dict(schema_version=1,finalized=self.finalized,termination=self.termination,
            partial_run=self.partial_run,started=self.started_wall is not None,
            single_floor=None if self.single_floor is None else self.single_floor.summary(self.last_wall,self.last_sim),
            floors=floors,returned_home=bool(mission_status.get('returned_home')),
            target_exploration_finished=bool(mission_status.get('exploration_finished')),
            elapsed_sim_seconds=elapsed,elapsed_wall_seconds=wall,
            real_time_factor=None if not wall else elapsed/wall,
            path_length_xy_m=self.path_xy,body_path_length_xyz_m=self.path_xyz,
            path_definition='10 Hz odometry; XY 1 cm deadband; gaps >2s and jumps >3m/s+0.5m excluded',
            **self.coverage.summary(),
            new_mapped_area_m2=None if self.baseline_area is None else self.coverage.area-self.baseline_area,
            newly_covered_reference_area_m2=new_reference,
            exploration_speed_m2_per_sim_s=None if not elapsed else area_gain/elapsed,
            exploration_speed_basis='reference_covered_area' if covered is not None else 'mapped_surface_area_no_reference',
            recent_exploration_speed_m2_per_sim_s=recent,
            covered_area_per_path_m=None if not self.path_xy or self.coverage.seen is None else
                self.coverage.summary()['covered_reference_area_m2']/self.path_xy,
            coverage_threshold_sim_seconds=dict(self.coverage_threshold_times),
            floor_arrival_sim_seconds=dict(self.arrivals),current_physical_floor=self.current_floor,
            cross_layer=dict(started=attempts,succeeded=successes,failed=failed,
                interrupted=sum(t['outcome']=='interrupted' for t in self.transitions),
                pending=int(self.transition is not None),
                success_rate=None if not attempts else successes/attempts,
                completed_success_rate=None if not completed else successes/completed,
                attempts=self.transitions+([dict(self.transition,outcome='pending')] if self.transition else [])),
            upward=dict(started=up_attempts,succeeded=len(up_success),
                failed=sum(t['outcome'] not in ('succeeded','interrupted') for t in upward),
                interrupted=sum(t['outcome']=='interrupted' for t in upward),pending=up_pending,
                success_rate=None if not up_attempts else len(up_success)/up_attempts,
                mean_sim_seconds=None if not up_success else
                    sum(t['duration_sim_seconds'] for t in up_success)/len(up_success)),
            mission_success=mission,success_criterion=criteria,target_floor=self.target_floor or None,
            includes_return=bool(return_required),
            planning=dict(attempts=len(plans),usable=sum(bool(p.get('usable')) for p in plans),
                success_rate=None if not plans else sum(bool(p.get('usable')) for p in plans)/len(plans),
                timing_samples=plan_times['count'],
                # Do not silently report a mean/P95 over only the timed successes.
                mean_ms=plan_times['mean'] if plan_times['count']==len(plans) else None,
                p95_ms=plan_times['p95'] if plan_times['count']==len(plans) else None,
                reasons=dict(Counter(p.get('reason','') for p in plans))),
            execution=dict(terminal_waypoints=len(feedback),
                succeeded=sum(p.get('status')=='succeeded' for p in feedback),
                failures=dict(Counter(p.get('reason','') for p in feedback if p.get('status')=='failed'))),
            timing_ms={key:distribution(values) for key,values in self.timings.items()},
            state_sim_seconds=dict(self.state_seconds),data_quality=dict(self.odom_anomalies))


class ResultWriter:
    """One compact JSON per run, atomically refreshed so interruptions survive."""
    def __init__(self,root,metadata,run_id=None):
        name=run_id or datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8]
        self.directory=Path(root).expanduser();self.directory.mkdir(parents=True,exist_ok=True)
        self.prefix=self.directory/name
        with self.prefix.with_suffix('.json').open('x') as handle:
            json.dump(dict(schema_version=4,finalized=False,termination='initializing'),handle)
        keys=('created_at_utc','method','world','world_sha256','git_head','localization_source',
              'start_floor_skip_option','metric_resolution','height_tolerance','reference_issue','reference_builder_version',
              'source_sha256','reference_sha256','floor_dwell_seconds','transition_timeout_seconds',
              'robot_reference_height','coverage_source','target_floor','trial_seed','single_floor')
        self.metadata={k:metadata[k] for k in keys if k in metadata}
        self.metadata['run_id']=name

    def event(self,kind,sim,data):pass
    def row(self,values):pass
    def close(self):pass

    def snapshot(self,summary,extra=None):
        keys=('started','finalized','termination','partial_run','target_exploration_finished',
              'returned_home','mission_success','success_criterion')
        result=dict(schema_version=4,**{k:summary.get(k) for k in keys},metadata=self.metadata)
        plan=summary['planning'];up=summary['upward']
        scope=summary.get('single_floor')
        result['single_floor']=(None if scope is None else {k:scope[k] for k in
            ('name','ground_height_m','state','completed','started_sim','ended_sim')})
        measured=scope if scope is not None else dict(
            reference_area_m2=summary['reference_area_m2'],observed_area_m2=summary['covered_reference_area_m2'],
            coverage_percent=summary['coverage_percent'],exploration_sim_seconds=summary['elapsed_sim_seconds'],
            exploration_wall_seconds=summary['elapsed_wall_seconds'])
        percent=lambda value:None if value is None else 100.*value
        result['metrics']=dict(
            reference_area_m2=measured['reference_area_m2'],
            observed_area_m2=measured['observed_area_m2'],
            coverage_percent=measured['coverage_percent'],
            planning_success_count=plan['usable'],planning_attempt_count=plan['attempts'],
            planning_success_percent=percent(plan['success_rate']),
            planning_mean_ms=plan['mean_ms'],planning_p95_ms=plan['p95_ms'],
            upward_success_count=up['succeeded'],upward_attempt_count=up['started'],
            upward_success_percent=percent(up['success_rate']),
            upward_mean_sim_seconds=up['mean_sim_seconds'],
            exploration_sim_seconds=measured['exploration_sim_seconds'],
            exploration_wall_seconds=measured['exploration_wall_seconds'])
        result['metric_labels']={
            'reference_area_m2':'待探索地面总面积（m²）','observed_area_m2':'已观测地面总面积（m²）',
            'coverage_percent':'探索覆盖率（%）','planning_success_count':'路径规划成功次数',
            'planning_attempt_count':'路径规划总次数','planning_success_percent':'路径规划成功率（%）',
            'planning_mean_ms':'平均路径规划耗时（ms）','planning_p95_ms':'P95路径规划耗时（ms）',
            'upward_success_count':'上楼成功次数','upward_attempt_count':'上楼总次数',
            'upward_success_percent':'上楼成功率（%）','upward_mean_sim_seconds':'成功上楼任务平均耗时（仿真s）',
            'exploration_sim_seconds':'探索耗时（仿真s）','exploration_wall_seconds':'探索耗时（墙钟s）'}
        result['accounting']=dict(upward_failed_count=up['failed'],upward_interrupted_count=up['interrupted'],
            upward_pending_count=up['pending'],planning_timing_samples=plan['timing_samples'],
            coverage_reference_available=measured['reference_area_m2'] is not None,
            includes_return=bool(summary.get('includes_return',False)))
        result['definitions']={
            'coverage':('仅评价'+scope['name']+'；结束时已观测地面面积/该层独立参考面积；包含到达前提前扫到的该层地面，按物理地面去重。' if scope else
                '参考范围内成熟实测地面投影面积/独立参考总面积；按XY及地面高度去重，切片别名不重复计数。'),
            'planning':'安全可执行路线数/全部已结束请求数；包括失败与超时，不等于执行成功率。耗时统计全部请求，P95采用线性插值。',
            'upward':'仅统计向上的物理跨层任务；成功数/全部尝试数（含失败、超时、中断）。重规划及补扫不另增任务；平均耗时仅统计成功任务。',
            'time':('从'+scope['name']+'普通探索开始，到该层首次局部及全局首轮收尾；包含选点、规划、行走、观测和等待，不含前后跨层及返航。未完成时为实际已用时间。' if scope else
                '执行就绪、任务活动且已有位姿时开始，至结束/中断/超时；包括选点、观测、行走、等待和跨层。返航配置由includes_return标明。'),
            'unavailable':'无独立参考、无尝试或无有效完整耗时样本时为null；未结束文件finalized=false，不视为完成试验。'}
        temp=Path(str(self.prefix)+'.json.tmp')
        try:
            temp.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
            temp.replace(self.prefix.with_suffix('.json'))
        finally:
            temp.unlink(missing_ok=True)
        return result
