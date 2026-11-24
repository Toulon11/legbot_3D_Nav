"""Sensor-aware views and evidence-backed observation tasks; no surface inference."""
import math
import numpy as np
from collections import OrderedDict

from .pct_frontier_index import frontier_return


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def ray_columns(start, target, settings):
    """Yield every grid column touched by the open ray, with its parameter interval."""
    origin=np.array([settings.origin_x,settings.origin_y])
    a=(start[:2]-origin)/settings.resolution+.5
    b=(target[:2]-origin)/settings.resolution+.5
    cell=np.floor(a).astype(int)
    delta=b-a
    step=np.sign(delta).astype(int)
    next_edge=[(cell[i]+(step[i]>0)-a[i])/delta[i] if step[i] else math.inf for i in (0,1)]
    stride=[1./abs(d) if d else math.inf for d in delta]
    edge_x=not step[0] and abs(a[0]-cell[0])<1e-9
    edge_y=not step[1] and abs(a[1]-cell[1])<1e-9
    first=0.
    while first<1.-1e-9:
        last=min(1.,*next_edge)
        x,y=map(int,cell)
        yield (x,y),first,last
        if edge_x:yield (x-1,y),first,last
        if edge_y:yield (x,y-1),first,last
        if edge_x and edge_y:yield (x-1,y-1),first,last
        if last>=1.-1e-9:
            break
        crossed=[i for i in (0,1) if next_edge[i]<=last+1e-9]
        if len(crossed)==2 and last>1e-9:
            yield (x+int(step[0]),y),last,last
            yield (x,y+int(step[1])),last,last
        for i in crossed:
            cell[i]+=step[i]
            next_edge[i]+=stride[i]
        first=last


class ObservationModel:
    def __init__(self, mapping):
        self.map = mapping
        self.task = None
        self.floor_attempt = None
        self.visibility_cache=OrderedDict()
        from .pct_blocker_review import BlockerReview
        self.blocker_review=BlockerReview(mapping)
        from .pct_observation_poses import ObservationPoseSearch
        self.pose_search=ObservationPoseSearch(self)

    def column_blockers(self, xy):
        cell=tuple(xy[:2])
        bands=self.map.surface_relations.physical_blockers(cell)
        if len(xy)>2:
            barrier=self.map.surface_relations.fresh_clearance_barrier(cell,float(xy[2]))
            if barrier is not None:bands=(*bands,barrier)
        return bands

    def ray_evidence(self, key):
        """Cache dependencies include both optical returns and known grid barriers."""
        if len(key)==4 and key[3]=='selection_layer':
            return self.map.frontier_selection_layer(key[:3])[0]
        if len(key)==4 and key[3]=='support_obstacle':
            return self.support_obstacle(key[:2],key[2])
        if len(key)==4 and key[3]=='known_obstacle':
            x,y,layer,_=key
            row=self.map.observed_cell((layer,x,y))
            return row is not None and (bool(row['solid']) or row['terrain_cost']>=50.)
        if len(key)==5 and key[4]=='ground_band_obstacle':
            x,y,layer,height,_=key
            return (self.ray_evidence((x,y,layer,'known_obstacle')) or
                    self.map.ground_band_obstacle((x,y),height))
        return self.column_blockers(key)

    def support_obstacle(self, xy, height):
        """A measured obstacle in the observer's ground space, across cuts."""
        tolerance=self.map.settings.surface_noise_tolerance+1e-7
        records=self.map._column_known_intervals.get(tuple(xy),())
        if self.map.ground_band_obstacle(xy,height):
            column=self.map.columns[tuple(xy)]
            step=min(.16,self.map.settings.step_max)
            bands=[(a,b) for a,b in column.obstacle_intervals
                   if a<height+self.map.settings.minimum_clearance-1e-7 and b>height+step+1e-7]
            # A measured short riser ending at a real upper tread is not a
            # wall on the observer's floor. Unobserved tops get no exemption.
            if any(b-a>step+1e-7 or not any(abs(g-b)<=tolerance for g,_,_,_,_ in records)
                   for a,b in bands):return True
        for g,c,_,lo,hi in records:
            if abs(g-height)>tolerance:continue
            # A valid cut inside this measured interval may itself be virtual.
            row=self.map.observed_cell((lo,*xy))
            if row is not None and (bool(row['solid']) or row['terrain_cost']>=50.):return True
        s=self.map.settings
        layer=self.map.frontier_selection_layer((s.origin_x+xy[0]*s.resolution,
                                                s.origin_y+xy[1]*s.resolution,height))[0]
        row=self.map.observed_cell((layer,*xy))
        return (row is not None and np.isfinite(row['G']) and abs(row['G']-height)<=tolerance and
                (bool(row['solid']) or row['terrain_cost']>=50.))

    def grid_view_blocked(self, point, target, layer, dependencies):
        """An observation stance may not look across a known obstacle cell.

        Only the XY projection and measured obstacles in the stance's ground
        space matter. Unknown inflation is not a measured wall. A different
        floor is not occlusion evidence for this view.
        """
        for xy,first,last in ray_columns(np.asarray(point),np.asarray(target),self.map.settings):
            key=(*xy,float(point[2]),'support_obstacle')
            if key not in dependencies:dependencies[key]=self.ray_evidence(key)
            if dependencies[key]:return True
        return False

    def ray_blocked(self, sensor, target, dependencies=None):
        """Trace observed surfaces, including vertical bands, independently of FOV."""
        sensor=np.asarray(sensor,dtype=float)
        target=np.asarray(target,dtype=float)
        target_xy=tuple(self.map.indices(target))
        for xy,first,last in ray_columns(sensor,target,self.map.settings):
            if xy==target_xy:
                continue
            first,last=max(first,1e-7),min(last,1.-1e-7)
            if first>last:
                continue
            bands=self.column_blockers(xy)
            if dependencies is not None:
                dependencies[xy]=bands
            z0,z1=sorted((sensor[2]+first*(target[2]-sensor[2]),sensor[2]+last*(target[2]-sensor[2])))
            if any(lo<=z1 and hi>=z0 for lo,hi in bands):
                return True
        return False

    def visible(self, point, yaw, targets, occlusion=True, ray_dependencies=None, sensor_pose=None,
                range_margin=0.):
        """Expected ground points in the mounted LiDAR frame (level body model)."""
        s = self.map.settings
        targets = np.asarray(targets, dtype=float).reshape(-1, 3)
        layer=self.map.frontier_selection_layer(point)[0] if occlusion else None
        key=(tuple(point),float(yaw),targets.tobytes(),sensor_pose,float(range_margin),layer)
        if occlusion and key in self.visibility_cache:
            cached,dependencies=self.visibility_cache[key]
            if all(self.ray_evidence(xy)==bands for xy,bands in dependencies.items()):
                self.visibility_cache.move_to_end(key)
                if ray_dependencies is not None:ray_dependencies.update(dependencies)
                return cached.copy()
            del self.visibility_cache[key]
        delta = targets - point
        c, sn = math.cos(yaw), math.sin(yaw)
        bx = delta[:, 0]*c + delta[:, 1]*sn - s.ground_lidar_x
        by = -delta[:, 0]*sn + delta[:, 1]*c
        bz = delta[:, 2] - s.observer_reference_height - s.ground_lidar_z
        cp, sp = math.cos(s.ground_lidar_pitch), math.sin(s.ground_lidar_pitch)
        sx, sz = cp*bx-sp*bz, sp*bx+cp*bz
        sensor = np.array(point) + [s.ground_lidar_x*c, s.ground_lidar_x*sn,
                                    s.observer_reference_height+s.ground_lidar_z]
        if sensor_pose is not None:
            sensor = np.asarray(sensor_pose[:3])
            sx, by, sz = ((targets-sensor) @ np.asarray(sensor_pose[3:]).reshape(3,3)).T
        az = np.arctan2(by, sx)
        el = np.arctan2(sz, np.hypot(sx, by))
        dist = np.sqrt(sx*sx+by*by+sz*sz)
        valid = (abs(az) <= s.ground_lidar_horizontal_half_angle) & \
            (el >= s.ground_lidar_elevation_min) & (el <= s.ground_lidar_elevation_max) & \
            (dist <= s.ground_lidar_max_range) & (dist > 0.)
        valid &= (dist>=s.ground_lidar_min_range+range_margin) & (dist<=s.ground_lidar_max_range-range_margin)
        if occlusion:
            dependencies={(*map(float,point),'selection_layer'):layer}
            for at in np.flatnonzero(valid):
                target=targets[at]
                if self.grid_view_blocked(point,target,layer,dependencies):
                    valid[at]=False
                    continue
            self.visibility_cache[key]=(valid.copy(),dependencies)
            if ray_dependencies is not None:ray_dependencies.update(dependencies)
            while len(self.visibility_cache)>256:self.visibility_cache.popitem(last=False)
        return valid

    def _visibility_view(self, point, targets, completed=(), occlusion=True, ray_dependencies=None,
                  admit_heading=None, minimum_gain=1):
        targets=np.asarray(targets,dtype=float).reshape(-1,3)
        if not len(targets):return None
        # Bounded heading proposals, all target points are scored in each view.
        bearings=np.arctan2(targets[:,1]-point[1],targets[:,0]-point[0])
        ids=np.linspace(0,len(targets)-1,min(16,len(targets)),dtype=int)
        proposals=[]
        for yaw in bearings[ids]:
            covered=self.visible(point,float(yaw),targets,False)
            if not covered.any():continue
            if self._tried_heading(point,yaw,completed):continue
            score=int(covered.sum())
            proposals.append((score,float(yaw),covered))
        if not proposals:return None
        proposals.sort(key=lambda p:-p[0])
        if not occlusion:
            _,yaw,covered=proposals[0]
            return dict(yaw=yaw,gain=int(covered.sum()),mask=covered)
        best=None
        for possible_gain,yaw,_ in (proposals if admit_heading else proposals[:4]):
            if admit_heading and (possible_gain<minimum_gain or (best and possible_gain<=best['gain'])):
                break
            covered=self.visible(point,yaw,targets,ray_dependencies=ray_dependencies)
            gain=int(covered.sum())
            if gain>=minimum_gain and (best is None or gain>best['gain']) and (not admit_heading or admit_heading(yaw)):
                best=dict(yaw=yaw,gain=gain,mask=covered)
        return best

    def floor_view(self, keys, point, yaw):
        index=self.map.frontier_index;s=self.map.settings
        targets=index.physical_targets(key for key in keys if key in index.nodes)
        unique=np.asarray([p for p,_ in targets]).reshape(-1,3)
        near=(np.linalg.norm(unique[:,:2]-point[:2],axis=1)<=index.radius)&(
            abs(unique[:,2]-point[2])<=s.step_max)
        near[near]=self.visible(point,yaw,unique[near])
        return {key for (_,aliases),visible in zip(targets,near) if visible for key in aliases},int(near.sum())

    def _tried_heading(self, point, yaw, completed):
        s=self.map.settings
        for done in completed:
            if len(done)<5:continue  # Legacy XYZ records are handled by the position query.
            left,right=(-done[4],done[4]) if len(done)==5 else done[4:6]
            if (left<=wrap(yaw-done[3])<=right and abs(point[2]-done[2])<=s.step_max and
                math.hypot(point[0]-done[0],point[1]-done[1])<=s.observation_revisit_radius):
                return True
        return False

    def _admitted_view(self, ground, targets, completed=(), headings=None, minimum_gain=1, diagnostics=None):
        body=np.asarray(ground,dtype=float)+[0.,0.,self.map.settings.observer_reference_height]
        found=self.pose_search.observation_poses(body,(0.,0.,0.),targets,
            stances=[(body,0,True)],completed=completed,headings=headings,
            minimum_gain=minimum_gain,max_results=1,diagnostics=diagnostics is not None)
        if diagnostics is not None:
            diagnostics['checks'].append({name:found[name] for name in
                ('stances_checked','rejections','rejected_poses','view_counts')})
        return found['views'].get(tuple(body))

    @staticmethod
    def _view_failure(diagnostics):
        checks=diagnostics['checks']
        if not checks:return 'observation_pose_unavailable'
        failures={}
        for check in checks:
            for reason,count in check['rejections'].items():
                failures[reason]=failures.get(reason,0)+count
        arrival={reason:count for reason,count in failures.items() if reason.startswith('arrival_')}
        if arrival:return max(arrival,key=arrival.get)
        counts=[check['view_counts'] for check in checks]
        if counts and all(c['headings']==c['repeated_headings'] for c in counts):
            return 'observation_heading_already_tried'
        if not any(c['max_in_field'] for c in counts):return 'frontier_outside_lidar_view'
        if not any(c['max_visible'] for c in counts):return 'frontier_occluded'
        return 'insufficient_visible_frontier'

    def begin(self, cid, point, forward=None, half_angle=0., current_yaw=None, floor_retry=False):
        index=self.map.frontier_index;s=self.map.settings
        attempt=self.floor_attempt if floor_retry and self.floor_attempt is not None and self.floor_attempt.cluster==cid else None
        fast=attempt is not None and attempt.fast
        self.floor_attempt=None
        cluster=index.task_cluster(cid)
        if attempt is not None:
            keys=sorted(index.remaining_floor_members({key:attempt.members[key] for key in attempt.local
                                                       if key in attempt.members}))
        else:
            keys=sorted(cluster['members']) if cluster else []
            if floor_retry:keys=[key for key in keys if key in index.floor_query_members.get(cid,())]
        if not keys and (attempt is not None or (forward is None and (floor_retry or cluster is None))):
            return dict(ready=False,reason='frontier_resolved',
                        floor_attempt=index.settle_floor_attempt(attempt,False))
        targets=np.array([[s.origin_x+key[1]*s.resolution,s.origin_y+key[2]*s.resolution,index.nodes[key].height]
                          for key in keys]).reshape(-1,3)
        # Only nearby compatible 3-D frontier surfaces belong to this view.
        near=np.linalg.norm(targets[:,:2]-point[:2],axis=1)<=index.radius
        if forward is None:
            near &= abs(targets[:,2]-point[2])<=s.step_max
        else:
            bearings=np.arctan2(targets[:,1]-point[1],targets[:,0]-point[0])
            near &= (np.cos(bearings-forward)>0) & (targets[:,2]>=point[2]-s.surface_noise_tolerance)
        targets=targets[near];keys=[key for key,keep in zip(keys,near) if keep]
        diagnostics=dict(remaining_frontiers=len(near),near_frontiers=len(keys),checks=[])
        if not keys and forward is None:
            return dict(ready=False,reason='frontier_outside_observation_range',diagnostic=diagnostics,
                        floor_attempt=index.settle_floor_attempt(attempt,True,tuple(point)))
        if attempt is None:
            unique,inverse=np.unique(targets,axis=0,return_inverse=True)
        else:
            groups=index.physical_targets(keys)
            unique=np.asarray([p for p,_ in groups]).reshape(-1,3)
            group_of={key:i for i,(_,aliases) in enumerate(groups) for key in aliases}
            inverse=np.asarray([group_of[key] for key in keys],dtype=int)
        side_yaw=None
        side_keys=[]
        side_gain=0
        if forward is None:
            completed=[]
            if current_yaw is not None and keys:
                members=set(keys)
                completed=[p for p in index._completed_near(members)
                           if any(k in members for k,_ in index.completed_views.get(p,()))]
            view=self._admitted_view(point,unique,completed,diagnostics=diagnostics)
            retry=index.retry_completion(cid,attempt.floor,keys) if attempt is not None else None
            if view is None and retry in completed:
                completed=[p for p in completed if p!=retry]
                view=self._admitted_view(point,unique,completed,diagnostics=diagnostics)
            if current_yaw is not None:
                minimum_gain=max(1,math.ceil(.8*view['gain']) if view is not None else 0)
                in_field=self.visible(point,current_yaw,unique,False)
                if in_field.sum()>=minimum_gain and not self._tried_heading(point,current_yaw,completed):
                    current=self._admitted_view(point,unique,completed,[current_yaw],minimum_gain,diagnostics)
                    if current is not None:view=current
                if view is not None and half_angle>0. and not fast:
                    # A few peripheral cells alone do not justify another stop.
                    minimum_added=max(2,math.ceil(.2*view['gain']))
                    for offset in (-half_angle,half_angle):
                        heading=wrap(view['yaw']+offset)
                        if self._tried_heading(point,heading,completed):
                            continue
                        added=self.visible(point,heading,unique,False) & ~view['mask']
                        if added.sum()<max(minimum_added,side_gain+1):
                            continue
                        side=self._admitted_view(point,unique,completed,[heading],minimum_added,diagnostics)
                        if side is None:continue
                        added &= side['mask']
                        gain=int(added.sum())
                        if gain>=minimum_added and gain>side_gain:
                            side_yaw=heading
                            side_gain=gain
                            side_keys=[(key,index.nodes[key].height) for key,keep in zip(keys,added[inverse]) if keep]
        else:
            # Keep the sweep centered on the verified direction of travel.
            # The three bearings may illuminate different higher surfaces.
            view=self._admitted_view(point,targets,headings=[forward],minimum_gain=0,diagnostics=diagnostics)
            visible=np.zeros(len(targets),dtype=bool)
            if view is not None:
                for offset in (0.,-half_angle,half_angle):
                    side=self._admitted_view(point,targets,headings=[forward+offset],minimum_gain=0,diagnostics=diagnostics)
                    if side is not None:visible |= side['mask']
                view=dict(yaw=forward,gain=len(np.unique(targets[visible],axis=0)))
        if view is None or (attempt is not None and view['gain']==0):
            diagnostics.update(physical_frontiers=len(unique),visible_count=view['gain'] if view else 0,
                               required_visible=1)
            return dict(ready=False,reason=self._view_failure(diagnostics) if view is None else 'insufficient_visible_frontier',
                        diagnostic=diagnostics,
                        floor_attempt=index.settle_floor_attempt(attempt,True,tuple(point)))
        mask=view['mask'][inverse] if forward is None else visible
        keys=[key for key,keep in zip(keys,mask) if keep]
        self.task=dict(keys=keys,heights=[index.nodes[k].height for k in keys],floor_attempt=attempt,
                       cluster=cid,members={k:index.nodes[k] for k in keys},
                       serial=self.map.integrated_serial,point=tuple(point),yaw=view['yaw'],
                       armed=False,view_yaws=[],side_yaw=side_yaw,side_keys=side_keys)
        return dict(ready=True,reason='ok',yaw=view['yaw'],gain=view['gain'],
                    side_yaw=side_yaw,side_gain=side_gain,diagnostic=diagnostics)

    def arm(self, point, yaw):
        if self.task is None:return False
        self._record_floor_view()
        side=self.task.get('side_yaw')
        if side is not None and abs(wrap(yaw-side)) < abs(wrap(yaw-self.task['yaw'])):
            for key,height in self.task['side_keys']:
                self.task['keys'].append(key)
                self.task['heights'].append(height)
            self.task['side_keys'].clear()
        self.task.setdefault('view_yaws',[]).append(float(yaw))
        self.task.update(serial=self.map.integrated_serial,point=tuple(point),yaw=float(yaw),armed=True)
        index=self.map.frontier_index;s=self.map.settings
        evidence=self.map.surface_observation
        for key in self.task['keys']:
            boundary=index.nodes.get(key)
            if boundary is None:
                continue
            target=(s.origin_x+key[1]*s.resolution,s.origin_y+key[2]*s.resolution,boundary.height)
            sources=[(x,y,sid) for x,y,_,sid in boundary.sources]
            evidence.expect(target,sources=sources,slices=[key[0]],kind='frontier')
        if self.task.get('floor_attempt') is not None:
            self._capture_floor_view(point,yaw)
        return True

    def _capture_floor_view(self, point, yaw):
        m=self.map;index=m.frontier_index;s=m.settings
        task=self.task
        keys=[key for key in task['keys'] if key in index.nodes]
        targets=[(s.origin_x+key[1]*s.resolution,s.origin_y+key[2]*s.resolution,index.nodes[key].height)
                 for key in keys]
        boundaries=[(key,index.nodes[key]) for key,visible in zip(keys,self.visible(point,yaw,targets)) if visible]
        baseline={}
        for xy in {key[1:3] for key,_ in boundaries}:
            column=m.columns.get(xy)
            baseline[xy]={r.id for r in column.values() if frontier_return(r.orientation)} if column else set()
        task['floor_view']=(task['serial'],boundaries,baseline)

    def _record_floor_view(self):
        view=self.task.pop('floor_view',None)
        m=self.map;index=m.frontier_index
        if view is None or m.integrated_serial<=view[0]:
            return
        serial,boundaries,baseline=view
        self.task['floor_frame_received']=True
        covered=self.task.setdefault('floor_covered',{})
        tolerance=m.settings.surface_noise_tolerance+1e-7
        for key,boundary in boundaries:
            xy=key[1:3]
            matches=covered.setdefault((xy,boundary),set())
            if any(index._same_boundary(boundary,index.nodes[k]) for k in index.by_column.get(xy,())):
                continue
            column=m.columns.get(xy)
            if column:
                matches.update((r.id,r.height) for r in column.values() if frontier_return(r.orientation)
                               and r.id not in baseline[xy] and r.last_seen>serial
                               and abs(r.height-boundary.height)<=tolerance)

    def progress(self):
        task=self.task
        if not task or not task['armed']:return dict(ready=False,reason='not_armed',gain=0,fresh=False)
        s=self.map.settings;resolved=set();fresh=False
        evidence=self.map.surface_observation
        for key,height in zip(task['keys'],task['heights']):
            x,y=key[1:3]
            target=(*self.map.position((0,x,y)),height)
            check=evidence.query(target,kind='frontier',layer=key[0])
            if check['ready']:resolved.add((x,y,height))
            if evidence.has_frontier_ground((x,y),height,task['serial'],layer=key[0]):fresh=True
            if check['state']=='refuted' and evidence.refuted((x,y),height,after=task['serial'],include_step=False):
                fresh=True
            # Frame must actually have integrated measured points near the
            # intended frontier at the right height, after alignment finished.
            for dx,dy in ((0,0),(1,0),(-1,0),(0,1),(0,-1)):
                if self.map.surface_observation.matching((x+dx,y+dy),height,task['serial'],kind='frontier'):fresh=True
        self._record_floor_view()
        gain=len(resolved)
        return dict(ready=bool(gain and fresh),reason='observed' if gain and fresh else 'waiting_local_observations',gain=gain,fresh=fresh)

    def finish(self, point, yaw, half_angle):
        if self.task is None:return None
        task=self.task
        index=self.map.frontier_index
        # Preserve the actual observation footprint and original frontier ownership.
        offsets=[wrap(v-yaw) for v in task.get('view_yaws',[yaw])]
        field=self.map.settings.ground_lidar_horizontal_half_angle
        left,right=(max(-math.pi,min(offsets)-field),min(math.pi,max(offsets)+field))
        if half_angle>=math.pi:left,right=-math.pi,math.pi
        record=(*map(float,point),float(yaw),left,right)
        attempt=task.get('floor_attempt')
        settlement=dict(ready=True,reason='completed',floor_attempt='untracked')
        if attempt is not None:
            self._record_floor_view()
            covered,resolved=index.floor_observation_counts(task.get('floor_covered',{}))
            minimum=min(covered,max(self.map.settings.floor_min_resolved_frontier_cells,
                                    math.ceil(.10*covered)))
            outcome=index.settle_floor_attempt(attempt,task.get('floor_frame_received',False),record,
                                               sufficient_progress=0<minimum<=resolved,
                                               failure_reason='floor_low_progress')
            settlement.update(floor_attempt=outcome,covered_frontier_cells=covered,
                              resolved_frontier_cells=resolved,minimum_resolved_frontier_cells=minimum)
        if settlement['floor_attempt']!='inconclusive':
            index.complete(0,0,record)
            index.completed_views[record]=tuple((key,height) for key,height in zip(task['keys'],task['heights'])
                                                if settlement['floor_attempt']!='blacklisted' or key in index.nodes)
        self.task=None
        return settlement
