"""One pose search and arrival-area policy for frontier, stair and recovery views."""
from collections import OrderedDict
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation


class ObservationPoseSearch:
    def __init__(self, model):
        self.model=model
        self.map=model.map
        self._cache=OrderedDict()

    @property
    def reference_height(self):
        return self.map.settings.observer_reference_height

    def _admission(self, xyz, **constraints):
        return self.map.point_safe_check(xyz,self.reference_height,**constraints)

    def observation_poses(self, base, attitude, targets, stances=None, floor=None, ascent=None,
                          preferred=None, tolerance=.15, completed=(), minimum_gain=1,
                          max_results=3, motions=None, headings=None, diagnostics=False, purpose='arrival'):
        """New arrivals require full coverage; current scans require actual support."""
        if purpose not in ('arrival','current_observation'):
            raise ValueError('invalid observation purpose')
        current=purpose=='current_observation'
        if current and (motions is not False or stances is not None or minimum_gain<1):
            raise ValueError('current observation only admits the actual stationary pose')
        m,s=self.map,self.map.settings
        range_margin=(0. if current else tolerance)+s.resolution/math.sqrt(2.)
        base=np.asarray(base,dtype=float)
        targets=np.asarray(targets,dtype=float).reshape(-1,3)
        result=dict(poses=[],views={},unknown_blockers=[],rays={},timing_ms={})
        def timed(phase,call,*args,**kwargs):
            began=time.monotonic()
            try:return call(*args,**kwargs)
            finally:
                result['timing_ms'][phase]=result['timing_ms'].get(phase,0.)+1000*(time.monotonic()-began)
        if diagnostics:
            result.update(rejections={},rejected_poses=[],stances_checked=0,
                          view_counts=dict(targets=len(targets),headings=0,repeated_headings=0,
                                           max_in_field=0,max_visible=0))

        def reject(reason,stand,check=None):
            if not diagnostics:return
            result['rejections'][reason]=result['rejections'].get(reason,0)+1
            if len(result['rejected_poses'])<3:
                layers=[]
                for layer,row in m.layers_at(*m.indices(stand)):
                    layers.append(dict(slice=int(layer),G=float(row['G']) if np.isfinite(row['G']) else None,
                        C=float(row['C']) if np.isfinite(row['C']) else None,
                        cost=float(row['inflated_terrain_cost']) if np.isfinite(row['inflated_terrain_cost']) else None,
                        unknown=bool(row['unknown']),solid=bool(row['solid'])))
                result['rejected_poses'].append(dict(stance=list(map(float,stand)),reason=reason,
                    layers=layers,unknown_blockers=(check or {}).get('unknown_blockers',[])[:8],
                    observation_failures=(check or {}).get('observation_failures',[])))
        if m.first_layer is None or (not len(targets) and minimum_gain>0):return result
        if stances is None:
            stances=self._stances(base,attitude,targets,floor,ascent,preferred,motions)
        roll,pitch,yaw=attitude
        for stand,action,can_turn in stances:
            stand=np.asarray(stand,dtype=float)
            if diagnostics:result['stances_checked']+=1
            if floor is not None and abs(stand[2]-self.reference_height-floor[0])>floor[1]+1e-7:
                reject('stance_outside_floor',stand)
                continue
            stop=(timed('stop_ms',self._arrival_stop,stand,s.robot_radius+s.safety_margin)[2] if current else
                  timed('stop_ms',self._arrival_area,stand,tolerance,None))
            if not stop['ready']:
                reject(stop['reason'],stand,stop)
                result['unknown_blockers'].extend(stop.get('unknown_blockers',()))
                continue
            if not current and not roll and not pitch:
                blind_area = timed('blind_ms',self._arrival_area,stand,tolerance,yaw)
                if not blind_area['ready'] and blind_area.get('heading_independent',False):
                    reject(blind_area['reason'],stand,blind_area)
                    result['unknown_blockers'].extend(blind_area.get('unknown_blockers',()))
                    continue
            ground=stand-[0.,0.,self.reference_height]
            proposals=self._headings(stand,targets,yaw,ascent,can_turn) if headings is None else headings
            best=None
            rejected_area=None
            in_view=False
            # Use the same visibility model for the cheap upper bound and
            # full occlusion check. Level poses can use its default transform.
            for heading in proposals:
                if diagnostics:result['view_counts']['headings']+=1
                if motions is False and ascent is not None and abs(math.remainder(heading-yaw,2*math.pi))>.70:
                    continue
                if ascent is not None and abs(math.remainder(heading-ascent,2*math.pi))>.70:
                    continue
                if self.model._tried_heading(ground,heading,completed):
                    if diagnostics:result['view_counts']['repeated_headings']+=1
                    continue
                sensor_pose=None
                if roll or pitch:
                    body=Rotation.from_euler('xyz',(roll,pitch,heading))
                    mount=Rotation.from_euler('y',s.ground_lidar_pitch)
                    sensor=stand+body.apply([s.ground_lidar_x,0.,s.ground_lidar_z])
                    sensor_pose=(*map(float,sensor),*((body*mount).as_matrix().ravel()))
                visible=timed('fov_ms',self.model.visible,ground,heading,targets,False,
                              sensor_pose=sensor_pose,range_margin=range_margin)
                if diagnostics:
                    result['view_counts']['max_in_field']=max(result['view_counts']['max_in_field'],int(visible.sum()))
                if visible.sum()<minimum_gain or (best is not None and visible.sum()<=best['gain']):
                    continue
                in_view=True
                mask=timed('occlusion_ms',self.model.visible,ground,heading,targets,
                           ray_dependencies=result['rays'],sensor_pose=sensor_pose,range_margin=range_margin)
                if diagnostics:
                    result['view_counts']['max_visible']=max(result['view_counts']['max_visible'],int(mask.sum()))
                gain=int(mask.sum())
                if gain<minimum_gain or (best is not None and gain<=best['gain']):continue
                check=stop if current else timed('blind_ms',self._arrival_area,stand,tolerance,heading,
                                         sensor_origin=None if sensor_pose is None else sensor_pose[:3])
                if not check['ready']:
                    rejected_area=check
                    result['unknown_blockers'].extend(check.get('unknown_blockers',()))
                    continue
                covered=targets[mask]
                target=(covered[np.argmin(np.linalg.norm(covered-targets.mean(axis=0),axis=1))]
                        if len(covered) else ground)
                best=dict(yaw=float(heading),gain=gain,mask=mask,target=target.tolist(),
                          visible_targets=covered)
            if best is None:
                if rejected_area is not None:reject(rejected_area['reason'],stand,rejected_area)
                else:reject('lidar_occluded' if in_view else 'no_admissible_lidar_view',stand)
                continue
            if motions is None and result['poses'] and any(
                    math.dist(stand[:2],view[3:5])<.28 for view in result['poses']):
                continue
            covered=targets[best['mask']] if best['gain'] else [ground]
            for target in covered[:1] if motions is None else covered:
                result['poses'].append([*map(float,target),*map(float,stand),best['yaw'],int(action)])
            result['views'][tuple(map(float,stand))]=best
            if motions is None and len(result['poses'])>=max_results:break
        result['unknown_blockers']=m.merge_unknown_blockers(result['unknown_blockers'],base)
        return result

    def _headings(self, stand, targets, yaw, ascent, can_turn):
        if not can_turn:return (yaw,)
        bearings=np.arctan2(targets[:,1]-stand[1],targets[:,0]-stand[0])
        ids=np.linspace(0,len(bearings)-1,min(16,len(bearings)),dtype=int)
        angles=[yaw,*bearings[ids]]
        if ascent is not None:
            angles.extend((ascent,ascent-.30,ascent+.30,ascent-.60,ascent+.60))
        return tuple(dict.fromkeys(float(math.remainder(h,2*math.pi)) for h in angles))

    def _stances(self, base, attitude, targets, floor, ascent, preferred, motions):
        m,s=self.map,self.map.settings
        if motions is not None:
            if not motions:return [(base,0,True)]
            stances=[]
            start=self._admission(base,relaxed=True,floor=floor)
            if start.get('ground_z') is None:return stances
            ground=start['ground_z'];yaw=attitude[2]
            if ascent is not None:
                side=np.array([-math.sin(ascent),math.cos(ascent),0.])
                backward=-np.array([math.cos(yaw),math.sin(yaw),0.])
                offsets=[(side*d,1) for d in (.22,-.22,.30,-.30,.40,-.40,.50,-.50,.60,-.60)]
                offsets.extend((backward*d,2) for d in (.22,.25,.30,.35,.40,.50,.60))
            else:
                offsets=[(np.array([math.cos(yaw+i*math.pi/4),math.sin(yaw+i*math.pi/4),0.])*d,
                          2 if i==4 else 1) for d in (.22,.35,.50,.60) for i in range(8)]
            for delta,action in offsets:
                stand=base+delta
                tolerance=min(.16,s.step_max)
                check=self._admission(stand,floor=(ground,tolerance) if ascent is not None else floor)
                if check['ready']:
                    stand[2]=check['ground_z']+self.reference_height
                    stances.append((stand,action,True))
            return stances
        choices={}
        valid_blocks={}
        for target in targets:
            lo,hi=m.indices(target[:2]-2.)//s.block_size,m.indices(target[:2]+2.)//s.block_size
            for layer in range(m.first_layer,m.last_layer+1):
                for bx in range(lo[0],hi[0]+1):
                    for by in range(lo[1],hi[1]+1):
                        key=(layer,bx,by)
                        block=m.blocks.get(key)
                        if block is None:continue
                        if key not in valid_blocks:
                            valid_blocks[key]=np.nonzero(m.traversable_block(key,block,floor))
                        valid=valid_blocks[key]
                        for i,j in zip(*valid):
                            cell=(bx*s.block_size+int(i),by*s.block_size+int(j))
                            if not m.surface_observation.is_ground_source(cell,int(block['g_surface_id'][i,j])):continue
                            ground=(s.origin_x+(bx*s.block_size+i)*s.resolution,
                                    s.origin_y+(by*s.block_size+j)*s.resolution,float(block['G'][i,j]))
                            if math.dist(ground[:2],target[:2])>2.:continue
                            score=math.dist(ground[:2],base[:2])+.2*math.dist(ground[:2],target[:2])
                            if preferred is not None:score+=2.*math.dist(ground,preferred)
                            choices[ground]=min(score,choices.get(ground,math.inf))
        # A nearest-first cap can spend every slot inside the target's blind
        # region. Retain measured stations at several distances and bearings
        # before the common FOV, stopping, occlusion and route checks run.
        center=np.median(targets[:,:2],axis=0)
        buckets={}
        for point in sorted(choices,key=lambda p:(choices[p],p)):
            dx,dy=np.asarray(point[:2])-center
            sector=int(math.floor((math.atan2(dy,dx)+math.pi)/(math.pi/2)))%4
            ring=int(math.hypot(dx,dy)/.4)
            buckets.setdefault((sector,ring),[]).append(point)
        ordered=[]
        for at in range(128):
            batch=[points[at] for points in buckets.values() if len(points)>at]
            if not batch:break
            ordered.extend(sorted(batch,key=lambda p:(choices[p],p)))
            if len(ordered)>=128:break
        return [(np.asarray((x,y,z+self.reference_height)),0,True) for x,y,z in ordered[:128]]

    def _arrival_geometry(self, xyz, xy_tolerance):
        m,s=self.map,self.map.settings
        stop_radius=s.robot_radius+s.safety_margin+xy_tolerance
        observation_radius=max(s.robot_radius+s.safety_margin,s.ground_lidar_min_range)+xy_tolerance
        reach=observation_radius+s.resolution
        xy=np.asarray(xyz[:2])
        lo,hi=m.indices(xy-reach)//s.block_size,m.indices(xy+reach)//s.block_size
        center=math.floor((xyz[2]-self.reference_height-s.origin_z)/s.slice_dh)+1
        bands=math.ceil(2*s.step_max/s.slice_dh)+1
        revision=(center,tuple(
            m.planning_block_versions.get((k,bx,by),0)
            for k in range(center-bands,center+bands+1)
            for bx in range(lo[0],hi[0]+1) for by in range(lo[1],hi[1]+1)
        ) if m.first_layer is not None else ())
        return stop_radius,observation_radius,revision

    def _arrival_area(self, xyz, xy_tolerance, heading=0., sensor_origin=None):
        mapping = self.map
        settings = mapping.settings
        stop_radius, observation_radius, revision = self._arrival_geometry(xyz, xy_tolerance)
        key = (*xyz, xy_tolerance)
        entry = self._cache.get(key)
        if entry is None or entry[0] != revision:
            entry = [revision, self._arrival_stop(xyz, stop_radius), {}, None]
            self._cache[key] = entry
        self._cache.move_to_end(key)
        if len(self._cache) > 1024:
            self._cache.popitem(last=False)
        _, (_, layers, stop_check), headings, coverage = entry
        if heading is None or not stop_check['ready']:
            return stop_check
        heading_key = (heading, sensor_origin)
        if heading_key in headings:
            result, rays = headings[heading_key]
            if all(self.model.ray_evidence(cell) == bands for cell, bands in rays.items()):
                return result
        if coverage is None:
            area = {(x, y) for x, y, _ in mapping.footprint_cells(xyz, observation_radius)}
            coverage = self._unobserved_area(xyz, layers, area)
            entry[3] = coverage
        sensor = (np.asarray(sensor_origin) if sensor_origin is not None else
                  np.asarray(xyz, dtype=float) + [settings.ground_lidar_x * math.cos(heading),
                                                 settings.ground_lidar_x * math.sin(heading),
                                                 settings.ground_lidar_z])
        bands = {cell: self.model.column_blockers(cell) for cell in coverage[1]}
        best = None
        rays = dict(bands)
        for layer, unseen in coverage[0]:
            reference = float(mapping.observed_cell((layer, *mapping.indices(xyz)))['G'])
            # With no surface crossing the open ray's height interval, yaw
            # cannot hide missing ground. Reuse that rejection for every yaw.
            can_occlude = (math.dist(sensor[:2], xyz[:2]) > observation_radius or
                           any(hi > min(reference, sensor[2]) + 1e-7 and
                               lo < max(reference, sensor[2]) - 1e-7
                               for values in bands.values() for lo, hi in values))
            missing = {}
            shadows = 0
            for cell in unseen:
                occupied = cell in mapping.columns and any(
                    lo <= reference + settings.minimum_clearance and
                    hi >= reference + settings.surface_noise_tolerance
                    for lo, hi in bands[cell])
                if occupied:
                    continue
                target = (*mapping.position((0, *cell)), reference)
                if can_occlude and self.model.ray_blocked(sensor, target, rays):
                    shadows += 1
                else:
                    missing[cell] = reference
            if best is None or len(missing) < len(best[1]):
                best = layer, missing, shadows, not can_occlude
            if not missing:
                break
        layer, missing, shadows, independent = best
        result = self._arrival_result(xyz, stop_radius, layer, missing, 0, observation_radius)
        result.update(observation_radius=observation_radius, occluded_cells=shadows,
                      heading_independent=independent)
        if len(headings) < 16:
            headings[heading_key] = result, rays
        return result

    def _unobserved_area(self, xyz, layers, area):
        mapping = self.map
        column_heights = {}
        coverage = []
        for layer, open_cells in layers:
            visited = set()
            pending = list(open_cells)
            while pending:
                x, y, reference = pending.pop()
                if (x, y) in visited:
                    continue
                visited.add((x, y))
                for dx, dy in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                    cell = x + dx, y + dy
                    if cell in visited or cell not in area:
                        continue
                    if cell not in column_heights:
                        column_heights[cell] = tuple(float(row['G']) for cut, row in mapping.layers_at(*cell)
                                                     if row['measured'] and not row['unknown'] and
                                                     mapping.surface_observation.is_ground_source(cell,int(row['g_surface_id'])) and
                                                     mapping.surface_observation.ground_observed(cell,float(row['G']),cut))
                    compatible = [height for height in column_heights[cell]
                                  if abs(height - reference) <= mapping.settings.step_max + 1e-5]
                    if compatible:
                        pending.append((*cell, min(compatible, key=lambda height: abs(height - reference))))
            # This outer disk checks sensor-blind evidence, not whether a
            # robot can walk from the stance to every cell in that disk.
            # A gap or a >step-height boundary cannot turn a separately
            # observed G/C interval back into an unobserved cell. The actual
            # parking footprint and route retain their full safety checks.
            visited.update(cell for cell in area if mapping.frontier_observed(layer,cell))
            coverage.append((layer, area - visited))
        return coverage, area

    def _arrival_stop(self, xyz, radius):
        m,s=self.map,self.map.settings
        check=self._admission(xyz,height_tolerance=.02,collect_blockers=True)
        if not check['ready']:
            return (),(),dict(check,reason='arrival_'+check['reason'])
        stopping={(x,y) for x,y,_ in m.footprint_cells(xyz,radius)}
        layers=[]
        footprints=set()
        best=None
        for layer in check['safe_layers']:
            blockers=[]
            height=float(m.observed_cell((layer,*m.indices(xyz)))['G'])
            missing,known,unsafe={},[],0
            for x,y in stopping:
                row=m.footprint_support(layer,x,y,height)
                if (row is None or row['unknown'] or not row['measured'] or
                        not m.surface_observation.is_ground_source((x,y),int(row['g_surface_id'])) or
                        not m.surface_observation.ground_observed((x,y),float(row['G']),layer)):
                    missing[x,y]=height
                elif row['solid'] or abs(float(row['G'])-height)>s.step_max+1e-5:
                    unsafe+=1
                    blockers.extend(m.unknown_blockers(layer,[(x,y)],height))
                else:
                    known.append((x,y,float(row['G'])))
            score=(unsafe,len(missing))
            if best is None or score<best[0]:
                best=score,layer,missing,blockers
            if score==(0,0) and tuple(known) not in footprints:
                footprints.add(tuple(known))
                layers.append((layer,known))
        (unsafe,_),layer,missing,blockers=best
        result=self._arrival_result(xyz,radius,layer,missing,unsafe)
        result['unknown_blockers']=[] if result['ready'] else m.merge_unknown_blockers(
            result['unknown_blockers']+blockers,xyz)
        return stopping,layers,result

    def _arrival_result(self, xyz, radius, layer, missing, unsafe, blind_radius=0.):
        m=self.map
        blockers=[[*m.position((layer,x,y)),height,layer] for (x,y),height in missing.items()]
        return dict(ready=not (unsafe or missing),layer=int(layer),
                    reason='arrival_unsafe_stop_area' if unsafe else
                           'arrival_unobserved_blind_area' if missing else 'arrival_area_ready',
                    stop_radius=radius,blind_radius=blind_radius,unobserved_cells=len(missing),
                    unsafe_cells=unsafe,unknown_blockers=m.merge_unknown_blockers(blockers,xyz),
                    observation_failures=[m.surface_observation.describe_ground((x,y),height,None if blind_radius else layer)
                        for (x,y),height in sorted(missing.items())[:12]])

