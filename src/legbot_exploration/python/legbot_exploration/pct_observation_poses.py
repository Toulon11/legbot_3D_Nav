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
                          max_results=3, motions=None, headings=None, diagnostics=False):
        """XYZ inputs use body height; targets use surface height. No safety bypass."""
        m,s=self.map,self.map.settings
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
                    layers=layers,unknown_blockers=(check or {}).get('unknown_blockers',[])[:8]))
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
            stop=timed('stop_ms',self._arrival_area,stand,tolerance,None)
            if not stop['ready']:
                reject(stop['reason'],stand,stop)
                result['unknown_blockers'].extend(stop.get('unknown_blockers',()))
                continue
            if not roll and not pitch:
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
                visible=timed('fov_ms',self.model.visible,ground,heading,targets,False,sensor_pose=sensor_pose)
                if diagnostics:
                    result['view_counts']['max_in_field']=max(result['view_counts']['max_in_field'],int(visible.sum()))
                if visible.sum()<minimum_gain or (best is not None and visible.sum()<=best['gain']):
                    continue
                in_view=True
                mask=timed('occlusion_ms',self.model.visible,ground,heading,targets,
                           ray_dependencies=result['rays'],sensor_pose=sensor_pose)
                if diagnostics:
                    result['view_counts']['max_visible']=max(result['view_counts']['max_visible'],int(mask.sum()))
                gain=int(mask.sum())
                if gain<minimum_gain or (best is not None and gain<=best['gain']):continue
                check=timed('blind_ms',self._arrival_area,stand,tolerance,heading,
                                         sensor_origin=None if sensor_pose is None else sensor_pose[:3])
                if not check['ready']:
                    rejected_area=check
                    result['unknown_blockers'].extend(check.get('unknown_blockers',()))
                    continue
                best=dict(yaw=float(heading),gain=gain,mask=mask)
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
                tolerance=min(.16,s.step_max) if action==2 else .04
                check=self._admission(stand,floor=(ground,tolerance) if ascent is not None else floor)
                if check['ready']:
                    stand[2]=check['ground_z']+self.reference_height
                    stances.append((stand,action,True))
            return stances
        choices={}
        for target in targets:
            lo,hi=m.indices(target[:2]-2.)//s.block_size,m.indices(target[:2]+2.)//s.block_size
            for layer in range(m.first_layer,m.last_layer+1):
                for bx in range(lo[0],hi[0]+1):
                    for by in range(lo[1],hi[1]+1):
                        block=m.blocks.get((layer,bx,by))
                        if block is None:continue
                        valid=block['measured'] & ~block['unknown'] & ~block['solid'] & ~block['hard'] & \
                              (block['inflated_terrain_cost']>=0) & (block['inflated_terrain_cost']<=20)
                        if floor is not None:valid &= abs(block['G']-floor[0])<=floor[1]
                        for i,j in zip(*np.nonzero(valid)):
                            ground=(s.origin_x+(bx*s.block_size+i)*s.resolution,
                                    s.origin_y+(by*s.block_size+j)*s.resolution,float(block['G'][i,j]))
                            if math.dist(ground[:2],target[:2])>2.:continue
                            score=math.dist(ground[:2],base[:2])+.2*math.dist(ground[:2],target[:2])
                            if preferred is not None:score+=2.*math.dist(ground,preferred)
                            choices[ground]=min(score,choices.get(ground,math.inf))
        return [(np.asarray((x,y,z+self.reference_height)),0,True)
                for x,y,z in sorted(choices,key=lambda p:(choices[p],p))[:128]]

    def _arrival_geometry(self, xyz, xy_tolerance):
        m,s=self.map,self.map.settings
        stop_radius=s.robot_radius+s.safety_margin+xy_tolerance
        observation_radius=max(s.robot_radius+s.safety_margin,s.ground_lidar_min_range)+xy_tolerance
        reach=observation_radius+s.resolution
        xy=np.asarray(xyz[:2])
        lo,hi=m.indices(xy-reach)//s.block_size,m.indices(xy+reach)//s.block_size
        revision=(m.first_layer,m.last_layer,tuple(
            m.planning_block_versions.get((k,bx,by),0)
            for k in range(m.first_layer,m.last_layer+1)
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
            if all(self.model.column_blockers(cell) == bands for cell, bands in rays.items()):
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
            reference = float(mapping.cell((layer, *mapping.indices(xyz)))['G'])
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
                        column_heights[cell] = tuple(float(row['G']) for _, row in mapping.layers_at(*cell)
                                                     if row['measured'] and not row['unknown'])
                    compatible = [height for height in column_heights[cell]
                                  if abs(height - reference) <= mapping.settings.step_max + 1e-5]
                    if compatible:
                        pending.append((*cell, min(compatible, key=lambda height: abs(height - reference))))
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
            height=float(m.cell((layer,*m.indices(xyz)))['G'])
            missing,known,unsafe={},[],0
            for x,y in stopping:
                row=m.cell((layer,x,y))
                if row is None or row['unknown'] or not row['measured']:
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
                    unsafe_cells=unsafe,unknown_blockers=m.merge_unknown_blockers(blockers,xyz))

