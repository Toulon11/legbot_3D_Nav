"""Fixed-origin online map adapter for the project's native PCT A*.

Registered frames are fused once into sparse height evidence. Planning holds
its owner's read lock; raw frames and full planning-map snapshots are not kept.
"""
import ctypes
from dataclasses import replace
from functools import cache
import importlib.util
import math
import os
from pathlib import Path
from statistics import median
import sys
import time
from .pct_frontier_index import DIRECTIONS
from .pct_frontiers import extract_frontiers
from .pct_stairs import tread_rise_bounds, level_platform, level_platform_cells
from .fixed_layer_map import FixedLayerMap, MapSettings

import numpy as np


PLATFORM_HEIGHT_TOLERANCE = .05
STAIR_PROBE_RADIUS = 3.4


def _load(name, path, package=False):
    spec = importlib.util.spec_from_file_location(
        name, path, submodule_search_locations=[str(path.parent)] if package else None)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class OnlinePct:
    def __init__(self, pct_root, reference_height=0.32, max_cells=20000000):
        if not np.isfinite(reference_height) or reference_height <= 0 or max_cells < 1:
            raise ValueError('invalid PCT adapter configuration')
        root = Path(pct_root)
        # Installed PCT libraries depend on bundled METIS/GTSAM. Preloading is
        # local to this process; no global environment or system files change.
        for parent in root.parents:
            runtime=parent / '.pct_runtime'
            if runtime.is_dir():
                os.environ.setdefault('CUPY_CACHE_DIR',str(runtime / 'kernel_cache'))
                # CuPy 13 does not discover pip NVRTC automatically. Preload
                # the isolated wheels by SONAME before compiling original kernels.
                for folder,name in (('cuda_runtime','libcudart.so.12'),
                                    ('cuda_nvrtc','libnvrtc-builtins.so.12.9'),
                                    ('cuda_nvrtc','libnvrtc.so.12')):
                    library=runtime/'nvidia'/folder/'lib'/name
                    if library.exists():
                        ctypes.CDLL(str(library),mode=ctypes.RTLD_GLOBAL)
            lib = parent / 'third_party/pct/install/lib/x86_64-linux-gnu'
            if lib.is_dir():
                for name in ('libmetis.so.5', 'libgtsam.so.4'):
                    if (lib / name).exists():
                        ctypes.CDLL(str(lib / name), mode=ctypes.RTLD_GLOBAL)
                break
        sys.path[:0] = [str(root / 'tomography/scripts'), str(root / 'planner/scripts'),
                        str(root / 'planner')]
        wrapper = _load('_exploration_pct_wrapper', root / 'planner/scripts/planner_wrapper.py')
        self.wrapper_type = wrapper.TomogramPlanner
        self.reference_height = reference_height
        self.map = FixedLayerMap(MapSettings(max_cells=max_cells,observer_reference_height=reference_height))
        self.version = None
        self.geometry_revision = 0
        self.planner = None
        self.build_ms = 0.0
        self.metadata = {}
        self._saved_stairs = {}
        self._pending_stairs = {}
        self._stair_recognitions = {}
        self._local_stair_task=None
        self._passive_stair_scout=False
        self._passive_stair_records={}
        self._passive_stair_floor=None

    def configure(self, settings):
        if self.map.accepted_frames:
            raise ValueError('pct_configuration_after_integration')
        self.map=FixedLayerMap(MapSettings(**dict(vars(self.map.settings),**settings)))

    def integrate(self, buffer, sensor_origin=None):
        count=self.map.integrate(np.frombuffer(buffer,dtype=np.float32).reshape(-1,3),sensor_origin=sensor_origin)
        if self.map.error:
            raise ValueError(self.map.error)
        return count

    def _make_planner(self):
        from .block_pct import BlockPlanner
        return BlockPlanner(self.map,self.wrapper_type)

    def commit_pending(self, buffer, version, _unused_start=None):
        """Called by the map owner while holding its exclusive commit lock."""
        self.prepare(buffer, version, allow_rebuild=True)

    def prepare(self, buffer, version, _unused_start=None, *, allow_rebuild=False):
        if isinstance(self.map,FixedLayerMap) and not allow_rebuild:
            if self.map._rebuild_pending:
                raise RuntimeError('pct_map_commit_failed')
            if self.map.floor_ground_update_pending:
                raise RuntimeError('pct_floor_geometry_pending')
        # Static surfaces update only in response to incoming point evidence.
        if (self.version==version and self.planner is not None and not self.map.dirty and
                not self.map.evidence_dirty and not getattr(self.map,'floor_ground_update_pending',False)):
            return
        started=time.monotonic()
        # C++ integrates each accepted frame exactly once; buffer here is empty.
        changed=self.map.build()
        if self.planner is None:
            self.planner=self._make_planner()
        self.geometry_revision=self.map.revision
        self.version=version
        self.build_ms=1000*(time.monotonic()-started)
        self.metadata=dict(self.map.stats,**self.map.counts,**self.map.last_work,version=version,build_ms=self.build_ms,
            accepted_frames=self.map.accepted_frames,accepted_points=self.map.accepted_points,
            backend='project_pct_fixed_online',map_error=self.map.error,
            geometry_version=self.map.revision,evidence_version=self.map.evidence_revision,
            frontier_index=dict(self.map.frontier_index.last_work,
                frontier_layers=len(self.map.frontier_index.nodes),clusters=len(self.map.frontier_index.clusters)),
            geometry_pending=bool(self.map.dirty),
            ignored_static_points=self.map.ignored_static_points,
            sensor_age=None if self.map.last_sensor_time is None else max(0.,self.map.clock()-self.map.last_sensor_time),
            integration_age=None if self.map.last_integration_time is None else max(0.,self.map.clock()-self.map.last_integration_time),
            update_mode='incremental_blocks' if changed else 'unchanged_geometry')

    def _admission(self, xyz, **constraints):
        return self.map.point_safe_check(xyz,self.reference_height,**constraints)

    def _admit(self, xyz):
        admission=self._admission(xyz)
        return admission['ready'],admission['layer']

    def _measured_ground(self, base):
        ground=np.asarray(base,dtype=float)-[0.,0.,self.reference_height]
        check=self._ground_support(base)
        if check['ready'] and check.get('ground_z') is not None:
            ground[2]=check['ground_z']
        return ground

    def _ground_support(self, base, floor=None):
        check=self._admission(base,floor=floor,relaxed=True)
        if check['ready']:
            row=self.map.cell((check['layer'],*self.map.indices(base)))
            if row is None or not row['measured'] or row['unknown']:
                return dict(check,ready=False,reason='tread_ground_unobserved')
            identity=check.get('surface_id',0)
            if not self.map.surface_observation.is_ground_source(tuple(self.map.indices(base)),identity):
                return dict(check,ready=False,reason='tread_ground_unobserved')
        return check

    def _tread_layers(self, x, y, height):
        """A historical return alone is not a currently usable tread."""
        m,s=self.map,self.map.settings
        return tuple(k for k,h,_ in m.support_samples(x,y)
                     if s.origin_z+k*s.slice_dh>height and
                     abs(h-height)<=s.surface_noise_tolerance+1e-7)

    def _tread_margin(self, point, yaw, tolerance=.15):
        """Rank interior stances including early arrival anywhere in the XY tolerance."""
        m,s=self.map,self.map.settings
        direction=np.array([math.cos(yaw),math.sin(yaw)])
        margins=[]
        for sign in (-1,1):
            distance=0.
            for step in range(1,9):
                cell=m.indices(np.asarray(point[:2])+sign*step*s.resolution*direction)
                if not self._tread_layers(*cell,point[2]):break
                distance=step*s.resolution
            margins.append(max(0.,distance+.5*s.resolution-tolerance))
        return min(margins)

    def start_ready(self, buffer, version, xyz, floor=None):
        self.prepare(buffer, version)
        return self._admission(xyz,floor=floor)['ready']

    def recovery_evidence(self, buffer, version, base, forward=None):
        self.prepare(buffer, version)
        m,s=self.map,self.map.settings
        if forward is None:
            return hash((m.revision,m.surface_observation.revision)) & ((1<<52)-1)
        radius=s.robot_radius+s.safety_margin
        lo,hi=m.indices(np.minimum(base,forward)-radius),m.indices(np.maximum(base,forward)+radius)
        first=math.floor((self._measured_ground(base)[2]-s.origin_z)/s.slice_dh)
        bounds=lo//s.block_size,hi//s.block_size
        revisions=tuple(m.planning_block_versions.get((k,x,y),0)
            for k in range(first,first+3)
            for x in range(bounds[0][0],bounds[1][0]+1)
            for y in range(bounds[0][1],bounds[1][1]+1))
        observations=tuple(m.surface_observation.block_versions.get((x,y),0)
            for x in range(bounds[0][0],bounds[1][0]+1) for y in range(bounds[0][1],bounds[1][1]+1))
        return hash((tuple(bounds[0]),tuple(bounds[1]),first,revisions,observations)) & ((1<<52)-1)

    def frontiers(self, buffer, version, start=None, minimum_distance=0., window_size=None,
                  query_budget=0., continuation=None, floor=None, geometry_only=False, platform_floor=None,
                  floor_selection=None, stair_heading=None, seek_ascent=False, review_remaining=None):
        self.prepare(buffer, version)
        if isinstance(self.map,FixedLayerMap):
            ground=None if start is None else self._measured_ground(start)
            start_check = {}
            if ground is not None and (window_size is not None or continuation is not None or floor is not None):
                start_check=self._admission(start,floor=platform_floor,
                                            collect_blockers=platform_floor is None and
                                                (not geometry_only or continuation is not None))
            managed_cross=continuation is not None and floor_selection is not None and len(floor_selection)>4
            selection=(self.observation_poses,self._arrival_geometry,*floor_selection[:4]
                       ) if floor_selection is not None and not geometry_only else None
            policy={}
            if managed_cross and platform_floor is None:
                attempts,max_step,min_rise=floor_selection[4:]
                policy=dict(attempt_scope=('cross_layer',floor),max_attempts=attempts,
                            ascent_limits=(max_step,min_rise) if platform_floor is None else None)
            elif floor_selection is not None and len(floor_selection)>4:
                policy=dict(max_attempts=floor_selection[4])
            if seek_ascent:
                attempts,max_step,min_rise=floor_selection[4:]
                min_rise=max(min_rise,self.map.settings.surface_noise_tolerance+1e-7)
                return dict(self.map.frontier_index.query(ground,minimum_distance,window_size,query_budget,
                    floor_selection=selection,attempt_scope=('floor_ascent',floor),max_attempts=attempts,
                    ascent_limits=(max_step,min_rise)),version=version,map=self.metadata)
            if platform_floor is not None:
                result=self.map.frontier_index.query(ground,minimum_distance,window_size,query_budget,
                    floor=platform_floor,floor_selection=selection,
                    prune_disconnected=start_check.get('ready',False),fast=self.map.settings.fast_exploration,
                    max_attempts=2,ordinary_floor=False)
                return dict(result,version=version,map=self.metadata)
            stair_started=time.monotonic()
            bounded=query_budget>0
            focus=(self.map.indices(ground),stair_heading) if ground is not None and bounded else None
            passive=floor is not None and continuation is None and not geometry_only
            stair_index=self.map.stair_index
            if floor is None:
                raw_flights=()
            elif passive and bounded:
                self.map.stair_candidates.set_floor(floor)
                raw_flights=self.map.stair_candidates.confirmed.values()
            else:
                raw_flights=stair_index.query(self.map,*floor,deadline=stair_started+query_budget*.5 if bounded else None,
                    tile_limit=4 if bounded else None,focus=focus)
            stair_index_ms=1000*(time.monotonic()-stair_started)
            qualification_started=time.monotonic()
            flights=(raw_flights if passive and bounded else
                     [qualified for flight in raw_flights
                      if (qualified:=self._qualified_stair(flight,floor)) is not None])
            confirmed=[flight for flight in flights if flight.confirmed]
            qualification_ms=1000*(time.monotonic()-qualification_started)
            fast=self.map.settings.fast_exploration and floor is not None and continuation is None and not geometry_only
            result=self.map.frontier_index.query(ground,minimum_distance,window_size,query_budget,
                continuation,floor if continuation is None else None,flights,
                evaluate_observers=not geometry_only and (continuation is None or floor is None or managed_cross),fast=fast,
                prune_disconnected=start_check.get('ready',False) and (floor is None or
                    abs(ground[2]-floor[0])<=floor[1]),
                floor_selection=selection,review_remaining=review_remaining,**policy)
            record_ms=0.
            if passive:
                on_departure=start_check.get('ready',False) and abs(ground[2]-floor[0])<=floor[1]
                if bounded:
                    work=self._maintain_stair_candidates(floor,ground,stair_heading,on_departure,
                                                        min(.020,query_budget*.25))
                    stair_index_ms+=work['stair_index_ms']
                    qualification_ms+=work['stair_qualification_ms']
                    record_ms=work['stair_candidate_record_ms']
                    result.setdefault('observer_phase_ms',{}).update(work)
                else:
                    began=time.monotonic()
                    self._record_stair_candidates(floor,flights,ground,stair_heading,on_departure)
                    record_ms=1000*(time.monotonic()-began)
            began=time.monotonic()
            saved=(list(self._saved_stairs.get(floor,())) if passive and bounded else
                   [flight for flight in self._saved_stairs.get(floor,())
                    if self._upward_stair_check(flight,floor)['ready']])
            qualification_ms+=1000*(time.monotonic()-began)
            result.setdefault('observer_phase_ms',{}).update(stair_index_ms=stair_index_ms,
                stair_qualification_ms=qualification_ms,stair_candidate_record_ms=record_ms)
            result['stair_targets']=[]
            result['stair_flights']=len(confirmed)
            result['stair_index_ms']=stair_index_ms
            result['pending_stair_candidates']=len(self._pending_stairs.get(floor,{}))
            result['saved_stair_entrances']=[
                [*self._stair_xyz(f.entry), math.atan2(f.direction[1], f.direction[0])]
                for f in sorted(saved,key=lambda f:self._stair_priority(f,ground))]
            result['unknown_blockers']=start_check.get('unknown_blockers', [])
            if ground is not None and floor is not None and (
                    geometry_only or continuation is not None or
                    (window_size is None and not result['scoped_frontier_cells'] and
                     not result['query_pending'] and not result['pending_review_clusters'])):
                if continuation is not None:
                    qualified=[flight for flight in confirmed if self._upward_stair_check(flight,floor)['ready']]
                    result['stair_targets']=self._stair_targets(qualified,ground,continuation=continuation)
                    result['stair_landing_targets']=self._stair_landing_targets(ground,floor,continuation)
                    result['stair_platform_visible']=self._stair_platform_visible(ground,floor,continuation[0])
            if ground is not None and continuation is not None:
                steps, unsafe, blockers = self._stair_step_targets(ground, minimum_distance, continuation)
                result['stair_step_targets'], result['stair_step_unsafe'] = steps, unsafe
                result['unknown_blockers'] = self.map.merge_unknown_blockers(
                    start_check.get('unknown_blockers', []) + blockers, ground)
            return dict(result,version=version,map=self.metadata)
        # Cache sparse target data with the native map version. No extra dense
        # map snapshot or alternate terrain planner is constructed.
        revision=getattr(self, 'geometry_revision', 0)
        if getattr(self, '_frontier_revision', None) != revision:
            started=time.monotonic()
            self._frontiers=extract_frontiers(self.planner,self.map.settings.step_max,self.map.settings.minimum_clearance)
            self._frontiers['extraction_ms']=1000*(time.monotonic()-started)
            self._frontier_revision = revision
        return dict(self._frontiers, version=version, map=self.metadata)

    def _stair_xyz(self, cell):
        x,y,z=cell
        s=self.map.settings
        return (s.origin_x+x*s.resolution,s.origin_y+y*s.resolution,z)

    def _same_stair_entrance(self, first, second):
        return (first.direction==second.direction and
                math.dist(first.entry[:2],second.entry[:2])*self.map.settings.resolution<=.8)

    def _stair_priority(self, flight, ground):
        distance=math.dist(self._stair_xyz(flight.entry)[:2],ground[:2]) if ground is not None else 0.
        return -len(flight.usable_treads),-len(flight.treads),distance,flight.entry,flight.direction

    def _qualified_stair(self, flight, floor):
        check=self._upward_stair_check(flight,floor)
        if not check['ready'] and not check.get('reviewable'):return None
        count=check.get('usable_count',len(flight.treads))
        return replace(flight,usable_count=None if count==len(flight.treads) else count)

    def _maintain_stair_candidates(self, floor, ground, yaw, on_floor, budget=.020):
        m=self.map;cache=m.stair_candidates
        cache.set_floor(floor)
        if self._passive_stair_floor!=floor:
            self._passive_stair_records={}
            self._passive_stair_floor=floor
        began=time.monotonic();deadline=began+budget
        work=dict(stair_index_ms=0.,stair_qualification_ms=0.,stair_candidate_record_ms=0.)
        pending=self._pending_stairs.setdefault(floor,{})

        def publish(group,flight):
            started=time.monotonic()
            old=self._passive_stair_records.pop(group,None)
            approach=pending.pop(old,None) if old is not None else None
            if flight is not None:
                if approach is None and on_floor and math.dist(self._stair_xyz(flight.treads[0])[:2],ground[:2])<=STAIR_PROBE_RADIUS:
                    approach=(*map(float,ground),float(yaw or 0.))
                pending[flight]=approach
                self._passive_stair_records[group]=flight
            work['stair_candidate_record_ms']+=1000*(time.monotonic()-started)

        def qualify(flight,scope):
            started=time.monotonic()
            result=self._qualified_stair(flight,scope)
            work['stair_qualification_ms']+=1000*(time.monotonic()-started)
            return result

        while cache.changes and time.monotonic()<deadline:
            group,flight=cache.changes.popitem(last=False)
            publish(group,flight)
        # Start with qualification work already queued by the last map commit.
        checked,published=cache.process(qualify,ground,began+budget*.5,publish)
        if time.monotonic()<deadline:
            worker=m.stair_scout_index if self._passive_stair_scout else m.stair_index
            self._passive_stair_scout=not self._passive_stair_scout
            started=time.monotonic()
            worker.advance(m,*floor,deadline=deadline,tile_limit=1,focus=(m.indices(ground),yaw))
            work['stair_index_ms']=1000*(time.monotonic()-started)
        more_checked,more_published=cache.process(qualify,ground,deadline,publish)
        work.update(stair_candidates_checked=checked+more_checked,
                    stair_candidates_updated=published+more_published,
                    stair_candidates_pending=len(cache.pending),
                    stair_maintenance_ms=1000*(time.monotonic()-began))
        return work

    def _rank_stair_entrances(self, flights, ground):
        representatives=[]
        buckets={}
        ranks={}
        width=max(1,math.ceil(.8/self.map.settings.resolution))
        for flight in sorted(dict.fromkeys(flights),key=lambda f:self._stair_priority(f,ground)):
            x,y=flight.entry[:2]
            bx,by=x//width,y//width
            rank=min((i for dx in (-1,0,1) for dy in (-1,0,1)
                      for i in buckets.get((flight.direction,bx+dx,by+dy),())
                      if self._same_stair_entrance(flight,representatives[i])),default=None)
            if rank is None:
                rank=len(representatives)
                representatives.append(flight)
                buckets.setdefault((flight.direction,bx,by),[]).append(rank)
            ranks[flight]=rank
        return ranks

    def _remember_stairs(self, task, ground, yaw):
        floor=task['floor']
        flight=self._qualified_stair(task['flight'],floor)
        if flight is None or not flight.confirmed:return False
        approach=self._stair_approach(task,task['home'],current=(*ground,yaw),search=True)
        if approach is None or not self._upward_stair_check(flight,floor)['ready']:
            return False
        saved=self._saved_stairs.setdefault(floor,{})
        match=next((f for f in saved if self._same_probe_entrance(f,flight)),None)
        previous=self._qualified_stair(match,floor) if match is not None else None
        if previous is not None and len(previous.usable_treads)>len(flight.usable_treads):
            flight=previous
        if match is not None and match!=flight:
            del saved[match]
        saved[flight]=approach
        return True

    def _entrance_groups(self, flights, ground):
        groups={}
        for flight,rank in self._rank_stair_entrances(flights,ground).items():
            group=groups.setdefault(rank,[])
            if len(group)<3:
                group.append(flight)
        return list(groups.values())

    def _same_probe_entrance(self, first, second):
        a,b=first.treads[0],second.treads[0]
        dx,dy=first.direction
        ux,uy=second.direction
        return (math.dist(a[:2],b[:2])*self.map.settings.resolution<=1.0 and
                abs(a[2]-b[2])<=self.map.settings.surface_noise_tolerance and
                (dx*ux+dy*uy)/(math.hypot(dx,dy)*math.hypot(ux,uy))>=.70)

    def _record_stair_candidates(self, floor, flights, ground, yaw, start_ready):
        """Keep geometric hints and observed approach poses; never plan or scan."""
        pending=self._pending_stairs.setdefault(floor,{})
        ranked=self._rank_stair_entrances(
            [qualified for f in flights if (qualified:=self._qualified_stair(f,floor)) is not None],ground)
        seen=set()
        for flight,group in ranked.items():
            # The ranker orders longer observed flights first. One physical
            # entrance is sufficient; the normal map indices retain the rest.
            if group in seen:continue
            seen.add(group)
            match=flight if flight in pending else next((f for f in pending if self._same_probe_entrance(f,flight)),None)
            approach=pending.get(match)
            previous=(self._qualified_stair(match,floor) if match is not None and match!=flight else None)
            if previous is not None and len(previous.usable_treads)>len(flight.usable_treads):
                flight=previous
            if match is not None and match!=flight:
                del pending[match]
            if (approach is None and start_ready and ground is not None and
                    math.dist(self._stair_xyz(flight.treads[0])[:2],ground[:2])<=STAIR_PROBE_RADIUS):
                approach=(*map(float,ground),float(yaw or 0.))
            pending[flight]=approach

    def _stair_candidates(self, floor, ground, yaw, incremental=False):
        m=self.map
        if incremental:
            on_floor=abs(ground[2]-floor[0])<=floor[1]
            self._maintain_stair_candidates(floor,ground,yaw,on_floor)
            return tuple(dict.fromkeys((*m.stair_candidates.candidates.values(),
                         *self._pending_stairs.get(floor,()),*self._saved_stairs.get(floor,()))))
        focus=(m.indices(ground),yaw)
        flights=m.stair_index.query(m,*floor,deadline=time.monotonic()+.10,tile_limit=4,focus=focus)
        hints=m.stair_scout_index.query(m,*floor,deadline=time.monotonic()+.10,tile_limit=4,focus=focus)
        for cache in (self._pending_stairs.get(floor,{}),self._saved_stairs.get(floor,{})):
            for old,approach in list(cache.items()):
                qualified=self._qualified_stair(old,floor)
                if qualified is None or qualified!=old:
                    del cache[old]
                if qualified is not None:
                    cache[qualified]=approach
        return tuple(dict.fromkeys(qualified for f in (*flights,*hints,*self._pending_stairs.get(floor,()),
                                *self._saved_stairs.get(floor,()))
                     if (qualified:=self._qualified_stair(f,floor)) is not None))

    def _upward_stair_check(self, flight, floor, origin=None, approach=None):
        """One qualification rule for fresh, cached and retried stair flights."""
        m,s=self.map,self.map.settings
        noise=s.surface_noise_tolerance;maximum=min(.16,s.step_max)
        detail=dict(floor_height=float(floor[0]),entry=list(map(float,self._stair_xyz(flight.entry))),
                    direction=list(map(int,flight.direction)),
                    treads=[list(map(float,self._stair_xyz(t))) for t in flight.treads])
        reason=None
        usable=0
        tail_reason=None
        if abs(flight.entry[2]-floor[0])>min(floor[1],noise)+1e-7:
            reason='stair_entry_outside_floor'
        elif not flight.treads or not noise-1e-7<=flight.treads[0][2]-floor[0]<=maximum+1e-7:
            reason='stair_first_tread_not_above_floor'
        elif flight.treads[0][2]-flight.entry[2]<=noise-1e-7:
            reason='stair_first_tread_within_entry_noise'
        elif not 1e-7<flight.treads[0][2]-flight.entry[2]<=maximum+1e-7:
            reason='stair_nonascending_or_unclimbable'
        if reason is None:
            if not any(row['measured'] and not row['unknown'] and
                       abs(float(row['G'])-flight.entry[2])<=noise+1e-7
                       and abs(float(row['G'])-floor[0])<=min(floor[1],noise)+1e-7
                       for _,row in m.layers_at(*flight.entry[:2])):
                reason='stair_entry_unobserved'
        if reason is None:
            tread_tolerance=min(.035,noise)+1e-7
            for x,y,height in flight.treads:
                previous=flight.entry[2] if usable==0 else flight.treads[usable-1][2]
                if not noise-1e-7<=height-previous<=maximum+1e-7:
                    tail_reason='stair_nonascending_or_unclimbable'
                    break
                column=m.columns.get((x,y))
                if not self._tread_layers(x,y,height):
                    observed=column is not None and any(
                        record.orientation in (0,1) and abs(record.height-height)<=tread_tolerance
                        for record in column.values())
                    tail_reason='stair_tread_no_traversable_slice' if observed else 'stair_tread_unobserved'
                    break
                usable+=1
        if reason is None and origin is not None:
            connectivity=m.floor_connectivity(floor)
            component=connectivity.component(m.indices(origin))
            entry=m.indices(approach) if approach is not None else flight.entry[:2]
            if component is None:reason='stair_entry_connectivity_unavailable'
            elif connectivity.component(entry)!=component:
                reason='stair_entry_disconnected'
        if approach is not None:
            detail['approach']=list(approach)
        detail.update(usable_count=usable,usable_treads=detail['treads'][:usable],
                      pending_treads=detail['treads'][usable:],tail_reason=tail_reason)
        return dict(detail,ready=reason is None and usable>0,
                    reviewable=reason is None and tail_reason is not None,
                    reason=reason or (tail_reason if not usable else 'upward_stair_candidate'))

    def _stair_approach_safe(self, stance, floor, tolerance):
        band=min(floor[1],self.map.settings.surface_noise_tolerance)
        body=(*stance[:2],stance[2]+self.reference_height)
        return (abs(stance[2]-floor[0])<=band+1e-7 and
                self._admission(body,floor=(floor[0],band),height_tolerance=.02)['ready'] and
                self._arrival_area(body,tolerance,stance[3])['ready'])

    def _stair_approach(self, task, origin, *, current=None, search=False):
        flight,floor=task['flight'],task['floor']
        yaw=math.atan2(flight.direction[1],flight.direction[0])
        tolerance=task['tolerance']
        first=self._stair_xyz(flight.treads[0])
        dx,dy=flight.direction
        connectivity=self.map.floor_connectivity(floor)
        component=connectivity.component(self.map.indices(origin))
        if component is None:
            return None

        def usable(stance):
            return (stance is not None and math.dist(stance[:2],first[:2])<=STAIR_PROBE_RADIUS and
                    (stance[0]-first[0])*dx+(stance[1]-first[1])*dy<=1e-7 and
                    not any(math.dist(stance[:2],p[:2])<=tolerance
                            for p in task.get('rejected_approaches',())) and
                    connectivity.component(self.map.indices(stance))==component and
                    self._stair_approach_safe(stance,floor,tolerance))

        for stance in (task.get('observed_approach'),task.get('saved_approach'),current,
                       (*task['home'],yaw)):
            if usable(stance):
                return stance
        if search:
            start=(*origin[:2],origin[2]+self.reference_height)
            goals=[self._stair_xyz(t) for t in flight.treads]
            band=min(floor[1],self.map.settings.surface_noise_tolerance)
            views=self.observation_poses(start,(0.,0.,yaw),goals,floor=(floor[0],band),
                ascent=yaw,preferred=first,tolerance=tolerance,max_results=8)
            for _,_,_,x,y,z,heading,_ in views['poses']:
                stance=(x,y,z-self.reference_height,heading)
                if usable(stance) and self.plan_surface(b'',self.version,start,(x,y,z),floor)[0]=='ok':
                    return stance
        return None

    def _stair_task_check(self, task):
        flight,floor,home=task['flight'],task['floor'],task['home']
        approach=self._stair_approach(task,home)
        result=self._upward_stair_check(flight,floor,home,approach)
        if approach is None and result['reason']=='stair_entry_disconnected':
            approach=self._stair_approach(task,home,search=True)
            if approach is not None:
                task['saved_approach']=approach
                return self._upward_stair_check(flight,floor,home,approach)
        return result

    def _stair_task_geometry(self, task):
        points=(task.get('failure_ground',task['home']),self._stair_xyz(task['flight'].treads[-1]))
        return tuple(self._arrival_geometry((p[0],p[1],p[2]+self.reference_height),task['tolerance'])[2]
                     for p in points)

    def _finish_stair_attempt(self, task, reason, ground=None):
        if not task.get('attempt_settled'):
            task['failures']=task.get('failures',0)+1
        if ground is not None:
            task['failure_ground']=tuple(ground)
        task.update(finished=True,attempt_settled=True,reason=reason,
                    checked_geometry=self._stair_task_geometry(task))
        task['failed_levels']=tuple(t[2] for t in task['flight'].treads)

    def _stair_probe_result(self, task, **result):
        diagnostic=dict(task.get('diagnostic',{}),failures=task.get('failures',0),
                        scans=task['scans'],confirmed=task.get('confirmed',False),
                        attempt_status=('exhausted' if task.get('failures',0)>=2 else
                                        'deferred' if task.get('finished') and task.get('failures') else 'active'),
                        reopen_reason=task.get('reopen_reason'))
        return dict(result,diagnostic=diagnostic)

    def _refresh_stair_task(self, task, ground, yaw, candidates=None):
        if candidates is None:
            candidates=self._stair_candidates(task['floor'],ground,yaw)
        compatible=[f for f in candidates if self._same_probe_entrance(f,task['flight'])]
        previous=self._qualified_stair(task['flight'],task['floor'])
        if previous is not None:
            compatible.insert(0,previous)
        if compatible:
            task['flight']=max(compatible,key=lambda f:(len(f.usable_treads),len(f.treads)))

    def floor_stair_probe(self, buffer, version, base, floor, yaw, depart=False, review=False,
                          tolerance=.15):
        """Advance the same stair-recognition task during exploration and departure."""
        self.prepare(buffer,version)
        m,s=self.map,self.map.settings
        ground=self._measured_ground(base)
        candidates=self._stair_candidates(floor,ground,yaw,incremental=True)
        saved=self._saved_stairs.get(floor,{})
        pending=self._pending_stairs.get(floor,{})
        task=self._local_stair_task
        indexing=(m.stair_candidates.floor==floor and (m.stair_candidates.needs_work() or
                  m.stair_index.pending or m.stair_scout_index.pending or
                  m.stair_index.floor!=floor or m.stair_scout_index.floor!=floor))
        if task is None and indexing and not any(f.usable_treads for f in candidates):
            return dict(ready=False,active=True,reason='stair_index_pending')
        if task is not None and task['floor']!=floor:
            self._local_stair_task=None
            task=None
        if task is not None:
            if not task.get('approach_pending'):
                self._refresh_stair_task(task,ground,yaw,candidates)
            # Observation-side movements and returning home are not progress.
            origin=task.get('progress_ground',task['home'])
            stance=task.get('last_stance')
            if (stance is not None and stance[2]>floor[0]+s.surface_noise_tolerance and
                    ground[2]>origin[2]+s.surface_noise_tolerance and
                    math.dist(ground[:2],stance[:2])<=.25 and abs(ground[2]-stance[2])<=.05):
                task.update(scans=0,progress_ground=tuple(ground))
            if (not task.get('approach_pending') and task['flight'].confirmed and task.get('verified') and
                    self._stair_task_check(task)['ready']):
                if not depart:
                    if self._remember_stairs(task,ground,yaw):
                        task.update(confirmed=True,finished=True,reason='stair_confirmed')
                    else:
                        self._finish_stair_attempt(task,'stair_no_safe_approach',ground)
                elif task['reason']=='stair_confirmed':
                    task['finished']=False
            if task['finished']:
                home=task['home']
                band=min(floor[1],s.surface_noise_tolerance)
                on_floor=(abs(ground[2]-floor[0])<=band+1e-7 and
                          self._admission(base,floor=(floor[0],band))['ready'])
                if on_floor:
                    self._local_stair_task=None
                    return self._stair_probe_result(task,ready=False,finished=True,reason=task['reason'])
                return self._stair_probe_result(task,ready=True,active=True,returning=True,
                    observations=[list(home)],yaws=[yaw],reason='local_stair_probe_return')
            result=self._local_stair_observers(task,ground,yaw,depart)
            if result.get('climbing') and not depart:
                return self.floor_stair_probe(buffer,version,base,floor,yaw,depart,review,tolerance)
            if result.get('ready'):return self._stair_probe_result(task,**result)
            self._finish_stair_attempt(task,result['reason'],ground)
            return self.floor_stair_probe(buffer,version,base,floor,yaw,depart,review,tolerance)
        records=self._stair_recognitions.setdefault(floor,[])
        nearby=[f for f in candidates if (depart and (f in pending or f in saved)) or
                math.dist(self._stair_xyz(f.treads[0])[:2],ground[:2])<=STAIR_PROBE_RADIUS]
        def previous_attempt(flight):
            return next((task for task in records if self._same_probe_entrance(flight,task['flight'])),None)

        ranked=sorted((group[0] for group in self._entrance_groups(nearby,ground)),
                      key=lambda flight:(depart and not flight.confirmed,
                                         (previous_attempt(flight) or {}).get('failures',0)))
        if depart:
            departures=sorted((flight for flight in saved if flight in candidates),
                              key=lambda flight:self._stair_priority(flight,ground))
            ranked=departures+[flight for flight in ranked if not any(
                self._same_probe_entrance(flight,stored) for stored in departures)]
        last_failure=None
        attempted=set()
        for flight in ranked:
            original=flight
            flight=self._qualified_stair(flight,floor)
            if flight is None:continue
            previous=previous_attempt(flight)
            if previous is not None and id(previous) in attempted:
                continue
            if previous is not None:
                if previous.get('confirmed') and not depart and self._upward_stair_check(previous['flight'],floor)['ready']:
                    continue
                task=previous
                if self._upward_stair_check(task['flight'],floor)['ready']:
                    previous=self._qualified_stair(task['flight'],floor)
                    flight=max((previous,flight),key=lambda f:(len(f.usable_treads),len(f.treads)))
                task['flight']=flight
                failures=task.get('failures',0)
                if task.get('finished') and failures:
                    old=task.get('failed_levels',())
                    new=tuple(t[2] for t in flight.treads)
                    extended=len(new)>len(old) and new[-1]>max(old,default=floor[0])+min(.035,s.surface_noise_tolerance)
                    geometry=self._stair_task_geometry(task)
                    improved_stance=None
                    if not extended and geometry!=task.get('checked_geometry'):
                        direction=math.atan2(flight.direction[1],flight.direction[0])
                        previous_ground=np.asarray(task.get('failure_ground',task['home']))
                        rejected=list(task.get('rejected',()))
                        if task.get('last_stance') is not None:
                            rejected.append(task['last_stance'])
                        improved_stance=self._verified_stair_stance(
                            previous_ground,direction,targets=[self._stair_xyz(t) for t in flight.treads],
                            floor_reference=floor[0],rejected=rejected)
                        task['checked_geometry']=geometry
                    if extended or improved_stance is not None:
                        task.update(failures=0,scans=0,rejected=[],views=[],verified=False,confirmed=False,
                                    rejected_approaches=[],
                                    reopen_reason='new_upward_tread' if extended else 'new_safe_upward_stance')
                    elif failures>=2 or not review:
                        continue
                    else:
                        task['reopen_reason']='floor_review'
                task.update(finished=False,attempt_settled=False)
            else:
                task=dict(floor=floor,flight=flight,home=tuple(ground),finished=False,
                          views=[],scans=0,failures=0,attempt_settled=False,
                          verified=False,confirmed=False,reason='stair_evidence_missing')
                records.append(task)
            attempted.add(id(task))
            task['tolerance']=tolerance
            if previous is None or task.get('failures',0)==0 or (depart and original in saved):
                if abs(ground[2]-floor[0])<=min(floor[1],s.surface_noise_tolerance)+1e-7:
                    task['home']=tuple(ground)
            if depart and original in saved:
                task['saved_approach']=saved[original]
                task['approach_pending']=abs(ground[2]-floor[0])<=min(floor[1],s.surface_noise_tolerance)+1e-7
            elif depart and pending.get(original) is not None:
                # A recorded pose is a hint. _stair_approach rechecks its
                # current safety, connectivity and route before it is used.
                task['saved_approach']=pending[original]
                task['approach_pending']=abs(ground[2]-floor[0])<=min(floor[1],s.surface_noise_tolerance)+1e-7
            result=self._local_stair_observers(task,ground,yaw,depart)
            if result.get('climbing') and not depart:
                continue
            if result.get('ready'):
                self._local_stair_task=task
                return self._stair_probe_result(task,**result)
            self._finish_stair_attempt(task,result['reason'],ground)
            last_failure=task
        if last_failure is not None:
            return self._stair_probe_result(last_failure,ready=False,finished=True,reason=last_failure['reason'])
        if indexing:
            return dict(ready=False,active=True,reason='stair_index_pending')
        return dict(ready=False,reason='no_local_stair_probe')

    def _local_stair_observers(self, task, ground, yaw, depart=False):
        """Use a safe tread stance as the stopping condition for each observation."""
        m,s=self.map,self.map.settings
        flight=task['flight']
        if flight.pending_treads:
            # Only the next blocked part becomes a local review target. The
            # complete geometric tail remains cached for later advancement.
            x,y,height=flight.pending_treads[0]
            point=self._stair_xyz((x,y,height))
            if math.dist(point[:2],ground[:2])<=STAIR_PROBE_RADIUS:
                sources=[(x,y,r.id) for r in m.surface_observation.matching((x,y),height)]
                layers=[k for k,row in m.layers_at(x,y) if abs(float(row['G'])-height)<=s.surface_noise_tolerance]
                m.surface_observation.expect(point,sources=sources,slices=layers,review=True)
                for layer in layers:
                    for target in m.unknown_blockers(layer,[(x,y)],height):
                        m.surface_observation.expect(target[:3],slices=[layer],review=True)
        if task.get('approach_pending'):
            geometry=self._upward_stair_check(flight,task['floor'])
            if not geometry['ready'] and not geometry.get('reviewable'):
                task['diagnostic']=geometry
                return dict(ready=False,reason=geometry['reason'])
            approach=self._stair_approach(task,ground,search=True)
            task['diagnostic']=self._upward_stair_check(flight,task['floor'],ground,approach)
            if approach is None:
                return dict(ready=False,reason='stair_no_safe_approach')
            task['saved_approach']=approach
            if math.dist(ground[:2],approach[:2])>task['tolerance'] or abs(ground[2]-approach[2])>.04:
                return dict(ready=True,active=True,approaching=True,
                            observations=[list(approach[:3])],yaws=[approach[3]],
                            reason='stair_approach_ready')
            task.update(approach_pending=False,home=tuple(ground),saved_approach=approach)
        task['diagnostic']=self._stair_task_check(task)
        task['diagnostic'].update(observer_ground=list(map(float,ground)),
            saved_candidate=flight in self._saved_stairs.get(task['floor'],()))
        if not task['diagnostic']['ready'] and not task['diagnostic'].get('reviewable'):
            return dict(ready=False,reason=task['diagnostic']['reason'])
        direction=math.atan2(flight.direction[1],flight.direction[0])
        if not flight.confirmed and self._stair_ends_on_known_level(flight,ground):
            return dict(ready=False,reason='stair_known_level_continuation')
        stance=self._verified_stair_stance(ground,direction,
            allow_level=ground[2]>task['floor'][0]+s.surface_noise_tolerance,
            targets=[self._stair_xyz(t) for t in (flight.usable_treads or flight.treads[:1])],rejected=task.get('rejected',()),
            floor_reference=task['floor'][0],diagnostic=task['diagnostic'])
        if stance is not None:
            x,y,height,heading=stance
            # The geometric representative may be at a rough/unsampled edge.
            # Replace only its representation with the actual, fully verified
            # stance on that same tread; never invent a new height or skip a
            # blocked earlier tread when counting the continuous safe prefix.
            at=min(range(len(flight.treads)),key=lambda i:abs(flight.treads[i][2]-height))
            if abs(flight.treads[at][2]-height)<=s.surface_noise_tolerance+1e-7:
                treads=list(flight.treads)
                treads[at]=(*map(int,m.indices((x,y))),float(height))
                qualified=self._qualified_stair(replace(flight,treads=tuple(treads)),task['floor'])
                if qualified is not None:
                    flight=task['flight']=qualified
                    task['diagnostic'].update(self._stair_task_check(task))
            task['verified']=True
            task['last_stance']=stance[:3]
            task['diagnostic']['verified_stance']=list(map(float,stance[:3]))
            if flight.confirmed:
                task['confirmed']=self._remember_stairs(task,ground,yaw)
                if not depart:
                    if not task['confirmed']:
                        return dict(ready=False,reason='stair_no_safe_approach')
                    task.update(finished=True,reason='stair_confirmed')
                    return dict(ready=True,active=True,climbing=True,observations=[[x,y,height]],
                                yaws=[heading],yaw=heading,reason='stair_confirmed')
            return dict(ready=True,active=True,climbing=depart and flight.confirmed,
                        observations=[[x,y,height]],yaws=[heading],yaw=heading,reason='stair_stance_ready')
        if flight.confirmed and task.get('verified') and not depart:
            task.update(finished=True,confirmed=True,reason='stair_confirmed')
            return dict(ready=False,reason='stair_confirmed')
        if task['scans']>=2 or task.get('failures',0)>=2:
            return dict(ready=False,reason='stair_observation_exhausted')
        goals=[self._stair_xyz(t) for t in flight.treads if t[2]>=ground[2]-s.surface_noise_tolerance]
        goals.extend(self.stair_review_points(b'',self.version,
            (*ground[:2],ground[2]+self.reference_height),direction))
        views=self.observation_poses((*ground[:2],ground[2]+self.reference_height),(0.,0.,yaw),goals,
            floor=(ground[2],s.step_max+s.surface_noise_tolerance),ascent=direction,
            preferred=self._stair_xyz(flight.treads[-1]),diagnostics=True)
        task['diagnostic']['observation_search']={key:views[key] for key in
            ('rejections','rejected_poses','stances_checked') if key in views}
        observations=[];headings=[]
        observed_targets=[]
        for tx,ty,tz,x,y,z,heading,_ in views['poses']:
            stance=(x,y,z-self.reference_height)
            if stance[2]<task['floor'][0]-min(task['floor'][1],s.surface_noise_tolerance)-1e-7:
                continue
            if not -s.surface_noise_tolerance<=stance[2]-ground[2]<=s.step_max+1e-7:
                continue
            if any(math.dist(stance[:2],p[:2])<.25 and abs(stance[2]-p[2])<.04 and
                   abs(math.remainder(heading-h,2*math.pi))<.35 for p,h in task['views']):
                continue
            observations.append(list(stance));headings.append(heading)
            observed_targets.append([tx,ty,tz])
        task['diagnostic']['observation_targets']=observed_targets
        if not observations:
            # A new stance can require evidence the current stance is able to
            # collect. Keep the actual-body and sight checks for that scan.
            diagnostic=task['diagnostic']
            # Keep the gaps local to the rejected tread stance as well as
            # the gaps closest to the robot returned by the wider search.
            blockers=[p for rejected in diagnostic.get('rejected_poses',())
                      for p in rejected.get('unknown_blockers',())]
            blockers.extend(diagnostic.get('arrival_blockers',[]))
            blockers.extend(views['unknown_blockers'])
            targets=list(dict.fromkeys(tuple(p[:3]) for p in blockers))
            start=(*ground[:2],ground[2]+self.reference_height)
            completed=[(*point,heading,.35) for point,heading in task['views']]
            stationary=self.observation_poses(start,(0.,0.,yaw),targets,
                ascent=direction,motions=False,purpose='current_observation',completed=completed)
            if not stationary['poses']:
                return dict(ready=False,reason='stair_no_safe_observation')
            tx,ty,tz,_,_,_,heading,_=stationary['poses'][0]
            task['diagnostic']['observation_targets']=[[tx,ty,tz]]
            return dict(ready=True,active=True,observations=[list(ground)],yaws=[heading],
                        yaw=heading,reason='stair_current_observation_ready')
        return dict(ready=True,active=True,observations=observations,yaws=headings,yaw=headings[0],
                    reason='stair_observation_ready')

    def _stair_ends_on_known_level(self, flight, ground):
        # A fully measured level continuation excludes an isolated step only
        # after reaching it. Missing cells remain insufficient evidence.
        m,s=self.map,self.map.settings
        x,y,height=flight.treads[-1]
        if abs(ground[2]-height)>s.surface_noise_tolerance:
            return False
        dx,dy=flight.direction
        tolerance=min(.035,s.surface_noise_tolerance)
        for distance in range(2,8):
            for side in (-2,0,2):
                heights=[float(row['G']) for _,row in m.layers_at(
                    x+distance*dx-side*dy,y+distance*dy+side*dx) if not row['unknown']]
                if not any(abs(h-height)<=tolerance for h in heights) or any(
                    s.surface_noise_tolerance<h-height<=s.step_max for h in heights):
                    return False
        return True

    def _verified_stair_stance(self, ground, yaw, allow_level=False, targets=None, rejected=(),
                               floor_reference=None,diagnostic=None):
        """Share target admission, observed arrival area and native route verification."""
        s=self.map.settings
        max_rise=min(.16,s.step_max)
        steps,_,_=self._stair_step_targets(ground,.15+s.resolution,
            (yaw,ground[2]-s.surface_noise_tolerance,.15))
        start=tuple(np.asarray(ground)+[0.,0.,self.reference_height])
        goals=list(targets or [tuple(p[:3]) for p in steps])
        # A stance on the observed tread looks beyond that tread. Requiring it
        # to see its own near-filtered surface would reject safe advancement.
        goals.extend(self.stair_review_points(b'',self.version,start,yaw))
        centers=[(*point,yaw) for point in (targets or ())]
        candidates=[]
        reasons={}
        def eligible(x,y,height):
            reason=None
            if height-ground[2]>max_rise+1e-7:
                reason='stance_above_next_step'
            elif floor_reference is not None and height<floor_reference+s.surface_noise_tolerance-1e-7:
                reason='stance_not_above_floor'
            elif any(math.dist((x,y),p[:2])<.20 and abs(height-p[2])<.04 for p in rejected):
                reason='previously_rejected_stance'
            elif not allow_level and height<=ground[2]+s.surface_noise_tolerance:
                reason='stance_not_above_current_ground'
            elif math.dist((x,y),ground[:2])<.15+s.resolution:
                reason='stance_too_close'
            elif (x-ground[0])*math.cos(yaw)+(y-ground[1])*math.sin(yaw)<.15:
                reason='stance_not_forward'
            if reason:reasons[reason]=reasons.get(reason,0)+1
            return reason is None
        margins={}
        def priority(point):
            key=tuple(point[:3])
            if key not in margins:margins[key]=self._tread_margin(key,yaw)
            return -margins[key]
        for x,y,height,heading in sorted((*centers,*steps),key=priority):
            if not eligible(x,y,height):continue
            candidates.append((np.asarray((x,y,height+self.reference_height)),0,True))
        search=self.observation_poses(start,(0.,0.,yaw),goals,stances=candidates,
            ascent=yaw,headings=[yaw],minimum_gain=0,max_results=12,diagnostics=diagnostic is not None)
        blockers=search['unknown_blockers']
        samples=search.get('rejected_poses',[])
        checked=search.get('stances_checked',0)
        if not search['poses']:
            first_search=search
            lower=ground[2]-s.surface_noise_tolerance if allow_level else ground[2]
            upper=ground[2]+max_rise
            search=self.observation_poses(start,(0.,0.,yaw),goals,
                floor=((lower+upper)*.5,(upper-lower)*.5),
                ascent=yaw,headings=[yaw],
                preferred=next((p for p in goals if p[2]>ground[2]+s.surface_noise_tolerance),None),
                diagnostics=diagnostic is not None)
            for reason,count in first_search.get('rejections',{}).items():
                reasons[reason]=reasons.get(reason,0)+count
            samples=(samples+search.get('rejected_poses',[]))[:3]
            checked+=search.get('stances_checked',0)
            blockers=self.map.merge_unknown_blockers(blockers+search['unknown_blockers'],ground)
        for reason,count in search.get('rejections',{}).items():reasons[reason]=reasons.get(reason,0)+count
        if diagnostic is not None:
            diagnostic.update(stance_candidates=len(centers)+len(steps),admitted_stances=len(search['poses']),
                              stances_checked=checked,rejections=reasons,rejected_poses=samples,
                              arrival_blockers=blockers,route_rejections={})
        for _,_,_,x,y,z,heading,_ in sorted(search['poses'],
                key=lambda view:priority((view[3],view[4],view[5]-self.reference_height))):
            height=z-self.reference_height
            if not eligible(x,y,height):continue
            began=time.monotonic()
            route_reason,_,route_meta=self.plan_surface(b'',self.version,start,(x,y,z))
            if route_reason=='ok':return x,y,height,heading
            if diagnostic is not None:
                routes=diagnostic['route_rejections'];routes[route_reason]=routes.get(route_reason,0)+1
                samples=diagnostic.setdefault('failed_routes',[])
                if len(samples)<3:
                    samples.append(dict(target=[float(x),float(y),float(z)],reason=route_reason,
                                        start_check=route_meta.get('start_check',{}).get('reason'),
                                        goal_check=route_meta.get('goal_check',{}).get('reason'),
                                        elapsed_ms=1000*(time.monotonic()-began)))
        return None

    def observation_poses(self, base, attitude, targets, stances=None, floor=None, ascent=None,
                          preferred=None, tolerance=.15, completed=(), minimum_gain=1,
                          max_results=3, motions=None, headings=None, version=None, diagnostics=False, purpose='arrival'):
        if version is not None:self.prepare(b'',version)
        return self.map.frontier_index.observation_model.pose_search.observation_poses(
            base,attitude,targets,stances,floor,ascent,preferred,tolerance,completed,
            minimum_gain,max_results,motions,headings,diagnostics,purpose)

    def _arrival_geometry(self, xyz, xy_tolerance):
        return self.map.frontier_index.observation_model.pose_search._arrival_geometry(xyz,xy_tolerance)

    def _arrival_area(self, xyz, xy_tolerance, heading=0.):
        return self.map.frontier_index.observation_model.pose_search._arrival_area(xyz,xy_tolerance,heading)

    def stair_review_points(self, buffer, version, base, heading):
        """Observation targets at the expected height of the next tread."""
        self.prepare(buffer, version)
        m, s = self.map, self.map.settings
        support=self._ground_support(base)
        if not support['ready']:return []
        reference = support['ground_z']
        start = np.asarray(base[:2], dtype=float)
        forward = np.array([math.cos(heading),math.sin(heading)])
        side = np.array([-forward[1],forward[0]])
        width = s.robot_radius+s.safety_margin
        min_rise,max_rise = tread_rise_bounds(s)
        cells = {}
        for d in np.arange(s.resolution,3*width+s.resolution*.5,s.resolution):
            for l in (0.,-width*.5,width*.5,-width,width):
                xy = tuple(m.indices(start+d*forward+l*side))
                if xy not in cells:
                    cells[xy]=(float(d),abs(l),[float(row['G']) for _,row in m.layers_at(*xy)
                                               if not row['unknown']])
        higher = [(d,l,h) for d,l,heights in cells.values() for h in heights
                  if min_rise<h-reference<=max_rise]
        search_range=(reference+min_rise,reference+max_rise)
        # Aiming at the middle of the search band does not assert a tread at
        # that height. Only actual returns may supply the next G.
        first,_,height = min(higher,default=(width,0.,sum(search_range)*.5))
        points = []
        for xy,(d,l,heights) in cells.items():
            if d<first-2*s.resolution:
                continue
            plausible = [h for h in heights if min_rise<h-reference<=2*max_rise]
            expected = min(plausible,key=lambda z:abs(z-height),default=sum(search_range)*.5)
            if (m.surface_relations.explains_step(xy,expected) or
                    m.surface_relations.fresh_clearance_barrier(xy,reference) is not None):
                continue
            point = (*m.position((0,*xy)),expected)
            points.append((abs(d-max(first,2.5*width))+l,point,not plausible))
        result=[]
        source=[(*map(int,m.indices(base)),support['surface_id'])] if support['ready'] else []
        for _,point,search in sorted(points)[:12]:
            if search:
                m.surface_observation.expect(point,sources=source,kind='search',height_range=search_range)
            result.append(list(point))
        return result

    def _stair_targets(self, flights, ground, continuation=None, rejected=None,
                       departure_height=None):
        s=self.map.settings
        min_rise,max_rise=tread_rise_bounds(s)
        initial=departure_height is not None

        def next_height(height):
            rise=height-ground[2]
            if initial:
                return (height-departure_height>=s.surface_noise_tolerance-1e-7 and
                        -s.surface_noise_tolerance<=rise<=max_rise)
            return min_rise < rise <= max_rise

        ordered=sorted(flights,key=lambda f:self._stair_priority(f,ground))
        choices={}
        for flight in ordered:
            if len(choices)>=8:continue
            yaw=math.atan2(flight.direction[1],flight.direction[0])
            if continuation is not None:
                forward, minimum_height, slack=continuation
                if (math.cos(yaw-forward)<.5 or
                        not flight.continues_from(tuple(self.map.indices(ground)),ground[2],
                                                  s.resolution,s.surface_noise_tolerance)):
                    continue
            dx,dy=flight.direction
            for x,y,z in flight.usable_treads:
                if not next_height(z):
                    continue
                offsets = (0,-1,-2,-3,-4,-5,-6,1,2,3)
                for advance in offsets:
                    p=self._stair_xyz((x+advance*dx,y+advance*dy,z))
                    if continuation is not None:
                        ahead=(p[0]-ground[0])*math.cos(forward)+(p[1]-ground[1])*math.sin(forward)
                        if p[2]<minimum_height or ahead<=slack:
                            continue
                    elif initial and z-ground[2]<=min_rise:
                        if (p[0]-ground[0])*dx+(p[1]-ground[1])*dy<=0.:
                            continue
                    check=self._admission((p[0],p[1],p[2]+self.reference_height),height_tolerance=.02,
                                          collect_blockers=rejected is not None)
                    if rejected is not None and not check['ready']:
                        rejected.append(check)
                    if check['ready'] and next_height(check['ground_z']):
                        choices.setdefault((*p[:2],check['ground_z']),yaw)
                        if len(choices)>=8:
                            break
                if len(choices)>=8:
                    break
        return [[*p,yaw] for p,yaw in choices.items()]

    def _stair_step_targets(self, ground, minimum_distance, continuation):
        """Find a measured next tread independently of the saved flight's representative cells."""
        m, s = self.map, self.map.settings
        min_rise,max_rise=tread_rise_bounds(s)
        yaw, minimum_height, forward_slack = continuation
        forward = np.array([math.cos(yaw), math.sin(yaw)])
        side = np.array([-forward[1], forward[0]])
        width = s.robot_radius + s.safety_margin
        reach = max(3 * width, minimum_distance + s.resolution)
        extent = math.ceil((reach + width) / s.resolution)
        cx, cy = m.indices(ground)
        candidates = {}
        boundary = []
        for x in range(cx - extent, cx + extent + 1):
            for y in range(cy - extent, cy + extent + 1):
                xy = m.position((0, x, y))
                delta = xy - ground[:2]
                ahead, lateral = float(delta @ forward), abs(float(delta @ side))
                distance = math.hypot(*delta)
                if not forward_slack < ahead <= reach or lateral > width or distance < minimum_distance:
                    continue
                rows = tuple(m.layers_at(x, y))
                same_band = False
                for _, row in rows:
                    height = float(row['G'])
                    same_band |= not row['unknown'] and abs(height-ground[2]) <= s.step_max+1e-5
                    if (row['terrain_cost']<0 or row['unknown'] or row['solid'] or
                            height < minimum_height or not -s.surface_noise_tolerance <= height-ground[2] <= max_rise):
                        continue
                    point = (*map(float, xy), height)
                    relief = float(row['relief_cost'])
                    score = (height-ground[2] <= min_rise, relief if relief >= 0 else math.inf, lateral, distance)
                    candidates[point] = min(score, candidates.get(point, score))
                if not same_band:
                    boundary.append((x, y))
        targets = []
        same_tread = []
        unsafe = 0
        blockers = []
        for point in sorted(candidates,key=lambda p:(candidates[p][0],-self._tread_margin(p,yaw),
                                                       *candidates[p][1:])):
            x, y, height = point
            level = point[2]-ground[2] <= min_rise
            views = same_tread if level else targets
            if (len(views) >= 6 or
                    any(math.hypot(x-view[0],y-view[1]) < 2*s.resolution for view in views)):
                continue
            check = self._admission((x, y, height + self.reference_height),
                                    height_tolerance=s.surface_noise_tolerance, collect_blockers=True)
            if not check['ready']:
                unsafe += 1
                blockers.extend(check.get('unknown_blockers', ()))
                continue
            views.append([x, y, check['ground_z'], yaw])
            if len(targets) == len(same_tread) == 6:
                break
        if not targets and not blockers:
            # A flight can end at the edge of the measured map, before a
            # single higher observation stance can be generated.
            for x, y in boundary:
                for dx, dy in DIRECTIONS:
                    for k, row in m.layers_at(x+dx, y+dy):
                        if (not row['unknown'] and 0<=row['passage_cost']<50 and
                                abs(row['G']-ground[2]) <= s.step_max+1e-5):
                            blockers.extend(m.unknown_blockers(k, [(x, y)], float(row['G'])))
        return targets+same_tread, unsafe, m.merge_unknown_blockers(blockers, ground)

    def complete_observation(self, buffer, version, cluster_id, revision, point):
        self.prepare(buffer,version)
        self.map.frontier_index.complete(cluster_id,revision,point)

    def _local_stair_heading(self, ground):
        """Direction from nearby measured, traversable tread edges, never a whole-map query."""
        m,s=self.map,self.map.settings
        radius=max(3*(s.robot_radius+s.safety_margin),4*s.resolution)
        extent=math.ceil(radius/s.resolution)
        cx,cy=map(int,m.indices(ground))
        lo=np.array([cx-extent,cy-extent])//s.block_size
        hi=np.array([cx+extent,cy+extent])//s.block_size
        key=(cx,cy,float(ground[2]),m.first_layer,m.last_layer,tuple(
            m.planning_block_versions.get((k,bx,by),0)
            for k in range(m.first_layer,m.last_layer+1)
            for bx in range(lo[0],hi[0]+1) for by in range(lo[1],hi[1]+1)))
        cached=getattr(self,'_stair_heading_cache',None)
        if cached is not None and cached[0]==key:return cached[1]
        tolerance=s.surface_noise_tolerance
        min_rise,max_rise=tread_rise_bounds(s)
        heights={}
        for x in range(cx-extent,cx+extent+1):
            for y in range(cy-extent,cy+extent+1):
                if math.hypot(x-cx,y-cy)*s.resolution>radius:continue
                samples=[h for _,h,_ in m.support_samples(x,y)
                         if abs(h-ground[2])<=max_rise+tolerance]
                if samples:heights[x,y]=min(samples,key=lambda h:abs(h-ground[2]))
        direction=np.zeros(2);total=0.;edges=0
        for (x,y),height in heights.items():
            for dx,dy in ((1,0),(0,1)):
                other=heights.get((x+dx,y+dy))
                if other is None:continue
                rise=other-height
                if not min_rise<abs(rise)<=max_rise+1e-7:continue
                if not min(height,other)-tolerance<=ground[2]<=max(height,other)+tolerance:continue
                distance=math.hypot(x+.5*dx-cx,y+.5*dy-cy)*s.resolution
                weight=abs(rise)/(.2+distance)**2
                direction+=math.copysign(weight,rise)*np.array([dx,dy])
                total+=weight;edges+=1
        confidence=float(np.linalg.norm(direction))/total if total else 0.
        result=dict(ready=edges>=3 and confidence>=.65,reason='measured_tread_direction',
                    yaw=math.atan2(direction[1],direction[0]),edges=edges,confidence=confidence)
        if not result['ready']:result['reason']='stair_direction_unobserved'
        self._stair_heading_cache=key,result
        return result

    def observation_task(self, buffer, version, operation, cid, xyz, yaw, half_angle, blockers=None):
        self.prepare(buffer,version)
        model=self.map.frontier_index.observation_model
        ground=np.asarray(xyz,dtype=float)-[0,0,self.reference_height]
        if operation=='floor_work_available':
            pending=self.map.frontier_index.floor_work_available((float(xyz[2]),half_angle))
            return dict(ready=pending,reason='floor_work_available' if pending else 'floor_work_unchanged')
        if operation=='begin_frontier_crossing':
            self.map.surface_observation.begin_crossing()
            return dict(ready=True,reason='frontier_crossing_started')
        if operation=='cancel_frontier_crossing':
            self.map.surface_observation.cancel_crossing()
            return dict(ready=True,reason='frontier_crossing_cancelled')
        if operation=='enter_frontier_floor':
            evidence=self.map.surface_observation
            new=evidence.enter_frontier_floor(xyz,half_angle)  # xyz already contains measured ground height.
            return dict(ready=True,reason='frontier_floor_entered' if new else 'frontier_floor_revisited',
                        **evidence.status())
        if operation=='stair_heading':
            if self.map.first_layer is None:return dict(ready=False,reason='map_unavailable')
            support=self._ground_support(xyz)
            if not support['ready']:return dict(ready=False,reason=support['reason'])
            ground[2]=support['ground_z']
            return self._local_stair_heading(ground)
        if operation=='observe_height':
            support=self._ground_support(xyz)
            evidence=self.map.surface_observation
            if not support['ready']:
                return dict(ready=False,reason='height_support_unobserved',**evidence.status())
            ground[2]=support['ground_z']
            result=evidence.visit_height(ground)
            return dict(result,ready=result['triggered'])
        if operation=='arrival_area':
            return self._arrival_area(xyz,half_angle,yaw)
        if operation=='blocker_available':
            ready=model.blocker_review.available(ground)
            return dict(ready=ready,reason='blocker_review_available' if ready else 'blocker_already_reviewed')
        if operation=='begin_blocker_review':
            return model.blocker_review.begin(blockers or ())
        if operation=='arm_blocker_review':return model.blocker_review.arm()
        if operation=='blocker_review_progress':return model.blocker_review.progress()
        if operation=='finish_blocker_review':return model.blocker_review.finish()
        if operation=='blocker_pending':
            if not model.blocker_review.available(ground):
                return dict(ready=False,reason='blocker_already_reviewed')
            m,s=self.map,self.map.settings
            x,y=m.indices(ground)
            cells={(int(x)+dx,int(y)+dy) for dx,dy in ((0,0),(0,1),(0,-1),(1,0),(-1,0))}
            layers={k for cx,cy in cells for k,_ in m.layers_at(cx,cy)}
            if not layers and m.first_layer is not None:
                layers.add(min(m.last_layer,max(m.first_layer,
                    math.floor((ground[2]-s.origin_z)/s.slice_dh)+1)))
            targets=[target for layer in layers for target in m.unknown_blockers(layer,cells,ground[2])]
            pending=any(math.dist(target[:2],ground[:2])<1e-6 and
                        abs(target[2]-ground[2])<=self.map.settings.surface_noise_tolerance+1e-7
                        for target in targets)
            return dict(ready=pending,reason='observation_pending' if pending else 'blocker_resolved')
        if operation in ('floor_reject_view','floor_reject_view_timeout'):
            ready=self.map.frontier_index.reject_floor_view(cid,xyz,operation.endswith('_timeout'),blockers or (),half_angle)
            return dict(ready=ready,reason=operation)
        if operation in ('arm_floor_stair_probe','floor_stair_probe_progress','finish_floor_stair_probe',
                         'finish_local_stair_return','cancel_local_stair_probe_scan','finish_stair_recognition',
                         'reject_stair_stance','recover_stair_stop'):
            task=self._local_stair_task
            if task is None:
                return dict(ready=False,reason='floor_stair_probe_not_started')
            if operation=='recover_stair_stop':
                ground=self._measured_ground(xyz)
                stand=ground+[0.,0.,self.reference_height]
                check=self._arrival_area(stand,task['tolerance'],yaw)
                if not check['ready'] or task.get('stop_recoveries',0)>=2:
                    return dict(check,ready=False,reason='stair_stop_recovery_unavailable')
                # A timeout is not an arrived receipt. Keep the physical task,
                # reject this stance and replan locally without resetting scan budgets.
                rejected='rejected_approaches' if task.get('approach_pending') else 'rejected'
                task.setdefault(rejected,[]).append(tuple(map(float,ground)))
                task.update(stop_recoveries=task.get('stop_recoveries',0)+1,
                            finished=False,verified=False,reason='stair_stop_safe_replan')
                return self._stair_probe_result(task,ready=True,reason='stair_stop_safe_replan')
            if operation=='reject_stair_stance':
                if task.get('approach_pending'):
                    task.setdefault('rejected_approaches',[]).append(tuple(ground))
                    return self._stair_probe_result(task,ready=True,reason='stair_approach_rejected')
                task.setdefault('rejected',[]).append(tuple(map(float,ground)))
                task['views'].append((tuple(map(float,ground)),float(yaw)))
                if task['finished']:
                    self._local_stair_task=None
                else:self._finish_stair_attempt(task,'stair_route_or_execution_rejected')
                return self._stair_probe_result(task,ready=True,reason='stair_stance_rejected')
            if operation=='arm_floor_stair_probe':
                self._floor_stair_probe_serial=self.map.integrated_serial
                return dict(ready=True,reason='floor_stair_probe_armed')
            if operation=='floor_stair_probe_progress':
                ground[2]=self._admission(xyz,relaxed=True).get('ground_z',ground[2])
                self._refresh_stair_task(task,ground,yaw)
                task['diagnostic']=self._stair_task_check(task)
                advance=task['diagnostic']['ready'] and self._verified_stair_stance(ground,math.atan2(
                    task['flight'].direction[1],task['flight'].direction[0]),
                    allow_level=ground[2]>task['floor'][0]+self.map.settings.surface_noise_tolerance,
                    targets=[self._stair_xyz(t) for t in task['flight'].treads],
                    rejected=task.get('rejected',()),floor_reference=task['floor'][0],diagnostic=task['diagnostic']) is not None
                return self._stair_probe_result(task,ready=advance,fresh=self.map.integrated_serial>getattr(self,'_floor_stair_probe_serial',self.map.integrated_serial),
                            reason='stair_stance_ready' if advance else 'stair_stance_missing')
            if operation=='finish_floor_stair_probe':
                ground[2]=self._admission(xyz,relaxed=True).get('ground_z',ground[2])
                stance=(*map(float,ground),float(yaw))
                if self._stair_approach_safe(stance,task['floor'],task['tolerance']):
                    task['observed_approach']=stance
                task['views'].append((stance[:3],stance[3]))
                task['scans']+=1
                return self._stair_probe_result(task,ready=True,reason='stair_view_completed')
            if operation=='finish_stair_recognition':
                task.update(confirmed=True,finished=True,reason='stair_confirmed')
                self._local_stair_task=None
                return dict(ready=True,reason='stair_recognition_completed')
            if operation=='finish_local_stair_return':
                if math.dist(ground[:2],task['home'][:2])<=half_angle+1e-7 and \
                        abs(ground[2]-task['home'][2])<=.05:
                    task['finished']=True
                    self._local_stair_task=None
                    return dict(ready=True,reason='local_stair_probe_returned')
                return dict(ready=False,reason='local_stair_return_unverified')
            task['views'].append((tuple(map(float,ground)),float(yaw)))
            task['scans']+=1
            self._finish_stair_attempt(task,'stair_observation_unavailable',ground)
            return self._stair_probe_result(task,ready=True,reason='local_stair_probe_inconclusive')
        if operation=='stair_step_progress':
            ground[2]=self._admission(xyz,relaxed=True).get('ground_z',ground[2])
            if self._stair_platform_visible(ground,None,yaw):
                return dict(ready=True,reason='stair_platform_visible')
            stance=self._verified_stair_stance(ground,yaw,allow_level=True)
            return dict(ready=stance is not None,reason='stair_stance_ready' if stance else 'stair_stance_missing')
        if operation=='arm_stair_view':
            m,s=self.map,self.map.settings
            ground=self._measured_ground(xyz)
            forward=np.array([math.cos(yaw),math.sin(yaw)])
            side=np.array([-forward[1],forward[0]])
            width=s.robot_radius+s.safety_margin
            lateral=np.arange(-width,width+s.resolution*.5,s.resolution)
            if half_angle:
                lateral=lateral[lateral*math.copysign(1.,half_angle)>=-.5*s.resolution]
            cells={tuple(m.indices(ground[:2]+distance*forward+offset*side))
                   for distance in np.arange(s.resolution,3*width+s.resolution*.5,s.resolution)
                   for offset in lateral}
            limit=min(.16,s.step_max)+1e-7
            next_tread={xy:tuple(r.height for r in m.surface_observation.observed(xy,kind='ground')
                                if s.surface_noise_tolerance<r.height-ground[2]<=limit) for xy in cells}
            targets={xy:heights for xy,heights in next_tread.items() if heights}
            self.stair_view=(m.integrated_serial,ground[2],targets or dict.fromkeys(cells,()))
            return dict(ready=True,reason='stair_view_armed')
        if operation=='stair_view_progress':
            if not hasattr(self,'stair_view'):
                return dict(ready=False,reason='stair_view_not_armed')
            serial, height, targets = self.stair_view
            m,s = self.map,self.map.settings
            fresh=any(any(abs(record.height-expected)<=s.surface_noise_tolerance for expected in heights)
                      if heights else -s.surface_noise_tolerance<=record.height-height<=min(.16,s.step_max)+s.surface_noise_tolerance
                      for xy,heights in targets.items()
                      for record in m.surface_observation.observed(xy,after=serial,kind='ground'))
            return dict(ready=fresh,fresh=fresh,
                        reason='stair_view_observed' if fresh else 'stair_view_no_local_returns')
        if operation in ('floor_attempt','floor_recovery_attempt'):
            attempt=model.floor_attempt
            if attempt is None or attempt.cluster!=cid:
                attempt=self.map.frontier_index.begin_floor_attempt(cid)
            self.map.frontier_index.clear_floor_view_search(cid)
            if operation=='floor_attempt' and attempt is not None and attempt.fast:
                visible,_=model.floor_view(attempt.local,ground,yaw)
                attempt=replace(attempt,local=visible)
            model.floor_attempt=attempt
            return dict(ready=model.floor_attempt is not None,reason='floor_attempt_recorded')
        if operation=='validate_fast_view':
            index=self.map.frontier_index;s=self.map.settings
            members=index.floor_query_members.get(cid,())
            _,gain=model.floor_view(members,ground,yaw)
            ready=gain>=max(1,min(s.fast_min_visible_cells,len(index.physical_targets(members))))
            return dict(ready=ready,reason='ready' if ready else 'no_visible_frontier',gain=gain)
        if operation=='floor_execution_failed':
            attempt=model.task.get('floor_attempt') if model.task is not None else model.floor_attempt
            outcome=(self.map.frontier_index.settle_floor_attempt(attempt,True,tuple(ground))
                     if attempt is not None and attempt.cluster==cid else 'untracked')
            return dict(ready=outcome in ('first_no_progress','deferred','blacklisted','cluster_changed'),
                        reason=operation,floor_attempt=outcome)
        if operation in ('floor_no_observation','floor_observer_timeout','floor_disconnected','floor_route_timeout',
                         'floor_no_route','floor_missing_connection','floor_known_barrier'):
            index=self.map.frontier_index
            attempt=model.floor_attempt
            if attempt is not None and attempt.cluster==cid:
                model.floor_attempt=None
            else:
                attempt=index.begin_floor_attempt(cid)
            outcome=index.settle_floor_attempt(attempt,True,tuple(ground) if half_angle>0. else None,
                                               failure_reason=operation)
            return dict(ready=outcome in ('first_no_progress','deferred','blacklisted','review_timed_out'),
                        reason=operation,floor_attempt=outcome)
        if operation=='begin':return model.begin(cid,ground)
        if operation in ('begin_flat','begin_floor'):
            return model.begin(cid,ground,half_angle=half_angle,current_yaw=yaw,
                               floor_retry=operation=='begin_floor')
        if operation=='begin_forward':
            tracked=model.floor_attempt is not None and model.floor_attempt.cluster==cid
            return model.begin(cid,ground,yaw,half_angle,floor_retry=tracked)
        if operation in ('cancel','cancel_recovery_view'):
            model.task=None
            if operation=='cancel':model.floor_attempt=None
            return dict(ready=True,reason='cancelled')
        if operation=='arm':return dict(ready=model.arm(ground,yaw),reason='armed')
        if operation=='progress':return model.progress()
        if operation=='finish':
            return model.finish(ground,yaw,half_angle) or dict(ready=False,reason='completed',floor_attempt=None)
        raise ValueError('invalid_observation_operation')

    def plan_surface(self, buffer, version, start, target, floor=None):
        return self.plan(buffer,version,start,target,floor,goal_height_tolerance=.02)

    def plan_surface_with_blockers(self, buffer, version, start, target, floor=None):
        return self.plan(buffer,version,start,target,floor,goal_height_tolerance=.02,collect_blockers=True)

    def plan_stair_surface(self, buffer, version, start, target, floor=None):
        return self.plan(buffer,version,start,target,floor,goal_height_tolerance=.02,
                         collect_blockers=True,dense=True)

    def plan_review_surface(self, buffer, version, start, target, floor, budget):
        return self.plan(buffer,version,start,target,floor,goal_height_tolerance=.02,
                         collect_blockers=True,budget=budget)

    def plan(self, buffer, version, start, target, floor=None, *, goal_height_tolerance=None,
             collect_blockers=False, dense=False, budget=None):
        deadline=None if budget is None else time.monotonic()+max(0.,budget)
        self.prepare(buffer, version)
        start_check=self._admission(start,floor=floor,collect_blockers=collect_blockers)
        goal_check=self._admission(target,floor=floor,height_tolerance=goal_height_tolerance,
                                   collect_blockers=collect_blockers)
        metadata=dict(self.metadata,start_check=start_check,goal_check=goal_check,
                      robot_reference_height=self.reference_height)
        if not start_check['ready']:
            if collect_blockers: metadata['unknown_blockers'] = start_check.get('unknown_blockers', [])
            return ('pct_route_outside_current_floor' if start_check['reason']=='surface_outside_current_floor'
                    else 'pct_unsafe_start'), [], metadata
        if not goal_check['ready']:
            if collect_blockers: metadata['unknown_blockers'] = goal_check.get('unknown_blockers', [])
            return ('pct_route_outside_current_floor' if goal_check['reason']=='surface_outside_current_floor'
                    else 'pct_unsafe_goal'), [], metadata
        ground_start = np.asarray(start, dtype=float).copy()
        ground_goal = np.asarray(target, dtype=float).copy()
        ground_start[2] -= self.reference_height
        ground_goal[2] -= self.reference_height
        route = self.planner.plan(ground_start, ground_goal, optimize=False,
                                  start_check=start_check, goal_check=goal_check, floor=floor,
                                  collect_blockers=collect_blockers, dense=dense, deadline=deadline)
        metadata['anchor_search'] = self.planner.search_diagnostics
        if route is None or len(route) == 0:
            if collect_blockers:
                metadata['unknown_blockers'] = self.planner.search_diagnostics.get('unknown_blockers', [])
            return ('pct_planning_timeout' if self.planner.search_diagnostics.get('timed_out') else
                    'pct_route_disconnected'), [], metadata
        route = np.asarray(route, dtype=float).copy()
        # Project wrapper exports ground + 0.50. Translate its reference once;
        # RDP and native A* remain exactly the project's implementation.
        route[:, 2] += self.reference_height - 0.50
        if not np.isfinite(route).all():
            return 'pct_invalid_route', [], metadata
        return 'ok', route.tolist(), metadata

    def recovery_targets(self, buffer, version, start):
        self.prepare(buffer,version)
        return self.map.target_candidates(start,self.reference_height)

    def point_check(self, buffer, version, point, floor=None):
        self.prepare(buffer,version)
        return self._admission(point,floor=floor)

    def tread_support_check(self, buffer, version, point, floor=None):
        self.prepare(buffer, version)
        return self._ground_support(point,floor)

    def platform_check(self, buffer, version, point, floor=None):
        self.prepare(buffer,version)
        center=self._admission(point,floor=floor)
        if not center['ready']:return center
        # A tread can be safe while its surrounding area is still a staircase.
        # Require real, safe, approximately level support around a body-sized
        # platform, rather than treating arrival at any step as a new floor.
        s=self.map.settings;radius=s.robot_radius+s.safety_margin+.30
        for angle in np.arange(8)*np.pi/4:
            p=np.asarray(point,dtype=float)+[radius*np.cos(angle),radius*np.sin(angle),0.]
            check=self._admission(p,floor=(center['ground_z'],min(.05,s.surface_noise_tolerance)))
            if not check['ready']:
                return dict(center,ready=False,reason='platform_not_confirmed')
        return dict(center,reason='safe_platform')

    def landing_check(self, buffer, version, point, departure_floor, heading=0., reference_height=None):
        self.prepare(buffer,version)
        center=self._admission(point,height_tolerance=PLATFORM_HEIGHT_TOLERANCE)
        return self._landing_geometry(point, departure_floor, heading, center,reference_height)

    def descending_landing_check(self, buffer, version, point, departure_floor, heading=0.):
        self.prepare(buffer,version)
        center=self._admission(point,height_tolerance=PLATFORM_HEIGHT_TOLERANCE)
        return self._landing_geometry(point,departure_floor,heading,center,direction=-1)

    def landing_recovery_check(self, buffer, version, point, departure_floor, heading=0., reference_height=None):
        self.prepare(buffer,version)
        # Only assess the platform geometry. The shift planner must separately
        # prove a safe exit from the starting footprint overlap.
        center=self._admission(point,relaxed=True,height_tolerance=.04)
        return self._landing_geometry(point, departure_floor, heading, center,reference_height)

    def _landing_geometry(self, point, departure_floor, heading, center,reference_height=None,*,direction=1):
        if not center['ready']:return center
        ground=center['ground_z']
        boundary=departure_floor[0]
        minimum=departure_floor[1] if direction<0 else 0.
        if min(direction*(ground-boundary),direction*(point[2]-self.reference_height-boundary))<=minimum:
            return dict(center,ready=False,reason='still_on_departure_floor')
        m=self.map;s=m.settings
        xy=tuple(m.indices(point))
        tolerance=PLATFORM_HEIGHT_TOLERANCE+1e-7
        reference=ground if reference_height is None else reference_height
        if abs(ground-reference)>tolerance:
            return dict(center,ready=False,reason='platform_height_changed',platform_reference=reference)
        heights={}

        def supported(x,y):
            height=min(self._landing_heights((x,y)),key=lambda z:(abs(z-reference),z),default=None)
            if height is None or abs(height-reference)>tolerance:
                return False
            heights[x,y]=height
            return True

        cells=level_platform_cells(s,xy,(math.cos(heading),math.sin(heading)),supported)
        if cells is None:
            return dict(center,ready=False,reason='platform_not_confirmed')
        platform_height=median(heights[xy] for xy in cells)
        if direction*(platform_height-boundary)<=minimum:
            return dict(center,ready=False,reason='still_on_departure_floor')
        return dict(center,reason='safe_platform',platform_height=platform_height)

    def _landing_disk(self, radius, offset=(0.,0.)):
        resolution=self.map.settings.resolution
        count=math.ceil(radius/resolution+.5)
        return [(dx,dy) for dx in range(-count,count+1) for dy in range(-count,count+1)
                if math.hypot(dx*resolution-offset[0],dy*resolution-offset[1])<=radius+1e-7]

    def _landing_heights(self, xy):
        return {float(row['G']) for _,row in self.map.layers_at(*xy)
                if row['measured'] and np.isfinite(row['G']) and 0<=row['terrain_cost']<=20}

    def _stair_platform_visible(self, ground, floor, heading):
        m,s=self.map,self.map.settings
        if floor is not None and ground[2]<=floor[0]+s.surface_noise_tolerance:
            return False
        tolerance=PLATFORM_HEIGHT_TOLERANCE+1e-7
        # Reuse platform dimensions: a wide but shallow tread is not a
        # platform. Only the observed-area requirement is smaller at entry.
        width=max(s.landing_min_width,2*(s.robot_radius+s.safety_margin))
        depth=max(s.landing_min_depth,1.7*(s.robot_radius+s.safety_margin))
        center=tuple(m.indices(ground))
        return level_platform(s,center,(math.cos(heading),math.sin(heading)),
            lambda x,y:any(abs(z-ground[2])<=tolerance for z in self._landing_heights((x,y))),
            (width,depth,width*depth))

    def _stair_landing_targets(self, ground, floor, continuation):
        m=self.map;s=m.settings
        yaw,minimum_height,backward_slack=continuation
        z=float(ground[2])
        if z<=floor[0]:
            return []
        center=tuple(m.indices(ground))
        origin=np.array([s.origin_x,s.origin_y])
        offset=ground[:2]-(origin+s.resolution*np.asarray(center))
        radius=s.landing_support_radius
        tolerance=PLATFORM_HEIGHT_TOLERANCE+1e-7
        heights_at=cache(self._landing_heights)

        def supported(x,y,height):
            return any(abs(z-height)<=tolerance for z in heights_at((x,y)))

        def has_level_ground(xy,height,disk):
            return bool(disk) and all(supported(xy[0]+dx,xy[1]+dy,height)
                                      for dx,dy in disk)

        direction=np.array([math.cos(yaw),math.sin(yaw)])
        if (has_level_ground(center,z,self._landing_disk(radius,offset)) and
                level_platform(s,center,direction,lambda x,y:supported(x,y,z))):
            return []  # Already inside a platform.
        disk=self._landing_disk(radius)
        # Prefer a stance with some room for SCAN's stopping tolerance.
        inner_disk=self._landing_disk(radius+.15)
        candidates=[]
        for dx,dy in self._landing_disk(radius+.90,offset):
            xy=center[0]+dx,center[1]+dy
            position=origin+s.resolution*np.asarray(xy)
            delta=position-ground[:2]
            distance=float(np.linalg.norm(delta))
            if distance<.25 or float(delta@direction)<-backward_slack:
                continue
            for height in sorted(heights_at(xy)):
                if height<=floor[0] or height<minimum_height or abs(height-z)>tolerance:
                    continue
                if not has_level_ground(xy,height,disk):
                    continue
                extra_room=has_level_ground(xy,height,inner_disk)
                candidates.append((not extra_room,distance,*position,height))
        targets=[]
        for _,_,x,y,height in sorted(candidates):
            if any(math.hypot(x-p[0],y-p[1])<.30 for p in targets):
                continue
            check=self._admission((x,y,height+self.reference_height),height_tolerance=tolerance)
            if check['ready']:
                targets.append([float(x),float(y),check['ground_z'],math.atan2(y-ground[1],x-ground[0])])
            if len(targets)>=4:
                break
        return targets

    def inspect(self, version, base, options):
        self.prepare(b'',version)
        from .pct_inspection import inspect
        return inspect(self,base,options)

    def _reverse_check(self, route, level=None, level_failure='recovery_shift_changes_level',
                       height_tolerance=.05):
        previous=None;initial_overlap=None
        for a,b in zip(route[:-1],route[1:]):
            count=max(1,int(np.ceil(np.linalg.norm(np.asarray(b)-a)/(self.map.settings.resolution*.4))))
            for t in np.linspace(0,1,count+1):
                p=np.asarray(a)*(1-t)+np.asarray(b)*t
                check=self.map.point_safe_check(p,self.reference_height,relaxed=True,
                                                floor=None if level is None else (level,height_tolerance))
                if check['reason']=='surface_outside_current_floor':return level_failure
                if not check['ready']: return check['reason']
                k=check['layer']
                # Unknown/obstacle distance never decreases while leaving
                # an initial inflation overlap. No new solid centre allowed.
                clearance=self.map.clearance_at(p,k)
                overlap=self.map.footprint_conflicts(p,k,keys=True)
                if initial_overlap is None: initial_overlap=overlap
                if not overlap.issubset(initial_overlap): return 'recovery_new_footprint_conflict'
                if previous is not None and clearance+0.015<min(previous,self.map.settings.robot_radius+self.map.settings.safety_margin):
                    return 'recovery_clearance_decreased'
                previous=clearance
        return None

    def plan_stair_reverse(self, buffer, version, start, target):
        self.prepare(buffer,version)
        a,b=np.asarray(start),np.asarray(target)
        delta=b-a
        length=np.linalg.norm(delta[:2])
        first=self._admission(a)
        metadata=dict(self.metadata,start_check=first)
        if not first['ready']:
            return 'pct_unsafe_start',[],metadata
        if not .20-1e-6<=length<=.60+1e-6:
            return 'pct_invalid_stair_reverse',[],metadata
        # Check the executor's early brake point and a 5 cm overshoot.
        overshoot=.05
        step_limit=min(.16,self.map.settings.step_max)
        for offset in (-.04,0.,overshoot):
            stop=self._admission(b+delta*(offset/length),floor=(first['ground_z'],step_limit))
            if not stop['ready']:
                metadata['stop_check']=stop
                return 'pct_unsafe_stair_reverse_stop',[],metadata
        # A body-height support lookup alone can still straddle a down-step.
        # Check the rear footprint against the configured step capability.
        direction=delta/length
        # GO2 rear hip is 19.34 cm behind the base. Allow a further 5.66 cm
        # for foot placement. The larger body radius still checks collisions.
        rear_reach=.25
        lateral=np.array([-direction[1],direction[0],0.])
        reach=length+overshoot+rear_reach
        for offset in (-.15,0.,.15):
            for distance in np.linspace(0.,reach,int(np.ceil(reach/(.4*self.map.settings.resolution)))+1):
                p=a+direction*distance+lateral*offset
                support=self.map.point_safe_check(p,self.reference_height,relaxed=True,
                                                  floor=(first['ground_z'],step_limit))
                if not support['ready']:
                    metadata['rear_support_check']=support
                    return 'pct_stair_reverse_rear_support_unverified',[],metadata
        reason=self._reverse_check([a,b+direction*overshoot],level=first['ground_z'],
                                   level_failure='stair_reverse_exceeds_step',
                                   height_tolerance=step_limit)
        return ('pct_'+reason,[],metadata) if reason else ('ok',[list(start),list(target)],metadata)

    def plan_shift(self, buffer, version, start, target):
        self.prepare(buffer,version)
        a,b=np.asarray(start),np.asarray(target)
        length=np.linalg.norm((b-a)[:2])
        first=self.map.point_safe_check(a,self.reference_height,relaxed=True)
        goal=self._admission(b)
        metadata=dict(self.metadata,start_check=first,goal_check=goal)
        if not first['ready']:
            return 'pct_unsafe_start',[],metadata
        if not .20-1e-6<=length<=.60+1e-6:
            return 'pct_invalid_recovery_shift',[],metadata
        # The short-action executor starts braking 7 cm before the target.
        early=self._admission(b-(b-a)*(.07/length))
        metadata['early_stop_check']=early
        if not goal['ready'] or not early['ready']:
            return 'pct_unsafe_shift_stop',[],metadata
        reason=self._reverse_check([a,b],level=first['ground_z'],
                                   level_failure='shift_exceeds_step',
                                   height_tolerance=min(.16,self.map.settings.step_max))
        return ('pct_'+reason,[],metadata) if reason else ('ok',[list(start),list(target)],metadata)
