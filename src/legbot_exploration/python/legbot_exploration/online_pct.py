"""Fixed-origin online map adapter for the project's native PCT A*.

Registered frames are fused once into sparse height evidence. Planning holds
its owner's read lock; raw frames and full planning-map snapshots are not kept.
"""
import ctypes
from dataclasses import replace
import importlib.util
import math
import os
from pathlib import Path
from statistics import median
import sys
import time
from .pct_frontier_index import DIRECTIONS
from .pct_frontiers import extract_frontiers
from .pct_platform import platform_cells
from .fixed_layer_map import FixedLayerMap, MapSettings
from .interval_map import IntervalLayerMap

import numpy as np


PLATFORM_HEIGHT_TOLERANCE = .05
SUPPORT_SEARCH_RADIUS = 3.4


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
        self.map = IntervalLayerMap(MapSettings(max_cells=max_cells,observer_reference_height=reference_height))
        self.version = None
        self.geometry_revision = 0
        self.planner = None
        self.build_ms = 0.0
        self.metadata = {}
        self._support_admissions={}
        self._support_passes={}
        self._ascent_selection=None

    def _ascent_layer(self, ground, minimum_height):
        """Rank a few cached cuts; keep one active until planning rejects it."""
        m,s=self.map,self.map.settings
        primary,_=m.frontier_selection_layer(ground)
        layers=[k for k in range(primary,primary+s.ascent_lookahead_slices) if k in m.layers]
        if not layers:layers=[primary]
        previous=self._ascent_selection
        generation=m.support_targets.revision
        refresh=(previous is None or math.dist(previous['anchor'][:2],ground[:2])>.25 or
                 abs(previous['anchor'][2]-ground[2])>s.surface_noise_tolerance or
                 previous['generation']!=generation or set(previous['layers'])!=set(layers))
        if refresh:
            same_anchor=(previous is not None and
                math.dist(previous['anchor'][:2],ground[:2])<=.25 and
                abs(previous['anchor'][2]-ground[2])<=s.surface_noise_tolerance and
                set(previous['layers'])==set(layers))
            retained_failed=set(previous['failed']) if same_anchor else set()
            retained_blockers=previous.get('blockers',{}) if same_anchor else {}
            supports=m.support_targets.candidates(ground,minimum_height,
                s.origin_z+max(layers)*s.slice_dh-1e-7,prefer_ascent=True)
            scores={k:-math.inf for k in layers}
            for _,_,_,point,_ in supports:
                xy=tuple(m.indices(point))
                for k in layers:
                    row=m.observed_cell((k,*xy))
                    if row is not None and not row['solid'] and abs(float(row['G'])-point[2])<1e-5:
                        scores[k]=max(scores[k],point[2])
            for k in layers:
                owner=m.frontier_index.slices.get(k)
                if owner is not None:
                    scores[k]=max(scores[k],max((float(c['frontier'][2]) for c in owner.clusters.values()
                        if float(c['frontier'][2])>=minimum_height),default=-math.inf))
            ordered=sorted(layers,key=lambda k:(-scores[k],k))
            previous=self._ascent_selection=dict(anchor=tuple(ground),generation=generation,
                layers=ordered,failed=retained_failed,selected=None,blockers=retained_blockers)
        # Preserve the current cut while its bounded search/route is pending.
        if previous['selected'] not in previous['layers'] or previous['selected'] in previous['failed']:
            previous['selected']=next((k for k in previous['layers'] if k not in previous['failed']),None)
        chosen=previous['selected']
        self.metadata['ascent_selection_slices']=dict(candidates=previous['layers'],selected=chosen,
            rejected=sorted(previous['failed']),height_limit=s.origin_z+max(layers)*s.slice_dh)
        return chosen

    def configure(self, settings):
        if self.map.accepted_frames:
            raise ValueError('pct_configuration_after_integration')
        self.map=IntervalLayerMap(MapSettings(**dict(vars(self.map.settings),**settings)))

    def integrate(self, buffer, sensor_origin=None, body_height=None, free_rays=None, query_height=None):
        count=self.map.integrate(np.frombuffer(buffer,dtype=np.float32).reshape(-1,3),
                                 sensor_origin=sensor_origin,body_height=body_height,
                                 free_rays=None if free_rays is None else np.frombuffer(free_rays,dtype=np.float32).reshape(-1,3))
        if self.map.error:
            raise ValueError(self.map.error)
        if query_height is not None:self.map.set_frontier_query_height(query_height)
        return count

    def _make_planner(self):
        from .block_pct import BlockPlanner
        return BlockPlanner(self.map,self.wrapper_type)

    def commit_pending(self, buffer, version, _unused_start=None):
        """Called by the map owner while holding its exclusive commit lock."""
        self._writer_commits_only=True
        self.prepare(buffer, version, allow_rebuild=True)
        if isinstance(self.map,FixedLayerMap):
            model=self.map.frontier_index.observation_model
            attempt=model.task.get('floor_attempt') if model.task else model.floor_attempt
            members=model.task.get('members') if model.task else attempt.members if attempt else None
            if members and not self.map.frontier_index.remaining_floor_members(members):
                if attempt is not None:
                    self.map.frontier_index.settle_floor_attempt(attempt,False,sufficient_progress=True)
                    model.floor_attempt=None
                return int(model.task['cluster'] if model.task else attempt.cluster)
        return 0

    def prepare(self, buffer, version, _unused_start=None, *, allow_rebuild=False):
        if isinstance(self.map,FixedLayerMap) and not allow_rebuild:
            if self.map._rebuild_pending:
                raise RuntimeError('pct_map_commit_failed')
            if getattr(self,'_writer_commits_only',False):
                if self.map.dirty or getattr(self.map,'frontier_query_pending',False):
                    raise RuntimeError('pct_map_commit_pending')
                # Expectations registered by readers can dirty frontier
                # metadata. They do not change physical geometry and are
                # flushed by the next writer, never rebuilt by this reader.
                if self.version==version and self.planner is not None:return
        # Static surfaces update only in response to incoming point evidence.
        if (self.version==version and self.planner is not None and not self.map.dirty and
                not self.map.evidence_dirty and not getattr(self.map,'frontier_query_pending',False)):
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
            backend='project_pct_interval_online',map_error=self.map.error,
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
            row=self.map.observed_cell((check['layer'],*self.map.indices(base)))
            if row is None or not row['measured'] or row['unknown']:
                return dict(check,ready=False,reason='tread_ground_unobserved')
            identity=check.get('surface_id',0)
            if not self.map.surface_observation.is_ground_source(tuple(self.map.indices(base)),identity):
                return dict(check,ready=False,reason='tread_ground_unobserved')
        return check

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
            main={}
            if ground is not None and selection is not None:
                layer,info=self.map.frontier_selection_layer(ground)
                main['selection_layer']=layer
                self.metadata['frontier_selection_slice']=dict(info,ground_height=float(ground[2]))
            policy={}
            if managed_cross and platform_floor is None:
                attempts,max_step,min_rise=floor_selection[4:]
                policy=dict(attempt_scope=('cross_layer',floor),max_attempts=attempts,
                            ascent_limits=(max_step,min_rise) if platform_floor is None else None)
            elif floor_selection is not None and len(floor_selection)>4:
                policy=dict(max_attempts=floor_selection[4])
            if seek_ascent:
                attempts,max_step,min_rise=floor_selection[4:]
                selected=self._ascent_layer(ground,floor[0]+floor[1]+1e-7)
                exhausted=selected is None
                main['selection_layer']=selected if selected is not None else self.map.frontier_selection_layer(ground)[0]
                self.metadata['frontier_selection_slice'].update(selected=main['selection_layer'],
                    address_height=self.map.settings.origin_z+main['selection_layer']*self.map.settings.slice_dh,
                    reason='ascent_slices_exhausted' if exhausted else 'ascent_lookahead')
                result=self.map.frontier_index.query(ground,minimum_distance,window_size,query_budget,
                    floor_selection=selection,attempt_scope=('floor_ascent',floor),max_attempts=attempts,
                    ascent_limits=(max_step,min_rise),evaluate_observers=not exhausted,**main)
                if exhausted:
                    result.update(clusters=[],query_pending=False,no_view_clusters=[],observer_timeout_clusters=[])
                result.update(upward_support_targets=[],support_query_pending=False,unknown_blockers=start_check.get('unknown_blockers',[]))
                if exhausted:
                    # Preserve the nearest failed cut's concrete footprint
                    # evidence instead of dropping it when the pass ends.
                    recorded=self._ascent_selection.get('blockers',{})
                    nearest=next((recorded[k] for k in sorted(recorded) if recorded[k]),[])
                    result['unknown_blockers']=self.map.merge_unknown_blockers(
                        result['unknown_blockers']+nearest,ground)
                # Observer search yields independently of already measured
                # upward supports and the blockers those supports expose.
                if ground is not None and not exhausted:
                    views,blockers,pending=self._support_views(ground,minimum_distance,
                        minimum_height=floor[0]+floor[1]+1e-7,budget=query_budget or .10,global_search=True,
                        selection_layer=selected)
                    result['upward_support_targets']=views
                    result['support_query_pending']=pending
                    result['query_pending'] |= pending and not views
                    result['unknown_blockers']=self.map.merge_unknown_blockers(result['unknown_blockers']+blockers,ground)
                    self._ascent_selection.setdefault('blockers',{})[selected]=result['unknown_blockers']
                return dict(result,version=version,map=self.metadata)
            if platform_floor is not None:
                result=self.map.frontier_index.query(ground,minimum_distance,window_size,query_budget,
                    floor=platform_floor,floor_selection=selection,
                    prune_disconnected=start_check.get('ready',False),fast=self.map.settings.fast_exploration,
                    max_attempts=2,ordinary_floor=False,**main)
                return dict(result,version=version,map=self.metadata)
            fast=self.map.settings.fast_exploration and floor is not None and continuation is None and not geometry_only
            result=self.map.frontier_index.query(ground,minimum_distance,window_size,query_budget,
                continuation,floor if continuation is None else None,(),
                evaluate_observers=not geometry_only,fast=fast,
                prune_disconnected=start_check.get('ready',False) and (floor is None or
                    abs(ground[2]-floor[0])<=floor[1]),
                floor_selection=selection,review_remaining=review_remaining,**policy,**main)
            result.update(upward_support_targets=[],unknown_blockers=start_check.get('unknown_blockers',[]))
            if ground is not None and continuation is not None:
                raised = floor is not None and ground[2] > floor[0] + floor[1] + 1e-7
                exhausted_frontiers = (not result['query_pending']
                                       and not any(c.get('observations') for c in result['clusters']))
                if raised or exhausted_frontiers:
                    views,blockers,pending=self._support_views(ground,minimum_distance,
                        minimum_height=continuation[1],budget=query_budget or .10,
                        global_search=window_size is None)
                    result['upward_support_targets']=views
                    result['query_pending'] |= pending and not views
                    result['unknown_blockers']=self.map.merge_unknown_blockers(
                        result['unknown_blockers']+blockers,ground)
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
        min_rise,max_rise = s.surface_noise_tolerance+1e-7,min(.16,s.step_max)+1e-7
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

    def _support_views(self, ground, minimum_distance, minimum_height, budget=.10, global_search=False,
                       selection_layer=None):
        """Validate cached upward supports through the public stopping check."""
        m,s=self.map,self.map.settings
        deadline=time.monotonic()+min(max(budget,.005),.10)
        low=max(minimum_height,ground[2]-s.surface_noise_tolerance)
        high=(ground[2]+min(.16,s.step_max)+1e-7 if selection_layer is None else
              s.origin_z+selection_layer*s.slice_dh-1e-7)
        radius=None if global_search else SUPPORT_SEARCH_RADIUS
        higher_surface = ground[2] + s.surface_noise_tolerance + 1e-7
        upward=m.support_targets.candidates(ground,max(low,higher_surface),high,minimum_distance,radius,
                                             prefer_ascent=selection_layer is not None)
        # Retain one finite candidate pass across map updates and timer ticks.
        # New geometry is checked when a candidate is visited; it must not
        # restart the cursor before the remaining candidates have had a turn.
        key=(selection_layer,round(minimum_height,3),global_search)
        progress=self._support_passes.get(key)
        if (progress is None or progress['cursor']>=len(progress['candidates']) or
                math.dist(progress['anchor'][:2],ground[:2])>.25 or
                abs(progress['anchor'][2]-ground[2])>s.surface_noise_tolerance):
            progress=dict(anchor=tuple(ground),candidates=upward,cursor=0,rejections={})
            self._support_passes[key]=progress
            while len(self._support_passes)>16:self._support_passes.pop(next(iter(self._support_passes)))
        candidates=progress['candidates']
        views=[];blockers=[];rejections=progress['rejections'];checked=0;first_admitted=None
        while progress['cursor']<len(candidates):
            if checked and time.monotonic()>=deadline:break
            _,_,_,point,k=candidates[progress['cursor']]
            progress['cursor']+=1;checked+=1
            if selection_layer is not None:
                row=m.observed_cell((selection_layer,*m.indices(point)))
                if row is None or row['solid'] or abs(float(row['G'])-point[2])>1e-5:continue
            body=(*point[:2],point[2]+self.reference_height)
            _,_,stamp=self._arrival_geometry(body,.15)
            cached=self._support_admissions.get(point)
            if cached is None or cached[0]!=stamp:
                check=self._arrival_area(body,.15,None)
                self._support_admissions[point]=(stamp,check)
                while len(self._support_admissions)>2048:
                    self._support_admissions.pop(next(iter(self._support_admissions)))
            else:check=cached[1]
            if not check['ready']:
                blockers.extend(check.get('unknown_blockers',()))
                rejections[point]=(check.get('unsafe_cells',1000),check.get('unobserved_cells',1000),
                                   point,check)
                continue
            rejections.pop(point,None)
            if any(math.dist(point[:2],p[:2])<.30 for p in views):continue
            if first_admitted is None:first_admitted=progress['cursor']-1
            heading=math.atan2(point[1]-ground[1],point[0]-ground[0])
            views.append([*point,heading])
            if len(views)>=6:break
        pending=progress['cursor']<len(candidates)
        checked_total=progress['cursor']
        # Keep an admitted batch stable while C++ resumes its bounded route
        # attempts. Only passes containing no safe stance advance unconditionally.
        if first_admitted is not None:progress['cursor']=first_admitted
        # Keep the best rejected stopping footprint explainable in live status.
        # This is bounded local evidence, not another map snapshot or query.
        def rejection_score(row):
            unsafe,unobserved,point,check=row
            known_barrier=('obstacle' in check.get('reason','') or
                bool(check.get('unsafe_cells',0)))
            return (known_barrier,math.dist(point[:2],ground[:2]),unobserved,point)
        rejected=[]
        for _,_,point,check in sorted(rejections.values(),key=rejection_score)[:2]:
            columns=[]
            for target in check.get('unknown_blockers',())[:3]:
                xy=tuple(map(int,m.indices(target)))
                column=m.columns.get(xy)
                columns.append(dict(xy=list(xy),records=[] if column is None else [
                    dict(id=r.id,height=r.height,low=r.low,high=r.high,orientation=r.orientation,
                         mature=r.mature,last_seen=r.last_seen,footprint=r.footprint)
                    for r in column.values() if abs(r.height-point[2])<=s.step_max+s.surface_noise_tolerance]))
            rejected.append(dict(point=list(point),reason=check.get('reason','stopping_check_rejected'),
                unobserved_cells=check.get('unobserved_cells'),unsafe_cells=check.get('unsafe_cells'),
                columns=columns))
        self.metadata['support_search']=dict(reference=list(map(float,ground)),candidates=len(candidates),
            admitted=len(views),pending=pending,checked=checked_total,batch_checked=checked,rejected=rejected)
        # Recover one concrete rejected support at a time. Combining all
        # footprints and keeping the nearest gaps can select unrelated walls.
        repairable=[r for r in rejections.values() if r[3].get('unknown_blockers') and
                    not rejection_score(r)[0]]
        best=min(repairable,key=rejection_score) if repairable else None
        focused=list(best[3].get('unknown_blockers',())) if best else blockers
        return views,m.merge_unknown_blockers(focused,ground),pending

    def complete_observation(self, buffer, version, cluster_id, revision, point):
        self.prepare(buffer,version)
        self.map.frontier_index.complete(cluster_id,revision,point)

    def refresh_frontier_displays(self, version, groups):
        """Filter selected marker snapshots against current unknown members."""
        self.prepare(b'',version)
        index=self.map.frontier_index;noise=self.map.settings.surface_noise_tolerance+1e-7
        result=[]
        for cid,points in groups:
            cluster=index.clusters.get(cid)
            members=cluster['members'] if cluster is not None else ()
            columns={}
            for key in members:columns.setdefault(key[1:3],[]).append(key)
            result.append([list(point) for point in points if any(
                abs(index.nodes[key].height-point[2])<=noise and
                not self.map.frontier_observed(key[0],key[1:3])
                for key in columns.get(tuple(map(int,self.map.indices(point))),()))])
        return result

    def observation_task(self, buffer, version, operation, cid, xyz, yaw, half_angle, blockers=None):
        self.prepare(buffer,version)
        if operation=='advance_ascent_slice':
            selection=self._ascent_selection
            if selection is None or selection['selected'] is None:
                return dict(ready=False,reason='ascent_slices_exhausted')
            selection['failed'].add(selection['selected'])
            remaining=[k for k in selection['layers'] if k not in selection['failed']]
            return dict(ready=bool(remaining),reason='try_next_ascent_slice' if remaining else 'ascent_slices_exhausted',
                        rejected=sorted(selection['failed']))
        model=self.map.frontier_index.observation_model
        ground=np.asarray(xyz,dtype=float)-[0,0,self.reference_height]
        if operation in ('begin','begin_flat','begin_floor','begin_forward','floor_attempt',
                         'floor_recovery_attempt','validate_fast_view','floor_execution_failed',
                         'arm','finish'):
            # A crouched or pitched body is not a new support surface. Use
            # the same measured G as scheduling; arrival still validates the
            # stopping footprint, blind area and visibility independently.
            ground=self._measured_ground(xyz)
        if operation=='floor_work_available':
            actual=self._measured_ground(np.asarray(xyz)+[0.,0.,self.reference_height])
            layer,_=self.map.frontier_selection_layer(actual)
            pending=self.map.frontier_index.floor_work_available((float(xyz[2]),half_angle),layer)
            return dict(ready=pending,reason='floor_work_available' if pending else 'floor_work_unchanged')
        if operation=='floor_task_pending':
            attempt=model.task.get('floor_attempt') if model.task else model.floor_attempt
            if attempt is None or attempt.cluster!=cid:
                return dict(ready=True,reason='frontier_task_untracked')
            pending=bool(self.map.frontier_index.remaining_floor_members(attempt.members))
            if not pending:
                self.map.frontier_index.settle_floor_attempt(attempt,False,sufficient_progress=True)
                model.floor_attempt=None
            return dict(ready=pending,reason='frontier_task_pending' if pending else 'frontier_resolved_in_transit')
        if operation=='enter_frontier_floor':
            evidence=self.map.surface_observation
            new=evidence.enter_frontier_floor(xyz,half_angle)  # xyz already contains measured ground height.
            return dict(ready=True,reason='frontier_floor_entered' if new else 'frontier_floor_revisited',
                        **evidence.status())
        if operation=='stair_heading':
            return dict(ready=True,reason='route_heading',yaw=float(yaw))

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
        if operation=='navigation_arrival':
            s=self.map.settings
            return model.pose_search._arrival_stop(xyz,s.robot_radius+s.safety_margin+half_angle)[2]
        if operation=='blocker_available':
            ready=model.blocker_review.available(ground)
            return dict(ready=ready,reason='blocker_review_available' if ready else 'blocker_already_reviewed')
        if operation=='begin_blocker_review':
            return model.blocker_review.begin(blockers or ())
        if operation=='arm_blocker_review':return model.blocker_review.arm()
        if operation=='blocker_review_progress':return model.blocker_review.progress()
        if operation=='finish_blocker_review':
            result=model.blocker_review.finish()
            # A fresh, matched return can repair an arrival footprint without
            # changing its G value or SupportTargets revision. Recheck cuts
            # on actual target evidence, never on an empty completed turn.
            if result.get('fresh') and result.get('gain',0)>0:
                self._ascent_selection=None
                self._support_passes.clear()
            return result
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
        if operation=='stair_step_progress':
            views,_,pending=self._support_views(ground,.25,ground[2]+.005)
            return dict(ready=bool(views),reason='upward_support_ready' if views else
                        'support_search_pending' if pending else 'support_unobserved')
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
            ready=gain>=1
            return dict(ready=ready,reason='ready' if ready else 'no_visible_frontier',gain=gain)
        if operation=='floor_execution_failed':
            attempt=model.task.get('floor_attempt') if model.task is not None else model.floor_attempt
            outcome=(self.map.frontier_index.settle_floor_attempt(attempt,True,tuple(ground))
                     if attempt is not None and attempt.cluster==cid else 'untracked')
            return dict(ready=outcome in ('first_no_progress','deferred','blacklisted','cluster_changed'),
                        reason=operation,floor_attempt=outcome)
        if operation in ('floor_no_observation','floor_observer_timeout','floor_disconnected','floor_route_timeout',
                         'floor_no_route','floor_missing_connection','floor_known_barrier','floor_missing_observation'):
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

    def _platform_patch(self, ground, minimum_area=None, reference=None):
        reference=float(ground[2]) if reference is None else reference
        heights={}
        def supported(x,y):
            found=min(self._landing_heights((x,y)),key=lambda z:abs(z-reference),default=None)
            if found is None or abs(found-reference)>PLATFORM_HEIGHT_TOLERANCE+1e-7:return False
            heights[x,y]=found
            return True
        cells=platform_cells(self.map.settings,tuple(self.map.indices(ground)),supported,minimum_area)
        return cells,heights

    def _platform_seed(self, ground):
        s=self.map.settings
        footprint_area = math.pi * (s.robot_radius + s.safety_margin)**2
        area = min(s.landing_min_area, max(s.resolution**2, footprint_area, s.landing_min_area / 2))
        cells,_=self._platform_patch(ground,area)
        return len(cells)*s.resolution**2+1e-7>=area

    def platform_check(self, buffer, version, point, floor=None):
        self.prepare(buffer,version)
        center=self._admission(point,floor=floor)
        if not center['ready']:return center
        ground=np.asarray((*point[:2],center['ground_z']))
        cells,_=self._platform_patch(ground)
        ready=len(cells)*self.map.settings.resolution**2+1e-7>=self.map.settings.landing_min_area
        return dict(center,ready=ready,reason='safe_platform' if ready else 'platform_not_confirmed')

    def landing_check(self, buffer, version, point, departure_floor, heading=0., reference_height=None):
        self.prepare(buffer,version)
        center=self._admission(point,height_tolerance=PLATFORM_HEIGHT_TOLERANCE)
        return self._landing_geometry(point,departure_floor,heading,center,reference_height)

    def descending_landing_check(self, buffer, version, point, departure_floor, heading=0.):
        self.prepare(buffer,version)
        center=self._admission(point,height_tolerance=PLATFORM_HEIGHT_TOLERANCE)
        return self._landing_geometry(point,departure_floor,heading,center,direction=-1)

    def landing_recovery_check(self, buffer, version, point, departure_floor, heading=0., reference_height=None):
        self.prepare(buffer,version)
        center=self._admission(point,relaxed=True,height_tolerance=.04)
        return self._landing_geometry(point,departure_floor,heading,center,reference_height)

    def _landing_geometry(self, point, departure_floor, heading, center, reference_height=None, *,direction=1):
        if not center['ready']:return center
        ground=center['ground_z'];boundary=departure_floor[0];minimum=departure_floor[1]
        if direction*(ground-boundary)<=minimum+1e-7:
            return dict(center,ready=False,reason='still_on_departure_floor')
        reference=ground if reference_height is None else reference_height
        if abs(ground-reference)>PLATFORM_HEIGHT_TOLERANCE+1e-7:
            return dict(center,ready=False,reason='platform_height_changed',platform_reference=reference)
        cells,heights=self._platform_patch((*point[:2],ground),reference=reference)
        area=len(cells)*self.map.settings.resolution**2
        if area+1e-7<self.map.settings.landing_min_area:
            return dict(center,ready=False,reason='platform_not_confirmed',platform_area=area)
        return dict(center,reason='safe_platform',platform_height=median(heights[xy] for xy in cells),platform_area=area)

    def _landing_heights(self, xy):
        mapping = self.map
        return {float(row['G']) for k, row in mapping.layers_at(*xy)
                if row['measured'] and mapping.observed_cell((k, *xy)) is not None
                and not row['solid'] and 0 <= mapping.planning_cost((k, *xy)) <= 20
                and mapping.surface_observation.is_ground_source(xy, int(row['g_surface_id']))}

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
