"""Fixed-origin sparse blocks. One mutable map, no planning snapshots.

The owner holds its write lock for integrate/build and read lock for queries.
Dense export is explicitly diagnostic; production consumers read blocks.
"""
from bisect import bisect_left
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, wait
from dataclasses import dataclass, fields
import os
import math
import time
import numpy as np
from pct_layer_selection import rank_layers
from .map_evidence import SurfaceColumn, changed_slice_indices, same_ground_surface


@dataclass(frozen=True)
class MapSettings:
    resolution: float = .10
    slice_dh: float = .50
    origin_x: float = 0.
    origin_y: float = 0.
    origin_z: float = 0.
    block_size: int = 32
    mapping_mode: str = 'static'
    slice_update_workers: int = 0  # Auto: up to four workers on available CPUs.
    maturity_min_points: int = 3
    surface_noise_tolerance: float = .05
    new_height_threshold: float = .10
    surface_normal_radius: float = .25
    surface_normal_min_points: int = 6
    surface_normal_vertical: float = .5
    surface_normal_thickness: float = .03
    relief_update_threshold: float = .25
    terrain_update_threshold: float = .25
    inflation_update_threshold: float = .25
    planning_budget_seconds: float = 1.5
    observation_revisit_radius: float = 1.0
    floor_min_resolved_frontier_cells: int = 5
    fast_exploration: bool = False
    fast_min_visible_cells: int = 3
    floor_failed_small_cluster_max_cells: int = 0
    floor_global_review_enabled: bool = True
    floor_four_direction_search: bool = False
    floor_direction_search_timeout: float = 1.5
    landing_support_radius: float = .60
    landing_min_area: float = 1.50
    landing_min_width: float = .80
    landing_min_depth: float = 1.00
    # Shared with the actual auxiliary sensor's xacro configuration.
    ground_lidar_x: float = .35
    ground_lidar_z: float = .20
    ground_lidar_pitch: float = .6108652381980153
    ground_lidar_horizontal_half_angle: float = .7853981633974483
    ground_lidar_elevation_min: float = -.0872664625997165
    ground_lidar_elevation_max: float = .3490658503988659
    ground_lidar_max_range: float = 12.
    ground_lidar_min_range: float = .70
    observer_reference_height: float = .32
    robot_radius: float = .35
    safety_margin: float = .05
    minimum_clearance: float = .50
    desired_clearance: float = .55
    step_max: float = .16
    slope_max: float = .40  # tangent, not radians
    traversability_kernel_size: int = 7
    standable_ratio: float = .5
    inflation_kernel_size: int = 13
    max_surface_records: int = 2000000
    max_cells: int = 20000000

    def __post_init__(self):
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, (int, float)) and not np.isfinite(value):
                raise ValueError('pct_nonfinite_configuration')
        if not (0 < self.ground_lidar_horizontal_half_angle <= math.pi and
                -math.pi/2 <= self.ground_lidar_elevation_min < self.ground_lidar_elevation_max <= math.pi/2 and
                self.ground_lidar_max_range > 0 and self.ground_lidar_min_range >= 0):
            raise ValueError('pct_invalid_observation_sensor')
        if self.mapping_mode != 'static':
            raise ValueError('pct_invalid_mapping_policy')
        if min(self.resolution, self.slice_dh, self.minimum_clearance, self.slope_max) <= 0:
            raise ValueError('pct_invalid_resolution')
        nonnegative = ('robot_radius', 'safety_margin', 'surface_noise_tolerance',
            'relief_update_threshold', 'terrain_update_threshold', 'inflation_update_threshold',
            'observation_revisit_radius')
        if min(getattr(self, n) for n in nonnegative) < 0 or self.desired_clearance < self.minimum_clearance:
            raise ValueError('pct_invalid_geometry_or_threshold')
        if self.surface_noise_tolerance >= self.minimum_clearance:
            raise ValueError('pct_surface_noise_tolerance_exceeds_clearance')
        if self.new_height_threshold <= 0:
            raise ValueError('pct_invalid_new_height_threshold')
        if self.surface_normal_radius<=0 or self.surface_normal_thickness<=0 or not 0<self.surface_normal_vertical<=1:
            raise ValueError('pct_invalid_surface_orientation')
        if int(self.slice_update_workers)!=self.slice_update_workers or not 0<=self.slice_update_workers<=16:
            raise ValueError('pct_invalid_update_workers')
        if int(self.surface_normal_min_points)!=self.surface_normal_min_points or not 3<=self.surface_normal_min_points<=12:
            raise ValueError('pct_invalid_surface_normal_samples')
        if self.planning_budget_seconds <= 0:
            raise ValueError('pct_invalid_planning_budget')
        if (not 0<=self.floor_failed_small_cluster_max_cells or
                int(self.floor_failed_small_cluster_max_cells)!=self.floor_failed_small_cluster_max_cells):
            raise ValueError('pct_invalid_floor_failed_small_cluster_max_cells')
        if not math.isfinite(self.floor_direction_search_timeout) or self.floor_direction_search_timeout<=0:
            raise ValueError('pct_invalid_floor_direction_search_timeout')
        if min(self.landing_support_radius,self.landing_min_area,
               self.landing_min_width,self.landing_min_depth) <= 0:
            raise ValueError('pct_invalid_landing_support')
        for n in ('block_size', 'maturity_min_points', 'floor_min_resolved_frontier_cells',
                  'fast_min_visible_cells',
                  'max_surface_records', 'max_cells', 'traversability_kernel_size', 'inflation_kernel_size'):
            value = getattr(self, n)
            if int(value) != value or value < 1:
                raise ValueError('pct_invalid_integer_setting:' + n)
        if self.traversability_kernel_size % 2 != 1 or self.inflation_kernel_size % 2 != 1 or not 0 <= self.standable_ratio <= 1:
            raise ValueError('pct_invalid_kernel')



FLOATS = ('G', 'C', 'pending_ceiling_z', 'passage_cost', 'relief_cost', 'terrain_cost', 'inflated_terrain_cost',
          'gx', 'gy', 'q', 'm', 'terrain_inflation_reference')
CELL_DTYPE = np.dtype([(n, '<f8') for n in FLOATS] +
    [('g_surface_id', '<i8'), ('c_surface_id', '<i8')] + [
    ('g_mature', '?'), ('c_mature', '?'),
    ('measured', '?'), ('flat', '?'), ('feature_valid', '?'),
    ('solid', '?'), ('unknown', '?'), ('hard', '?'), ('solid_hard', '?'), ('unknown_hard', '?')])


def different(a, b):
    return not (a == b or (np.isnan(a) and np.isnan(b)))


def cost_changed(old, new, threshold):
    if old == new:
        return False
    category = lambda v: -1 if v < 0 else (2 if v >= 50 else (1 if v > 20 else 0))
    return category(old) != category(new) or abs(old-new) > threshold


class FixedLayerMap:
    def __init__(self, settings=None, clock=time.monotonic, *, update_workers=None, native_updates=True):
        self.settings = settings or MapSettings()
        self.clock = clock
        self.columns, self.column_layers, self.blocks = {}, {}, {}
        self._column_geometry = {}
        self.layers = {}
        self.counts = {}
        self._auto_workers=update_workers is None and self.settings.slice_update_workers==0
        if update_workers is None:
            available=len(os.sched_getaffinity(0))
            update_workers=min(available,self.settings.slice_update_workers or 4)
        if int(update_workers)!=update_workers or not 1<=update_workers<=16:
            raise ValueError('pct_invalid_update_workers')
        self.update_workers=int(update_workers)
        self._pool=None
        self._cost_kernel=None
        self._geometry_kernel=None
        self._boundary_kernel=None
        self._rebuild_pending=False
        if native_updates:
            try:
                from _legbot_layer_kernels import update_geometry, update_costs, find_boundaries
            except ModuleNotFoundError as exc:
                if exc.name != '_legbot_layer_kernels':
                    raise
            else:
                self._cost_kernel=update_costs
                self._geometry_kernel=update_geometry
                self._boundary_kernel=find_boundaries
        # Local cut demands persist even where no column has been observed
        # yet. They contain no heights or occupancy and create no surfaces.
        self._native_slice_masks, self._required_slice_masks = {}, {}
        self.surface_records = self.accepted_points = self.accepted_frames = 0
        self.evidence_revision = self.revision = 0
        self.last_sensor_time = self.last_integration_time = self.last_commit_time = None
        self.dirty, self.geometry_changed = set(), set()
        self.evidence_dirty = set()
        self._floor_ground_pending = {}
        self._floor_ceiling_pending = {}
        self._floor_ground_refresh = defaultdict(set)
        self._new_floor_bands = set()
        self.error, self.stats, self.last_work = '', {}, {}
        self.lower = self.upper = self.first_layer = self.last_layer = None
        self.ignored_static_points = 0
        self.integrated_serial = 0
        self.orientation_counts={}
        self.orientation_ms=0.

        from .surface_relations import SurfaceRelations
        self.surface_relations=SurfaceRelations(self)
        from .surface_observation import SurfaceObservation
        self.surface_observation = SurfaceObservation(self)
        from .pct_frontier_index import FrontierIndex
        self.frontier_index = FrontierIndex(self)
        from .pct_stairs import StairIndex, StairCandidateCache
        self.stair_candidates=StairCandidateCache(self.settings)
        self.stair_index = StairIndex()
        self.stair_scout_index = StairIndex(include_unclassified=True)
        from .pct_connectivity import ConnectivityIndex
        self.connectivity=ConnectivityIndex(self)
        self._floor_connectivity=None
        self.planning_block_versions={}

    def floor_connectivity(self, floor):
        from .pct_connectivity import ConnectivityIndex
        if self._floor_connectivity is None or self._floor_connectivity.floor!=floor:
            self._floor_connectivity=ConnectivityIndex(self,floor)
            self._floor_connectivity.update_tiles({(x,y) for _,x,y in self.blocks})
        return self._floor_connectivity

    def ground_pending(self, xy, layer):
        return layer in self._floor_ground_pending.get(xy,())

    def schedule_floor_ground(self, band):
        self._new_floor_bands.add(band)

    @property
    def floor_ground_update_pending(self):
        return bool(self._new_floor_bands or self._floor_ground_refresh)

    def _queue_floor_ground_resets(self):
        """Queue the affected G and C of existing cells on both sides of the floor."""
        b=self.settings.block_size
        for band in self._new_floor_bands:
            for (layer,bx,by),block in self.blocks.items():
                ground=block['G']
                if layer>=band.first_slice:
                    affected=block['measured'] & (~np.isfinite(ground) | (ground<=band.upper_ground+1e-7))
                else:
                    demand=self._required_slice_masks.get((bx,by),{}).get(band.first_slice)
                    if demand is None:
                        continue
                    ceiling=block['C']
                    affected=demand & np.isfinite(ground) & (
                        ~np.isfinite(ceiling) | (ceiling>=band.height-self.settings.surface_noise_tolerance))
                for x,y in np.argwhere(affected):
                    xy=(bx*b+int(x),by*b+int(y))
                    self._floor_ground_refresh[xy].add(layer)

    def _has_lower_ground_at_arrival(self, layers, records, band):
        first = bisect_left(layers, band.first_slice)
        if not first:
            return False
        cut = self.settings.origin_z+layers[first-1]*self.settings.slice_dh
        # An allocated lower cut without a ground sample does not count.
        return any(record.orientation != -1 and record.height <= cut+1e-7 and
                   record.first_seen <= band.arrival_frame for record in records)

    def _working_surfaces(self, xy, layer, ground, ceiling, layers, records):
        """Select the committed G/C for this cut from cumulative evidence."""
        height=ground.height if ground is not None else math.nan
        band=next((scope for scope in self.surface_observation.ground_bands
                   if scope.includes(layer,height)),None)
        pending = band is not None and not band.ground_seen(ground) and (
            ground is None or self._has_lower_ground_at_arrival(layers, records, band))
        if pending:
            self._floor_ground_pending.setdefault(xy,set()).add(layer)
            self._clear_pending_cut(self._floor_ceiling_pending,xy,layer)
            return None,None,math.nan
        self._clear_pending_cut(self._floor_ground_pending,xy,layer)
        pending_ceiling=self._pending_ceiling(xy,layer,ground,ceiling)
        if math.isfinite(pending_ceiling):
            self._floor_ceiling_pending.setdefault(xy,set()).add(layer)
        else:
            self._clear_pending_cut(self._floor_ceiling_pending,xy,layer)
        return ground,ceiling,pending_ceiling

    @staticmethod
    def _clear_pending_cut(pending,xy,layer):
        layers=pending.get(xy)
        if layers:
            layers.discard(layer)
            if not layers:del pending[xy]

    def _pending_ceiling(self,xy,layer,ground,ceiling):
        if ground is None:
            return math.nan
        s=self.settings
        x,y=xy
        demands=self._required_slice_masks.get((x//s.block_size,y//s.block_size),{})
        for band in reversed(self.surface_observation.ground_bands):
            if layer>=band.first_slice or ground.height>=band.height-s.surface_noise_tolerance:
                continue
            affected=demands.get(band.first_slice)
            if affected is None or not affected[x%s.block_size,y%s.block_size]:
                continue
            if ceiling is not None:
                difference=ceiling.height-band.height
                if difference < -s.surface_noise_tolerance:
                    break
                if abs(difference)<=s.surface_noise_tolerance and (
                        ceiling.orientation==-1 or ceiling.last_seen>band.cutoff):
                    break
            if self.surface_observation.ceiling_band_seen_free(xy,band.height,band.cutoff):
                continue
            return band.height
        return math.nan

    def clearance_needs_observation(self,row):
        return (row is not None and math.isfinite(row['pending_ceiling_z']) and
                row['pending_ceiling_z']-row['G']<self.settings.minimum_clearance and
                (not math.isfinite(row['C']) or row['C']-row['G']>=self.settings.minimum_clearance))

    def cell(self, key, create=False):
        k, x, y = map(int, key)
        b = self.settings.block_size
        block_key = (k, x//b, y//b)
        block = self.blocks.get(block_key)
        if block is None:
            if not create:
                return None
            if (len(self.blocks)+1)*b*b > self.settings.max_cells:
                raise ValueError('pct_map_cell_limit')
            block = self._allocate_block(block_key)
        return block[x % b, y % b]

    def _allocate_block(self, key):
        from .tomographic_slice import TomographicSlice
        k,bx,by = key
        layer = self.layers.get(k)
        if layer is None:
            layer = self.layers[k] = TomographicSlice(self,k)
        block = layer.allocate((bx,by))
        self.blocks[key] = block  # PCT's existing sparse accessor borrows the same arrays.
        return block

    def _run_slice_tasks(self, operation, inputs):
        workers = min(self.update_workers,len(inputs))
        # At small batch sizes, Python marshalling and thread wakeups
        # outweigh the native parallel work on recorded LiDAR frames.
        if self._auto_workers and (
                self._cost_kernel is None or sum(len(args[1]) for args in inputs)<8192):
            workers = min(workers,1)
        if workers<2:
            return [operation(*args) for args in inputs],workers
        if self._pool is None:
            self._pool = ThreadPoolExecutor(max_workers=self.update_workers, thread_name_prefix='pct-slice')
        futures = []
        try:
            for args in inputs:
                futures.append(self._pool.submit(operation,*args))
        finally:
            # Drain every writer even if submitting or running a job fails.
            wait(futures)
        return [future.result() for future in futures],workers

    def iter_cells(self, ground_only=False):
        b = self.settings.block_size
        for (k, bx, by), block in self.blocks.items():
            mask = np.isfinite(block['G']) if ground_only else block['measured']
            for i, j in np.argwhere(mask):
                yield (k, bx*b+int(i), by*b+int(j)), block[i, j]

    def layers_at(self, x, y):
        for k in sorted(self.column_layers.get((int(x), int(y)), ())):
            row = self.cell((k, x, y))
            if row is not None:
                yield k, row

    def indices(self, xyz):
        s = self.settings
        return np.floor((np.asarray(xyz[:2])-[s.origin_x, s.origin_y])/s.resolution+.5).astype(int)

    def position(self, key):
        _, x, y = key
        return [self.settings.origin_x+x*self.settings.resolution,
                self.settings.origin_y+y*self.settings.resolution]

    def integrate(self, points, now=None, sensor_origin=None):
        now = self.clock() if now is None else float(now)
        self.last_sensor_time = now
        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        if not len(points):
            return self.surface_records
        s=self.settings
        # Origin-less calls are the explicit offline, prelabelled-surface API.
        # Production range-filtered LiDAR input always supplies its paired pose.
        orientations=np.ones(len(points),dtype=np.int8)
        started=time.perf_counter()
        if sensor_origin is not None:
            origin=np.asarray(sensor_origin,dtype=float)
            if origin.shape!=(3,) or not np.isfinite(origin).all():
                raise ValueError('pct_invalid_sensor_origin')
            from .surface_orientation import classify
            orientations=classify(points,origin,s.surface_normal_radius,s.surface_normal_min_points,
                                  s.surface_normal_vertical,s.surface_normal_thickness)
        cell_xy=(points[:,:2]-[s.origin_x,s.origin_y])/s.resolution+.5
        xy=np.floor(cell_xy).astype(np.int64)
        footprints=tops=None
        if sensor_origin is not None:
            subcells=np.minimum(3,((cell_xy-xy)*4).astype(np.int64))
            footprints=1 << (subcells[:,0]+4*subcells[:,1])
            tops=np.ceil((points[:,2].astype(float)-1e-7)*100).astype(np.int64)
        keys,inv=np.unique(xy,axis=0,return_inverse=True)
        order=np.argsort(inv,kind='stable')
        boundaries=np.r_[0,np.cumsum(np.bincount(inv))]
        self.orientation_ms=1000*(time.perf_counter()-started)
        self.orientation_counts={name:int((orientations==value).sum()) for name,value in
                                 (('support',1),('underside',-1),('vertical',-2),('ambiguous',0))}
        # Near capacity, stage only touched columns to reject an over-capacity
        # frame atomically. Normal fusion modifies the one live map directly.
        staged={} if self.surface_records+len(points)>s.max_surface_records else None
        total=self.surface_records
        used=0;changed=set()
        evidence=self.surface_observation
        for i,xy_key in enumerate(keys):
            key=tuple(map(int,xy_key))
            existing=self.columns.get(key)
            if staged is not None:
                column=existing.clone() if existing else SurfaceColumn()
            else:
                column=existing
                if column is None:
                    column=self.columns[key]=SurfaceColumn()
            before=len(column)
            accepted=0
            for point_id in order[boundaries[i]:boundaries[i+1]]:
                z=points[point_id,2]
                previous=len(column)
                accepted+=column.add(float(z),s.surface_noise_tolerance,s.maturity_min_points,
                                     self.integrated_serial+1,int(orientations[point_id]),
                                     int(footprints[point_id]) if footprints is not None else 0,
                                     int(tops[point_id]) if tops is not None else None)
                total+=len(column)-previous
                if total>s.max_surface_records:
                    self.error='pct_surface_record_limit'
                    return self.surface_records
            if staged is not None:
                staged[key]=column
            used+=accepted
            if accepted or len(column)!=before:
                changed.add(key)
        if staged is not None:
            self.columns.update(staged)
        self.integrated_serial += 1
        # A frozen surface can get a new hit without changing its geometry.
        if self._floor_ground_pending or self._floor_ceiling_pending:
            for x,y in keys:
                xy=(int(x),int(y))
                if xy in self._floor_ground_pending:
                    self._floor_ground_refresh[xy].update(self._floor_ground_pending[xy])
                if xy in self._floor_ceiling_pending:
                    self._floor_ground_refresh[xy].update(self._floor_ceiling_pending[xy])
        affected=self.surface_relations.invalidate(changed)
        self.evidence_dirty.update(affected)
        self.surface_observation.refresh_relations(affected)
        observed=self.surface_observation.observe_frame(
            points,None if sensor_origin is None else origin,self.integrated_serial,orientations)
        self.evidence_dirty.update(observed)
        for xy in observed:
            if xy in self._floor_ceiling_pending:
                self._floor_ground_refresh[xy].update(self._floor_ceiling_pending[xy])
        self.surface_records=total
        self.dirty.update(changed)
        self.ignored_static_points+=len(points)-used
        if used:
            self.accepted_frames+=1
            self.accepted_points+=used
            self.evidence_revision+=1
        self.last_integration_time=now
        self.error=''
        return self.surface_records

    def build(self, now=None):
        try:
            if self._rebuild_pending:
                self._reset_slice_products()
            result=self._build(now)
        except Exception:
            # Retain fused evidence and rebuild from it after a failed writer.
            self._rebuild_pending=True
            raise
        self._rebuild_pending=False
        return result

    def _reset_slice_products(self):
        self.blocks.clear()
        self.layers.clear()
        self.column_layers.clear()
        self._column_geometry.clear()
        self._floor_ground_pending.clear()
        self._floor_ceiling_pending.clear()
        self._floor_ground_refresh.clear()
        self._native_slice_masks.clear()
        self._required_slice_masks.clear()
        self.planning_block_versions.clear()
        self.lower=self.upper=self.first_layer=self.last_layer=None
        self.dirty=set(self.columns)
        self.frontier_index.reset_geometry()
        from .pct_connectivity import ConnectivityIndex
        from .pct_stairs import StairIndex
        self.connectivity=ConnectivityIndex(self)
        self._floor_connectivity=None
        self.stair_index=StairIndex()
        self.stair_scout_index=StairIndex(include_unclassified=True)

    def _build(self, now):
        now = self.clock() if now is None else now
        if not self.dirty and not self.evidence_dirty and not self.floor_ground_update_pending:
            if self.first_layer is None:
                raise ValueError('pct_empty_cloud')
            self.last_work = dict(base_cells=0, feature_cells=0, terrain_cells=0, inflation_cells=0,
                                  hard_cells=0, base_update_ms=0.,cost_update_ms=0.,frontier_update_ms=0.,
                                  slice_workers=0,frontier_workers=0,native_slice_costs=self._cost_kernel is not None,
                                  geometry_candidate_cells=0,geometry_update_cells=0,geometry_skipped_cells=0)
            self.frontier_index.commit([])
            return False
        s = self.settings
        from .map_costs import expand_layers
        started=time.perf_counter()
        self._queue_floor_ground_resets()
        evidence_columns=set(self.dirty) | self.evidence_dirty
        work=expand_layers(self,self.dirty)
        batches=defaultdict(dict)
        pending_geometry={}
        frontiers=defaultdict(set)
        geometry_candidate_cells=0
        missing_blocks=set()
        b=s.block_size
        frontier_resets=defaultdict(set)
        for xy in work | self._floor_ground_refresh.keys():
            column=self.columns.get(xy)
            if not column:
                continue
            geometry=column.observed_geometry(s.surface_noise_tolerance)
            _,records=geometry
            signature=tuple((r.height,r.id,r.mature,r.orientation) for r in records)
            layers=tuple(sorted(self.column_layers[xy]))
            previous,previous_layers=self._column_geometry.get(xy,((),()))
            changed=set(changed_slice_indices(previous,signature,layers,previous_layers,s.origin_z,s.slice_dh))
            changed.update(self._floor_ground_refresh.get(xy,()))
            added=set(layers).difference(previous_layers)
            for band in self.surface_observation.ground_bands:
                if band.first_slice in added:
                    changed.update(k for k in layers if k<band.first_slice)
            pending_geometry[xy]=(signature,layers)
            geometry_candidate_cells+=len(layers)
            # Frontier observations can change while each retained cut keeps the same G/C.
            if xy in work:
                for k in layers:
                    frontiers[k].add(xy)
            for k in changed:
                ground,ceiling=column.at(s.origin_z+k*s.slice_dh,geometry,s.surface_noise_tolerance)
                was_pending=self.ground_pending(xy,k)
                ground,ceiling,pending_ceiling=self._working_surfaces(xy,k,ground,ceiling,layers,records)
                if was_pending!=self.ground_pending(xy,k):
                    frontiers[k].add(xy)
                    frontier_resets[k].add(xy)
                surfaces=tuple((r.height,r.id,r.mature) if r else (np.nan,0,False)
                               for r in (ground,ceiling))
                batches[k][xy]=(*surfaces,pending_ceiling)
                block_key=(k,xy[0]//b,xy[1]//b)
                if block_key not in self.blocks:
                    missing_blocks.add(block_key)
        if (len(self.blocks)+len(missing_blocks))*b*b>s.max_cells:
            raise ValueError('pct_map_cell_limit')
        for key in sorted(missing_blocks):
            self._allocate_block(key)
        prepared=time.perf_counter()

        def update_costs(layer, evidence):
            return layer,layer.update_costs(evidence,self._cost_kernel,self._geometry_kernel)

        results,cost_workers=self._run_slice_tasks(update_costs, [(self.layers[k],batches[k]) for k in sorted(batches)])
        costs_done=time.perf_counter()
        self.last_work=dict(base_cells=0,feature_cells=0,terrain_cells=0,inflation_cells=0,hard_cells=0)
        self.geometry_changed=set()
        for layer,(base,changes,cost_work) in results:
            self.geometry_changed.update((layer.index,*xy) for xy in base | changes)
            for name,count in cost_work.items():
                self.last_work[name]+=count
        # A hit can resolve a frontier even if its cut's G/C did not change.
        # Include cuts which have a local dependency on the touched column.
        for layer,(_,changes,_) in results:
            frontiers[layer.index].update(changes)
        for x,y in evidence_columns | work:
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    for k in self.column_layers.get((x+dx,y+dy),()):
                        frontiers.setdefault(k,set()).add((x,y))
            for key in self.frontier_index.by_column.get((x,y),()):
                frontiers.setdefault(key[0],set()).add((x,y))
        def update_frontiers(index, columns):
            return index,index.update(columns,self._boundary_kernel,frontier_resets.get(index.layer,()))

        frontier_results,frontier_workers=self._run_slice_tasks(update_frontiers,
            [(self.layers[k].frontiers,columns) for k,columns in sorted(frontiers.items()) if k in self.layers])
        self.frontier_index.commit(frontier_results)
        frontier_done=time.perf_counter()
        self.frontier_index.last_work['update_ms']=1000*(frontier_done-costs_done)
        self.counts={}
        for layer in self.layers.values():
            for name,count in layer.counts.items():
                self.counts[name]=self.counts.get(name,0)+count
        self.connectivity.update(self.geometry_changed)
        if self._floor_connectivity is not None:
            self._floor_connectivity.update(self.geometry_changed)
        scout_columns = evidence_columns | {(x,y) for _,x,y in self.geometry_changed}
        self.stair_index.update(self,scout_columns)
        self.stair_scout_index.update(self,scout_columns)
        qualification_columns={(x,y) for _,x,y in self.geometry_changed}
        qualification_columns.update(xy for xy,geometry in pending_geometry.items()
                                     if self._column_geometry.get(xy)!=geometry)
        self.stair_candidates.invalidate(qualification_columns)
        revision=self.revision+bool(self.geometry_changed)
        for key in {(k,x//b,y//b) for k,x,y in self.geometry_changed}:
            self.planning_block_versions[key]=revision
        self.last_work.update(base_update_ms=1000*(prepared-started),
            cost_update_ms=1000*(costs_done-prepared),frontier_update_ms=1000*(frontier_done-costs_done),
            slice_workers=cost_workers,frontier_workers=frontier_workers,
            geometry_candidate_cells=geometry_candidate_cells,
            geometry_update_cells=sum(len(batch) for batch in batches.values()),
            geometry_skipped_cells=geometry_candidate_cells-sum(len(batch) for batch in batches.values()),
            native_slice_costs=self._cost_kernel is not None)
        self.stats = dict(surface_records=self.surface_records, accepted_points=self.accepted_points,
            accepted_frames=self.accepted_frames, evidence_version=self.evidence_revision,
            geometry_version=revision, allocated_blocks=len(self.blocks),
            map_cells=len(self.blocks)*s.block_size*s.block_size,
            retained_slices=0 if self.first_layer is None else self.last_layer-self.first_layer+1,
            mapping_mode=s.mapping_mode, maturity_min_points=s.maturity_min_points,
            surface_noise_tolerance=s.surface_noise_tolerance,
            ignored_static_points=self.ignored_static_points,
            orientation_ms=self.orientation_ms,orientation_points=self.orientation_counts,
            connectivity=self.connectivity.last_work,
            update_mode='incremental_blocks' if self.geometry_changed else 'unchanged_geometry', **self.last_work)
        self.revision=revision
        if self.geometry_changed:
            self.last_commit_time=now
        self._column_geometry.update(pending_geometry)
        self.dirty.clear()
        self.evidence_dirty.clear()
        self._floor_ground_refresh.clear()
        self._new_floor_bands.clear()
        return bool(self.geometry_changed)

    def point_safe_check(self, xyz, reference_height=.32, relaxed=False, *,
                         floor=None, height_tolerance=None, collect_blockers=False):
        x, y = self.indices(xyz)
        requested = xyz[2]-reference_height
        result = dict(ready=False, cell=[int(x), int(y)], requested_ground_z=float(requested), layer=0)
        options = {int(k): row for k, row in self.layers_at(x, y) if np.isfinite(row['G'])}
        def missing_ground(result):
            if collect_blockers and self.first_layer is not None and math.isfinite(requested):
                s = self.settings
                k = math.floor((requested-s.origin_z)/s.slice_dh)+1
                k = min(self.last_layer, max(self.first_layer, k))
                result['unknown_blockers'] = list(self.unknown_blockers(k,[(x,y)],requested))
            return result

        if not options:
            return missing_ground(dict(result, reason='no_ground_sample'))
        tolerance = self.settings.step_max+1e-5
        if height_tolerance is not None:
            tolerance = min(tolerance, height_tolerance+1e-7)
        checked = {}
        unknown_footprints = {}

        def read_height(layer):
            return float(options[layer]['G'])

        def describe(k):
            row = options[k]
            ceiling=row['C']
            return dict(result, layer=int(k), ground_z=float(row['G']),
                cost=float(row['inflated_terrain_cost']), source=1,
                solid=bool(row['solid']), unknown_inflated=bool(row['hard']),
                clearance=float(ceiling-row['G']) if np.isfinite(ceiling) else None,
                clearance_observed=bool(np.isfinite(ceiling) and not np.isfinite(row['pending_ceiling_z'])),
                clearance_review_height=(float(row['pending_ceiling_z'])
                                         if np.isfinite(row['pending_ceiling_z']) else None),
                mature=bool(row['g_mature']),
                surface_id=int(row['g_surface_id']))

        def safe_cost(k):
            row = options[k]
            check = checked[k] = describe(k)
            if floor is not None and abs(check['ground_z']-floor[0]) > floor[1]+1e-7:
                check['reason'] = 'surface_outside_current_floor'
            elif row['solid']:
                check['reason'] = 'solid_or_steep_surface'
            elif self.clearance_needs_observation(row):
                check['reason'] = 'ceiling_observation_pending'
                unknown_footprints.setdefault(k,set()).add((x,y))
            elif not self.surface_observation.is_ground_source((x,y),int(row['g_surface_id'])):
                check['reason'] = 'ground_surface_unverified'
                unknown_footprints.setdefault(k,set()).add((x,y))
            elif not relaxed and not self.surface_observation.ground_observed((x,y),check['ground_z'],k):
                check['reason'] = 'surface_observation_pending'
                unknown_footprints.setdefault(k,set()).add((x,y))
            elif not relaxed:
                if collect_blockers:
                    unknown_keys, solid_keys, count = self.footprint_conflicts(xyz,k,separate=True)
                    unknown, solid = (bool(unknown_keys), count), (bool(solid_keys), len(solid_keys))
                    if unknown_keys or solid_keys:
                        unknown_footprints[k] = unknown_keys | solid_keys
                else:
                    unknown, solid = self.footprint_conflicts(xyz,k)
                check.update(checked_cells=unknown[1], physical_footprint_conflict=bool(solid[0]))
                if unknown[0]:
                    check['reason'] = 'unknown_footprint'
                elif solid[0]:
                    check['reason'] = 'obstacle_footprint'
                elif row['inflated_terrain_cost'] < 0 or row['inflated_terrain_cost'] > 20:
                    check['reason'] = 'terrain_cost_exceeded'
            if 'reason' in check:
                if collect_blockers and row['solid']:
                    unknown_footprints.setdefault(k,set()).add((x,y))
                return None
            check.update(ready=True, reason='ready')
            # Relaxed recovery accepts a surface whose inflation is pending.
            # Rank it after layers with a computed cost.
            return check['cost'] if check['cost'] >= 0 else 1e9

        safe = rank_layers(options, requested, tolerance, read_height, safe_cost)
        if safe:
            best = checked[safe[0]]
            aliases = [k for k, row in options.items() if
                       same_ground_surface(row['g_surface_id'], row['G'],
                                           best['surface_id'], best['ground_z'])]
            return dict(best, safe_layers=safe, equivalent_surface_slices=len(aliases),
                        equivalent_safe_layers=[k for k in safe if k in aliases])
        if not checked:
            closest = min(options, key=lambda k: abs(read_height(k)-requested))
            return missing_ground(dict(describe(closest), reason='surface_height_mismatch', safe_layers=[]))
        failure = min(checked.values(), key=lambda c: (
            c['reason']=='surface_outside_current_floor', c.get('physical_footprint_conflict', False),
            abs(c['ground_z']-requested)))
        result = dict(failure, safe_layers=[])
        if result['reason'] in ('ground_surface_unverified','surface_observation_pending','unknown_footprint'):
            result['surface_evidence']=self.surface_observation.describe_ground((x,y),requested)
        if unknown_footprints:
            result['unknown_blockers'] = self.merge_unknown_blockers(
                (record for k, keys in unknown_footprints.items()
                 for record in self.unknown_blockers(k,keys,read_height(k))), xyz)
        return result

    def footprint_cells(self, xyz, radius):
        s = self.settings
        x, y = map(int,self.indices(xyz))
        half = math.ceil(radius/s.resolution+1)
        for dx in range(-half, half+1):
            for dy in range(-half, half+1):
                px,py = self.position((0,x+dx,y+dy))
                distance = math.hypot(max(abs(px-xyz[0])-s.resolution*.5,0),
                                      max(abs(py-xyz[1])-s.resolution*.5,0))
                if distance<=radius+1e-8:
                    yield x+dx,y+dy,distance

    def footprint_conflicts(self, xyz, layer, keys=False, *, separate=False):
        s = self.settings
        unknown, solid, count = set(), set(), 0
        for x,y,_ in self.footprint_cells(xyz,s.robot_radius+s.safety_margin):
            count += 1
            row = self.cell((layer,x,y))
            if (row is None or row['unknown'] or self.clearance_needs_observation(row) or
                    (not row['solid'] and not self.surface_observation.is_ground_source((x,y),int(row['g_surface_id'])))):
                unknown.add((x,y))
            if row is not None and row['solid']:
                solid.add((x,y))
        if separate:
            return unknown, solid, count
        return unknown | solid if keys else ((bool(unknown), count), (bool(solid), len(solid)))

    def unknown_blockers(self, layer, cells, reference):
        """Targets for missing ground or blocked geometry; never change traversability."""
        s = self.settings
        relations = self.surface_relations
        for x, y in sorted(cells):
            for pending in self.surface_observation.pending_at((x,y),reference,layer):
                yield [*self.position((layer,x,y)),pending.height,int(layer)]
            row = self.cell((layer,x,y))
            if self.clearance_needs_observation(row):
                yield [*self.position((layer,x,y)),float(row['pending_ceiling_z']),int(layer)]
                continue
            if row is not None and not row['unknown'] and np.isfinite(row['G']) and not self.surface_observation.is_ground_source((x,y),int(row['g_surface_id'])):
                yield [*self.position((layer,x,y)),reference,int(layer)]
                continue
            if row is not None and not row['unknown'] and np.isfinite(row['G']):
                if any(not other['unknown'] and not other['solid'] and
                       self.surface_observation.is_ground_source((x,y),int(other['g_surface_id'])) and
                       abs(other['G']-row['G'])<=s.surface_noise_tolerance+1e-7 and
                       0<=other['inflated_terrain_cost']<=20 and not other['hard']
                       for _,other in self.layers_at(x,y)):
                    continue
                if row['passage_cost']>=50 and np.isfinite(row['C']):
                    if self.surface_relations.fresh_clearance_barrier((x,y),reference) is not None:
                        continue
                    yield [*self.position((layer,x,y)),float(row['C']),int(layer)]
                    continue
                if row['passage_cost']<0 or row['relief_cost']<50:
                    continue
                neighbors=[(x+dx,y+dy,self.cell((layer,x+dx,y+dy)))
                           for dx,dy in ((0,1),(0,-1),(1,0),(-1,0))]
                observed=[(nx,ny,other) for nx,ny,other in neighbors
                          if other is not None and not other['unknown'] and
                          np.isfinite(other['G']) and 0<=other['passage_cost']<50]
                height=max(float(row['G']),reference,
                           max((float(other['G']) for _,_,other in observed),default=reference))
                if relations.classify((x,y),height).state!='known_step':
                    yield [*self.position((layer,x,y)),height,int(layer)]
                for nx,ny,other in observed:
                    if (height-other['G']>s.surface_noise_tolerance+1e-7 and
                            relations.classify((nx,ny),height).state!='known_step'):
                        yield [*self.position((layer,nx,ny)),height,int(layer)]
                continue
            heights = []
            for dx, dy in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                row = self.cell((layer, x+dx, y+dy))
                if (row is not None and not row['unknown'] and not row['solid'] and
                        abs(row['G']-reference) <= s.step_max+1e-5):
                    heights.append(float(row['G']))
            height = min(heights, key=lambda z: (abs(z-reference), -z)) if heights else reference
            # The expected tread may already be represented in another cut.
            if any(not row['unknown'] and abs(row['G']-height) <= s.surface_noise_tolerance
                   for _, row in self.layers_at(x, y)):
                continue
            if relations.classify((x,y),height).state!='known_step':
                yield [s.origin_x+x*s.resolution, s.origin_y+y*s.resolution, height, int(layer)]

    def merge_unknown_blockers(self, records, point):
        ordered = sorted(records, key=lambda p: (math.hypot(p[0]-point[0], p[1]-point[1]), *p))
        result = []
        for p in ordered:
            if any(p[:2] == q[:2] and abs(p[2]-q[2]) <= self.settings.surface_noise_tolerance
                   for q in result):
                continue
            result.append(p)
            if len(result) == 12:
                break
        return result

    def planning_cost(self, key):
        row = self.cell(key)
        if row is None or row['unknown'] or row['solid'] or row['inflated_terrain_cost'] < 0:
            return 50.
        return 50. if row['hard'] else float(row['inflated_terrain_cost'])

    def clearance_at(self, xyz, layer):
        s = self.settings
        limit = s.robot_radius+s.safety_margin+s.resolution
        half = math.ceil(limit/s.resolution)+1
        x, y = self.indices(xyz)
        distance = limit
        for dx in range(-half, half+1):
            for dy in range(-half, half+1):
                row = self.cell((layer, x+dx, y+dy))
                if row is None or row['solid'] or row['unknown']:
                    px = s.origin_x+(x+dx)*s.resolution
                    py = s.origin_y+(y+dy)*s.resolution
                    distance = min(distance, math.hypot(max(abs(px-xyz[0])-s.resolution*.5, 0),
                                                       max(abs(py-xyz[1])-s.resolution*.5, 0)))
        return distance

    def support_samples(self, x, y):
        """Measured, non-underside G represented by a traversable upper cut."""
        s=self.settings
        return tuple((k,float(row['G']),int(row['g_surface_id'])) for k,row in self.layers_at(x,y)
                     if row['measured'] and not row['unknown'] and np.isfinite(row['G']) and
                     s.origin_z+k*s.slice_dh>float(row['G']) and
                     self.surface_observation.is_ground_source((x,y),int(row['g_surface_id'])) and
                     0<=self.planning_cost((k,x,y))<=20)

    def target_candidates(self, xyz, reference_height):
        if getattr(self,'_target_revision',None)!=self.revision:
            self._target_surfaces=[(key,self.position(key)+[float(row['G'])])
                for key,row in self.iter_cells(True) if self.planning_cost(key)<=20]
            self._target_revision=self.revision
        choices = {}
        for key,p in self._target_surfaces:
            if abs(p[2]-(xyz[2]-reference_height)) > self.settings.step_max+1e-5:
                continue
            choices[(key[1], key[2], round(p[2]/.02))] = (math.dist(p[:2], xyz[:2]), p)
        selected = []
        ordered=sorted(choices.values())
        # A bounded candidate pool must also cover farther observed regions;
        # the controller, not this query, scores and selects the destination.
        nearby=ordered[:min(len(ordered),512)]
        spread=[ordered[i] for i in np.linspace(0,max(0,len(ordered)-1),64,dtype=int)] if ordered else []
        for _, point in nearby:
            if any(math.dist(point, old) < .4 for old in selected):
                continue
            selected.append(point)
            if len(selected) == 24:
                break
        for _,point in spread:
            if any(math.dist(point,old)<.4 for old in selected):continue
            selected.append(point)
            if len(selected)>=64:
                break
        return selected

    def export_dense(self):
        """Explicit test/GPU-comparison export; never used by production readers."""
        if self.first_layer is None:
            raise ValueError('pct_empty_cloud')
        shape = (self.last_layer-self.first_layer+1, *map(int, self.upper-self.lower))
        if math.prod(shape) > self.settings.max_cells:
            raise ValueError('pct_dense_export_limit')
        output = {n: np.full(shape, np.nan if n in ('G', 'C') else -1., dtype=np.float64)
                  for n in ('G', 'C', 'passage_cost', 'relief_cost', 'terrain_cost', 'inflated_terrain_cost')}
        for (k, x, y), row in self.iter_cells():
            index = (k-self.first_layer, x-self.lower[0], y-self.lower[1])
            for name in output:
                output[name][index] = row[name]
        return output
