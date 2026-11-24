"""Fixed-origin sparse blocks. One mutable map, no planning snapshots.

The owner holds its write lock for integrate/build and read lock for queries.
Dense export is explicitly diagnostic; production consumers read blocks.
"""
from collections import defaultdict
from bisect import bisect_left
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
    allow_open_ceiling: bool = True  # Interval mode requires actual ray clearance for top passages.
    open_ceiling_share_height: float = 5.0  # Inclusive vertical limit above measured G, metres.
    column_continuity_threshold: float = .10
    reliable_observation_range: float = 5.0
    slice_update_workers: int = 0  # Auto: up to four workers on available CPUs.
    maturity_min_points: int = 3
    surface_noise_tolerance: float = .05
    new_height_threshold: float = .10
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
    ascent_lookahead_slices: int = 3
    landing_min_area: float = 1.50
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
        if not isinstance(self.allow_open_ceiling,bool):
            raise ValueError('pct_invalid_open_ceiling_policy')
        if min(self.resolution, self.slice_dh, self.minimum_clearance, self.slope_max,
               self.open_ceiling_share_height, self.column_continuity_threshold) <= 0:
            raise ValueError('pct_invalid_resolution')
        nonnegative = ('robot_radius', 'safety_margin', 'surface_noise_tolerance',
            'relief_update_threshold', 'terrain_update_threshold', 'inflation_update_threshold',
            'observation_revisit_radius', 'reliable_observation_range')
        if min(getattr(self, n) for n in nonnegative) < 0 or self.desired_clearance < self.minimum_clearance:
            raise ValueError('pct_invalid_geometry_or_threshold')
        if self.surface_noise_tolerance >= self.minimum_clearance:
            raise ValueError('pct_surface_noise_tolerance_exceeds_clearance')
        if self.new_height_threshold <= 0:
            raise ValueError('pct_invalid_new_height_threshold')
        if int(self.slice_update_workers)!=self.slice_update_workers or not 0<=self.slice_update_workers<=16:
            raise ValueError('pct_invalid_update_workers')
        if self.planning_budget_seconds <= 0:
            raise ValueError('pct_invalid_planning_budget')
        if (not 0<=self.floor_failed_small_cluster_max_cells or
                int(self.floor_failed_small_cluster_max_cells)!=self.floor_failed_small_cluster_max_cells):
            raise ValueError('pct_invalid_floor_failed_small_cluster_max_cells')
        if not math.isfinite(self.floor_direction_search_timeout) or self.floor_direction_search_timeout<=0:
            raise ValueError('pct_invalid_floor_direction_search_timeout')
        if int(self.ascent_lookahead_slices)!=self.ascent_lookahead_slices or not 1<=self.ascent_lookahead_slices<=5:
            raise ValueError('pct_invalid_ascent_lookahead_slices')
        if self.landing_min_area <= 0:
            raise ValueError('pct_invalid_landing_support')
        for n in ('block_size', 'maturity_min_points', 'floor_min_resolved_frontier_cells',
                  'fast_min_visible_cells',
                  'max_surface_records', 'max_cells', 'traversability_kernel_size', 'inflation_kernel_size'):
            value = getattr(self, n)
            if int(value) != value or value < 1:
                raise ValueError('pct_invalid_integer_setting:' + n)
        if self.traversability_kernel_size % 2 != 1 or self.inflation_kernel_size % 2 != 1 or not 0 <= self.standable_ratio <= 1:
            raise ValueError('pct_invalid_kernel')



FLOATS = ('G', 'C', 'passage_cost', 'relief_cost', 'terrain_cost', 'inflated_terrain_cost',
          'gx', 'gy', 'q', 'm', 'terrain_inflation_reference', 'C_clear', 'node_z')
CELL_DTYPE = np.dtype([(n, '<f8') for n in FLOATS] +
    [('g_surface_id', '<i8'), ('c_surface_id', '<i8')] + [
    ('g_mature', '?'), ('c_mature', '?'),
    ('measured', '?'), ('flat', '?'), ('feature_valid', '?'),
    ('solid', '?'), ('unknown', '?'), ('hard', '?'), ('solid_hard', '?'), ('unknown_hard', '?'),
    ('interval_node', '?')])


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
        self._column_intervals = {}
        self._column_known_intervals = {}
        self._column_ground_intervals = {}
        self._column_frontier_signatures = {}
        self._obstacle_changes = {}
        self._obstacle_tiles = {}
        self._obstacle_costs = {}
        self._blocked_cell = np.zeros((),dtype=CELL_DTYPE)
        for name in FLOATS:self._blocked_cell[name]=np.nan
        for name in ('passage_cost','terrain_cost','inflated_terrain_cost'):
            self._blocked_cell[name]=50.
        self._blocked_cell['relief_cost']=0.
        for name in ('measured','solid','hard','solid_hard'):
            self._blocked_cell[name]=True
        self._blocked_cell.flags.writeable=False
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
        self.surface_records = self.accepted_points = self.accepted_frames = 0
        self.evidence_revision = self.revision = 0
        self.last_sensor_time = self.last_integration_time = self.last_commit_time = None
        self.dirty, self.geometry_changed = set(), set()
        self.evidence_dirty = set()
        self.error, self.stats, self.last_work = '', {}, {}
        self.lower = self.upper = self.first_layer = self.last_layer = None
        self.ignored_static_points = 0
        self.integrated_serial = 0
        self.geometry_returns=0
        self.evidence_preparation_ms=0.

        from .surface_relations import SurfaceRelations
        self.surface_relations=SurfaceRelations(self)
        from .surface_observation import SurfaceObservation
        self.surface_observation = SurfaceObservation(self)
        from .pct_frontier_index import FrontierIndex
        self.frontier_index = FrontierIndex(self)
        from .pct_support_targets import SupportTargets
        self.support_targets=SupportTargets(self)
        from .pct_connectivity import ConnectivityIndex
        self.connectivity=ConnectivityIndex(self)
        self._floor_connectivity=None
        self.planning_block_versions={}
        self._seam_costs={}
        s = self.settings
        self._footprint_offsets = tuple((x,y) for x,y,_ in self.footprint_cells(
            (s.origin_x,s.origin_y),s.robot_radius+s.safety_margin))
        self._footprint_offset_set=frozenset(self._footprint_offsets)
        from .map_costs import inflation_weights
        weights={(x,y):w for x,y,w in inflation_weights(s)}
        self._seam_footprint=tuple((x,y,weights.pop((x,y),0.),True) for x,y in self._footprint_offsets)+tuple(
            (x,y,w,False) for (x,y),w in weights.items())

    def floor_connectivity(self, floor):
        from .pct_connectivity import ConnectivityIndex
        if self._floor_connectivity is None or self._floor_connectivity.floor!=floor:
            self._floor_connectivity=ConnectivityIndex(self,floor)
            self._floor_connectivity.update_tiles({(x,y) for _,x,y in self.blocks})
        return self._floor_connectivity

    def observed_cell(self, key, ground_height=None):
        """Read known G/C space or a persistent same-XY blocked interval.

        Return a committed reference or temporary max-G/min-C view. Querying
        never allocates a neighbor or mutates geometry. A blocked interval is
        known even without G/C; it never manufactures a support height.
        """
        k, x, y = key
        cut = self.settings.origin_z + k*self.settings.slice_dh
        return self.observed_at_height((x,y),cut,ground_height,layer_hint=k)

    def observed_at_height(self, xy, cut, ground_height=None, *, layer_hint=None):
        """Query an exact world Z without rounding it to a storage address."""
        x,y=xy
        own=self._effective_row(self.cell((layer_hint,x,y))) if layer_hint is not None else None
        column=self.columns.get((x,y))
        blocked=column is not None and column.obstacle_at(cut) is not None
        ground = ceiling = own if self._covers_cut(own, cut, ground_height) else None
        indexed = self._column_intervals.get((x, y))
        if indexed is not None:
            grounds, upper, entries = indexed
            at = bisect_left(grounds, cut-1e-7)-1
            # The usual case is one interval covering this cut, even in a
            # column with several independent rooms. Keep that query cheap.
            if (ground is not None and at>=0 and
                    (float(ground['G']),float(ground['C']))==entries[at][:2] and
                    (at==0 or upper[at-1]<=cut+1e-7)):
                return self._obstacle_view(ground) if blocked else ground
            # Only intervals containing this cut participate. In particular,
            # endpoints from independent rooms can never bridge their gap.
            while at >= 0 and upper[at] > cut+1e-7:
                for layer in entries[at][2]:
                    origin = self._effective_row(self.cell((layer,x,y)))
                    if not self._covers_cut(origin, cut, ground_height):continue
                    if ground is None or origin['G']>ground['G']:ground=origin
                    if ceiling is None or origin['C']<ceiling['C']:ceiling=origin
                at -= 1
        if ground is None:return self._blocked_cell if blocked else None
        if ceiling['C']==ground['C']:return self._obstacle_view(ground) if blocked else ground
        # A temporary reference result, never a materialized map cell. Keep
        # the original G identity and recheck the tighter measured clearance.
        row=ground.copy()
        for field in ('C','c_surface_id','c_mature'):row[field]=ceiling[field]
        clearance=float(row['C']-row['G'])
        passage=50. if clearance<self.settings.minimum_clearance else max(0.,20.*(self.settings.desired_clearance-clearance))
        row['passage_cost']=passage
        row['terrain_cost']=passage+row['relief_cost'] if row['relief_cost']>=0 else (50. if passage>=50 else -1.)
        row['inflated_terrain_cost']=max(row['inflated_terrain_cost'],row['terrain_cost'])
        row['solid']=row['terrain_cost']>=50
        row['solid_hard'] |= row['solid']
        row['hard'] |= row['solid']
        return self._obstacle_view(row) if blocked else row

    def _obstacle_view(self, row):
        # Preserve actual/borrowed G/C without mutating the original work cell.
        if row['solid'] and min(row[f] for f in ('passage_cost','terrain_cost','inflated_terrain_cost'))>=50:
            return row
        row=row.copy()
        for field in ('passage_cost','terrain_cost','inflated_terrain_cost'):
            row[field]=max(50.,float(row[field]))
        row['solid']=row['solid_hard']=row['hard']=True
        return row

    def _effective_row(self, row):
        return row

    def obstacle_at(self, key):
        k,x,y=key
        column=self.columns.get((int(x),int(y)))
        return column.obstacle_at(self.settings.origin_z+k*self.settings.slice_dh) if column else None

    def obstacle_footprint(self, key):
        """Cached robot-disk exclusion using only nearby obstacle columns."""
        if key in self._obstacle_costs:return self._obstacle_costs[key]
        k,x,y=key;s=self.settings;b=s.block_size
        center=self.observed_cell(key)
        reference=float(center['G']) if center is not None else math.nan
        reach=math.ceil((s.robot_radius+s.safety_margin)/s.resolution+.5)
        for bx in range((x-reach)//b,(x+reach)//b+1):
            for by in range((y-reach)//b,(y+reach)//b+1):
                for ox,oy in self._obstacle_tiles.get((k,bx,by),()):
                    if (ox-x,oy-y) in self._footprint_offset_set:
                        row=self.observed_cell((k,ox,oy))
                        if (math.isfinite(reference) and row is not None and
                                self.tread_above_riser(k,ox,oy,row,reference) is not None):continue
                        self._obstacle_costs[key]=True
                        return True
        self._obstacle_costs[key]=False
        return False

    def _covers_cut(self, row, cut, ground_height=None):
        if (row is None or row['unknown'] or not np.isfinite(row['G']) or
                np.isnan(row['C']) or row['C']<=row['G'] or cut<=row['G']+1e-7):return False
        if ground_height is not None and abs(float(row['G'])-ground_height)>self.settings.surface_noise_tolerance+1e-7:
            return False
        if np.isfinite(row['C']):return cut<row['C']-1e-7
        return cut<=row['G']+self.settings.open_ceiling_share_height+1e-7

    def frontier_observed(self, layer, xy):
        """Ordinary frontiers use geometric knownness, never a target G."""
        return self.frontier_cell(layer,xy) is not None

    def frontier_cell(self, layer, xy):
        return self.observed_cell((layer,*xy))

    def frontier_cost(self, layer, xy, row=None):
        return self.planning_cost((layer,*xy))

    def _index_column_intervals(self, xy):
        """Refresh only a changed column; deduplicate intervals across cuts."""
        previous=self._column_known_intervals.get(xy,())
        intervals = {}
        known={}
        signature=[]
        s=self.settings
        for k,row in self.layers_at(*xy):
            row=self._effective_row(row)
            if not self._covers_cut(row,s.origin_z+k*s.slice_dh):continue
            pair = (float(row['G']),float(row['C']))
            identity=int(row['g_surface_id'])
            signature.append((k,*pair,identity))
            lo=math.floor((pair[0]-s.origin_z+1e-7)/s.slice_dh)+1
            hi=(math.ceil((pair[1]-s.origin_z-1e-7)/s.slice_dh)-1 if math.isfinite(pair[1]) else
                math.floor((pair[0]+s.open_ceiling_share_height-s.origin_z+1e-7)/s.slice_dh))
            known.setdefault((*pair,identity,lo,hi),None)
            intervals.setdefault(pair,[]).append(k)
        if intervals:
            entries = tuple((g,c,tuple(ks)) for (g,c),ks in sorted(intervals.items()))
            upper=[];maximum=-math.inf
            for g,c,_ in entries:
                limit=c if math.isfinite(c) else g+s.open_ceiling_share_height+2e-7
                maximum=max(maximum,limit);upper.append(maximum)
            self._column_intervals[xy]=(tuple(g for g,_,_ in entries),tuple(upper),entries)
        else:self._column_intervals.pop(xy,None)
        if known:
            records=tuple(sorted(known))
            self._column_known_intervals[xy]=records
            groups={}
            for g,c,identity,lo,hi in records:
                token=(identity,g if not identity else 0.)
                groups.setdefault(token,set()).add((lo,hi))
            self._column_ground_intervals[xy]={key:tuple(sorted(value)) for key,value in groups.items()}
            self._column_frontier_signatures[xy]=tuple(signature)
        else:
            self._column_known_intervals.pop(xy,None)
            self._column_ground_intervals.pop(xy,None)
            self._column_frontier_signatures.pop(xy,None)
        current=self._column_known_intervals.get(xy,())
        changed_records=set(previous)^set(current)
        affected={k for _,_,_,lo,hi in changed_records for k in range(lo,hi+1)}
        return affected,previous!=current

    def ground_band_obstacle(self, xy, height):
        """Measured occupancy above a support, independent of cut aliases.

        Return bands occupy space; a room's free (G,C) interval does not.
        A lower floor's ceiling below this ground cannot block it. Treads
        within the step capability remain usable supports.
        """
        column=self.columns.get(tuple(xy))
        if column is None:return False
        low=float(height)+min(.16,self.settings.step_max)
        high=float(height)+self.settings.minimum_clearance
        return any(lo<high-1e-7 and hi>low+1e-7
                   for lo,hi in column.obstacle_intervals)

    def frontier_height_compatible(self, layer, target, source, height, identity):
        """Auxiliary frontier filter only; inferred heights never become G.

        S contains known cuts of the source ground, including bounded open
        aliases. Open records of another ground never supply an auxiliary
        height. No evidence in S leaves the normal unknown boundary.
        """
        # Compatibility maps deduplicate an already measured ground across
        # cuts. The live radar-height view instead requires an actual interval
        # hit at z_q; a same-G record outside that range cannot settle it.
        if self.ground_band_obstacle(target,height):return False
        records=self._column_known_intervals.get(target,())
        if getattr(self,'frontier_query_height',None) is None and any(abs(g-height)<=self.settings.surface_noise_tolerance+1e-7
               for g,_,_,_,_ in records):
            return False
        groups=self._column_ground_intervals.get(source,{})
        spans=groups.get((identity,height if not identity else 0.),())
        if not spans:return True
        cut=getattr(self,'frontier_query_height',None)
        if cut is None:cut=self.settings.origin_z+layer*self.settings.slice_dh
        else:layer=math.floor((cut-self.settings.origin_z)/self.settings.slice_dh)
        grounds=[];ceilings=[]
        for g,c,_,lo,hi in records:
            # Missing C does not establish a different target surface.
            if not math.isfinite(c):continue
            contacts=[(max(lo,a),min(hi,b)) for a,b in spans if max(lo,a)<=min(hi,b)]
            if not contacts:continue
            if c<cut-1e-7 and any(a<layer for a,_ in contacts):ceilings.append(c)
            else:grounds.append(g)
        auxiliary=max(ceilings) if ceilings else max(grounds,default=None)
        return auxiliary is None or abs(auxiliary-height)<=min(.16,self.settings.step_max)+1e-7

    def frontier_selection_layer(self, ground):
        """One cut for scheduling, with a valid measured-interval fallback."""
        s=self.settings
        preferred=math.floor((float(ground[2])+.16-s.origin_z+1e-7)/s.slice_dh)+1
        xy=tuple(map(int,self.indices(ground)))
        choices=[]
        for g,c,_,lo,hi in self._column_known_intervals.get(xy,()):
            if abs(g-ground[2])>s.surface_noise_tolerance+1e-7:continue
            selected=min(hi,max(lo,preferred))
            choices.append((abs(selected-preferred),abs(g-ground[2]),selected,c))
        if not choices:
            return preferred,dict(preferred=preferred,selected=preferred,
                height=s.origin_z+preferred*s.slice_dh,reason='ground_cut_unobserved')
        _,_,selected,ceiling=min(choices)
        return selected,dict(preferred=preferred,selected=selected,
            height=s.origin_z+selected*s.slice_dh,
            reason='preferred' if selected==preferred else
                   'ceiling_limited' if math.isfinite(ceiling) else 'observed_open_ground_cut')

    def ground_pending(self, xy, layer):
        return xy in self.columns and self.observed_cell((layer,*xy)) is None

    def _ground_at_cut(self, layer, ground, ceiling):
        if ground is None:
            return None, None
        cut = self.settings.origin_z + layer*self.settings.slice_dh
        if cut <= ground.height+1e-7:
            return None, None
        if ((ceiling is not None and cut < ceiling.height-1e-7) or
                (ceiling is None and self.settings.allow_open_ceiling)):
            return ground, ceiling
        # Measured ceiling geometry still lives in the original column.
        return None, None

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
            # All actual returns participate in the ordered G/C geometry.
            # Sensor pose remains necessary for measured footprints and scans,
            # but neither its Z nor fitted normals gate support selection.
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
        self.evidence_preparation_ms=1000*(time.perf_counter()-started)
        self.geometry_returns=len(points)
        # Near capacity, stage only touched columns to reject an over-capacity
        # frame atomically. Normal fusion modifies the one live map directly.
        staged={} if self.surface_records+len(points)>s.max_surface_records else None
        total=self.surface_records
        used=0;changed=set();obstacle_changes={}
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
                                     int(tops[point_id]) if tops is not None else None,
                                     support_hit_z=(float(z) if sensor_origin is not None and
                                                    origin[2]>float(z)+1e-7 else None))
                total+=len(column)-previous
                if total>s.max_surface_records:
                    self.error='pct_surface_record_limit'
                    return self.surface_records
            if staged is not None:
                staged[key]=column
            previous_intervals=column.obstacle_intervals
            if self._remember_obstacles(column, points[order[boundaries[i]:boundaries[i+1]],2]):
                obstacle_changes[key]=previous_intervals
                changed.add(key)
            used+=accepted
            if accepted or len(column)!=before:
                changed.add(key)
        if staged is not None:
            self.columns.update(staged)
        blocking_changed=False
        for xy,previous in obstacle_changes.items():
            self._obstacle_changes.setdefault(xy,previous)
            tile=xy[0]//s.block_size,xy[1]//s.block_size
            def cuts(intervals):
                return {k for low,high in intervals for k in range(
                    math.ceil((low-s.origin_z-1e-7)/s.slice_dh),
                    math.floor((high-s.origin_z+1e-7)/s.slice_dh)+1)}
            before,after=cuts(previous),cuts(self.columns[xy].obstacle_intervals)
            for k in before-after:
                cells=self._obstacle_tiles[k,*tile]
                cells.remove(xy)
                if not cells:del self._obstacle_tiles[k,*tile]
            for k in after-before:self._obstacle_tiles.setdefault((k,*tile),set()).add(xy)
            blocking_changed |= before!=after
        if blocking_changed:
            self._obstacle_costs.clear()
            self._seam_costs.clear()
        self.integrated_serial += 1
        affected=self.surface_relations.invalidate(changed)
        self.evidence_dirty.update(affected)
        self.surface_observation.refresh_relations(affected)
        observed=self.surface_observation.observe_frame(
            points,None if sensor_origin is None else origin,self.integrated_serial,orientations)
        self.evidence_dirty.update(observed)
        self.surface_records=total
        self.dirty.update(changed)
        self.ignored_static_points+=len(points)-used
        if used or obstacle_changes:
            self.accepted_frames+=1
            self.accepted_points+=used
            self.evidence_revision+=1
        self.last_integration_time=now
        self.error=''
        return self.surface_records

    def _remember_obstacles(self, column, heights):
        s=self.settings
        return column.remember_obstacles(heights,s.minimum_clearance,s.surface_noise_tolerance)

    def _column_cuts(self, xy, geometry):
        from .map_costs import column_cuts
        return column_cuts(self,xy,geometry)

    def _changed_cuts(self, previous, current, layers, previous_layers):
        s=self.settings
        return changed_slice_indices(previous,current,layers,previous_layers,s.origin_z,s.slice_dh)

    def _geometry_signature(self, xy, geometry):
        return self.columns[xy].geometry_signature(geometry,self.settings.surface_noise_tolerance)

    def _commit_node_metadata(self, batches, changes):
        pass

    def _commit_planning_versions(self, revision):
        pass

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
        self._column_intervals.clear()
        self._column_known_intervals.clear()
        self._column_ground_intervals.clear()
        self._column_frontier_signatures.clear()
        self.planning_block_versions.clear()
        self._seam_costs.clear()
        self._obstacle_costs.clear()
        self.lower=self.upper=self.first_layer=self.last_layer=None
        self.dirty=set(self.columns)
        self.frontier_index.reset_geometry()
        from .pct_connectivity import ConnectivityIndex
        from .pct_support_targets import SupportTargets
        self.connectivity=ConnectivityIndex(self)
        self._floor_connectivity=None
        self.support_targets=SupportTargets(self)

    def _build(self, now):
        now = self.clock() if now is None else now
        self._prepare_frontier_query()
        if not self.dirty and not self.evidence_dirty:
            if not self.columns:
                raise ValueError('pct_empty_cloud')
            self.last_work = dict(base_cells=0, feature_cells=0, terrain_cells=0, inflation_cells=0,
                                  hard_cells=0, base_update_ms=0.,cost_update_ms=0.,frontier_update_ms=0.,
                                  slice_workers=0,frontier_workers=0,native_slice_costs=self._cost_kernel is not None,
                                  geometry_candidate_cells=0,geometry_update_cells=0,geometry_skipped_cells=0)
            self.frontier_index.commit([])
            return False
        s = self.settings
        started=time.perf_counter()
        evidence_columns=set(self.dirty) | self.evidence_dirty
        work = self.dirty
        batches=defaultdict(dict)
        pending_geometry={}
        frontiers=defaultdict(set)
        geometry_candidate_cells=0
        missing_blocks=set()
        b=s.block_size
        frontier_resets=defaultdict(set)
        previous_slices=set(self.layers)
        for xy in work:
            column=self.columns.get(xy)
            if not column:
                continue
            geometry=column.observed_geometry(s.surface_noise_tolerance)
            cut_surfaces=self._column_cuts(xy,geometry)
            # Open ground may live just below a grid boundary while its
            # G+16 cm scheduling cut is above it. Keep a frontier index for
            # that cut, without allocating/filling any neighbor work cells.
            for ground,ceiling in cut_surfaces.values():
                if ceiling is not None:continue
                k=math.floor((ground.height+.16-s.origin_z+1e-7)/s.slice_dh)+1
                if k not in self.layers:
                    from .tomographic_slice import TomographicSlice
                    self.layers[k]=TomographicSlice(self,k)
                    self.first_layer=min(self.first_layer,k)
                    self.last_layer=max(self.last_layer,k)
                frontiers[k].add(xy)
            self.column_layers[xy]=set(cut_surfaces)
            _,records=geometry
            signature=self._geometry_signature(xy,geometry)
            layers=tuple(sorted(self.column_layers[xy]))
            previous,previous_layers=self._column_geometry.get(xy,((),()))
            changed=set(self._changed_cuts(previous,signature,layers,previous_layers))
            current=set(layers)
            removed=set(previous_layers)-current
            pending_geometry[xy]=(signature,layers)
            geometry_candidate_cells+=len(layers)
            # Frontier observations can change while each retained cut keeps the same G/C.
            if xy in work:
                for k in layers:
                    frontiers[k].add(xy)
            for k in removed:
                frontiers[k].add(xy)
                frontier_resets[k].add(xy)
                batches[k][xy]=((math.nan,0,False),(math.nan,0,False))
            for k in changed:
                ground,ceiling=cut_surfaces[k]
                surfaces=tuple((r.height,r.id,r.mature) if r else (np.nan,0,False)
                               for r in (ground,ceiling))
                if ground is not None and ceiling is None and s.allow_open_ceiling:
                    surfaces=(surfaces[0],(math.inf,0,False))
                batches[k][xy]=surfaces
                block_key=(k,xy[0]//b,xy[1]//b)
                if block_key not in self.blocks:
                    missing_blocks.add(block_key)
        if (len(self.blocks)+len(missing_blocks))*b*b>s.max_cells:
            raise ValueError('pct_map_cell_limit')
        for key in sorted(missing_blocks):
            self._allocate_block(key)
        prepared=time.perf_counter()

        def update_geometry(layer, evidence):
            return layer.index,layer.update_geometry(evidence,self._geometry_kernel)

        committed,_ = self._run_slice_tasks(update_geometry, [(self.layers[k],batch) for k,batch in sorted(batches.items())])
        changes = dict(committed)
        self._commit_node_metadata(batches,changes)
        affected = {k+offset for k in changes for offset in (-1,0,1) if k+offset in self.layers}

        def update_costs(layer, changes):
            return layer,layer.update_costs(self,changes)

        results,cost_workers=self._run_slice_tasks(update_costs, [(self.layers[k],changes) for k in sorted(affected)])
        costs_done=time.perf_counter()
        self.last_work=dict(base_cells=0,feature_cells=0,terrain_cells=0,inflation_cells=0,hard_cells=0)
        self.geometry_changed=set()
        for layer,(base,changes,cost_work) in results:
            self.geometry_changed.update((layer.index,*xy) for xy in base | changes)
            for name,count in cost_work.items():
                self.last_work[name]+=count
        # Interval-only changes (including C-only columns with no G) are real
        # planning changes. Invalidate only the affected cuts and robot disks.
        for (x,y),old in self._obstacle_changes.items():
            for k in self.layers:
                # Low-cut wall evidence also affects high-cut frontiers.
                if (x,y) in self.frontier_index.by_column or any(
                        k in self.column_layers.get((x+dx,y+dy),())
                        for dx in (-1,0,1) for dy in (-1,0,1)):
                    frontiers[k].add((x,y))
                cut=s.origin_z+k*s.slice_dh
                was_blocked=any(lo-1e-7<=cut<=hi+1e-7 for lo,hi in old)
                if was_blocked != (self.obstacle_at((k,x,y)) is not None):
                    frontiers[k].add((x,y))
                    self.geometry_changed.update((k,x+dx,y+dy) for dx,dy in self._footprint_offsets)
        cost_columns={(x,y) for _,x,y in self.geometry_changed}
        for xy in work:
            aliases,changed_interval=self._index_column_intervals(xy)
            if changed_interval:
                for k in aliases:
                    frontiers[k].add(xy)
                    self.geometry_changed.add((k,*xy))
        # Neighbor cost/standability changes also affect virtual aliases,
        # even when no ray hit that column in this batch.
        cost_layers=defaultdict(set)
        for layer,x,y in self.geometry_changed:cost_layers[x,y].add(layer)
        for xy,touched_layers in cost_layers.items():
            for _,_,_,lo,hi in self._column_known_intervals.get(xy,()):
                if not any(lo<=k<=hi for k in touched_layers):continue
                for k in self.layers:
                    if lo<=k<=hi:
                        frontiers[k].add(xy)
                        self.geometry_changed.add((k,*xy))
        # Initialize a new cut's logical index once from the column interval
        # catalog. Existing cuts subsequently receive only local changes.
        for k in self.layers.keys()-previous_slices:
            frontiers[k].update(xy for xy,records in self._column_known_intervals.items()
                                if any(lo<=k<=hi for _,_,_,lo,hi in records))
        if self.geometry_changed:
            self._seam_costs.clear()
            self._obstacle_costs.clear()
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
        self._query_frontier_updates(frontiers, evidence_columns | work | cost_columns,
                                     self.layers.keys()-previous_slices)
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
        self.connectivity.update(self.geometry_changed,support_changed=self.geometry_changed)
        if self._floor_connectivity is not None:
            self._floor_connectivity.update(self.geometry_changed,support_changed=self.geometry_changed)
        self.support_targets.update(self.geometry_changed)
        revision=self.revision+bool(self.geometry_changed)
        for key in {(k,x//b,y//b) for k,x,y in self.geometry_changed}:
            self.planning_block_versions[key]=revision
        self._commit_planning_versions(revision)
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
            allow_open_ceiling=s.allow_open_ceiling,
            surface_noise_tolerance=s.surface_noise_tolerance,
            ignored_static_points=self.ignored_static_points,
            evidence_preparation_ms=self.evidence_preparation_ms,geometry_returns=self.geometry_returns,
            connectivity=self.connectivity.last_work,
            update_mode='incremental_blocks' if self.geometry_changed else 'unchanged_geometry', **self.last_work)
        self.revision=revision
        if self.geometry_changed:
            self.last_commit_time=now
        self._column_geometry.update(pending_geometry)
        self.dirty.clear()
        self.evidence_dirty.clear()
        self._obstacle_changes.clear()
        return bool(self.geometry_changed)

    def _prepare_frontier_query(self):
        pass

    def _query_frontier_updates(self, frontiers, columns, new_layers):
        pass

    def point_safe_check(self, xyz, reference_height=.32, relaxed=False, *,
                         floor=None, height_tolerance=None, collect_blockers=False):
        x, y = self.indices(xyz)
        requested = xyz[2]-reference_height
        result = dict(ready=False, cell=[int(x), int(y)], requested_ground_z=float(requested), layer=0)
        options = {int(k): observed for k,_ in self.layers_at(x,y)
                   if (observed:=self.observed_cell((k,x,y))) is not None and np.isfinite(observed['G'])}
        def missing_ground(result):
            if collect_blockers and self.first_layer is not None and math.isfinite(requested):
                s = self.settings
                k = math.floor((requested-s.origin_z)/s.slice_dh)+1
                k = min(self.last_layer, max(self.first_layer, k))
                result['unknown_blockers'] = list(self.unknown_blockers(k,[(x,y)],requested))
            return result

        if not options:
            k=math.floor((requested-self.settings.origin_z)/self.settings.slice_dh)+1
            if self.obstacle_at((k,x,y)) is not None:
                return dict(result,layer=k,reason='solid_or_steep_surface',solid=True,cost=50.)
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
            cost=float(row['inflated_terrain_cost'])
            if not relaxed and cost>20 and self.seam_candidate(k,float(row['G'])):
                cost=self.planning_cost((k,x,y))
            return dict(result, layer=int(k), ground_z=float(row['G']),
                cost=cost, source=1,
                solid=bool(row['solid']), unknown_inflated=bool(row['hard']),
                clearance=float(ceiling-row['G']) if np.isfinite(ceiling) else None,
                clearance_observed=bool(np.isfinite(ceiling) and
                    (not row['interval_node'] or row['c_surface_id']!=0)),
                clearance_lower_bound=float(row['C_clear']-row['G']) if row['interval_node'] else None,
                node_height=float(row['node_z']) if row['interval_node'] else None,
                mature=bool(row['g_mature']),
                surface_id=int(row['g_surface_id']))

        def safe_cost(k):
            row = options[k]
            check = checked[k] = describe(k)
            if floor is not None and abs(check['ground_z']-floor[0]) > floor[1]+1e-7:
                check['reason'] = 'surface_outside_current_floor'
            elif row['solid']:
                check['reason'] = 'solid_or_steep_surface'
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
                elif check['cost'] < 0 or check['cost'] > 20:
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
        center = self.observed_cell((layer,*self.indices(xyz)))
        reference = float(center['G']) if center is not None else math.nan
        unknown, solid, count = set(), set(), 0
        for x,y,_ in self.footprint_cells(xyz,s.robot_radius+s.safety_margin):
            count += 1
            row = self.footprint_support(layer,x,y,reference)
            if (row is None or row['unknown'] or
                    (not row['solid'] and not self.surface_observation.is_ground_source((x,y),int(row['g_surface_id'])))):
                unknown.add((x,y))
            if row is not None and row['solid']:
                solid.add((x,y))
        if separate:
            return unknown, solid, count
        return unknown | solid if keys else ((bool(unknown), count), (bool(solid), len(solid)))

    def footprint_support(self, layer, x, y, reference):
        """Read a neighboring cut only for a footprint crossing a cut boundary."""
        row = self.observed_cell((layer,x,y))
        s = self.settings
        if self.obstacle_at((layer,x,y)) is not None:
            upper=self.tread_above_riser(layer,x,y,row,reference) if math.isfinite(reference) else None
            return upper if upper is not None else row
        if row is not None and (not math.isfinite(reference) or
                                abs(float(row['G'])-reference)<=s.step_max+1e-7):
            if row['passage_cost']<50 or not math.isfinite(row['C']):
                return row
            upper=self.tread_above_riser(layer,x,y,row,reference)
            return upper if upper is not None else row
        # The disk can span two successive steps. Individual route edges and
        # stop checks still enforce their original single-step constraints.
        candidates = (self.observed_cell((k,x,y)) for k in (layer-1,layer+1))
        if row is not None and not row['solid'] and abs(float(row['G'])-reference)>2*s.step_max+1e-7:
            row = None
        return min((r for r in candidates if r is not None and
                    abs(float(r['G'])-reference)<=2*s.step_max+1e-7),
                   key=lambda r:abs(float(r['G'])-reference),default=row)

    def tread_above_riser(self, layer, x, y, row, reference):
        """An observed higher tread can support a step over a measured riser."""
        if row is None or not np.isfinite(row['G']) or not np.isfinite(row['C']):return None
        upper=self.observed_cell((layer+1,x,y))
        if (upper is None or upper['solid'] or upper['unknown'] or
                not 0<=upper['terrain_cost']<=20 or
                upper['G']<row['C']-1e-7 or upper['G']<=row['G']+1e-7 or
                abs(float(upper['G'])-reference)>self.settings.step_max+1e-7):
            return None
        column=self.columns.get((x,y))
        # Preserve a measured underside. A non-underside at or below the
        # next tread may be the face of that step in the lower cut.
        if column is None:
            return None
        orientations={r.id:r.orientation for r in column.values()}
        if (orientations.get(int(row['c_surface_id'])) in (-2,0,1) and
                orientations.get(int(upper['g_surface_id']))==1):
            return upper
        return None

    def nearby_cut_height(self, layer, x, y, reference):
        rows = (self.cell((k,x,y)) for k in (layer-1,layer+1))
        heights = (float(row['G']) for row in rows if row is not None)
        return min((h for h in heights if abs(h-reference)<=self.settings.step_max+1e-5),
                   key=lambda h:abs(h-reference),default=math.nan)

    def seam_candidate(self, layer, ground):
        s = self.settings
        cut = s.origin_z+layer*s.slice_dh
        return (min(abs(cut-ground),abs(cut-s.slice_dh-ground))<=2*s.step_max+1e-7 and
                (layer-1 in self.layers or layer+1 in self.layers))

    def unknown_blockers(self, layer, cells, reference):
        """Targets for missing ground or blocked geometry; never change traversability."""
        s = self.settings
        relations = self.surface_relations
        for x, y in sorted(cells):
            if self.obstacle_at((layer,x,y)) is not None:continue
            for pending in self.surface_observation.pending_at((x,y),reference,layer):
                yield [*self.position((layer,x,y)),pending.height,int(layer)]
            row = self.cell((layer,x,y))
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
        row = self.observed_cell(key)
        if row is None or row['unknown'] or row['solid'] or row['inflated_terrain_cost'] < 0:
            return 50.
        if self.obstacle_footprint(key):return 50.
        if not row['hard'] and row['inflated_terrain_cost']<=20:
            return float(row['inflated_terrain_cost'])
        layer,x,y = key
        if (not (row['unknown_hard'] or row['solid_hard']) or not self.seam_candidate(layer,float(row['G'])) or
                not 0<=row['terrain_cost']<=20):
            return 50. if row['hard'] else float(row['inflated_terrain_cost'])
        if key not in self._seam_costs:
            cost = float(row['terrain_cost'])
            for dx,dy,weight,in_footprint in self._seam_footprint:
                support = self.footprint_support(layer,x+dx,y+dy,float(row['G']))
                if in_footprint and (support is None or support['unknown'] or support['solid']):
                    cost = 50.
                    break
                if support is not None:
                    cost=max(cost,weight*float(support['terrain_cost']))
            self._seam_costs[key] = cost
        return self._seam_costs[key]

    def traversable_block(self, key, block, floor=None):
        """Use the same footprint support for selecting and routing to poses."""
        k,bx,by=key
        s=self.settings
        cut=s.origin_z+k*s.slice_dh
        upper=np.where(block['interval_node'] & np.isnan(block['C']),block['C_clear'],block['C'])
        eligible = (block['measured'] & ~block['unknown'] & ~block['solid'] &
                    np.isfinite(block['G']) & ~np.isnan(upper) &
                    (block['G']+1e-7<cut) & (cut<upper-1e-7) &
                    (block['inflated_terrain_cost']>=0))
        if floor is not None:
            eligible &= abs(block['G']-floor[0])<=floor[1]+1e-7
        traversable=eligible & ~block['hard'] & (block['inflated_terrain_cost']<=20)
        for i,j in zip(*np.nonzero(traversable)):
            if self.obstacle_footprint((k,bx*s.block_size+int(i),by*s.block_size+int(j))):
                traversable[i,j]=False
        if k-1 in self.layers or k+1 in self.layers:
            gap=np.minimum(abs(block['G']-cut),abs(block['G']-cut+s.slice_dh))
            seam=(eligible & (block['unknown_hard'] | block['solid_hard']) &
                  (block['terrain_cost']>=0) & (block['terrain_cost']<=20) & (gap<=2*s.step_max+1e-7))
            for i,j in zip(*np.nonzero(seam)):
                traversable[i,j]=self.planning_cost((k,bx*s.block_size+int(i),by*s.block_size+int(j)))<=20
        return traversable

    def clearance_at(self, xyz, layer):
        s = self.settings
        limit = s.robot_radius+s.safety_margin+s.resolution
        half = math.ceil(limit/s.resolution)+1
        x, y = self.indices(xyz)
        center = self.observed_cell((layer,x,y))
        reference = float(center['G']) if center is not None else math.nan
        distance = limit
        for dx in range(-half, half+1):
            for dy in range(-half, half+1):
                row = self.footprint_support(layer,x+dx,y+dy,reference)
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
                     s.origin_z+k*s.slice_dh+1e-7>=float(row['G']) and
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
        separation=max(self.settings.resolution*1.5,.15)
        for _, point in nearby:
            if any(math.dist(point, old) < separation for old in selected):
                continue
            selected.append(point)
            if len(selected) == 96:
                break
        for _,point in spread:
            if any(math.dist(point,old)<separation for old in selected):continue
            selected.append(point)
            if len(selected)>=128:
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
