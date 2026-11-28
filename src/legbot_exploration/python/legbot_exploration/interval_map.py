"""Adaptive, per-XY passage nodes built from accumulated measured intervals.

The inherited sparse index is a compatibility address, not a node elevation.
There is one stored node per admissible interval; node_z is its actual centre.
Fixed-height queries can borrow that node only inside its evidenced clearance.
No neighboring XY cell is allocated to fill a storage block or a cost halo.
"""
from dataclasses import dataclass, replace
from bisect import bisect_left, bisect_right, insort
import math
from types import SimpleNamespace
import numpy as np

from .fixed_layer_map import FixedLayerMap, MapSettings, different
from .interval_rays import merge_intervals, trace_free_columns


@dataclass(frozen=True)
class PassageNode:
    ground: float
    ceiling: float  # NaN when no measured upper endpoint exists.
    clear_to: float  # Absolute upper bound of the confirmed free space.
    midpoint: float
    ground_id: int
    ceiling_id: int
    ground_mature: bool
    ceiling_mature: bool

    def surfaces(self):
        return (SimpleNamespace(height=self.ground,id=self.ground_id,mature=self.ground_mature),
                SimpleNamespace(height=self.ceiling,id=self.ceiling_id,mature=self.ceiling_mature))


class IntervalLayerMap(FixedLayerMap):
    def __init__(self, settings=None, *args, **kwargs):
        settings=settings or MapSettings()
        if settings.slice_dh>settings.minimum_clearance*.5:
            # A valid interval must contain an internal query address even
            # when both measured endpoints lie exactly on bucket boundaries.
            settings=replace(settings,slice_dh=settings.minimum_clearance*.5)
        super().__init__(settings,*args,**kwargs)
        self._free_columns={}
        self._top_observed={}
        self._interval_nodes={}
        self._ordered_passages={}
        self._pending_open={}
        self._pending_ground={}
        self._pending_ground_count=0
        self._confirmed_grounds={}
        self._interval_node_count=0
        # Only the designated primary LiDAR sets this view. Physical nodes,
        # costs and A* addresses remain independent of the current sensor Z.
        self.frontier_query_height=None
        self._pending_frontier_query_height=None
        self._frontier_height_events={}
        self._frontier_column_events={}
        self._frontier_event_bands=[]
        self._frontier_endpoint_buckets={}
        self._frontier_bucket_keys={}
        self._frontier_query_changes=set()
        self._frontier_query_checked=0
        self.radar_frontier_layer=None
        self._radar_known_cache={}
        self._ray_kernel=trace_free_columns
        if kwargs.get('native_updates',True):
            try:
                from _legbot_layer_kernels import trace_free_columns as native_trace
            except ImportError:
                pass
            else:self._ray_kernel=native_trace

    def set_frontier_query_height(self, height):
        """Stage a primary sensor's timestamp-matched world Z for commit."""
        if not math.isfinite(height):raise ValueError('pct_invalid_frontier_query_height')
        self._pending_frontier_query_height=float(height)

    def _reset_slice_products(self):
        super()._reset_slice_products()
        self._frontier_height_events.clear()
        self._frontier_column_events.clear()
        self._ordered_passages.clear()
        self._frontier_event_bands=[]
        self._frontier_endpoint_buckets={}
        self._frontier_bucket_keys={}
        self.radar_frontier_layer=None
        self._radar_known_cache.clear()
        if self.frontier_query_height is not None:
            self._frontier_query_changes.update(self.columns)

    @property
    def frontier_query_pending(self):
        return self._pending_frontier_query_height is not None

    def _prepare_frontier_query(self):
        height=self._pending_frontier_query_height
        if height is None:return
        old=self.frontier_query_height
        if old is None:
            changed=set(self.columns)
        elif old==height:
            changed=set()
        else:
            low,high=sorted((old,height));dh=self.settings.slice_dh
            first=bisect_left(self._frontier_event_bands,math.floor(low/dh))
            last=bisect_right(self._frontier_event_bands,math.floor(high/dh))
            changed=set()
            for band in self._frontier_event_bands[first:last]:
                keys=self._frontier_bucket_keys.get(band)
                if keys is None:
                    keys=self._frontier_bucket_keys[band]=tuple(sorted(self._frontier_endpoint_buckets[band]))
                lo=bisect_left(keys,low);hi=bisect_right(keys,high)
                for z in keys[lo:hi]:changed.update(self._frontier_height_events[z])
        self.frontier_query_height=height
        self._pending_frontier_query_height=None
        self._frontier_query_checked=len(changed)
        self._frontier_query_changes.update(changed)
        self.evidence_dirty.update(changed)

    def _index_column_intervals(self, xy):
        result=super()._index_column_intervals(xy)
        # Sorted endpoint events let a moving query visit only columns whose
        # known/unknown/obstacle classification can actually have changed.
        endpoints=set()
        indexed=self._column_intervals.get(xy)
        if indexed is not None:
            for g,c,_ in indexed[2]:endpoints.update((g+1e-7,c-1e-7))
        column=self.columns.get(xy)
        if column is not None:
            for low,high in column.obstacle_intervals:
                endpoints.update((low-1e-7,high+1e-7))
        endpoints={z for z in endpoints if math.isfinite(z)}
        previous=self._frontier_column_events.get(xy,set())
        for z in previous-endpoints:
            owners=self._frontier_height_events[z];owners.discard(xy)
            if not owners:
                del self._frontier_height_events[z]
                band=math.floor(z/self.settings.slice_dh)
                values=self._frontier_endpoint_buckets[band];values.remove(z)
                self._frontier_bucket_keys.pop(band,None)
                if not values:
                    del self._frontier_endpoint_buckets[band]
                    self._frontier_event_bands.pop(bisect_left(self._frontier_event_bands,band))
        for z in endpoints-previous:
            if z not in self._frontier_height_events:
                self._frontier_height_events[z]=set()
                band=math.floor(z/self.settings.slice_dh)
                if band not in self._frontier_endpoint_buckets:
                    self._frontier_endpoint_buckets[band]=set();insort(self._frontier_event_bands,band)
                self._frontier_endpoint_buckets[band].add(z)
                self._frontier_bucket_keys.pop(band,None)
            self._frontier_height_events[z].add(xy)
        if endpoints:self._frontier_column_events[xy]=endpoints
        else:self._frontier_column_events.pop(xy,None)
        return result

    def _query_frontier_updates(self, frontiers, columns, new_layers):
        if self.frontier_query_height is None:return
        touched=columns | self._frontier_query_changes
        if self.radar_frontier_layer is None and self.layers:
            self.radar_frontier_layer=min(self.layers)
            touched=touched | self.columns.keys()
        for xy in touched:self._radar_known_cache.pop(xy,None)
        # The current Z defines one boundary topology. Other cut indexes are
        # aliases of this view, not independently rebuilt copies of it.
        frontiers.clear()
        if self.radar_frontier_layer is not None:
            frontiers[self.radar_frontier_layer].update(touched)
        for k,index in self.frontier_index.slices.items():
            if k!=self.radar_frontier_layer and (index.nodes or index.frontier_inputs):
                frontiers[k].update(index.frontier_inputs)
        self._frontier_query_changes.clear()

    def frontier_cell(self, layer, xy):
        if self.frontier_query_height is None:return super().frontier_cell(layer,xy)
        if xy not in self._radar_known_cache:
            self._radar_known_cache[xy]=self.observed_at_height(xy,self.frontier_query_height)
        return self._radar_known_cache[xy]

    def frontier_cost(self, layer, xy, row=None):
        if self.frontier_query_height is None:return super().frontier_cost(layer,xy,row)
        row=self.frontier_cell(layer,xy) if row is None else row
        if row is None or row['unknown'] or not math.isfinite(float(row['G'])):return -1.
        # Safety belongs to the referenced physical node, not an unrelated
        # fixed query address which may intersect a lower riser or ceiling.
        owner=math.floor((float(row['G'])-self.settings.origin_z+1e-7)/self.settings.slice_dh)+1
        return self.planning_cost((owner,*xy))

    def frontier_selection_layer(self, ground):
        layer,info=super().frontier_selection_layer(ground)
        if self.frontier_query_height is not None:
            info=dict(info,address_height=info['height'],height=self.frontier_query_height,
                      reason='primary_lidar_height')
        return layer,info

    def _remember_obstacles(self, column, heights):
        s=self.settings
        previous=column.obstacle_intervals
        previous_walls=column._obstacle_samples
        # _obstacle_samples is the authoritative compact list of wall/column
        # intervals, including singleton surfaces. Noise grouping is separate.
        column.remember_obstacles(heights,s.column_continuity_threshold+2e-7,
                                  s.surface_noise_tolerance)
        # Too-small gaps are known unusable space, not unknown frontiers.
        # Keep their exclusion spans separate from the raw wall intervals.
        merged=[]
        for low,high in column._obstacle_samples:
            if merged and low-merged[-1][1]<s.minimum_clearance-1e-7:
                merged[-1]=(merged[-1][0],high)
            else:merged.append((low,high))
        column.obstacle_intervals=tuple((lo,hi) for lo,hi in merged
            if hi-lo>s.surface_noise_tolerance+1e-7)
        return previous!=column.obstacle_intervals or previous_walls!=column._obstacle_samples

    def integrate(self, points, now=None, sensor_origin=None, body_height=None, free_rays=None):
        points=np.asarray(points,dtype=np.float32).reshape(-1,3)
        points=points[np.isfinite(points).all(axis=1)]
        if body_height is not None and not math.isfinite(body_height):
            raise ValueError('pct_invalid_body_height')
        s=self.settings
        free={}
        rays=points
        if free_rays is not None:
            extra=np.asarray(free_rays,dtype=np.float32).reshape(-1,3)
            extra=extra[np.isfinite(extra).all(axis=1)]
            if len(extra):rays=np.concatenate((points,extra))
        if sensor_origin is not None and len(rays) and s.reliable_observation_range>0:
            origin=np.asarray(sensor_origin,dtype=float)
            if origin.shape!=(3,) or not np.isfinite(origin).all():
                raise ValueError('pct_invalid_sensor_origin')
            free=self._ray_kernel(rays,origin,s.resolution,s.origin_x,s.origin_y,s.origin_z,
                                  s.reliable_observation_range,s.max_cells)
            if len(self._free_columns)+sum(xy not in self._free_columns for xy in free)>s.max_cells:
                raise ValueError('pct_free_column_limit')
        count=super().integrate(points,now,sensor_origin)
        if self.error:return count
        if sensor_origin is None and len(points):
            # The existing origin-less API imports explicitly prelabelled
            # finite surfaces. Production LiDAR always has a paired origin;
            # absence of new online rays must never promote an old endpoint.
            for xy in set(map(tuple,self.indices_array(points))):
                xy=tuple(map(int,xy));column=self.columns[xy]
                trusted=self._confirmed_grounds.setdefault(xy,{})
                for record in column.values():
                    if record.last_seen==self.integrated_serial:
                        if trusted.get(record.id)!=record.high:self.dirty.add(xy)
                        trusted[record.id]=record.high
        # Retain free evidence only from actual rays. Neither a body pose nor
        # the sensor range creates free voxels. Updating these bounds dirties
        # the affected ground columns, even when frozen G means did not move.
        for xy,spans in free.items():
            previous=self._free_columns.get(xy,())
            current=merge_intervals(previous,spans)
            if previous!=current:
                self._free_columns[xy]=current
                if xy in self.columns:self.dirty.add(xy)
        h=float(body_height) if body_height is not None else None
        if h is not None and len(points):
            for xy in set(map(tuple,self.indices_array(points))):
                xy=tuple(map(int,xy))
                previous=self._top_observed.get(xy,-math.inf)
                # A paired pose permits the top candidate on this observed
                # column; it does not clear unknown XYs or establish C.
                if h>previous:
                    self._top_observed[xy]=h
                    walls=self.columns[xy]._obstacle_samples
                    if walls and previous<walls[-1][1]<=h:self.dirty.add(xy)
        return count

    def indices_array(self, points):
        s=self.settings
        return np.floor((points[:,:2]-[s.origin_x,s.origin_y])/s.resolution+.5).astype(np.int64)

    def _open_clearance(self, xy, ground):
        s=self.settings
        # A return endpoint occupies its voxel. Permit only that one voxel's
        # quantisation gap next to G, never an arbitrary unobserved gap.
        spans=self._free_columns.get(xy,())
        limit=math.floor((ground+s.resolution+1e-7-s.origin_z)/s.resolution)
        at=bisect_right(spans,(limit,math.inf))-1
        if at>=0:
            low,high=spans[at]
            bottom=s.origin_z+low*s.resolution
            top=s.origin_z+(high+1)*s.resolution
            if bottom<=ground+s.resolution+1e-7 and top>ground:
                return min(top,ground+s.open_ceiling_share_height)
        return math.nan

    def _column_cuts(self, xy, geometry):
        s=self.settings;column=self.columns[xy]
        walls=column._obstacle_samples
        heights,records=geometry
        nodes={};pending=None;pending_ground=[]
        if records and walls:
            trusted=self._confirmed_grounds.setdefault(xy,{})
            live_ids={r.id for r in records}
            for identity in trusted.keys()-live_ids:del trusted[identity]
            def endpoint(height):
                pad=s.surface_noise_tolerance+1e-7
                lo=bisect_left(heights,height-pad);hi=bisect_right(heights,height+pad)
                nearby=records[lo:hi]
                if not nearby:
                    at=bisect_left(heights,height)
                    nearby=records[max(0,at-1):at+1]
                return min(nearby,key=lambda r:(min(abs(r.low-height),abs(r.high-height)),
                                               abs(r.height-height),r.id))
            def add(g,c,clear):
                support=endpoint(g)
                upper=endpoint(c) if math.isfinite(c) else None
                node=PassageNode(g,c,clear,g+(clear-g)*.5,support.id,
                    upper.id if upper else 0,support.mature,upper.mature if upper else False)
                k=math.floor((g-s.origin_z+1e-7)/s.slice_dh)+1
                if k in nodes:raise ValueError('pct_interval_address_collision')
                old=self._interval_nodes.get(xy,{}).get(k)
                if (old is not None and old.ground==g and old.clear_to==clear and
                    (old.ceiling==c or math.isnan(old.ceiling) and math.isnan(c)) and
                    (old.ground_id,old.ceiling_id,old.ground_mature,old.ceiling_mature)==
                    (node.ground_id,node.ceiling_id,node.ground_mature,node.ceiling_mature)):
                    node=old
                nodes[k]=node
                trusted[support.id]=g
            def support_evidence(ground):
                support=endpoint(ground)
                previous=trusted.get(support.id,math.nan)
                confirmed=(math.isfinite(previous) and
                           abs(previous-ground)<=s.surface_noise_tolerance+1e-7)
                # Free rays through this column prove headroom, not a hit on
                # its lower surface. Only that surface's own return from
                # above may establish new support provenance. Every return
                # still participates in the ordered occupied intervals.
                hit=support.support_hit_z
                direct=(hit is not None and abs(hit-ground)<=s.surface_noise_tolerance+1e-7)
                return confirmed,direct
            # Linear scan of ordered occupied spans. Only internal spaces
            # with sufficient clearance receive navigation nodes.
            for (_,ground),(ceiling,_) in zip(walls,walls[1:]):
                if ceiling-ground+1e-7<s.minimum_clearance:continue
                confirmed,direct=support_evidence(ground)
                # Both enclosing endpoints were measured. The direct lower
                # hit proves support provenance; a vertically complete ray
                # column is required only when the upper boundary is absent.
                # An oblique stair observation need not sweep every height.
                if confirmed or direct:
                    add(ground,ceiling,ceiling)
                else:pending_ground.append((ground,ceiling,self._open_clearance(xy,ground)))
            ground=walls[-1][1]
            if s.allow_open_ceiling and self._top_observed.get(xy,-math.inf)>=ground-1e-7:
                confirmed,direct=support_evidence(ground)
                clear=self._open_clearance(xy,ground)
                if ((confirmed or direct) and math.isfinite(clear) and
                        clear-ground+1e-7>=s.minimum_clearance):
                    add(ground,math.nan,clear)
                else:pending=(ground,clear)
        if pending is None:self._pending_open.pop(xy,None)
        else:self._pending_open[xy]=pending
        self._pending_ground_count+=len(pending_ground)-len(self._pending_ground.get(xy,()))
        if pending_ground:self._pending_ground[xy]=tuple(pending_ground)
        else:self._pending_ground.pop(xy,None)
        self._interval_node_count+=len(nodes)-len(self._interval_nodes.get(xy,()))
        self._interval_nodes[xy]=nodes
        self._ordered_passages[xy]=tuple((n.ground,k,n.clear_to) for k,n in sorted(nodes.items()))
        if nodes:
            self.first_layer=min(nodes) if self.first_layer is None else min(self.first_layer,min(nodes))
            self.last_layer=max(nodes) if self.last_layer is None else max(self.last_layer,max(nodes))
            # Scheduling retains one query plane above G+16 cm. This index
            # owns no copied G/C cells: it reads the same interval node.
            from .tomographic_slice import TomographicSlice
            for node in nodes.values():
                k=math.floor((node.ground+.16-s.origin_z+1e-7)/s.slice_dh)+1
                if s.origin_z+k*s.slice_dh>=node.clear_to-1e-7:continue
                if k not in self.layers:self.layers[k]=TomographicSlice(self,k)
                self.first_layer=min(self.first_layer,k);self.last_layer=max(self.last_layer,k)
        x,y=xy
        self.lower=np.array([x,y]) if self.lower is None else np.minimum(self.lower,[x,y])
        self.upper=np.array([x+1,y+1]) if self.upper is None else np.maximum(self.upper,[x+1,y+1])
        return {k:n.surfaces() for k,n in nodes.items()}

    def _geometry_signature(self, xy, geometry):
        return tuple((k,n.ground,n.ceiling if math.isfinite(n.ceiling) else None,n.clear_to,
                      n.ground_id,n.ceiling_id,n.ground_mature,n.ceiling_mature)
                     for k,n in sorted(self._interval_nodes[xy].items()))

    def _changed_cuts(self, previous, current, layers, previous_layers):
        old={row[0]:row for row in previous};new={row[0]:row for row in current}
        return [k for k in layers if old.get(k)!=new[k]]

    def _commit_node_metadata(self, batches, changes):
        for k,batch in batches.items():
            base,ground=changes[k]
            for xy in batch:
                row=self.cell((k,*xy));node=self._interval_nodes.get(xy,{}).get(k)
                clear=node.clear_to if node else math.nan
                middle=node.midpoint if node else math.nan
                if (different(float(row['C_clear']),clear) or different(float(row['node_z']),middle)
                        or bool(row['interval_node'])!=(node is not None)):
                    base.add(xy)
                row['C_clear'],row['node_z'],row['interval_node']=clear,middle,node is not None

    def observed_at_height(self, xy, cut, ground_height=None, *, layer_hint=None):
        # Physical passages are disjoint and ordered: at most one can own Z.
        # Never join endpoints from different rooms or materialise an alias.
        xy=tuple(xy)
        entries=self._ordered_passages.get(xy,())
        at=bisect_left(entries,(cut-1e-7,))-1
        column=self.columns.get(xy)
        blocked=column is not None and column.obstacle_at(cut) is not None
        if at>=0:
            g,k,upper=entries[at]
            if (cut<upper-1e-7 and (ground_height is None or
                    abs(g-ground_height)<=self.settings.surface_noise_tolerance+1e-7)):
                row=self._effective_row(self.cell((k,*xy)))
                if self._covers_cut(row,cut,ground_height):
                    return self._obstacle_view(row) if blocked else row
        return self._blocked_cell if blocked else None

    def _effective_row(self, row):
        if row is None:return None
        if row['interval_node'] and math.isnan(row['C']):
            if not math.isfinite(row['C_clear']):return None
            row=row.copy();row['C']=row['C_clear']
        return row

    def _covers_cut(self, row, cut, ground_height=None):
        return super()._covers_cut(self._effective_row(row),cut,ground_height)

    def footprint_support(self, layer, x, y, reference):
        # A compatibility query plane can intersect the riser BELOW an
        # already measured tread. Foot support belongs to the actual passage
        # node, not that plane. Keep the inherited two-step disk-height bound;
        # actual movement edges still enforce the single-step limit.
        if math.isfinite(reference):
            spans=self._column_known_intervals.get((x,y),())
            limit=2*self.settings.step_max+1e-7
            lo=bisect_left(spans,(reference-limit,))
            hi=bisect_right(spans,(reference+limit,math.inf))
            choices=[]
            for g,_,_,_,_ in spans[lo:hi]:
                k=math.floor((g-self.settings.origin_z+1e-7)/self.settings.slice_dh)+1
                row=self.observed_cell((k,x,y))
                if row is not None and not row['unknown']:choices.append(row)
            if choices:return min(choices,key=lambda row:abs(float(row['G'])-reference))
        return super().footprint_support(layer,x,y,reference)

    def tread_above_riser(self, layer, x, y, row, reference):
        blocked=self.obstacle_at((layer,x,y))
        if blocked is None:return None
        spans=self._column_known_intervals.get((x,y),())
        step=self.settings.step_max+1e-7
        lo=bisect_left(spans,(reference-step,))
        hi=bisect_right(spans,(reference+step,math.inf))
        for g,_,_,_,_ in spans[lo:hi]:
            if g<blocked[1]-1e-7:continue
            owner=math.floor((g-self.settings.origin_z+1e-7)/self.settings.slice_dh)+1
            support=self.observed_cell((owner,x,y))
            if (support is not None and not support['solid'] and not support['unknown'] and
                    0<=support['terrain_cost']<=20):return support
        return None

    def _commit_planning_versions(self, revision):
        # FixedLayerMap commits versions for the changed physical/virtual
        # height addresses. No column-wide generation: upper-room updates
        # must not invalidate routes which only read the lower room.
        pass

    def build(self, now=None):
        result=super().build(now)
        self.stats.update(mapping_mechanism='xy_passage_intervals',
            interval_nodes=self._interval_node_count,pending_open_intervals=len(self._pending_open),
            pending_ground_intervals=self._pending_ground_count,
            free_evidence_columns=len(self._free_columns),
            column_continuity_threshold=self.settings.column_continuity_threshold,
            node_height_kind='per_column_interval_midpoint')
        self.stats['query_address_spacing']=self.settings.slice_dh
        self.stats['frontier_query_height']=self.frontier_query_height
        self.stats['frontier_height_changed_columns']=self._frontier_query_checked
        self.stats['shared_frontier_topologies']=int(self.radar_frontier_layer is not None)
        self.stats['frontier_cut_references']=len(self.layers)
        self._frontier_query_checked=0
        return result
