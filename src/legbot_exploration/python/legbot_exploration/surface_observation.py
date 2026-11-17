"""Shared, height-specific observation evidence; hypotheses never write G/C."""
from collections import defaultdict
from bisect import bisect_left, insort
from dataclasses import dataclass, field
import math
import numpy as np

FRONTIER_GROUND_TOLERANCE = .16


@dataclass(frozen=True)
class FloorGroundBand:
    height: float
    lower_ground: float
    upper_ground: float
    cutoff: int
    arrival_frame: int
    first_slice: int

    def includes(self, layer, ground):
        # G is nondecreasing within a column. A higher independent G
        # terminates this floor's dependency range, including all its aliases.
        return layer >= self.first_slice and (
            not math.isfinite(ground) or ground <= self.upper_ground+1e-7)

    def ground_seen(self, ground):
        if ground is None:
            return False
        # Near-platform hits from the approach are reusable. After arrival,
        # a new hit can resolve a work cell at its measured G.
        cutoff = self.cutoff if ground.height >= self.lower_ground-1e-7 else self.arrival_frame
        return ground.last_seen > cutoff


@dataclass
class SurfaceExpectation:
    id: int
    xy: tuple
    height: float
    sources: tuple
    serial: int
    kind: str = 'ground'
    slices: set = field(default_factory=set)
    required_mask: int = 0xffff
    state: str = 'pending'
    resolved_at: int = 0
    free_masks: list = field(default_factory=lambda: [0, 0, 0])
    higher_mask: int = 0
    reviewed: bool = False
    height_range: tuple | None = None
    actual_serial: int | None = None
    center_range: tuple | None = None
    neighbor_inferred: bool = False

    def __post_init__(self):
        if self.center_range is None:
            self.center_range = (self.height, self.height)


class SurfaceObservation:
    """Query accumulated surfaces and watch only explicitly requested local targets.

    Negative evidence uses real incoming rays, ending at their actual returns.
    Three planes span the expected height band; every requested 4x4 subcell
    must be covered on all three planes. No raw cloud or full-map ray history
    persists. A new, higher observed surface can supersede a ground hypothesis
    only after covering its requested footprint, never erase a real surface.
    """
    def __init__(self, mapping):
        self.map = mapping
        self.by_column = defaultdict(list)
        self.records = {}
        self.pending_tiles = defaultdict(set)
        self.next_id = 1
        self.revision = 0
        self.block_versions = {}
        self.visited_heights = []
        self.highest_measured = None
        self.height_triggers = 0
        self.last_height = None
        self.height_serial = 0
        self.frontier_floors = {}
        self.ground_bands = ()
        self.last_registered_floor_frame = None
        self.last_work = dict(ray_bands=0, ray_targets=0, changed_columns=0)

    @staticmethod
    def _compatible(first, second):
        if not first or not second:
            return True
        a = {(x, y): sid for x, y, sid in first}
        b = {(x, y): sid for x, y, sid in second}
        common = a.keys() & b.keys()
        return bool(common) and all(a[xy] == b[xy] for xy in common)

    def observed(self, xy, after=None, kind='geometry'):
        column = self.map.columns.get(tuple(xy))
        return tuple(r for r in column.values()
                     if math.isfinite(r.height) and
                     (kind not in ('ground','search') or r.orientation in (0, 1)) and
                     (kind != 'frontier' or r.orientation != -1) and
                     (after is None or r.last_seen > after)) if column else ()

    def matching(self, xy, height, after=None, kind='ground'):
        tolerance = self.map.settings.surface_noise_tolerance + 1e-7
        return tuple(r for r in self.observed(xy,after,kind) if abs(r.height-height) <= tolerance)

    def floor_ground_band(self, height):
        nearest = min((floor for floor, (tolerance, _, _) in self.frontier_floors.items()
                       if floor is not None and abs(floor-height) <= tolerance+1e-7),
                      key=lambda floor: abs(floor-height), default=None)
        return next((band for band in self.ground_bands if band.height==nearest),None)

    def first_ground_slice(self, height):
        s=self.map.settings
        # Match SurfaceColumn.at: a return at the cut can supply G.
        return math.ceil((height-s.origin_z-1e-7)/s.slice_dh)

    def begin_crossing(self):
        self.frontier_floors.setdefault(None,(0.,self.map.integrated_serial,None))

    def cancel_crossing(self):
        self.frontier_floors.pop(None,None)

    def enter_frontier_floor(self, ground, tolerance):
        height = float(ground[2])
        crossing=self.frontier_floors.pop(None,None)
        if any(abs(floor-height) <= max(tolerance, band)+1e-7
               for floor, (band, _, _) in self.frontier_floors.items()):
            return False
        cutoff=None
        if crossing is not None:
            cutoff=crossing[1]
        elif self.frontier_floors:
            cutoff=(self.last_registered_floor_frame if self.last_registered_floor_frame is not None
                    else self.map.integrated_serial)
        self.frontier_floors[height] = (tolerance, cutoff, self.map.integrated_serial)
        heights=sorted(self.frontier_floors,reverse=True)
        bands=[]
        above=math.inf
        for h in heights:
            band_tolerance,start,arrival=self.frontier_floors[h]
            if start is not None:
                lower_ground=h-min(band_tolerance,self.map.settings.surface_noise_tolerance)
                bands.append(FloorGroundBand(h,lower_ground,min(h+band_tolerance,(h+above)*.5),start,arrival,
                                             self.first_ground_slice(h)))
            above=h
        self.ground_bands=tuple(bands)
        if cutoff is not None:
            affected = self._discard_inferred_floor_records(height,tolerance)
            self.map.evidence_dirty.update(affected)
            self.map.schedule_floor_ground(self.floor_ground_band(height))
        return True

    def _discard_inferred_floor_records(self, height, tolerance):
        affected=set()
        first_slice=self.first_ground_slice(height)
        retired=[record for record in self.records.values()
                 if record.neighbor_inferred and abs(record.height-height)<=tolerance+1e-7]
        for record in retired:
            lower_slices={k for k in record.slices if k<first_slice}
            if lower_slices:
                if record.slices!=lower_slices:
                    record.slices=lower_slices
                    self._changed(record.xy)
                    affected.add(record.xy)
                continue
            del self.records[record.id]
            identities=self.by_column[record.xy]
            identities.remove(record.id)
            if not identities:
                del self.by_column[record.xy]
            self._discard_pending(record)
            self._changed(record.xy)
            affected.add(record.xy)
        return affected

    def has_frontier_ground(self, xy, height, after=None, *, height_span=None, layer=None):
        """An observed ground can cover an ordinary frontier without matching its surface."""
        low, high = (height, height) if height_span is None else height_span
        tolerance = FRONTIER_GROUND_TOLERANCE + 1e-7
        if layer is None:layer=self.first_ground_slice(height)
        if self.map.ground_pending(xy,layer):
            return False
        return any(high-tolerance <= r.height <= low+tolerance
                   for r in self.observed(xy,after,kind='ground'))

    def frontier_settled(self, xy, height, sources=(), *, layer=None):
        if layer is None:layer=self.first_ground_slice(height)
        return not self.map.ground_pending(xy,layer) and (
            self.has_frontier_ground(xy,height,layer=layer) or self.refuted(xy,height,sources,include_step=False))

    def is_ground_source(self, xy, identity):
        # Explicit imported/test planner cells may have no column reference.
        # Real online cells always retain their physical surface identity.
        return not identity or any(r.id==identity and r.orientation!=-1 for r in self.observed(xy))

    def local_sources(self, point, kind='ground'):
        x,y=map(int,self.map.indices(point))
        return tuple((x+dx,y+dy,r.id) for dx in (-1,0,1) for dy in (-1,0,1)
                     for r in self.matching((x+dx,y+dy),point[2],kind=kind))

    def ground_conflicts(self, xy, height, layer=None):
        """A newly conflicting local hypothesis cannot borrow an old lower hit.

        Separate floors remain independent. A fresh, different ground return
        may verify this actual support without disproving another sheet.
        """
        noise=self.map.settings.surface_noise_tolerance+1e-7
        for identity in self.by_column.get(tuple(xy), ()):
            r=self.records[identity]
            if (r.state=='pending' and r.kind=='ground' and
                    (layer is None or not r.slices or layer in r.slices) and
                    noise<r.height-height<=self.map.settings.step_max+1e-7 and
                    not self.matching(xy,height,r.serial)):
                yield r

    def ground_observed(self, xy, height, layer=None):
        return next(self.ground_conflicts(xy,height,layer),None) is None

    def describe_ground(self, xy, reference, layer=None):
        """Explain the same support gates used by arrival, without changing G/C."""
        s=self.map.settings
        rows=[]
        for k,row in self.map.layers_at(*xy):
            if layer is not None and k!=layer:continue
            if not np.isfinite(row['G']):continue
            h=float(row['G']);sid=int(row['g_surface_id'])
            source=next((r for r in self.observed(xy) if r.id==sid),None)
            conflicts=list(self.ground_conflicts(xy,h,k))
            reason=('ground_unobserved' if row['unknown'] or not row['measured'] else
                    'ground_source_invalid' if not self.is_ground_source(xy,sid) else
                    'surface_observation_pending' if conflicts else
                    'surface_height_mismatch' if abs(h-reference)>s.step_max+1e-5 else 'ground_observed')
            rows.append(dict(slice=int(k),G=h,cost=float(row['inflated_terrain_cost']),surface_id=sid,
                             orientation=None if source is None else int(source.orientation),
                             last_seen=None if source is None else int(source.last_seen),reason=reason,
                             pending_expectations=[dict(id=r.id,height=r.height,serial=r.serial,
                                 slices=sorted(r.slices),sources=r.sources,actual_serial=r.actual_serial)
                                 for r in conflicts]))
        compatible=[r for r in rows if abs(r['G']-reference)<=s.step_max+1e-5]
        reason=('surface_disconnected' if any(r['reason']=='ground_observed' for r in compatible) else
                min(compatible,key=lambda r:abs(r['G']-reference))['reason'] if compatible else
                'surface_height_mismatch' if rows else 'ground_unobserved')
        return dict(cell=list(map(int,xy)),reference_height=float(reference),reason=reason,layers=rows)

    def pending_at(self, xy, reference, layer=None):
        return tuple(r for identity in self.by_column.get(tuple(xy), ()) for r in (self.records[identity],)
                     if r.state=='pending' and (layer is None or not r.slices or layer in r.slices) and
                     abs(r.height-reference)<=self.map.settings.step_max+1e-7)

    def boundary_signature(self, xy):
        return tuple((r.height,r.sources,r.state,r.resolved_at)
                     for identity in self.by_column.get(tuple(xy), ()) for r in (self.records[identity],)
                     if r.state in ('refuted','superseded','step_boundary'))

    def _tile(self, xy):
        b = self.map.settings.block_size
        return xy[0]//b, xy[1]//b

    def _settle(self, record, state, serial):
        record.state, record.resolved_at = state, serial
        self._discard_pending(record)
        self._changed(record.xy)

    def _discard_pending(self, record):
        tile = self._tile(record.xy)
        pending = self.pending_tiles.get(tile)
        if pending is not None:
            pending.discard(record.id)
            if not pending:
                del self.pending_tiles[tile]

    def _changed(self, xy):
        self.revision += 1
        self.block_versions[self._tile(xy)] = self.revision
        self.map.evidence_dirty.add(tuple(xy))

    def _refresh_relation(self, record):
        if record.neighbor_inferred:
            if record.state not in ('pending','frontier_covered'):
                return
            covered=self.has_frontier_ground(record.xy,record.height,height_span=record.center_range)
            if covered and record.state=='pending':
                self._settle(record,'frontier_covered',self.map.integrated_serial)
            elif not covered and record.state=='frontier_covered':
                self._reopen(record)
            return
        if record.kind!='ground' or record.height_range is not None or record.state not in ('pending','step_boundary'):
            return
        relation=self.map.surface_relations.classify(record.xy,record.height)
        if self.map.surface_relations.explains_step(record.xy,record.height,record.sources):
            if record.state!='step_boundary':self._settle(record,'step_boundary',self.map.integrated_serial)
        elif record.state=='step_boundary':
            # Later evidence can reveal overlapping floors. Reopen just this
            # physical hypothesis without deleting surfaces or resetting attempts.
            if relation.state=='matched':self._settle(record,'matched',self.map.integrated_serial)
            else:self._reopen(record)

    def _reopen(self, record):
        record.state='pending'
        record.serial=self.map.integrated_serial
        record.free_masks,record.higher_mask,record.actual_serial=[0,0,0],0,None
        self.pending_tiles[self._tile(record.xy)].add(record.id)
        self._changed(record.xy)

    def refresh_relations(self, columns):
        for xy in columns:
            for identity in self.by_column.get(xy,()):self._refresh_relation(self.records[identity])

    def arm_review(self, record, serial):
        if record.state!='pending':return
        if record.serial!=serial or record.actual_serial is not None:
            self._changed(record.xy)
        record.serial=serial
        record.free_masks,record.higher_mask,record.actual_serial=[0,0,0],0,None

    def expect(self, point, *, sources=(), slices=(), review=False, kind='ground', required_mask=0xffff,
               height_range=None, neighbor_inferred=False):
        height = float(point[2])
        neighbor_inferred = neighbor_inferred or kind=='frontier'
        if neighbor_inferred and self.floor_ground_band(height) is not None:
            return None
        xy = tuple(map(int, self.map.indices(point)))
        sources = tuple(sorted(set(tuple(map(int, source)) for source in sources)))
        noise = self.map.settings.surface_noise_tolerance + 1e-7
        for identity in self.by_column.get(xy, ()):
            record = self.records[identity]
            low,high=min(record.center_range[0],height),max(record.center_range[1],height)
            widening=(low,high)!=record.center_range
            if (record.kind == kind and high-low <= noise and record.height_range==height_range and
                    record.neighbor_inferred==neighbor_inferred and
                    record.required_mask==required_mask and
                    (not widening or record.state=='pending') and
                    self._compatible(record.sources, sources)):
                if widening:
                    # A proof for the previous band cannot prove the widened band.
                    record.center_range=(low,high)
                    record.free_masks,record.higher_mask=[0,0,0],0
                    self._changed(xy)
                record.slices.update(map(int, slices))
                if review and not record.reviewed and record.state == 'matched':
                    record.serial = self.map.integrated_serial
                    record.state = 'pending'
                    record.free_masks, record.higher_mask = [0, 0, 0], 0
                    record.actual_serial=None
                    self.pending_tiles[self._tile(xy)].add(record.id)
                    self._changed(xy)
                record.reviewed |= review
                self._refresh_relation(record)
                return record
        record = SurfaceExpectation(self.next_id, xy, height, sources, self.map.integrated_serial,
                                    kind=kind, slices=set(map(int, slices)), required_mask=required_mask,
                                    reviewed=review,height_range=height_range,neighbor_inferred=neighbor_inferred)
        self.next_id += 1
        self.records[record.id] = record
        self.by_column[xy].append(record.id)
        if not review and self._matches(record):
            record.state = 'matched'
        else:
            self.pending_tiles[self._tile(xy)].add(record.id)
            self._changed(xy)
        self._refresh_relation(record)
        return record

    def refuted(self, xy, height, sources=(), after=None, *, include_step=True):
        if self.matching(xy,height,kind='geometry'):
            return False
        if include_step and self.map.surface_relations.explains_step(xy,height,sources):return True
        return any(r.kind!='search' and r.state in ('refuted', 'superseded') and
                   r.center_range[0]-1e-7<=height<=r.center_range[1]+1e-7 and self._covers_height(r,height) and
                   self._compatible(r.sources, sources) and (after is None or r.resolved_at > after)
                   for identity in self.by_column.get(tuple(xy), ()) for r in (self.records[identity],))

    def ceiling_band_seen_free(self,xy,height,after):
        return any(r.state=='refuted' and r.kind!='search' and r.resolved_at>after and
                   r.center_range[0]-1e-7<=height<=r.center_range[1]+1e-7 and self._covers_height(r,height)
                   for identity in self.by_column.get(xy,()) for r in (self.records[identity],))

    def _band(self, record):
        noise=self.map.settings.surface_noise_tolerance
        lo,hi=record.height_range if record.height_range is not None else (
            record.center_range[0]-noise,record.center_range[1]+noise)
        return round((lo+hi)*.5/.01), math.ceil((hi-lo)*.5/.01)

    def _covers_height(self, record, height):
        band,half=self._band(record)
        return abs(band*.01-height)+self.map.settings.surface_noise_tolerance <= half*.01+.005+1e-7

    def _has_actual_in_band(self, record):
        noise=self.map.settings.surface_noise_tolerance
        lo,hi=record.height_range if record.height_range is not None else (
            record.center_range[0]-noise,record.center_range[1]+noise)
        return any(lo-1e-7<=r.height<=hi+1e-7 for r in self.observed(record.xy))

    def _matches(self, record, after=None):
        if record.height_range is None:
            noise=self.map.settings.surface_noise_tolerance+1e-7
            lo,hi=record.center_range
            return tuple(r for r in self.observed(record.xy,after,record.kind)
                         if hi-noise<=r.height<=lo+noise)
        lo,hi=record.height_range
        return tuple(r for r in self.observed(record.xy,after,record.kind) if lo<=r.height<=hi)

    def query(self, point, *, after=None, sources=(), kind='ground', expectation=None, layer=None):
        if expectation is not None:
            sources,kind=expectation.sources,expectation.kind
        xy = tuple(map(int, self.map.indices(point)))
        ordinary=kind=='frontier' or (expectation is not None and expectation.neighbor_inferred)
        if layer is None:layer=self.first_ground_slice(point[2])
        if ordinary and self.map.ground_pending(xy,layer):
            return dict(state='unresolved',ready=False,
                        surfaces=[],new_heights=[],lower_observed=False,higher_observed=False)
        matches = (self._matches(expectation,after) if expectation is not None and expectation.height_range is not None
                   else self.matching(xy,float(point[2]),after,kind))
        fresh = self.observed(xy,after,kind) if after is not None else ()
        state='unresolved'
        if matches:
            state='matched'
        elif ordinary and self.has_frontier_ground(xy,point[2],after,layer=layer):
            state='frontier_covered'
        elif self.refuted(xy,point[2],sources,after,include_step=not ordinary):
            state='refuted'
        if (not matches and expectation is not None and expectation.state in ('refuted','superseded') and
                (expectation.height_range is not None or self._covers_height(expectation,point[2])) and
                (after is None or expectation.resolved_at>after)):
            state='refuted'
        noise = self.map.settings.surface_noise_tolerance + 1e-7
        return dict(state=state, ready=state != 'unresolved',
                    surfaces=[dict(id=r.id, height=r.height, last_seen=r.last_seen) for r in matches],
                    new_heights=sorted({r.height for r in fresh}),
                    lower_observed=any(r.height < point[2]-noise for r in fresh),
                    higher_observed=any(r.height > point[2]+noise for r in fresh))

    def visit_height(self, ground):
        """One trigger per newly reached height; all anchors below are real G."""
        height = float(ground[2])
        if any(h is not None and abs(h-height)<=min(tolerance,.04)+1e-7
               for h,(tolerance,_,_) in self.frontier_floors.items()):
            self.last_registered_floor_frame=self.map.integrated_serial
        self.highest_measured=height if self.highest_measured is None else max(height,self.highest_measured)
        if not self.visited_heights:
            self.visited_heights.append(height)
            self.height_serial=self.map.integrated_serial
            return dict(triggered=False, reason='height_baseline', **self.status())
        at=bisect_left(self.visited_heights,height)
        neighbors=self.visited_heights[max(0,at-1):at+1]
        if min(abs(height-h) for h in neighbors)<=self.map.settings.new_height_threshold+1e-7:
            return dict(triggered=False, reason='height_already_visited', **self.status())
        insort(self.visited_heights,height)
        self.height_triggers += 1
        self.last_height = height
        self.height_serial=self.map.integrated_serial
        if self.floor_ground_band(height) is not None:
            return dict(triggered=True, reason='new_measured_height', registered_targets=0, **self.status())
        m, s = self.map, self.map.settings
        radius = max(s.robot_radius+s.safety_margin, s.ground_lidar_min_range)+.15
        cells = {(x, y) for x, y, _ in m.footprint_cells(ground, radius)}
        # Grow a real, same-height component, not an inferred surface patch.
        center = tuple(map(int, m.indices(ground)))
        anchors, pending = {}, [center]
        seen = set()
        while pending:
            xy = pending.pop()
            if xy in seen or xy not in cells:
                continue
            seen.add(xy)
            choices = [(k, row) for k, row in m.layers_at(*xy) if s.origin_z+k*s.slice_dh>height+1e-7 and
                       row['measured'] and not row['unknown'] and
                       not row['solid'] and self.is_ground_source(xy,int(row['g_surface_id'])) and
                       abs(float(row['G'])-height) <= s.surface_noise_tolerance+1e-7]
            if not choices:
                continue
            k, row = min(choices, key=lambda item: (abs(float(item[1]['G'])-height), item[0]))
            h,sid=float(row['G']),int(row['g_surface_id'])
            cuts={cut for cut,other in choices if int(other['g_surface_id'])==sid and
                  abs(float(other['G'])-h)<=s.surface_noise_tolerance+1e-7}
            anchors[xy] = (h,sid,cuts)
            pending.extend((xy[0]+dx, xy[1]+dy) for dx, dy in ((0,1),(0,-1),(1,0),(-1,0)))
        targets=defaultdict(list)
        for xy, (h, sid, cuts) in anchors.items():
            if self.floor_ground_band(h) is not None:
                continue
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    target = xy[0]+dx, xy[1]+dy
                    if target not in cells:
                        continue
                    if self.has_frontier_ground(target,h):
                        continue
                    # All anchors belong to this measured connected patch.
                    # Merge compatible height spans before registering, so a
                    # different neighboring source does not create another task.
                    group=next((g for g in targets[target] if
                                max(g['high'],h)-min(g['low'],h)<=s.surface_noise_tolerance+1e-7),None)
                    if group is None:
                        group=dict(height=h,low=h,high=h,sources=set(),slices=set())
                        targets[target].append(group)
                    group['low']=min(group['low'],h);group['high']=max(group['high'],h)
                    group['sources'].add((*xy,sid));group['slices'].update(cuts)
        made=set()
        for target,groups in targets.items():
            for group in groups:
                record=self.expect((*m.position((0,*target)),group['height']),sources=group['sources'],
                                   slices=group['slices'],review=True,neighbor_inferred=True)
                made.add(record.id)
        return dict(triggered=True, reason='new_measured_height', registered_targets=len(made), **self.status())

    def status(self):
        return dict(highest_visited=self.highest_measured,
                    height_serial=self.height_serial,
                    height_anchor=self.last_height if self.last_height is not None else next(iter(self.visited_heights),None),
                    visited_heights=list(self.visited_heights),height_triggers=self.height_triggers,
                    frontier_floors=[dict(height=height,observation_start_frame=self.frontier_floors[height][1])
                                     for height in sorted(h for h in self.frontier_floors if h is not None)],
                    trigger_threshold=self.map.settings.new_height_threshold,
                    expectations=len(self.records), pending=sum(map(len, self.pending_tiles.values())),
                    **self.last_work)

    def pending_near(self, point, radius):
        m, s = self.map, self.map.settings
        lo, hi = m.indices(np.asarray(point[:2])-radius)//s.block_size, m.indices(np.asarray(point[:2])+radius)//s.block_size
        return [r for x in range(lo[0], hi[0]+1) for y in range(lo[1], hi[1]+1)
                for identity in self.pending_tiles.get((x, y), ()) for r in (self.records[identity],)
                if math.dist(m.position((0, *r.xy)), point[:2]) <= radius]

    def observe_frame(self, points, origin, serial, orientations=None):
        """Incremental positive hits and sparse negative ray coverage."""
        self.last_work = dict(ray_bands=0, ray_targets=0, changed_columns=0)
        if not self.pending_tiles or not len(points):
            return set()
        m, s = self.map, self.map.settings
        if origin is None:
            # Offline prelabelled surfaces carry no rays and cannot disprove a hypothesis.
            lo, hi = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
            center = (lo+hi)*.5
            active = self.pending_near(center, float(np.linalg.norm(hi-lo))*.5+s.resolution)
        else:
            reach = float(np.linalg.norm(points-origin, axis=1).max())+s.resolution
            active = self.pending_near(origin, reach)
        changed = set()
        pending = []
        for record in active:
            if self._matches(record,record.serial):
                self._settle(record, 'matched', serial)
                changed.add(record.xy)
            else:
                if record.actual_serial is None and self.observed(record.xy,record.serial,record.kind):
                    record.actual_serial=serial
                    self._changed(record.xy)
                    changed.add(record.xy)
                pending.append(record)
        if origin is None or not pending:
            self.last_work['changed_columns'] = len(changed)
            return changed
        noise = s.surface_noise_tolerance
        groups = defaultdict(list)
        for record in pending:
            groups[self._band(record)].append(record)
        delta = points.astype(float)-origin
        dz = delta[:, 2]
        valid_z = abs(dz) > 1e-9
        for (band, half), records in groups.items():
            by_xy = defaultdict(list)
            for record in records:
                by_xy[record.xy].append(record)
            # Quantization only widens the checked band, never narrows it.
            for plane, z in enumerate((band*.01-half*.01-.005, band*.01, band*.01+half*.01+.005)):
                t = np.divide(z-origin[2], dz, out=np.zeros(len(dz)), where=valid_z)
                selected = valid_z & (t > 1e-7) & (t < 1.-1e-7)
                self._cover(by_xy, origin[:2]+delta[selected, :2]*t[selected, None], plane)
            # A fully hit higher surface supersedes only an unobserved ground hypothesis.
            support = np.ones(len(points),dtype=bool) if orientations is None else orientations==1
            higher = points[support & (points[:, 2] > band*.01+half*.01+.005) &
                            (points[:, 2] <= band*.01+s.step_max)]
            self._cover(by_xy, higher[:, :2], None)
            for record in records:
                if (all(mask & record.required_mask == record.required_mask for mask in record.free_masks) and
                        not self._has_actual_in_band(record)):
                    self._settle(record, 'refuted', serial)
                    changed.add(record.xy)
                elif (record.kind == 'ground' and record.higher_mask & record.required_mask == record.required_mask and
                      not self._has_actual_in_band(record)):
                    self._settle(record, 'superseded', serial)
                    changed.add(record.xy)
        self.last_work = dict(ray_bands=len(groups), ray_targets=len(pending), changed_columns=len(changed))
        return changed

    def _cover(self, by_xy, points, plane):
        if not len(points):
            return
        s = self.map.settings
        cells = (points-[s.origin_x, s.origin_y])/s.resolution+.5
        xy = np.floor(cells).astype(np.int64)
        sub = np.clip(((cells-xy)*4).astype(np.int64), 0, 3)
        wanted=np.asarray(list(by_xy),dtype=np.int64)
        packed=(xy[:,0]<<32) | (xy[:,1]&0xffffffff)
        wanted=(wanted[:,0]<<32) | (wanted[:,1]&0xffffffff)
        keep=np.isin(packed,wanted)
        xy,sub=xy[keep],sub[keep]
        if not len(xy):return
        keys, inverse = np.unique(xy, axis=0, return_inverse=True)
        masks = np.zeros(len(keys), dtype=np.uint32)
        np.bitwise_or.at(masks, inverse, np.left_shift(np.uint32(1), sub[:, 0]+4*sub[:, 1]))
        for key, mask in zip(keys, masks):
            for record in by_xy.get(tuple(map(int, key)), ()):
                if plane is None:
                    record.higher_mask |= int(mask)
                else:
                    record.free_masks[plane] |= int(mask)
