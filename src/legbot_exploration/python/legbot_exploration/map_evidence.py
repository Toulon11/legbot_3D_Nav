"""Ordered, shared height surfaces for static online mapping.

Only immature records retain up to num-1 scalar samples, so merging candidates
can respect the first-num maturity rule. Mature records retain no sample list.
"""
from bisect import bisect_left, bisect_right
from dataclasses import dataclass, field


def same_ground_surface(first_id, first_height, second_id, second_height):
    """Compare the G sources of two cuts at the same XY."""
    return first_id == second_id and (first_id != 0 or abs(first_height-second_height) <= 1e-7)


def changed_slice_indices(previous, current, layers, previous_layers, origin_z, slice_dh):
    """Find cuts whose selected height, surface identity or maturity can change.

    A record at h supplies C below h and G above h up to the next return.
    Include those intervals in both sequences, since a mean can move, a group
    can merge, and noise filtering can remove a record. Newly allocated cuts
    always read the complete accumulated column.
    """
    added = set(layers).difference(previous_layers)
    if previous == current:
        return sorted(added)
    if not previous:
        return list(layers)
    changed = set(previous).symmetric_difference(current)
    boundaries = [origin_z+k*slice_dh+1e-7 for k in layers]
    intervals = []
    for records in (previous, current):
        for i, record in enumerate(records):
            if record not in changed:
                continue
            low = records[i-1][0] if i else -float('inf')
            # A changed ordered endpoint affects the neighboring intervals.
            high=records[i+1][0] if i+1<len(records) else float('inf')
            intervals.append((bisect_left(boundaries, low), bisect_left(boundaries, high)))
    end = 0
    for first, last in sorted(intervals):
        added.update(layers[max(first, end):last])
        end = max(end, last)
    return sorted(added)


@dataclass(slots=True)
class SurfaceRecord:
    id: int
    height: float
    count: int
    low: float
    high: float
    mature_at: int | None = None
    pending: list = field(default_factory=list)
    last_seen: int = 0  # Integrated-frame serial; does not change frozen geometry.
    orientation: int = 1  # Compatibility field; integrated returns no longer have side roles.
    footprint: int = 0  # 4 by 4 mask of measured hit locations within the XY cell.
    top_cm: int | None = None
    support_hit_z: float | None = None  # Direct hit arriving from above, not a traversed free voxel.

    @property
    def mature(self):
        return self.mature_at is not None

    def add(self, z, sequence, num):
        self.count += 1
        self.height += (z-self.height)/self.count
        self.low, self.high = min(self.low, z), max(self.high, z)
        self.pending.append((sequence, z))
        if self.count >= num:
            self.mature_at = sequence
            self.pending.clear()

    def clone(self):
        return SurfaceRecord(self.id,self.height,self.count,self.low,self.high,
                             self.mature_at,list(self.pending),self.last_seen,self.orientation,
                             self.footprint,self.top_cm,self.support_hit_z)

    def __getitem__(self, index):
        return (self.low, self.high)[index]


class SurfaceColumn:
    """One authoritative height per physical surface, shared by every slice."""
    __slots__ = ('surfaces', 'heights', 'next_id', 'sequence', 'obstacle_intervals', '_obstacle_samples')

    def __init__(self):
        self.surfaces, self.heights = [], []
        self.next_id, self.sequence = 1, 0
        # Robot-blocking spans, not filled optical occupancy. Keep them apart
        # from G/C and the frozen surface means used by ordinary mapping.
        self.obstacle_intervals = ()
        self._obstacle_samples = ()

    def __len__(self):
        return len(self.surfaces)

    def values(self):
        return iter(self.surfaces)

    def clone(self):
        other=SurfaceColumn()
        other.surfaces=[s.clone() for s in self.surfaces]
        other.heights=list(self.heights)
        other.next_id,other.sequence=self.next_id,self.sequence
        other.obstacle_intervals=self.obstacle_intervals
        other._obstacle_samples=self._obstacle_samples
        return other

    def remember_obstacles(self, heights, clearance, noise_tolerance=0.):
        """Join successive same-XY returns whose gap is below robot clearance.

        Singleton endpoints are retained so later static observations can
        extend a span. A gap equal to clearance stays open. The small epsilon
        only stabilizes the strict comparison for float32 point clouds.
        """
        spans=sorted((*self._obstacle_samples,*((float(z),float(z)) for z in heights)))
        merged=[]
        for low,high in spans:
            if merged and low-merged[-1][1] < clearance-1e-7:
                merged[-1]=(merged[-1][0],max(merged[-1][1],high))
            else:merged.append((low,high))
        self._obstacle_samples=tuple(merged)
        # Preserve the existing single-ground noise rule. Points belonging to
        # one G's noise band are a horizontal surface, not vertical obstacles.
        # A distinct underside/explicit obstacle must never be discarded here.
        def ground_noise(low,high):
            return (high-low<=noise_tolerance+1e-7 and
                    any(r.orientation==1 and max(abs(r.height-low),abs(high-r.height))<=noise_tolerance+1e-7
                        for r in self.surfaces) and
                    not any(r.orientation in (-1,-2) and low-1e-7<=r.height<=high+1e-7
                            for r in self.surfaces))
        result=tuple((low,high) for low,high in merged if not ground_noise(low,high))
        if result==self.obstacle_intervals:return False
        self.obstacle_intervals=result
        return True

    def obstacle_at(self, height):
        """Closed, finite blocking interval containing height, if any."""
        at=bisect_right(self.obstacle_intervals,(height+1e-7,float('inf')))-1
        if at>=0:
            low,high=self.obstacle_intervals[at]
            if low-1e-7<=height<=high+1e-7:return low,high
        return None

    def geometry_signature(self, geometry, tolerance):
        return tuple((r.height,r.id,r.mature) for r in geometry[1])

    def _sort(self):
        self.surfaces.sort(key=lambda s:(s.height,s.id))
        self.heights=[s.height for s in self.surfaces]

    def _coalesce(self, tolerance, num):
        self.surfaces.sort(key=lambda s:(s.height,s.id))
        i=0
        while i+1<len(self.surfaces):
            a,b=self.surfaces[i:i+2]
            if max(a.high,b.high)-min(a.low,b.low) > tolerance+1e-7:
                i+=1
                continue
            mature=[s for s in (a,b) if s.mature]
            if mature:
                kept=min(mature,key=lambda s:(s.mature_at,s.id))
                kept.low,kept.high=min(a.low,b.low),max(a.high,b.high)
            else:
                kept=min((a,b),key=lambda s:s.id)
                samples=sorted(a.pending+b.pending)[:num]
                kept.count=len(samples)
                kept.height=sum(z for _,z in samples)/kept.count
                kept.low=min(z for _,z in samples);kept.high=max(z for _,z in samples)
                kept.mature_at=samples[-1][0] if kept.count>=num else None
                kept.pending=[] if kept.mature else samples
            kept.last_seen=max(a.last_seen,b.last_seen)
            kept.footprint=a.footprint | b.footprint
            tops=[top for top in (a.top_cm,b.top_cm) if top is not None]
            kept.top_cm=max(tops,default=None)
            support_hits=[z for z in (a.support_hit_z,b.support_hit_z) if z is not None]
            kept.support_hit_z=max(support_hits,default=None)
            self.surfaces[i:i+2]=[kept]
            self.surfaces.sort(key=lambda s:(s.height,s.id))
            i=max(0,i-1)
        self._sort()

    def add(self, z, tolerance, num, frame=0, orientation=1, footprint=0, top_cm=None,
            support_hit_z=None):
        # Retain the argument for stored-record/API compatibility; geometry no
        # longer separates height groups by a ground/ceiling role classifier.
        orientation=1
        at=bisect_left(self.heights,z)
        lo=bisect_left(self.heights,z-tolerance-1e-7)
        hi=bisect_right(self.heights,z+tolerance+1e-7)
        # Distance to a moving mean alone admits a chain of points whose total
        # span exceeds the noise band. Apply the same full-span rule to both
        # individual hits and group coalescing, including frozen mature means.
        candidates=[i for i in range(lo,hi) if max(self.surfaces[i].high,z)-min(self.surfaces[i].low,z)<=tolerance+1e-7]
        nearest=min(candidates,key=lambda i:(abs(self.heights[i]-z),self.surfaces[i].id)) if candidates else None
        if nearest is not None and abs(self.heights[nearest]-z)<=tolerance+1e-7:
            surface=self.surfaces[nearest]
            surface.last_seen=frame
            new_footprint=surface.footprint | footprint
            new_top=top_cm if surface.top_cm is None else (
                max(surface.top_cm,top_cm) if top_cm is not None else surface.top_cm)
            new_support=(support_hit_z if surface.support_hit_z is None else
                         max(surface.support_hit_z,support_hit_z) if support_hit_z is not None else
                         surface.support_hit_z)
            low,high=min(surface.low,z),max(surface.high,z)
            changed=(new_footprint!=surface.footprint or new_top!=surface.top_cm or
                     new_support!=surface.support_hit_z)
            surface.footprint,surface.top_cm=new_footprint,new_top
            surface.support_hit_z=new_support
            if surface.mature:
                # Bounds constrain subsequent grouping while the mature
                # mean remains frozen; no side-role dependencies remain.
                surface.low,surface.high=low,high
                return changed
            self.sequence+=1
            surface.add(z,self.sequence,num)
        else:
            self.sequence+=1
            surface=SurfaceRecord(self.next_id,z,0,z,z,last_seen=frame,orientation=orientation,
                                  footprint=footprint,top_cm=top_cm,support_hit_z=support_hit_z)
            self.next_id+=1
            surface.add(z,self.sequence,num)
            self.surfaces.insert(at,surface)
        self._sort()
        self._coalesce(tolerance,num)
        return True

    def observed_geometry(self, noise_tolerance):
        """All measured height groups, already noise-bounded during fusion."""
        return tuple(r.height for r in self.surfaces),list(self.surfaces)

    def at(self, cut, geometry, noise_tolerance=.05):
        heights,records=geometry
        at=bisect_right(heights,cut+1e-7)
        return (records[at-1] if at else None,records[at] if at<len(records) else None)

    def at_cuts(self, cuts, geometry, origin_z, slice_dh, noise_tolerance=.05):
        """Select nearest lower G and nearest upper C in one ordered sweep."""
        heights,records=geometry
        pos=0
        for k in cuts:
            cut=origin_z+k*slice_dh
            while pos<len(records) and heights[pos]<=cut+1e-7:pos+=1
            yield k,(records[pos-1] if pos else None),(records[pos] if pos<len(records) else None)
