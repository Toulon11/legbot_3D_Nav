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

    A record at h supplies C below h; eligible support can supply G from h
    up to the next support. Undersides may still bound clearance below a cut.
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
            # A skipped underside does not terminate G's dependency range.
            # Its ceiling barrier can likewise affect cuts up to the next
            # real support. Legacy 3-tuples describe unclassified geometry.
            high=next((r[0] for r in records[i+1:] if len(r)<4 or r[3]!=-1),float('inf'))
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
    orientation: int = 1  # +1 support, -1 underside, -2 vertical, 0 ambiguous.
    footprint: int = 0  # 4 by 4 mask of measured hit locations within the XY cell.
    top_cm: int | None = None
    first_seen: int = 0

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
                             self.footprint,self.top_cm,self.first_seen)

    def __getitem__(self, index):
        return (self.low, self.high)[index]


class SurfaceColumn:
    """One authoritative height per physical surface, shared by every slice."""
    __slots__ = ('surfaces', 'heights', 'next_id', 'sequence')

    def __init__(self):
        self.surfaces, self.heights = [], []
        self.next_id, self.sequence = 1, 0

    def __len__(self):
        return len(self.surfaces)

    def values(self):
        return iter(self.surfaces)

    def clone(self):
        other=SurfaceColumn()
        other.surfaces=[s.clone() for s in self.surfaces]
        other.heights=list(self.heights)
        other.next_id,other.sequence=self.next_id,self.sequence
        return other

    def _sort(self):
        self.surfaces.sort(key=lambda s:(s.height,s.id))
        self.heights=[s.height for s in self.surfaces]

    def _undersides(self):
        return [s.height for s in self.surfaces if s.orientation == -1]

    def _coalesce(self, tolerance, num):
        # Opposite sides of a thin floor must not merge as measurement noise.
        undersides=self._undersides()
        self.surfaces.sort(key=lambda s:(s.orientation,s.height,s.id))
        i=0
        while i+1<len(self.surfaces):
            a,b=self.surfaces[i:i+2]
            if (a.orientation!=b.orientation or
                    max(a.high,b.high)-min(a.low,b.low) > tolerance+1e-7 or
                    (a.orientation==1 and bisect_left(undersides,a.height-1e-7)!=
                     bisect_left(undersides,b.height-1e-7))):
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
            kept.first_seen=min(a.first_seen,b.first_seen)
            kept.footprint=a.footprint | b.footprint
            tops=[top for top in (a.top_cm,b.top_cm) if top is not None]
            kept.top_cm=max(tops,default=None)
            self.surfaces[i:i+2]=[kept]
            self.surfaces.sort(key=lambda s:(s.orientation,s.height,s.id))
            i=max(0,i-1)
        self._sort()

    def add(self, z, tolerance, num, frame=0, orientation=1, footprint=0, top_cm=None):
        at=bisect_left(self.heights,z)
        lo=bisect_left(self.heights,z-tolerance-1e-7)
        hi=bisect_right(self.heights,z+tolerance+1e-7)
        # A measured underside separates support observed below it from support
        # observed above it, even within the height noise tolerance.
        # Distance to a moving mean alone admits a chain of points whose total
        # span exceeds the noise band. Apply the same full-span rule to both
        # individual hits and group coalescing, including frozen mature means.
        candidates=[i for i in range(lo,hi) if self.surfaces[i].orientation==orientation and
                    max(self.surfaces[i].high,z)-min(self.surfaces[i].low,z)<=tolerance+1e-7]
        if orientation==1 and candidates and (undersides:=self._undersides()):
            band=bisect_left(undersides,z-1e-7)
            candidates=[i for i in candidates if bisect_left(undersides,self.heights[i]-1e-7)==band]
        nearest=min(candidates,key=lambda i:(abs(self.heights[i]-z),self.surfaces[i].id)) if candidates else None
        if nearest is not None and abs(self.heights[nearest]-z)<=tolerance+1e-7:
            surface=self.surfaces[nearest]
            surface.last_seen=frame
            new_footprint=surface.footprint | footprint
            new_top=top_cm if surface.top_cm is None else (
                max(surface.top_cm,top_cm) if top_cm is not None else surface.top_cm)
            low,high=min(surface.low,z),max(surface.high,z)
            changed=(new_footprint!=surface.footprint or new_top!=surface.top_cm)
            surface.footprint,surface.top_cm=new_footprint,new_top
            if surface.mature:
                # Bounds constrain subsequent grouping; extending them within
                # this band keeps the mean frozen. For ambiguous/vertical
                # records, high also participates in near-ground suppression.
                changed |= orientation in (0,-2) and high!=surface.high
                surface.low,surface.high=low,high
                return changed
            self.sequence+=1
            surface.add(z,self.sequence,num)
        else:
            self.sequence+=1
            surface=SurfaceRecord(self.next_id,z,0,z,z,last_seen=frame,orientation=orientation,
                                  footprint=footprint,top_cm=top_cm,first_seen=frame)
            self.next_id+=1
            surface.add(z,self.sequence,num)
            self.surfaces.insert(at,surface)
        self._sort()
        self._coalesce(tolerance,num)
        return True

    def observed_geometry(self, noise_tolerance):
        records=[]
        support=None
        for record in self.surfaces:
            if record.orientation==1:
                support=record
            elif (support and record.orientation in (0,-2) and
                  (support.mature or not record.mature) and
                  record.high-support.height<=noise_tolerance+1e-7):
                # Suppress near-ground noise once for the whole column.
                # A separate, observed underside is retained.
                continue
            records.append(record)
        return tuple(r.height for r in records),records

    def at(self, cut, geometry, noise_tolerance=.05):
        # G must describe a possible support, never an observed underside.
        # Keep all original evidence for ceiling and line-of-sight queries.
        heights,records=geometry
        at=bisect_right(heights,cut+1e-7)
        ground=next((r for r in reversed(records[:at]) if r.orientation!=-1),None)
        ceiling=records[at] if at<len(records) else None
        if ground is not None:
            # Skipping an underside must not erase a real low ceiling below
            # the cut. Near-coincident faces remain one height-resolution band;
            # their real support supplies G, not the underside's identity.
            barrier=next((r for r in records[:at] if r.orientation==-1 and
                          r.height>ground.height+noise_tolerance+1e-7),None)
            if barrier is not None:ceiling=barrier
        return ground,ceiling
