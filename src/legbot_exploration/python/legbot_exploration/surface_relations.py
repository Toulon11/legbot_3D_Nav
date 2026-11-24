"""Shared physical surface relations; costs never manufacture a surface or wall."""
from collections import defaultdict
from dataclasses import dataclass
import math
import weakref


NEIGHBORS=tuple((x,y) for x in (-1,0,1) for y in (-1,0,1))
DIRECTIONS=tuple(p for p in NEIGHBORS if p!=(0,0))


@dataclass(frozen=True)
class SurfaceRelation:
    state: str
    actual_height: float | None = None
    direction: tuple | None = None
    dependencies: frozenset = frozenset()


class SurfaceRelations:
    def __init__(self, mapping):
        self._owner=weakref.ref(mapping)
        self.cache={}
        self.readers=defaultdict(set)

    @property
    def map(self):return self._owner()

    def invalidate(self, columns):
        keys=set().union(*(self.readers.get(xy,()) for xy in columns)) if columns else set()
        affected=set(columns)
        for key in keys:
            result=self.cache.pop(key)
            affected.add(key[0])
            for xy in result.dependencies:
                self.readers[xy].discard(key)
                if not self.readers[xy]:del self.readers[xy]
        return affected

    def classify(self, xy, expected):
        xy=tuple(map(int,xy));expected=float(expected)
        key=xy,expected
        if key in self.cache:return self.cache[key]
        m,s=self.map,self.map.settings
        tolerance=s.surface_noise_tolerance+1e-7
        dependencies=set();memo={}
        def hits(cell):
            if cell not in memo:
                dependencies.add(cell)
                column=m.columns.get(cell)
                memo[cell]=tuple(column.values()) if column else ()
            return memo[cell]
        def ground(cell,height):
            return any(r.orientation==1 and abs(r.height-height)<=tolerance for r in hits(cell))
        def finish(state,actual=None,direction=None):
            result=SurfaceRelation(state,actual,direction,frozenset(dependencies))
            self.cache[key]=result
            for cell in dependencies:self.readers[cell].add(key)
            return result
        measured=[r.height for r in hits(xy) if r.orientation==1]
        if not measured:return finish('unobserved')
        actual=min(measured,key=lambda h:abs(h-expected))
        if abs(expected-actual)<=tolerance:return finish('matched',actual)
        if abs(expected-actual)>min(.16,s.step_max)+1e-7:return finish('separate_surface',actual)
        labels={}
        for dx,dy in NEIGHBORS:
            cell=xy[0]+dx,xy[1]+dy
            if any(r.orientation==-1 and min(actual,expected)-tolerance<=r.height<=max(actual,expected)+tolerance
                   for r in hits(cell)):break
            own,other=ground(cell,actual),ground(cell,expected)
            if own==other:break
            labels[dx,dy]=int(other)
        else:
            if 3<=sum(labels.values())<=6:
                for nx,ny in DIRECTIONS:
                    if all(label==int(dx*nx+dy*ny>0) for (dx,dy),label in labels.items()):
                        return finish('known_step',actual,(nx,ny))
        if expected<=actual:return finish('boundary_unconfirmed',actual)
        # A one-cell mixed stripe needs spatially separated returns and an
        # independent high-only row beyond it. Missing/overlapping sheets stay unknown.
        low,high=actual,expected
        underside_margin=tolerance
        def bands(cell):
            records=hits(cell)
            if any(r.orientation==-1 and low+underside_margin<r.height<=high+tolerance for r in records):
                return False,False,0,0
            present=[False,False];masks=[0,0]
            for r in records:
                if r.orientation!=1:continue
                distances=abs(r.height-low),abs(r.height-high)
                side=int(distances[1]<distances[0])
                if distances[side]<=tolerance and abs(distances[0]-distances[1])>1e-7:
                    present[side]=True;masks[side]|=r.footprint
            return *present,*masks
        x,y=xy
        local={p:bands((x+p[0],y+p[1])) for p in NEIGHBORS}
        for nx,ny in DIRECTIONS:
            front=[p for p in NEIGHBORS if p[0]*nx+p[1]*ny>0]
            if any(local[p][:2]!=(True,False) for p in NEIGHBORS if p not in front):continue
            edge=[local[p] for p in front]
            mixed=[(lo,hi) for has_low,has_high,lo,hi in edge if has_low and has_high]
            if not mixed or not all(item[1] for item in edge):continue
            if not all(self._separated_footprints(lo,hi,(nx,ny)) for lo,hi in mixed):continue
            ahead={(x+dx+nx,y+dy+ny) for dx,dy in front}
            confirmed=[bands(cell)[:2] for cell in ahead]
            if all(item==(False,True) for item in confirmed):
                return finish('known_step',actual,(nx,ny))
        return finish('boundary_unconfirmed',actual)

    def explains_step(self, xy, expected, sources=()):
        relation = self.classify(xy, expected)
        if relation.state != 'known_step':
            return False
        dx, dy = relation.direction
        return all((x-xy[0])*dx+(y-xy[1])*dy > 0 for x,y,_ in sources)

    @staticmethod
    def _separated_footprints(low_mask, high_mask, direction):
        if min(low_mask.bit_count(),high_mask.bit_count())<2:return False
        dx,dy=direction
        low=[i%4*dx+i//4*dy for i in range(16) if low_mask & (1<<i)]
        high=[i%4*dx+i//4*dy for i in range(16) if high_mask & (1<<i)]
        return any(4*sum(p<=split for p in low)>=3*len(low) and
                   4*sum(p>split for p in high)>=3*len(high)
                   for split in range(min(low+high),max(low+high)))

    def physical_blockers(self, xy):
        column=self.map.columns.get(tuple(xy))
        if not column:return ()
        s=self.map.settings;pad=min(.5*s.resolution,s.surface_noise_tolerance)
        return tuple((r.low-pad,max(r.high,.01*r.top_cm if r.top_cm is not None else r.high)+pad)
                     if r.orientation==-2 else (r.height,r.height) for r in column.values())

    def measured_floor_barrier(self, xy, ground, cutoff):
        m,s=self.map,self.map.settings
        column=m.columns.get(xy)
        if not column:
            return False
        # Require continuous vertical evidence in the body-height band.
        # A stored traversal span can seal this floor without a surface-side
        # classifier. Freshness still belongs to this diagnostic request.
        highest=ground+min(s.step_max,.16)
        top=ground+max(s.minimum_clearance,s.observer_reference_height)
        if any(low<=highest+1e-7 and high>=top-1e-7
               for low,high in column.obstacle_intervals):
            recent=sorted((r.low,r.high) for r in column.values() if r.last_seen>cutoff)
            spans=[]
            for low,high in recent:
                if spans and low-spans[-1][1]<s.minimum_clearance-1e-7:
                    spans[-1]=(spans[-1][0],max(spans[-1][1],high))
                else:spans.append((low,high))
            if any(low<=highest+1e-7 and high>=top-1e-7 for low,high in spans):return True
        return self.fresh_clearance_barrier(xy,ground,after=cutoff) is not None

    def fresh_clearance_barrier(self, xy, ground, *, after=None):
        """Confirm clearance obstruction across the relevant observed cuts."""
        m,s=self.map,self.map.settings
        height=max(s.minimum_clearance,s.observer_reference_height,.16)
        first=math.floor((ground-s.origin_z)/s.slice_dh)+1
        last=math.floor((ground+height-s.origin_z)/s.slice_dh)+1
        if first==last:return None
        column=m.columns.get(tuple(xy))
        if not column:return None
        serial=(max(1,m.surface_observation.height_serial or m.integrated_serial)
                if after is None else after+1)
        source_ids=set()
        for k in range(first,last+1):
            row=m.cell((k,*xy))
            if (row is None or row['unknown'] or not row['measured'] or
                    not math.isfinite(row['G']) or not math.isfinite(row['C']) or
                    not row['passage_cost']>=50 or row['C']-row['G']>=s.minimum_clearance or
                    abs(float(row['G'])-ground)>s.step_max+s.surface_noise_tolerance):return None
            source_ids.add((int(row['g_surface_id']),int(row['c_surface_id'])))
        records={r.id:r for r in column.values()}
        for ground_id,ceiling_id in source_ids:
            support,ceiling=records.get(ground_id),records.get(ceiling_id)
            if (support is None or ceiling is None or support.orientation!=1 or
                    min(support.last_seen,ceiling.last_seen)<serial):return None
        return ground,ground+height
