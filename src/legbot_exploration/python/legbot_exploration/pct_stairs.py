"""Recognize ascending runs of real treads from the committed PCT surfaces."""
from collections import defaultdict, deque, OrderedDict
from dataclasses import dataclass
from itertools import islice
import math
import time

from .pct_frontier_index import DIRECTIONS

def tread_rise_bounds(settings):
    return settings.surface_noise_tolerance+1e-7, min(.16,settings.step_max)+1e-7


def level_platform(settings, center, direction, supported, dimensions=None):
    return level_platform_cells(settings, center, direction, supported, dimensions) is not None


def level_platform_cells(settings, center, direction, supported, dimensions=None):
    """Check a bounded, four-connected patch and a continuous width/depth within it."""
    s = settings
    resolution = s.resolution
    yaw = math.atan2(direction[1],direction[0])
    octant = round(yaw/(math.pi/4))
    dx,dy = round(math.cos(octant*math.pi/4)),round(math.sin(octant*math.pi/4))
    pitch = resolution/math.hypot(dx,dy)
    min_width,min_depth,min_area=dimensions or (s.landing_min_width,s.landing_min_depth,s.landing_min_area)
    width = math.ceil(min_width/pitch-1e-7)
    depth = math.ceil(min_depth/pitch-1e-7)
    area = math.ceil(min_area/resolution**2-1e-7)
    limit = math.ceil(max(min_width,min_depth,math.sqrt(min_area))
                      /resolution)+math.ceil(.60/resolution)
    if not supported(*center):
        return None
    seen = {center}
    pending = [center]
    rows = {}
    connected = set()
    while pending:
        x,y = pending.pop()
        rows.setdefault(x*dx+y*dy,set()).add(y*dx-x*dy)
        connected.add((x,y))
        for nx,ny in ((-1,0),(1,0),(0,-1),(0,1)):
            next_xy = (x+nx,y+ny)
            if next_xy in seen or max(abs(next_xy[0]-center[0]),abs(next_xy[1]-center[1]))>limit:
                continue
            seen.add(next_xy)
            if supported(*next_xy):
                pending.append(next_xy)
    if len(connected)<area:
        return None
    # With diagonal axes, even rows have even columns and odd rows have
    # odd columns. Intersect each parity separately before measuring width.
    stride = 2 if dx and dy else 1
    for first in sorted(rows):
        shared = set()
        for start in range(first,first+min(stride,depth)):
            shared.update(rows.get(start,set()).intersection(
                *(rows.get(row,()) for row in range(start+stride,first+depth,stride))))
        run = 0
        previous = None
        for column in sorted(shared):
            run = run+1 if previous is not None and column==previous+1 else 1
            if run>=width:
                return connected
            previous = column
    return None


@dataclass(frozen=True)
class StairFlight:
    entry: tuple
    direction: tuple
    treads: tuple
    half_width: int
    usable_count: int | None = None

    @property
    def confirmed(self):
        return len(self.usable_treads) >= 2

    @property
    def usable_treads(self):
        return self.treads if self.usable_count is None else self.treads[:self.usable_count]

    @property
    def pending_treads(self):
        return () if self.usable_count is None else self.treads[self.usable_count:]

    def owns_boundary(self, x, y, sources, on_floor):
        """Only hand off the unknown boundary directly ahead of the entrance."""
        dx, dy = self.direction
        rx, ry = x-self.entry[0], y-self.entry[1]
        stride = dx*dx+dy*dy
        floor_sources=[(sx,sy) for sx,sy,h,*_ in sources if on_floor(h)]
        return (0 < rx*dx+ry*dy <= stride and abs(rx*dy-ry*dx) <= self.half_width*stride and
                bool(floor_sources) and all((x-sx)*dx+(y-sy)*dy > 0 for sx,sy in floor_sources))

    def entrance_cells(self):
        """Grid cells from the entrance row through the first measured tread."""
        dx, dy = self.direction
        x, y, _ = self.entry
        tx, ty, _ = self.treads[0]
        width = self.half_width
        end = (tx-x)*dx+(ty-y)*dy
        for i in range(min(x,tx)-width*abs(dy),max(x,tx)+width*abs(dy)+1):
            for j in range(min(y,ty)-width*abs(dx),max(y,ty)+width*abs(dx)+1):
                if (0 <= (i-x)*dx+(j-y)*dy <= end and
                        abs((i-x)*dy-(j-y)*dx) <= width*(dx*dx+dy*dy)):
                    yield i,j

    def continues_from(self, xy, ground, resolution, tolerance):
        """Require an upcoming tread on the measured run that supports this stance."""
        dx,dy=self.direction
        stride=dx*dx+dy*dy
        for x,y,height in (self.entry,*self.treads[:-1]):
            rx,ry=xy[0]-x,xy[1]-y
            if (abs(height-ground)<=tolerance and abs(rx*dy-ry*dx)<=self.half_width*stride and
                    abs(rx*dx+ry*dy)*resolution<=.50*math.sqrt(stride)):
                return True
        return False

class StairCandidateCache:
    """Incremental geometric candidates, exact column dependencies and pending admission."""
    def __init__(self, settings):
        self.settings=settings
        self.floor=None
        self.tiles={}
        self.owners=defaultdict(set)
        self.dependents=defaultdict(set)
        self.pending=OrderedDict()
        self.qualified={}
        self.groups={}
        self.group_bins=defaultdict(set)
        self.membership={}
        self.dirty_groups=OrderedDict()
        self.candidates={}
        self.confirmed={}
        self.changes=OrderedDict()
        self.next_group=0

    def set_floor(self, floor):
        if self.floor!=floor:
            self.__init__(self.settings)
            self.floor=floor

    def _bin(self, flight):
        size=max(1,math.ceil(1./self.settings.resolution))
        x,y,_=flight.treads[0]
        return x//size,y//size

    def _same_entrance(self, first, second):
        a,b=first.treads[0],second.treads[0]
        dx,dy=first.direction;ux,uy=second.direction
        return (math.dist(a[:2],b[:2])*self.settings.resolution<=1. and
                abs(a[2]-b[2])<=self.settings.surface_noise_tolerance and
                (dx*ux+dy*uy)/math.hypot(dx,dy)/math.hypot(ux,uy)>=.70)

    def _queue(self, flight):
        old=self.qualified.pop(flight,None)
        self.pending.setdefault(flight,None)
        group=self.membership[flight]
        if old is not None:self.dirty_groups.setdefault(group,None)
        if old is not None and self.candidates.get(group)==old:
            self.candidates.pop(group,None)
            self.confirmed.pop(group,None)
            self.changes[group]=None

    def invalidate(self, columns):
        affected=set()
        for xy in columns:affected.update(self.dependents.get(tuple(xy),()))
        for flight in affected:self._queue(flight)

    def replace(self, owner, flights):
        current=set(flights)
        previous=self.tiles.get(owner,set())
        for flight in previous-current:
            self.owners[flight].discard(owner)
            if self.owners[flight]:continue
            del self.owners[flight]
            group=self.membership.pop(flight)
            self.groups[group][1].discard(flight)
            self.pending.pop(flight,None)
            self.qualified.pop(flight,None)
            for x,y,_ in (flight.entry,*flight.treads):
                self.dependents[x,y].discard(flight)
                if not self.dependents[x,y]:del self.dependents[x,y]
            self.candidates.pop(group,None)
            self.confirmed.pop(group,None)
            self.changes[group]=None
            self.dirty_groups.setdefault(group,None)
        for flight in current-previous:
            self.owners[flight].add(owner)
            if flight in self.membership:continue
            bx,by=self._bin(flight)
            group=next((gid for dx in (-1,0,1) for dy in (-1,0,1)
                        for gid in self.group_bins.get((bx+dx,by+dy),())
                        if self._same_entrance(flight,self.groups[gid][0])),None)
            if group is None:
                self.next_group+=1;group=self.next_group
                self.groups[group]=(flight,set())
                self.group_bins[bx,by].add(group)
            self.groups[group][1].add(flight)
            self.membership[flight]=group
            for x,y,_ in (flight.entry,*flight.treads):self.dependents[x,y].add(flight)
            self._queue(flight)
        if current:self.tiles[owner]=current
        else:self.tiles.pop(owner,None)

    def process(self, qualify, ground, deadline, publish):
        checked=published=0
        def priority(flight):
            x,y,_=flight.entry
            return math.hypot(x*self.settings.resolution+self.settings.origin_x-ground[0],
                              y*self.settings.resolution+self.settings.origin_y-ground[1])

        while (self.pending or self.dirty_groups) and time.monotonic()+1e-9<deadline:
            if self.dirty_groups:
                group,_=self.dirty_groups.popitem(last=False)
                anchor,members=self.groups[group]
                best=max((self.qualified[f] for f in members if f in self.qualified),
                         key=lambda f:(len(f.usable_treads),len(f.treads),-priority(f)),default=None)
                if best is None:
                    self.candidates.pop(group,None);self.confirmed.pop(group,None)
                else:
                    self.candidates[group]=best
                    if best.confirmed:self.confirmed[group]=best
                    else:self.confirmed.pop(group,None)
                publish(group,best)
                self.changes.pop(group,None)
                published+=1
                if not members:
                    self.group_bins[self._bin(anchor)].discard(group)
                    del self.groups[group]
                continue
            flight=min(islice(self.pending,64),key=priority)
            del self.pending[flight]
            result=qualify(flight,self.floor)
            checked+=1
            if result is not None:self.qualified[flight]=result
            self.dirty_groups.setdefault(self.membership[flight],None)
        return checked,published

    def needs_work(self):
        return bool(self.pending or self.dirty_groups)


class StairIndex:
    """Cache entrance candidates by tile and invalidate the columns each search read."""
    def __init__(self, include_unclassified=False):
        self.include_unclassified = include_unclassified
        self.floor = None
        self.surfaces = {}
        self.seeds = {}
        self.flights = {}
        self.reads = {}
        self.readers = {}
        self.dirty = set()
        self._unseeded = deque()
        self._unseeded_tiles=deque()
        self.column_tiles=defaultdict(set)
        self.revision = 0
        self._building={}
        self._building_reads=set()
        self._building_dependencies={}
        self._building_users=defaultdict(set)
        self._building_tile=None
        self._published_seeds={}

    @property
    def pending(self):
        return bool(self._unseeded or self._unseeded_tiles or self.dirty)

    def cached_flights(self):
        return list(dict.fromkeys(f for flights in self.flights.values() for f in flights))

    def update(self, mapping, columns):
        s=mapping.settings
        for x,y in columns:self.column_tiles[x//s.block_size,y//s.block_size].add((x,y))
        if self.floor is None:
            return
        floor = self.floor[0]
        _, max_rise = tread_rise_bounds(s)
        for xy in columns:
            heights = {h for _,h,_ in mapping.support_samples(*xy)}
            if self.include_unclassified:
                # Keep pending geometric hints alongside the traversable index.
                # This lets verified tread runs win before inspecting raw lips.
                heights.update(r.height for r in mapping.columns.get(xy, {}).values()
                               if r.orientation in (0,1))
            heights=tuple(sorted(heights))
            if self.surfaces.get(xy, ()) == heights:
                continue
            affected=set(self._building_users.get(xy,()))
            if (xy[0]//s.block_size,xy[1]//s.block_size)==self._building_tile:
                affected.update((xy,direction) for direction in DIRECTIONS)
            for key in affected:
                self._building.pop(key,None)
                for dependency in self._building_dependencies.pop(key,()):
                    self._building_users[dependency].discard(key)
                    if not self._building_users[dependency]:
                        del self._building_users[dependency]
            if heights:
                self.surfaces[xy] = heights
            else:
                self.surfaces.pop(xy, None)
            self.dirty.update(self.readers.get(xy, ()))
            tile = (xy[0]//s.block_size, xy[1]//s.block_size)
            first = tuple(z for z in heights if s.surface_noise_tolerance-1e-7 <= z-floor <= max_rise)
            if first:
                self.seeds.setdefault(tile, {})[xy] = first
                self.dirty.add(tile)
            elif xy in self.seeds.get(tile, {}):
                del self.seeds[tile][xy]
                self.dirty.add(tile)

    def advance(self, mapping, floor, tolerance, deadline=None, tile_limit=None, focus=None):
        def tile_order(tile):
            if focus is None:
                return tile
            xy,yaw=focus
            size=mapping.settings.block_size
            dx,dy=((i+.5)*size-p for i,p in zip(tile,xy))
            behind=dx*math.cos(yaw)+dy*math.sin(yaw)<0 if yaw is not None else False
            return math.hypot(dx,dy)+size*behind,tile

        if self.floor != (floor, tolerance):
            column_tiles=self.column_tiles
            self.__init__(self.include_unclassified)
            self.column_tiles=column_tiles
            self.floor = floor, tolerance
            if not column_tiles:
                # A standalone diagnostic index may be created after fusion.
                self.update(mapping,mapping.columns)
            self._unseeded_tiles=deque(sorted(column_tiles,key=tile_order))
        candidates=mapping.stair_candidates
        candidates.set_floor((floor,tolerance))
        while self._unseeded or self._unseeded_tiles:
            if not self._unseeded:
                self._unseeded.extend(self.column_tiles[self._unseeded_tiles.popleft()])
            self.update(mapping,[self._unseeded.popleft() for _ in range(min(64,len(self._unseeded)))])
            if deadline is not None and time.monotonic()>=deadline:
                break
        processed = 0
        ordered=sorted(self.dirty,key=tile_order)
        if self._building_tile in self.dirty:
            ordered.remove(self._building_tile);ordered.insert(0,self._building_tile)
        for tile in ordered:
            if ((tile_limit is not None and processed >= tile_limit) or
                (deadline is not None and time.monotonic() >= deadline)):
                break
            if self._building_tile!=tile:
                self._building_tile=tile
                self._building={}
                self._building_reads=set()
                self._building_dependencies={}
                self._building_users=defaultdict(set)
            seeds = self.seeds.get(tile)
            if not seeds:
                self.seeds.pop(tile, None)
                self.flights.pop(tile, None)
            old_seeds=self._published_seeds.setdefault(tile,set())
            for xy in old_seeds-set(seeds or ()):
                for direction in DIRECTIONS:
                    candidates.replace((self.include_unclassified,tile,xy,direction),())
                old_seeds.remove(xy)
            reads = self._building_reads
            key_reads=set()

            def heights(x, y):
                xy = x, y
                reads.add(xy)
                key_reads.add(xy)
                return self.surfaces.get(xy, ())

            for xy,levels in (seeds or {}).items():
                for direction in DIRECTIONS:
                    key=xy,direction
                    if key in self._building:continue
                    if deadline is not None and time.monotonic()>=deadline:return
                    key_reads=set()
                    self._building[key]=_find_stairs(mapping.settings,floor,tolerance,{xy:levels},heights,
                                                    directions=(direction,))
                    self._building_dependencies[key]=key_reads
                    for dependency in key_reads:self._building_users[dependency].add(key)
                    candidates.replace((self.include_unclassified,tile,xy,direction),self._building[key])
                    old_seeds.add(xy)
            for xy in self.reads.pop(tile, ()):
                self.readers[xy].discard(tile)
                if not self.readers[xy]:del self.readers[xy]
            self.flights[tile]=[flight for batch in self._building.values() for flight in batch]
            self.reads[tile] = reads
            for xy in reads:
                self.readers.setdefault(xy, set()).add(tile)
            self.dirty.remove(tile)
            self._building_tile=None
            self._building={};self._building_reads=set()
            self._building_dependencies={}
            self._building_users=defaultdict(set)
            processed+=1
            self.revision += 1

    def query(self, mapping, floor, tolerance, deadline=None, tile_limit=None, focus=None):
        self.advance(mapping,floor,tolerance,deadline,tile_limit,focus)
        return self.cached_flights()


def find_stairs(mapping, floor, floor_tolerance):
    return StairIndex().query(mapping, floor, floor_tolerance)


def _find_stairs(s, floor, floor_tolerance, first_treads, heights, directions=DIRECTIONS):
    tread_tolerance = min(.035, s.surface_noise_tolerance)
    _, max_rise = tread_rise_bounds(s)
    first_rise = s.surface_noise_tolerance-1e-7
    if max_rise < first_rise:
        return []

    def near(x, y, h):
        return any(abs(z-h) <= tread_tolerance for z in heights(x, y))

    flights = []
    platforms = {}
    for dx, dy in directions:
        pitch = s.resolution*math.hypot(dx, dy)
        # Gaps are allowed only for recognizing geometry. The planner still
        # requires supported motion and never uses these gaps as free space.
        entrance_gap = int(.80/pitch + 1e-7)
        tread_gap = int(.45/pitch + 1e-7)
        half_width = max(1, math.ceil(.25/pitch))
        min_depth = max(2, math.ceil(.18/pitch))
        max_depth = max(min_depth, math.ceil((max(1.0,s.landing_min_depth)+.45)/pitch))
        entries = set()
        for (tx,ty), levels in first_treads.items():
            for i in range(1,entrance_gap+2):
                x,y=tx-i*dx,ty-i*dy
                for z in heights(x, y):
                    if abs(z-floor)<=min(floor_tolerance,s.surface_noise_tolerance)+1e-7 and any(1e-7<t-z<=max_rise for t in levels):
                        entries.add((x,y,z))

        for x,y,ground in sorted(entries):
            def supported(i, h):
                return all(near(x+i*dx-j*dy, y+i*dy+j*dx, h)
                           for j in (-half_width, 0, half_width))

            # The current floor reference is already measured at the robot.
            # Keep a measured entry anchor even if this row has sparse ground
            # returns. Every tread still needs support on both sides.
            at, h, treads = 1, ground, []
            tread_depths = []
            while True:
                next_tread = None
                gap = tread_gap if treads else entrance_gap
                for start in range(at, at+gap+1):
                    levels = [z for z in heights(x+start*dx,y+start*dy)
                              if (tread_tolerance+1e-7 if treads else 1e-7) < z-h <= max_rise and
                              (treads or first_rise <= z-floor <= max_rise) and
                              all(supported(i,z) for i in range(start,start+min_depth))]
                    for z in levels:
                        # Missing rows are fine. Reject a measured height
                        # discontinuity between the two candidate treads.
                        between = (heights(x+i*dx,y+i*dy) for i in range(at,start))
                        if all(not row or any(h-tread_tolerance<=v<=z+tread_tolerance for v in row)
                               for row in between):
                            next_tread = start, z
                            break
                    if next_tread is not None:
                        break
                if next_tread is None:
                    break
                at,h=next_tread
                end = at+min_depth
                while end-at < max_depth and supported(end, h):
                    end += 1
                middle = (at+end-1)//2
                tread = (x+middle*dx, y+middle*dy, h)
                treads.append(tread)
                # A measured level platform terminates this run even if
                # another rising sequence is visible across it.
                depth = (end-at)*pitch
                previous_depth = sorted(tread_depths)[len(tread_depths)//2] if tread_depths else depth
                if depth+1e-7 >= max(s.landing_min_depth,previous_depth+.25):
                    key = (tread, dx, dy)
                    if key not in platforms:
                        platforms[key] = level_platform(s,tread[:2],(dx,dy),lambda i,j: near(i,j,h))
                    if platforms[key]:
                        break
                tread_depths.append(depth)
                at = end
            if treads:
                flights.append(StairFlight((x, y, ground), (dx, dy), tuple(treads), half_width))
    return flights
