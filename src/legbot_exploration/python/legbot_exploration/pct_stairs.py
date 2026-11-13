"""Recognize ascending runs of real treads from the committed PCT surfaces."""
from collections import deque
from dataclasses import dataclass
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

    @property
    def confirmed(self):
        return len(self.treads) >= 3

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
        self.revision = 0

    @property
    def pending(self):
        return bool(self._unseeded or self.dirty)

    def update(self, mapping, columns):
        if self.floor is None:
            return
        floor = self.floor[0]
        s = mapping.settings
        _, max_rise = tread_rise_bounds(s)
        for xy in columns:
            heights = tuple(r.height for r in mapping.columns.get(xy, {}).values()
                            if r.orientation == 1 or (self.include_unclassified and r.orientation == 0))
            if self.surfaces.get(xy, ()) == heights:
                continue
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

    def query(self, mapping, floor, tolerance, deadline=None, tile_limit=None, focus=None):
        def tile_order(tile):
            if focus is None:
                return tile
            xy,yaw=focus
            size=mapping.settings.block_size
            dx,dy=((i+.5)*size-p for i,p in zip(tile,xy))
            behind=dx*math.cos(yaw)+dy*math.sin(yaw)<0 if yaw is not None else False
            return math.hypot(dx,dy)+size*behind,tile

        if self.floor != (floor, tolerance):
            self.__init__(self.include_unclassified)
            self.floor = floor, tolerance
            if deadline is None:
                self.update(mapping, mapping.columns)
            else:
                if focus is None:
                    self._unseeded=deque(mapping.columns)
                else:
                    size=mapping.settings.block_size
                    tiles={}
                    for x,y in mapping.columns:
                        tiles.setdefault((x//size,y//size),[]).append((x,y))
                    self._unseeded=deque(xy for tile in sorted(tiles,key=tile_order) for xy in tiles[tile])
        while self._unseeded:
            self.update(mapping,[self._unseeded.popleft() for _ in range(min(1024,len(self._unseeded)))])
            if deadline is not None and time.monotonic()>=deadline:
                break
        processed = 0
        for tile in sorted(self.dirty,key=tile_order):
            if ((tile_limit is not None and processed >= tile_limit) or
                (deadline is not None and time.monotonic() >= deadline)):
                break
            self.dirty.remove(tile)
            processed += 1
            for xy in self.reads.pop(tile, ()):
                self.readers[xy].remove(tile)
                if not self.readers[xy]:
                    del self.readers[xy]
            seeds = self.seeds.get(tile)
            if not seeds:
                self.seeds.pop(tile, None)
                self.flights.pop(tile, None)
                continue
            reads = set()

            def heights(x, y):
                xy = x, y
                reads.add(xy)
                return self.surfaces.get(xy, ())

            self.flights[tile] = _find_stairs(mapping.settings, floor, tolerance, seeds, heights)
            self.reads[tile] = reads
            for xy in reads:
                self.readers.setdefault(xy, set()).add(tile)
        if processed:
            self.revision += 1
        return list(dict.fromkeys(f for flights in self.flights.values() for f in flights))


def find_stairs(mapping, floor, floor_tolerance):
    return StairIndex().query(mapping, floor, floor_tolerance)


def _find_stairs(s, floor, floor_tolerance, first_treads, heights):
    tread_tolerance = min(.035, s.surface_noise_tolerance)
    _, max_rise = tread_rise_bounds(s)
    first_rise = s.surface_noise_tolerance-1e-7
    if max_rise < first_rise:
        return []

    def near(x, y, h):
        return any(abs(z-h) <= tread_tolerance for z in heights(x, y))

    flights = []
    platforms = {}
    for dx, dy in DIRECTIONS:
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
