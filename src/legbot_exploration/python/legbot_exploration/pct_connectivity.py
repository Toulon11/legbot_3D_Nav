"""Incremental necessary reachability check, never an alternative planner.

Project the union of native traversable slice cells onto XY, optionally within
the same height band as a floor-constrained search. Include native gateway
positions, which can be entered through an equivalent slice at the same XY.
Every native edge moves to an eight-neighbour XY, so a disconnected projection
rules out a route. Native A* still checks the slices, heights and gateway rules.

Only touched 32x32 tiles are relabelled. Queries walk the small graph of tile
components, not the full map, and no geometry snapshot is constructed.
"""
import weakref
import time
import numpy as np
from scipy.ndimage import label

NEIGHBOURS=tuple((x,y) for x in (-1,0,1) for y in (-1,0,1) if x or y)


class ConnectivityIndex:
    def __init__(self, mapping, floor=None):
        self._owner=weakref.ref(mapping)
        self.floor=floor
        self.tiles={}
        self.graph={}
        self.components={}
        self.masks={}
        self.revision=0
        self.last_work={}

    @property
    def map(self):return self._owner()

    def node(self, xy):
        x,y=map(int,xy);b=self.map.settings.block_size;tile=(x//b,y//b)
        labels=self.tiles.get(tile)
        value=int(labels[x%b,y%b]) if labels is not None else 0
        return (*tile,value) if value else None

    def component(self, xy):
        return self._component(self.node(xy))

    def _component(self, root):
        if root is None:return None
        if root not in self.components:
            pending=[root];members={root}
            while pending:
                for other in self.graph[pending.pop()]:
                    if other not in members:members.add(other);pending.append(other)
            identity=min(members)
            self.components.update((key,identity) for key in members)
        return self.components[root]

    def possible(self, start, target):
        # A missing start cannot establish a normal start. Leave admission
        # (and relaxed recovery) to its owner instead of hiding that error.
        first=self.component(start)
        return first is None or first==self.component(target)

    def mask(self, tile, component):
        if (tile,component) in self.masks:return self.masks[(tile,component)]
        labels=self.tiles.get(tile)
        if labels is None:return np.zeros((self.map.settings.block_size,)*2,bool)
        allowed=[value for value in np.unique(labels) if value and self._component((*tile,int(value)))==component]
        result=np.isin(labels,allowed)
        self.masks[(tile,component)]=result
        return result

    def update(self, changed):
        b=self.map.settings.block_size
        self.update_tiles({(int(x)//b,int(y)//b) for _,x,y in changed})

    def update_tiles(self, candidates):
        began=time.monotonic()
        m=self.map;b=m.settings.block_size
        dirty=set()
        for tile in candidates:
            mask=np.zeros((b,b),bool)
            for k in range(m.first_layer,m.last_layer+1):
                block=m.blocks.get((k,*tile))
                if block is None:continue
                known=np.isfinite(block['G'])&~block['solid']&~block['unknown']
                safe=known&~block['hard']&(block['inflated_terrain_cost']>=0)&(block['inflated_terrain_cost']<=20)
                if self.floor is not None:
                    safe &= abs(block['G']-self.floor[0])<=self.floor[1]+1e-7
                # A gateway's safe alias shares its XY and passes the same floor filter.
                mask |= safe
            old=self.tiles.get(tile)
            if old is not None and np.array_equal(old!=0,mask):continue
            if old is None and not mask.any():continue
            if old is not None:
                for value in np.unique(old):
                    key=(*tile,int(value))
                    if not value:continue
                    for other in self.graph.pop(key):self.graph[other].discard(key)
            labels,count=label(mask,structure=np.ones((3,3),bool))
            self.tiles[tile]=labels
            for value in range(1,count+1):self.graph[(*tile,value)]=set()
            dirty.add(tile)
        # Connect tile borders, including diagonal contacts across corners.
        border={(x,y) for x in range(b) for y in (0,b-1)}|{(x,y) for x in (0,b-1) for y in range(b)}
        for bx,by in dirty:
            for x,y in border:
                key=self.node((bx*b+x,by*b+y))
                if key is None:continue
                for dx,dy in NEIGHBOURS:
                    if 0<=x+dx<b and 0<=y+dy<b:continue
                    other=self.node((bx*b+x+dx,by*b+y+dy))
                    if other is not None:self.graph[key].add(other);self.graph[other].add(key)
        if dirty:self.revision+=1;self.components.clear();self.masks.clear()
        self.last_work=dict(checked_tiles=len(candidates),relabelled_tiles=len(dirty),revision=self.revision,
                            update_ms=1000*(time.monotonic()-began))

    def known_barrier(self, start, targets, deadline):
        """Prove a separation only if a bounded search is sealed by measured obstacles."""
        if not targets:
            return False
        m=self.map
        points=np.asarray([tuple(start),*targets])
        lower=points.min(axis=0)-3
        shape=points.max(axis=0)+4-lower
        if int(np.prod(shape))>4096:
            return False
        # Missing, stale and inconclusive cells are open in this diagnostic.
        open_cells=np.ones(tuple(shape),dtype=bool)
        for x,y in np.ndindex(*shape):
            if time.monotonic()>=deadline:
                return False
            xy=(int(lower[0]+x),int(lower[1]+y))
            if self.node(xy) is None and m.surface_relations.measured_floor_barrier(xy,self.floor[0],0):
                open_cells[x,y]=False
        at=tuple(np.asarray(start)-lower)
        ends=[tuple(np.asarray(xy)-lower) for xy in targets]
        open_cells[at]=True
        for end in ends:open_cells[end]=True
        labels,_=label(open_cells,structure=np.ones((3,3),bool))
        outside=set(np.r_[labels[0,:],labels[-1,:],labels[:,0],labels[:,-1]])-{0}
        root=labels[at]
        return all(labels[end]!=root and (root not in outside or labels[end] not in outside)
                   for end in ends)
