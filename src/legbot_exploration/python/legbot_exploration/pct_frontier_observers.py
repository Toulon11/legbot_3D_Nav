"""Lazy, local observer geometry independent of robot and completion state."""
import math
import weakref
import time
from collections import OrderedDict
import numpy as np


def observation_effort(distance, gain):
    """Bound the credit for a large cluster; every visit has a fixed cost."""
    return (1.+distance)/math.sqrt(max(1,min(gain,64)))


class FrontierObservers:
    def __init__(self, index):
        self._index = weakref.ref(index)
        self.cache = {}
        self.dependencies = {}
        self.built = 0
        self.invalidated = 0
        self.last_unreachable = 0
        self.searches=OrderedDict()
        self.pending=False
        self.geometry_ms=self.visibility_ms=0.
        self.phase_ms={}
        self.unknown_blockers=[]

    def drop(self, key):
        entry = self.cache.pop(key, None)
        if entry is None:
            return
        for dependency in entry['dependencies']:
            users = self.dependencies[dependency]
            users.remove(key)
            if not users:
                del self.dependencies[dependency]
        self.invalidated += 1
        for search in list(self.searches):
            if key in self.searches[search]['groups']:
                del self.searches[search]

    def discard_search(self, members):
        for signature in tuple(self.searches):
            if signature[0]==members:
                del self.searches[signature]

    def prune(self):
        fragments = self._index().components.fragments
        for key in list(self.cache):
            if key[0] not in fragments:
                self.drop(key)

    def invalidate_frontiers(self, changed):
        index = self._index()
        for key in changed:
            fid = index.components.node_fragment.get(key)
            if fid is not None:
                self.drop((fid, key[0]))

    def invalidate_surfaces(self, changed):
        """Only cached candidates on the changed cut and within source radius."""
        index = self._index()
        s = index.map.settings
        b = s.block_size
        touched = {}
        for k, x, y, h in changed:
            for key in self.dependencies.get((k, x // b, y // b), ()):
                touched.setdefault(key, []).append((x, y, h))
        for key, cells in touched.items():
            entry = self.cache[key]
            values = np.asarray(cells)
            lo, hi = entry['bounds']
            distance = np.linalg.norm(np.maximum(np.maximum(lo-values[:, :2], values[:, :2]-hi), 0), axis=1)
            if ((distance <= index.radius / s.resolution + 1e-6) &
                (values[:, 2] >= entry['heights'][0]-s.step_max-1e-5) &
                (values[:, 2] <= entry['heights'][1]+s.step_max+1e-5)).any():
                self.drop(key)

    @staticmethod
    def distances(points, seeds, step):
        nearest = np.full(len(points), np.inf)
        for offset in range(0, len(seeds), 64):
            group = seeds[offset:offset+64]
            d = np.linalg.norm(points[:, None, :2]-group[None, :, :2], axis=2)
            d[abs(points[:, None, 2]-group[None, :, 2]) > step+1e-5] = np.inf
            nearest = np.minimum(nearest, d.min(axis=1))
        return nearest

    def candidates(self, fid, k, members):
        index = self._index()
        m = index.map
        s = m.settings
        b = s.block_size
        key = fid, k
        if key not in self.cache:
            sources = sorted(n for n in index.components.fragments[fid] if n[0] == k)
            data = np.asarray([(n[1], n[2], index.nodes[n].height) for n in sources])
            lo, hi = data[:, :2].min(0), data[:, :2].max(0)
            seeds = data.copy()
            seeds[:, :2] = seeds[:, :2]*s.resolution+[s.origin_x, s.origin_y]
            half = index.radius / s.resolution
            low = np.floor((lo-half)/b).astype(int)
            high = np.floor((hi+half)/b).astype(int)
            dependencies = set()
            choices = []
            for bx in range(low[0], high[0]+1):
                for by in range(low[1], high[1]+1):
                    dependency = k, bx, by
                    dependencies.add(dependency)  # Missing blocks may later acquire observations.
                    block = m.blocks.get(dependency)
                    if block is None:
                        continue
                    valid = ~block['unknown'] & ~block['solid'] & ~block['hard'] & \
                            (block['inflated_terrain_cost'] >= 0) & (block['inflated_terrain_cost'] <= 20)
                    ii, jj = np.nonzero(valid)
                    points = np.column_stack(((bx*b+ii)*s.resolution+s.origin_x,
                                              (by*b+jj)*s.resolution+s.origin_y, block['G'][ii, jj]))
                    costs = block['inflated_terrain_cost'][ii, jj]
                    for offset in range(0, len(points), 256):
                        batch = points[offset:offset+256]
                        nearest = self.distances(batch, seeds, s.step_max)
                        keep = nearest <= index.radius+1e-8
                        if keep.any():
                            choices.append(np.column_stack((batch[keep], nearest[keep], costs[offset:offset+256][keep])))
            values = np.vstack(choices) if choices else np.empty((0, 5))
            cells=np.floor((values[:,:2]-[s.origin_x,s.origin_y])/s.resolution+.5).astype(int)
            cost_rows=[]
            for tile in {tuple(v) for v in cells//b}:
                at=np.flatnonzero((cells//b==tile).all(axis=1))
                local=cells[at]%b
                cost_rows.append((tile,at,local[:,0],local[:,1]))
            self.cache[key] = dict(values=values, sources=set(sources), dependencies=dependencies,cost_rows=cost_rows,
                                   bounds=(lo, hi), heights=(data[:, 2].min(), data[:, 2].max()))
            for dependency in dependencies:
                self.dependencies.setdefault(dependency, set()).add(key)
            self.built += 1
        entry = self.cache[key]
        values = entry['values']
        # Geometry stays valid across soft cost drift. Read live costs in
        # batches rather than rebuilding distances or retaining stale scores.
        for tile,at,ii,jj in entry['cost_rows']:
            values[at,4]=m.blocks[(k,*tile)]['inflated_terrain_cost'][ii,jj]
        if members != entry['sources'] and len(values):
            # A 3-D query can clip through a tile. Reuse safe map candidates,
            # but only credit frontiers actually inside that query's window.
            seeds = np.asarray([(s.origin_x+n[1]*s.resolution, s.origin_y+n[2]*s.resolution,
                                 index.nodes[n].height) for n in sorted(members)])
            values = values.copy()
            values[:, 3] = self.distances(values[:, :3], seeds, s.step_max)
            values = values[values[:, 3] <= index.radius+1e-8]
        return values

    def groups(self, members):
        index = self._index()
        groups = {}
        for n in members:
            groups.setdefault((index.components.node_fragment[n], n[0]), set()).add(n)
        return groups

    def reachable_mask(self, xyz, valid, connectivity, component):
        m = self._index().map
        b = m.settings.block_size
        s = m.settings
        cells = np.floor((xyz[:, :2]-[s.origin_x,s.origin_y])/s.resolution+.5).astype(int)
        tiles = cells // b
        reachable = np.zeros(len(xyz), dtype=bool)
        for tile in {tuple(v) for v in tiles[valid]}:
            at = valid & (tiles == tile).all(axis=1)
            local = cells[at] % b
            reachable[at] = connectivity.mask(tile, component)[local[:, 0], local[:, 1]]
        return reachable

    def disconnected(self, members, connectivity, component, floor):
        # Prove separation using the full geometric pose set. The local
        # window, minimum travel and prior viewpoints are scheduling filters.
        has_station = False
        for (fid, k), group in self.groups(members).items():
            xyz = self.candidates(fid, k, group)[:, :3]
            valid = abs(xyz[:, 2]-floor[0]) <= floor[1]
            if valid.any():
                if self.reachable_mask(xyz, valid, connectivity, component).any():
                    return False
                has_station = True
        return has_station

    def select(self, members, base=None, minimum_distance=0., completed=(), component=None,
               window=None, deadline=None, continuation=None, floor=None, minimum_gain=1, physical=None,
               connectivity=None, admission=None, rejected=(), ascent_limits=None):
        began=time.monotonic()
        self.pending=False;self.geometry_ms=self.visibility_ms=0.
        self.phase_ms={}
        self.ray_dependencies={}
        self.unknown_blockers=[]
        index = self._index()
        s = index.map.settings
        groups = self.groups(members)
        connectivity = connectivity if connectivity is not None else index.map.connectivity
        choices = {}
        self.last_views = {}
        unreachable = set()
        for (fid, k), group in sorted(groups.items()):
            data = self.candidates(fid, k, group)
            if not len(data):
                continue
            xyz = data[:, :3]
            valid = np.ones(len(data), dtype=bool)
            if window is not None:
                valid &= ((xyz >= window[0]-1e-8) & (xyz <= window[1]+1e-8)).all(axis=1)
            if floor is not None:
                valid &= abs(xyz[:, 2]-floor[0]) <= floor[1]
            if base is not None:
                valid &= np.linalg.norm(xyz[:, :2]-base[:2], axis=1) >= minimum_distance
                if continuation is not None:
                    yaw,minimum_height,backward_slack=continuation
                    direction=np.array([math.cos(yaw),math.sin(yaw)])
                    delta=xyz[:,:2]-base[:2]
                    valid &= (xyz[:,2]>=minimum_height) & (delta@direction>=-backward_slack)
                    if ascent_limits is not None:
                        max_step,_=ascent_limits
                        valid &= xyz[:,2]<=base[2]+max_step+1e-7
            for done in completed:
                if len(done)>3:continue
                valid &= (np.linalg.norm(xyz[:, :2]-done[:2], axis=1) > s.observation_revisit_radius) | \
                         (abs(xyz[:, 2]-done[2]) > s.step_max)
            if component is not None:
                before = valid.copy()
                valid = self.reachable_mask(xyz, valid, connectivity, component)
                unreachable.update(tuple(p) for p in xyz[before & ~valid])
            rank = abs(data[:, 3]-1.5)+.01*data[:, 4]
            if base is not None:
                rank += np.linalg.norm(xyz-base, axis=1)
            for n in np.flatnonzero(valid):
                point = tuple(map(float, xyz[n]))
                choices[point] = min(choices.get(point, math.inf), float(rank[n]))
        def order(p):
            level=ascent_limits is not None and p[2]-base[2]<ascent_limits[1]
            return level,choices[p],p

        ordered = sorted(choices,key=order)
        self.last_unreachable = len(unreachable)
        self.geometry_ms=1000*(time.monotonic()-began)
        if not ordered:
            return []
        visibility_started=time.monotonic()
        seeds=np.asarray([point for point,_ in physical]) if physical is not None else np.asarray(sorted({
            (s.origin_x+n[1]*s.resolution,s.origin_y+n[2]*s.resolution,index.nodes[n].height) for n in members}))
        done=[p for p in completed if len(p)<5 or any(k in members for k,_ in index.completed_views.get(p,()))]
        # Candidate visibility is a property of that pose and target set.
        # Base position, travel limit and connected component only filter/rank
        # poses above; changing them must not discard valid per-pose rays.
        signature=(frozenset(members),frozenset(completed),continuation,floor,minimum_gain,
                   None if admission is None else admission[2],ascent_limits)
        model=index.observation_model
        search=self.searches.setdefault(signature,dict(groups=set(groups),tested=set(),visible={},rays={},
                                                       scores={},pose_rays={}))
        self.searches.move_to_end(signature)
        while len(self.searches)>64:self.searches.popitem(last=False)
        if admission is None:
            engine=model.pose_search
            admission=(engine.observation_poses,engine._arrival_geometry,.15,True)
        return self._safe_views(ordered,choices,seeds,done,search,admission,rejected,
                                minimum_gain,deadline,base,visibility_started,order,
                                continuation[0] if ascent_limits is not None else None,
                                floor is not None and continuation is None)

    def _safe_views(self, ordered, costs, seeds, done, search, admission, rejected,
                    minimum_gain, deadline, base, began, order, ascent_yaw, prefer_gain):
        index=self._index()
        m,model=index.map,index.observation_model
        find_poses,geometry,tolerance,allow_new=admission
        memo=search.setdefault('admitted',{})
        views={}
        computed=False
        for pose in ordered:
            if pose in rejected:
                continue
            coarse=search['scores'].get(pose)
            if coarse is not None and coarse<minimum_gain:
                continue
            prior=memo.get(pose)
            body=tuple(np.asarray(pose)+[0.,0.,m.settings.observer_reference_height])
            stamp=geometry(body,tolerance)[2] if prior else None
            if prior and prior[0]==stamp and all(model.column_blockers(xy)==bands for xy,bands in prior[2].items()):
                _,view,_,blockers=prior
            else:
                if not allow_new or (computed and deadline is not None and time.monotonic()>=deadline):
                    self.pending=True
                    break
                computed=True
                near=(np.linalg.norm(seeds[:,:2]-pose[:2],axis=1)<=index.radius)&(
                    abs(seeds[:,2]-pose[2])<=m.settings.step_max)
                if coarse is None:
                    rough=model._visibility_view(pose,seeds[near],done,occlusion=False)
                    coarse=search['scores'][pose]=rough['gain'] if rough else 0
                if coarse<minimum_gain:
                    continue
                if stamp is None:stamp=geometry(body,tolerance)[2]
                found=find_poses(body,(0.,0.,ascent_yaw or 0.),seeds[near],
                    stances=[(np.asarray(body),0,True)],ascent=ascent_yaw,
                    tolerance=tolerance,completed=done,minimum_gain=minimum_gain,max_results=1)
                for phase,elapsed in found.get('timing_ms',{}).items():
                    self.phase_ms[phase]=self.phase_ms.get(phase,0.)+elapsed
                view=found['views'].get(body)
                rays=found['rays']
                blockers=found['unknown_blockers']
                memo[pose]=(stamp,view,rays,blockers)
            if view:
                views[pose]=view
                if len(views)>=3:
                    break
            elif blockers:
                self.unknown_blockers=m.merge_unknown_blockers(self.unknown_blockers+blockers,pose if base is None else base)
        self.visibility_ms=1000*(time.monotonic()-began)
        self.last_views=views
        return [list(pose) for pose in sorted(views,key=lambda p:(order(p)[0],
            observation_effort(costs[p],views[p]['gain']) if prefer_gain else
            costs[p]-.03*views[p]['gain'],p))]
