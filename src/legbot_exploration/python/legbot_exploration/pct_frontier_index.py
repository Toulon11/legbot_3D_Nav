"""Incremental unknown-side frontiers over the owner's committed sparse map.

Only frontier membership, components and observer caches persist here. Map
cells stay in the owner; no dense map or second planning snapshot is created.
The owner updates this index before releasing its map commit lock.
"""
from dataclasses import dataclass, field
from collections import Counter, deque, defaultdict
import math
import time
from statistics import median
import weakref
import numpy as np

SLICE_ID_STRIDE = 1 << 24

DIRECTIONS = tuple((x,y) for x in (-1,0,1) for y in (-1,0,1) if x or y)
NEIGHBORHOOD = ((0, 0),) + DIRECTIONS
EMPTY_WORK = dict(checked_columns=0,changed_frontier_layers=0,topology_changed_layers=0,
                  boundary_input_columns=0,relabelled_tiles=0,reclustered_layers=0,
                  component_fragments_visited=0,updated_clusters=0,observer_invalidated=0,
                  summaries_ms=0.,membership_ms=0.,clusters_ms=0.,update_ms=0.,
                  affected_clusters=0,component_nodes_visited=0,cluster_rebuilds=0,
                  local_connectivity_checks=0,local_connectivity_preserved=0,
                  physical_columns_checked=0,physical_clusters_updated=0,physical_tasks_ms=0.)


def frontier_return(orientation):
    """A return from a distinct underside cannot resolve the tread above it."""
    return orientation != -1


@dataclass(frozen=True)
class Boundary:
    height: float
    # Keep cut-specific support references; source_states reads current costs.
    sources: tuple
    layer: int | None = None


@dataclass(frozen=True)
class FloorAttempt:
    cluster: int
    floor: tuple
    members: dict
    local: set
    fast: bool = False
    failures: int = 0
    max_attempts: int = 2
    bounded_review: bool = False
    failed_small: bool = False
    tested_poses: frozenset = frozenset()
    resume_poses: tuple = ()

    @property
    def review(self):
        return self.failures > 0


@dataclass
class FloorViewSearch:
    cluster: int
    scope: tuple
    members: dict
    elapsed: float = 0.
    last_slice: int = 0
    rejected: set = field(default_factory=set)
    first_rejected: tuple | None = None
    blockers: list = field(default_factory=list)
    route_timeout: bool = False
    direction_center: tuple | None = None
    direction_poses: tuple | None = None
    direction_results: dict = field(default_factory=dict)
    tested_poses: set = field(default_factory=set)
    resume_poses: tuple = ()
    pose_limit: int | None = None


class FrontierCluster(dict):
    """Observer positions are a query result, never eager map-commit work."""
    def __init__(self, index, **values):
        super().__init__(values)
        self._index = weakref.ref(index)

    def __getitem__(self, key):
        if key == 'observations':
            index = self._index()
            return index._observers(self['members'], completed=index._completed_near(self['members']))
        if key == 'completed':
            return self._index()._completed_near(self['members'])
        return super().__getitem__(key)


class SliceFrontiers:
    """Geometry and connected components owned by a single cut."""

    def __init__(self, mapping, layer, namespace, catalog):
        self._owner=weakref.ref(mapping)
        self._catalog=weakref.ref(catalog)
        self.layer=layer
        self.nodes={}
        self.by_column={}
        self.membership={}
        self.clusters={}
        self.safe_signatures={}
        self.frontier_inputs={}
        self.known_count=self.safe_count=0
        self.next_id=namespace
        self.revision=0
        self.last_work=dict(EMPTY_WORK)
        from .pct_frontier_components import FrontierComponents
        self.components=FrontierComponents(self, namespace)

    @property
    def map(self):
        return self._owner()

    def _summary(self, xy, cache):
        if xy in cache:return cache[xy]
        m=self.map;s=m.settings;potential=[];safe=[];observed=set();barriers=set()
        k=self.layer
        if m.ground_pending(xy,k):
            result=([],(),(),())
            cache[xy]=result
            return result
        row=m.cell((k,*xy)) if k in m.column_layers.get(xy,()) else None
        if row is not None and np.isfinite(row['G']):
            h=float(row['G'])
            barriers.add(h)
            if np.isfinite(row['C']):barriers.add(float(row['C']))
            if (not row['solid'] and m.surface_observation.is_ground_source(xy,int(row['g_surface_id'])) and
                    (not np.isfinite(row['C']) or row['C']-h>=s.minimum_clearance)):
                cost=m.planning_cost((k,*xy))
                source=(k,h,int(row['g_surface_id']),float(cost))
                if 0<=cost<=20:safe.append(source)
                potential.append(source)
        # Keep physical surfaces even when no fixed cut selects them as G.
        # They can seed a view task, but never acquire an invented safe cost.
        # Measured hits provide independent occlusion/body evidence.
        column=m.columns.get(xy)
        if column:
            records=list(m.surface_observation.observed(xy))
            barriers.update(r.height for r in records)
            observed.update((r.height,r.orientation) for r in records)
            for at,r in enumerate(records):
                if r.orientation!=1:continue
                cut=m.surface_observation.first_ground_slice(r.height)
                if k!=cut or (at+1<len(records) and records[at+1].height-r.height<s.minimum_clearance):
                    continue
                if not any(other['g_surface_id']==r.id for _,other in m.layers_at(*xy)):
                    potential.append((k,r.height,r.id,50.))
        result=(potential,tuple(sorted(observed)),tuple(safe),tuple(sorted(barriers)))
        cache[xy]=result
        return result

    def _boundaries(self, xy, cache):
        s=self.map.settings
        evidence=self.map.surface_observation
        sources={}
        for dx,dy in DIRECTIONS:
            nx,ny=xy[0]+dx,xy[1]+dy
            for k,h,sid,cost in self._summary((nx,ny),cache)[0]:
                sources.setdefault(k,[]).append((nx,ny,h,sid,cost))
        if not sources:
            return {}
        _,observed,_,barriers=self._summary(xy,cache)
        result={}
        for k,entries in sources.items():
            # Surface equivalence is a measurement tolerance, not step ability.
            # Bound each group's complete height span to prevent chained merges.
            # Distinct surfaces in one source column must never share a task.
            groups=[]
            for entry in sorted(entries,key=lambda e:(e[2],e[0],e[1],e[3])):
                for group in reversed(groups):
                    if entry[2]-group[0][2]>s.surface_noise_tolerance+1e-7:continue
                    if any(e[:2]==entry[:2] and e[3]!=entry[3] for e in group):continue
                    group.append(entry);break
                else:groups.append([entry])
            heights=[median([e[2] for e in group]) for group in groups]
            resolved=set()
            for h,orientation in observed:
                if not frontier_return(orientation):continue
                distances=[abs(h-height) for height in heights]
                closest=min(distances)
                matches=[at for at,d in enumerate(distances) if d<=closest+1e-7]
                # Exact matching assigns a return to its unique closest group.
                if closest<=s.surface_noise_tolerance+1e-7 and len(matches)==1:
                    resolved.add(matches[0])
            for at,group in enumerate(groups):
                height=heights[at]
                refs=tuple((x,y,sid) for x,y,_,sid,_ in group)
                if at in resolved or any(height+s.step_max<h<=height+s.minimum_clearance for h in barriers):
                    continue
                if evidence.frontier_settled(xy,height,refs,layer=k):
                    continue
                # Stable source identity permits several heights in one cut/XY.
                # IDs are column-local, so include the source XY in the key.
                anchor=min((e[0],e[1],e[3]) for e in group)
                result[(k,*xy,anchor)]=Boundary(height,tuple(sorted(entry[:4] for entry in group)),k)
        return result

    def source_states(self, key):
        """Read current costs from retained cut references, not stale copies."""
        return tuple(source+(self.map.planning_cost((key[0],source[0],source[1])),)
                     for source in self.nodes[key].sources)

    def neighbours(self, key):
        _,x,y,_=key;h=self.nodes[key].height;s=self.map.settings
        a={(e[0],e[1]):e[3] for e in self.nodes[key].sources}
        own=self.by_column.get((x,y),())
        for dx,dy in ((0,0),)+DIRECTIONS:
            nearby=self.by_column.get((x+dx,y+dy),())
            for other in nearby:
                if other==key:continue
                delta=abs(self.nodes[other].height-h)
                # Distinct height tasks cannot bridge two floors at one XY.
                limit=s.surface_noise_tolerance+1e-7 if not (dx or dy) else s.step_max+1e-5
                if delta>limit:continue
                b={(e[0],e[1]):e[3] for e in self.nodes[other].sources}
                common=a.keys() & b.keys()
                if any(a[xy]!=b[xy] for xy in common):continue
                # Adjacent boundary groups can share a physical source.
                if common:
                    yield other;continue
                # Match sheets by mutual nearest elevation. A generous step
                # bound alone would join two parallel, closely spaced sheets.
                if any(abs(self.nodes[n].height-h)<delta-1e-7 for n in nearby):continue
                if any(abs(self.nodes[n].height-self.nodes[other].height)<delta-1e-7
                       for n in own):continue
                yield other

    def _drop_cluster(self, cid):
        cluster=self.clusters.pop(cid)
        self.changed_clusters.add(cid)
        for key in cluster['members']:self.membership.pop(key,None)
        return cluster

    def _refresh(self, cid, members, old=None):
        s=self.map.settings
        physical=sorted({(key[1],key[2],self.nodes[key].height) for key in members})
        data=np.asarray(physical,dtype=float)
        xyz=data.copy();xyz[:,:2]=data[:,:2]*s.resolution+[s.origin_x,s.origin_y]
        at=int(np.argmin(np.linalg.norm(xyz-np.median(xyz,axis=0),axis=1)))
        tiles={(key[1]//s.block_size,key[2]//s.block_size) for key in members}
        cluster=FrontierCluster(self._catalog(),id=cid,members=set(members),tiles=tiles,revision=self.revision,
            frontier=xyz[at].tolist(),cell=data[at,:2].astype(int).tolist(),gain=len(physical)*s.resolution,
            cells=len(physical),display=xyz[np.linspace(0,len(xyz)-1,min(len(xyz),256),dtype=int)].tolist())
        self.clusters[cid]=cluster
        for key in members:self.membership[key]=cid
        self.changed_clusters.add(cid)

    def update(self, changed_columns, boundary_kernel=None, force_columns=()):
        self.changed_clusters=set()
        self.stale_observers=set()
        if not changed_columns:
            self.last_work=dict(EMPTY_WORK)
            return set(), set(), set()
        began=time.monotonic()
        self.revision=self._catalog().revision+1
        cache={};observer_changes=set();observer_heights=set()
        boundary_dirty=set(force_columns)
        # Raw hits and soft costs do not necessarily alter a boundary. Compare
        # only the fields read by boundary checks before expanding the 3x3.
        for xy in changed_columns:
            potential,observed,safe,barriers=self._summary(xy,cache)
            previous_safe=self.safe_signatures.get(xy,())
            before_geometry={entry[:3] for entry in previous_safe}
            after_geometry={entry[:3] for entry in safe}
            if before_geometry!=after_geometry:
                observer_changes.update((entry[0],*xy,entry[1]) for entry in before_geometry^after_geometry)
                previous_safety={(entry[1],entry[2]) for entry in previous_safe}
                current_safety={(entry[1],entry[2]) for entry in safe}
                observer_heights.update((*xy,entry[0]) for entry in previous_safety^current_safety)
            if safe:self.safe_signatures[xy]=safe
            else:self.safe_signatures.pop(xy,None)
            signature=(tuple(entry[:3] for entry in potential),observed,barriers,
                       self.map.surface_observation.boundary_signature(xy))
            previous_input=self.frontier_inputs.get(xy,((),(),(),()))
            self.known_count+=len(potential)-len(previous_input[0])
            self.safe_count+=len(safe)-len(previous_safe)
            if previous_input!=signature:
                boundary_dirty.add(xy)
                if any(signature):self.frontier_inputs[xy]=signature
                else:self.frontier_inputs.pop(xy,None)
        summaries_done=time.monotonic()
        columns=set(boundary_dirty)
        columns.update((x+dx,y+dy) for x,y in boundary_dirty for dx,dy in DIRECTIONS)
        replacements=self._boundary_batch(columns,cache,boundary_kernel)
        work,changed,topology,elapsed=self._replace_boundaries(replacements)
        self.last_work=dict(checked_columns=len(columns),changed_frontier_layers=len(changed),
            topology_changed_layers=len(topology),boundary_input_columns=len(boundary_dirty),
            summaries_ms=1000*(summaries_done-began),membership_ms=elapsed,
            clusters_ms=1000*(time.monotonic()-summaries_done)-elapsed,
            observer_invalidated=0, **work,update_ms=1000*(time.monotonic()-began))
        return changed, observer_changes, observer_heights

    def _boundary_batch(self, columns, cache, kernel):
        if kernel is None or not columns:
            return {xy:self._boundaries(xy,cache) for xy in columns}
        neighbourhood=columns | {(x+dx,y+dy) for x,y in columns for dx,dy in DIRECTIONS}
        summaries={}
        for xy in neighbourhood:
            potential,observed,_,barriers=self._summary(xy,cache)
            if potential or observed or barriers:
                summaries[xy]=([(height,sid) for _,height,sid,_ in potential],observed,barriers)
        settings=self.map.settings
        evidence=self.map.surface_observation
        found=kernel(columns,summaries,settings.surface_noise_tolerance,
                     settings.step_max,settings.minimum_clearance)
        replacements={xy:{} for xy in columns}
        for xy,groups in found.items():
            for height,sources in groups:
                refs=tuple((x,y,sid) for x,y,_,sid in sources)
                if evidence.frontier_settled(xy,height,refs,layer=self.layer):
                    continue
                anchor=min((x,y,sid) for x,y,_,sid in sources)
                replacements[xy][(self.layer,*xy,anchor)]=Boundary(height,tuple(sources),self.layer)
        return replacements

    def _replace_boundaries(self, replacements):
        began=time.monotonic()
        changed=set();geometry=set();added=set();removed=set()
        for xy,fresh in replacements.items():
            old=self.by_column.get(xy,set())
            for key in old|fresh.keys():
                before,after=self.nodes.get(key),fresh.get(key)
                if before==after:continue
                changed.add(key)
                if before is None:added.add(key)
                if after is None:removed.add(key)
                if before is None or after is None or before.height!=after.height or before.sources!=after.sources:
                    geometry.add(key)
        # Adding a sheet can change the nearest-sheet match of existing nodes.
        # Recheck those local endpoints too, before replacing any node.
        edge_candidates=set(geometry)
        for key in geometry:
            for dx,dy in ((0,0),)+DIRECTIONS:
                edge_candidates.update(self.by_column.get((key[1]+dx,key[2]+dy),()))
        old_edges={key:set(self.neighbours(key)) if key in self.nodes else None for key in edge_candidates}
        old_clusters={self.membership[key] for key in geometry if key in self.membership}
        # A changed height invalidates only the fragment/cut candidate geometry.
        fragments = {(self.components.node_fragment[k], k[0]) for k in geometry
                     if k in self.components.node_fragment}
        self.stale_observers.update(fragments)
        for xy,fresh in replacements.items():
            for key in self.by_column.pop(xy,()):self.nodes.pop(key,None)
            self.nodes.update(fresh)
            if fresh:self.by_column[xy]=set(fresh)
        topology={key for key in edge_candidates if old_edges[key] !=
                  (set(self.neighbours(key)) if key in self.nodes else None)}
        self.components.replace(removed,added)
        membership_ms=1000*(time.monotonic()-began)
        work=self.components.update(topology,old_edges)
        # Heights can change without altering any edge. Refresh metadata only;
        # no component traversal or observer search is needed in that case.
        metadata_clusters=old_clusters|{self.membership[k] for k in geometry if k in self.membership}
        for cid in metadata_clusters:
            if cid in self.clusters and self.clusters[cid]['revision']!=self.revision:
                old=self._drop_cluster(cid);self._refresh(cid,old['members'],old)
        return work,changed,topology,membership_ms

class FragmentLookup:
    def __init__(self, namespaces):
        self.namespaces=namespaces

    def get(self, fid, default=None):
        owner=self.namespaces.get(fid % SLICE_ID_STRIDE)
        return owner.fragments.get(fid,default) if owner is not None else default

    def __contains__(self, fid):
        return self.get(fid) is not None

    def __getitem__(self, fid):
        members=self.get(fid)
        if members is None:
            raise KeyError(fid)
        return members


class SliceComponentsView:
    def __init__(self, index):
        self._index=weakref.ref(index)
        self.by_namespace={}
        self.fragments=FragmentLookup(self.by_namespace)
        self.node_fragment=self

    def __getitem__(self, key):
        owner=self._index().slices[key[0]]
        return owner.components.node_fragment[key]

    def get(self, key, default=None):
        owner=self._index().slices.get(key[0])
        return owner.components.node_fragment.get(key,default) if owner else default

    def tile(self, key):
        return self._index().slices[key[0]].components.tile(key)


class FrontierIndex:
    """Read view and physical task history shared by all slice indexes."""

    source_states = SliceFrontiers.source_states

    def __init__(self, mapping, observation_radius=2.):
        self._owner=weakref.ref(mapping)
        self.radius=observation_radius
        self._next_namespace=1
        self.reset_geometry()
        from .pct_observation import ObservationModel
        self.observation_model=ObservationModel(mapping)
        self.completed_points=set()
        self.completed_views={}
        self.completed_tiles={}
        self.completion_revision=0
        self.floor_failures={}
        self.floor_blacklist={}
        self.floor_review_timeouts={}
        self.floor_failed_small={}
        self.floor_known_barriers={}
        self.floor_disconnected={}
        self.floor_retry_tiles=defaultdict(set)
        self.floor_retry_dependencies={}
        self.floor_reopened=0

    def reset_geometry(self):
        # Completed views and failure history are keyed by physical evidence.
        self.slices={}
        self.nodes={}
        self.by_column={}
        self.membership={}
        self.clusters={}
        self.components=SliceComponentsView(self)
        from .pct_frontier_observers import FrontierObservers
        self.observer_cache=FrontierObservers(self)
        self.floor_query_members={}
        self.floor_query=None
        self.floor_pending=None
        self.floor_view_searches={}
        self.floor_view_slice=0
        self.query_scope=None
        self.query_visited={}
        self._physical_task_cache=({}, {}, {}, {})
        self._physical_column_edges={}
        self._physical_contacts={}
        self.known_count=self.safe_count=0
        self.revision=getattr(self,'revision',0)
        self.last_work=dict(EMPTY_WORK)

    @property
    def map(self):
        return self._owner()

    def add_slice(self, k):
        namespace=self._next_namespace
        if namespace>=SLICE_ID_STRIDE:
            raise ValueError('pct_slice_namespace_limit')
        self._next_namespace+=1
        index = self.slices[k] = SliceFrontiers(self.map, k, namespace, self)
        self.components.by_namespace[namespace] = index.components
        return index

    def neighbours(self, key):
        return self.slices[key[0]].neighbours(key)

    def task_cluster(self, cid):
        tasks, _, aliases, _ = self.physical_tasks()
        # Callers holding a slice ID can resolve it to the same physical task.
        return tasks.get(aliases.get(cid,cid))

    def task_id(self, key):
        return self.physical_tasks()[2][self.membership[key]]

    def update(self, changed_columns, frontier_columns=None):
        """Serial diagnostic entry point for tests that directly alter cells."""
        columns = set(changed_columns) | set(frontier_columns or ())
        results = [(index, index.update(columns)) for index in self.slices.values()] if columns else []
        self.commit(results)

    def commit(self, results):
        self.last_work=dict(EMPTY_WORK)
        if not results:
            return
        began=time.monotonic()
        invalidated=self.observer_cache.invalidated
        observer_changes=set()
        observer_heights=set()
        self.revision+=1
        changed_clusters=set()
        changed_columns=set()
        for index,(changed,surfaces,heights) in results:
            changed_columns.update(key[1:3] for key in changed)
            for key in changed:
                xy=key[1:3]
                if key in index.nodes:
                    self.nodes[key]=index.nodes[key]
                    self.by_column.setdefault(xy,set()).add(key)
                    self.membership[key]=index.membership[key]
                else:
                    self.nodes.pop(key,None)
                    self.membership.pop(key,None)
                    self.by_column[xy].discard(key)
                    if not self.by_column[xy]:
                        del self.by_column[xy]
            for cid in index.changed_clusters:
                self.clusters.pop(cid,None)
                cluster=index.clusters.get(cid)
                if cluster:
                    self.clusters[cid]=cluster
                    for key in cluster['members']:
                        if self.membership.get(key)!=cid:
                            changed_columns.add(key[1:3])
                        self.membership[key]=cid
            changed_clusters.update(index.changed_clusters)
            observer_changes.update(surfaces)
            observer_heights.update(heights)
            for key,value in index.last_work.items():
                self.last_work[key]+=value
            for observer in index.stale_observers:
                self.observer_cache.drop(observer)
        if changed_clusters or changed_columns:
            self._update_physical_tasks(changed_clusters,changed_columns)
        s=self.map.settings
        self._forget_completed(observer_heights,s.robot_radius+s.safety_margin+s.resolution)
        self.observer_cache.invalidate_surfaces(observer_changes)
        self._reopen_floor_failures(observer_changes)
        self.known_count=sum(index.known_count for index in self.slices.values())
        self.safe_count=sum(index.safe_count for index in self.slices.values())
        self.last_work['observer_invalidated']=self.observer_cache.invalidated-invalidated
        self.last_work['slice_work_ms']=self.last_work['update_ms']
        self.last_work['commit_ms']=1000*(time.monotonic()-began)

    def _completed_near(self, members):
        """Completion ownership is spatial, independent of component IDs."""
        s=self.map.settings;b=s.block_size
        # A completed pose may lie outside the frontier's observer radius
        # yet still be within the revisit radius of a candidate inside it.
        half=math.ceil((self.radius+s.observation_revisit_radius)/s.resolution)
        lo=[min(k[axis] for k in members)-half for axis in (1,2)]
        hi=[max(k[axis] for k in members)+half for axis in (1,2)]
        points=set()
        for bx in range(lo[0]//b,hi[0]//b+1):
            for by in range(lo[1]//b,hi[1]//b+1):points.update(self.completed_tiles.get((bx,by),()))
        return sorted(points)

    def _watch_floor_failure(self, floor, key):
        xy=key[1:3]
        token=(xy,floor,self.nodes[key])
        if token in self.floor_retry_dependencies:return
        s=self.map.settings
        radius=self.radius+max(s.robot_radius+s.safety_margin,s.ground_lidar_min_range)+.15
        half=math.ceil(radius/s.resolution)
        x,y=xy;b=s.block_size
        tiles={(bx,by) for bx in range((x-half)//b,(x+half)//b+1)
               for by in range((y-half)//b,(y+half)//b+1)}
        self.floor_retry_dependencies[token]=tiles
        for tile in tiles:self.floor_retry_tiles[tile].add(token)

    def _reopen_floor_failures(self, changes):
        s=self.map.settings;b=s.block_size
        touched=defaultdict(list)
        for layer,x,y,height in changes:
            row=self.map.cell((layer,x,y))
            if row is not None and not row['unknown'] and self.map.planning_cost((layer,x,y))<=20:
                for token in self.floor_retry_tiles.get((x//b,y//b),()):
                    touched[token].append((x,y,height))
        radius=self.radius+max(s.robot_radius+s.safety_margin,s.ground_lidar_min_range)+.15
        resumed=set()
        for token,points in touched.items():
            xy,floor,boundary=token
            if not any(math.hypot(x-xy[0],y-xy[1])*s.resolution<=radius and
                       abs(height-boundary.height)<=s.step_max+1e-7 for x,y,height in points):
                continue
            for table in (self.floor_failures,self.floor_blacklist,self.floor_review_timeouts,self.floor_failed_small):
                records=table.get(xy)
                if records is None:continue
                matches=[record for record in records if self._same_boundary(boundary,record[1])]
                for record in matches:
                    if isinstance(records,dict):del records[record]
                    else:records.discard(record)
                if not records:table.pop(xy,None)
            for key in self.by_column.get(xy,()):
                if self._same_boundary(self.nodes[key],boundary):
                    resumed.add(self.task_id(key))
            for tile in self.floor_retry_dependencies.pop(token):
                self.floor_retry_tiles[tile].discard(token)
                if not self.floor_retry_tiles[tile]:del self.floor_retry_tiles[tile]
        for cid in resumed:
            self.clear_floor_view_search(cid)
            self.query_visited.pop(cid,None)
        self.floor_reopened+=len(resumed)
        self.last_work['reopened_floor_clusters']=len(resumed)

    def floor_work_available(self, floor):
        for cluster in self.physical_tasks()[0].values():
            for key in cluster['members']:
                node=self.nodes[key]
                if (abs(node.height-floor[0])<=floor[1] and
                        any(abs(source[2]-floor[0])<=floor[1] for source in node.sources) and
                        not self._disconnected_on_floor(floor,key) and
                        not self._floor_records(self.floor_blacklist,floor,key) and
                        self._failure_count(floor,key)==0):
                    return True
        return False

    def _forget_completed(self, columns, radius):
        if not self.completed_points:return
        s=self.map.settings;b=s.block_size;span=math.ceil(radius/(b*s.resolution))+1
        candidates=set()
        for x,y,height in columns:
            px,py=s.origin_x+x*s.resolution,s.origin_y+y*s.resolution
            for bx in range(x//b-span,x//b+span+1):
                for by in range(y//b-span,y//b+span+1):
                    for point in self.completed_tiles.get((bx,by),()):
                        if abs(point[2]-height)<=s.step_max and math.hypot(point[0]-px,point[1]-py)<=radius:
                            candidates.add(point)
        # A neighbouring cut becoming safe or a cost moving is not itself
        # evidence that this completed view has changed. Recheck the actual
        # completed position against the current physical surface.
        remove=set()
        for point in candidates:
            check=self.map.point_safe_check(point,0.)
            if not check['ready'] or abs(check.get('ground_z',point[2])-point[2])>self.map.settings.surface_noise_tolerance:
                remove.add(point)
        for point in remove:
            self.completed_points.remove(point)
            self.completed_views.pop(point,None)
            x,y=self.map.indices(point);tile=(x//b,y//b)
            self.completed_tiles[tile].remove(point)
            if not self.completed_tiles[tile]:del self.completed_tiles[tile]
        if remove:self.completion_revision+=1

    def _observers(self, members, base=None, minimum_distance=0., completed=(), component=None,
                   window=None, deadline=None, continuation=None, floor=None, minimum_gain=1, physical=None,
                   connectivity=None, admission=None, rejected=(), ascent_limits=None,
                   direction_search=None, direction_deadline=None):
        return self.observer_cache.select(members,base,minimum_distance,completed,component,window,deadline,
                                          continuation,floor,minimum_gain,physical,connectivity,admission,
                                          rejected,ascent_limits,direction_search,direction_deadline)

    def complete(self, cid, revision, point):
        point=tuple(map(float,point))
        self.completed_points.add(point)
        self.completion_revision+=1
        x,y=self.map.indices(point);b=self.map.settings.block_size
        self.completed_tiles.setdefault((x//b,y//b),set()).add(point)
        # Completion is filtered spatially at query time. It never invalidates
        # safe map geometry, nor caches for another floor or a distant view.

    def begin_floor_attempt(self, cid):
        cluster=self.task_cluster(cid)
        if cluster is not None:
            cid=cluster['id']
        local=self.floor_query_members.get(cid)
        if not local or cluster is None or self.floor_query is None:return None
        floor,belongs,fast,failures,max_attempts,bounded_review=self.floor_query
        members={key:self.nodes[key] for key in cluster['members'] if belongs(key) and
                 not self._disconnected_on_floor(floor,key) and
                 not self._floor_records(self.floor_blacklist,floor,key) and
                 self._failure_count(floor,key)==failures}
        small_limit=self.map.settings.floor_failed_small_cluster_max_cells
        small=bounded_review and failures>0 and small_limit and self._small_floor_members(
            cluster['members'],belongs,small_limit)
        search=self.floor_view_searches.get(cid) if bounded_review else None
        tested=frozenset(search.tested_poses|search.rejected) if search is not None else frozenset()
        resume=tuple(p for p in (search.direction_poses or ()) if p not in tested) if search is not None else ()
        return FloorAttempt(cid,floor,members,local,fast,failures,max_attempts,bounded_review,bool(small),tested,resume)

    def reject_floor_view(self, cid, pose, timed_out, blockers, elapsed=0.):
        search=self.floor_view_searches.get(cid)
        if search is None:return False
        pose=tuple(pose)
        if pose not in search.rejected:
            search.elapsed+=max(0.,elapsed)
        search.rejected.add(pose)
        if search.first_rejected is None:search.first_rejected=pose
        search.route_timeout|=timed_out
        search.blockers=self.map.merge_unknown_blockers(search.blockers+list(blockers),pose)
        return True

    def clear_floor_view_search(self, cid):
        self.floor_view_searches.pop(cid,None)

    def _same_boundary(self, first, second):
        a={source[:2]:source[3] for source in first.sources}
        b={source[:2]:source[3] for source in second.sources}
        common=a.keys() & b.keys()
        return (abs(first.height-second.height)<=self.map.settings.surface_noise_tolerance+1e-7
                and bool(common) and all(a[xy]==b[xy] for xy in common))

    def _floor_records(self, records, floor, key):
        node=self.nodes[key]
        # Task scope controls scheduling, not the identity of an observation.
        # A physical boundary keeps its history across platform/floor handoff.
        return [identity for identity in records.get(key[1:3],())
                if self._same_boundary(node,identity[1])]

    def _disconnected_on_floor(self, floor, key):
        if any(record[0]==floor for record in self._floor_records(self.floor_known_barriers,floor,key)):
            return True
        height=self.nodes[key].height
        tolerance=self.map.settings.surface_noise_tolerance+1e-7
        return any(band==floor and abs(height-z)<=tolerance
                   for band,z in self.floor_disconnected.get(key[1:3],()))

    def _retire_disconnected(self, floor, members):
        for key in members:
            if self._disconnected_on_floor(floor,key):continue
            xy=key[1:3]
            self.floor_disconnected.setdefault(xy,set()).add((floor,self.nodes[key].height))
            failed=self.floor_failures.get(xy)
            if failed:
                for identity in self._floor_records(self.floor_failures,floor,key):
                    del failed[identity]
                if not failed:del self.floor_failures[xy]

    def remaining_floor_members(self, members):
        return {key for old,boundary in members.items() for key in self.by_column.get(old[1:3],())
                if self._same_boundary(self.nodes[key],boundary)}

    def _failure_count(self, scope, key):
        records=self.floor_failures.get(key[1:3],{})
        return max((records[r][0] for r in self._floor_records(self.floor_failures,scope,key)),default=0)

    def retry_completion(self, cid, floor, members=None):
        cluster=self.task_cluster(cid)
        if members is None:
            members=cluster['members'] if cluster else ()
        for key in sorted(members):
            if key not in self.nodes:continue
            records=self._floor_records(self.floor_failures,floor,key)
            views=[self.floor_failures[key[1:3]][identity][1] for identity in records]
            for view in views:
                if view is not None:return view
        return None

    def settle_floor_attempt(self, attempt, count_failure, completed_view=None, sufficient_progress=False,
                             failure_reason=None):
        if attempt is None:return 'untracked'
        self.clear_floor_view_search(attempt.cluster)
        if not count_failure and not sufficient_progress:return 'inconclusive'
        # Match physical evidence in the column. Component IDs and cut aliases
        # can change while the robot is travelling to the observation.
        remaining=self.remaining_floor_members(attempt.members)
        if not remaining:return 'progress' if sufficient_progress else 'cluster_changed'
        if count_failure and failure_reason!='floor_known_barrier':
            for key in remaining:self._watch_floor_failure(attempt.floor,key)
        if attempt.bounded_review and attempt.review and failure_reason in (
                'floor_observer_timeout','floor_route_timeout','floor_missing_connection'):
            for key in remaining:
                self.floor_review_timeouts.setdefault(key[1:3],set()).add((attempt.floor,self.nodes[key]))
            return 'review_timed_out'
        if attempt.review:
            for key in remaining:
                xy=key[1:3]
                records=self.floor_failures.get(xy,{})
                for identity in self._floor_records(self.floor_failures,attempt.floor,key):
                    del records[identity]
                if not records:self.floor_failures.pop(xy,None)
        if sufficient_progress:return 'progress'
        failures=attempt.failures+1
        exhausted=failures>=attempt.max_attempts or failure_reason=='floor_known_barrier'
        for key in remaining:
            xy=key[1:3]
            record=(attempt.floor,self.nodes[key])
            if failure_reason=='floor_known_barrier':
                self.floor_known_barriers.setdefault(xy,set()).add(record)
                continue
            if exhausted:
                self.floor_blacklist.setdefault(xy,set()).add(record)
                if attempt.failed_small:self.floor_failed_small.setdefault(xy,set()).add(record)
            else:
                self.floor_failures.setdefault(xy,{})[record]=(failures,completed_view,failure_reason,
                                                              attempt.tested_poses,attempt.resume_poses)
        if not exhausted:
            return 'first_no_progress' if failures==1 else 'deferred'
        if failure_reason=='floor_disconnected':
            self._retire_disconnected(attempt.floor,remaining)
        return 'blacklisted'

    def physical_targets(self, members):
        """Return one XYZ per physical boundary, with all cut aliases."""
        groups={}
        s=self.map.settings
        for key in sorted(members,key=lambda k:(k[1:3],self.nodes[k].height,k)):
            node=self.nodes[key]
            column=groups.setdefault(key[1:3],[])
            for _,aliases in column:
                if all(self._same_boundary(node,self.nodes[alias]) for alias in aliases):
                    aliases.append(key)
                    break
            else:column.append((key,[key]))
        return [((s.origin_x+x*s.resolution,s.origin_y+y*s.resolution,self.nodes[rep].height),aliases)
                for (x,y),column in groups.items() for rep,aliases in column]

    def _small_floor_members(self, members, belongs, limit):
        """Classify the complete floor cluster, stopping once it is large enough."""
        floor_members, columns = set(), set()
        for key in members:
            if belongs(key):
                floor_members.add(key)
                columns.add(key[1:3])
                if len(columns)>limit:
                    return set()
        return floor_members if len(self.physical_targets(floor_members))<=limit else set()

    def floor_observation_counts(self, covered):
        # One XY can have duplicate cuts and independent physical surfaces.
        columns={}
        for (xy,boundary),matches in sorted(covered.items(),key=lambda item:item[0][1].height):
            groups=columns.setdefault(xy,[])
            for group in reversed(groups):
                if all(self._same_boundary(boundary,other) for other in group['boundaries']):
                    group['boundaries'].append(boundary)
                    group['matches'].update(matches)
                    break
            else:
                groups.append(dict(height=boundary.height,boundaries=[boundary],matches=set(matches)))
        resolved=0
        evidence=self.map.surface_observation
        for xy,groups in columns.items():
            # A real return can resolve only one physical task in this column.
            candidates=sorted((abs(height-g['height']),i,sid) for i,g in enumerate(groups)
                              if all(not self.map.ground_pending(xy,b.layer) for b in g['boundaries'])
                              for sid,height in g['matches'])
            resolved_groups={i for i,g in enumerate(groups) if any(
                evidence.frontier_settled(xy,b.height,[(x,y,sid) for x,y,_,sid in b.sources],layer=b.layer)
                for b in g['boundaries'])}
            matched_surfaces=set()
            for _,i,sid in candidates:
                if i not in resolved_groups and sid not in matched_surfaces:
                    resolved_groups.add(i);matched_surfaces.add(sid)
            resolved+=len(resolved_groups)
        return sum(map(len,columns.values())),resolved

    def physical_tasks(self):
        """Committed task/spatial indexes; queries never rebuild their contents."""
        return self._physical_task_cache

    def _update_physical_tasks(self, changed_clusters, columns):
        began=time.monotonic()
        tasks,spatial,owners,aliases=self._physical_task_cache
        columns=set(columns)
        touched=set(changed_clusters)
        for xy in columns:
            for first,last in self._physical_column_edges.get(xy,()):
                touched.update((first,last))
            touched.update(self.membership[key] for key in self.by_column.get(xy,()))

        for xy in columns:
            previous=self._physical_column_edges.pop(xy,set())
            current=set()
            for _,members in self.physical_targets(self.by_column.get(xy,())):
                ids=sorted({self.membership[key] for key in members})
                current.update((ids[0],other) for other in ids[1:])
            for first,last in previous-current:
                for source,target in ((first,last),(last,first)):
                    neighbours=self._physical_contacts[source]
                    neighbours[target]-=1
                    if not neighbours[target]:
                        del neighbours[target]
                    if not neighbours:
                        del self._physical_contacts[source]
            for first,last in current-previous:
                self._physical_contacts.setdefault(first,Counter())[last]+=1
                self._physical_contacts.setdefault(last,Counter())[first]+=1
            if current:
                self._physical_column_edges[xy]=current

        # Include whole old tasks when a changed column joins or separates
        # their aliases. Everything else retains its cached task and views.
        affected=set(touched)
        pending=deque(touched)
        visited_tasks=set()
        while pending:
            cid=pending.popleft()
            neighbours=set(self._physical_contacts.get(cid,()))
            task_id=owners.get(cid)
            if task_id is not None and task_id not in visited_tasks:
                visited_tasks.add(task_id)
                neighbours.update(aliases[task_id])
            new=neighbours-affected
            affected.update(new)
            pending.extend(new)
        for task_id in visited_tasks:
            task=tasks.pop(task_id)
            for tile in task['tiles']:
                spatial[tile].remove(task_id)
                if not spatial[tile]:
                    del spatial[tile]
            for cid in aliases.pop(task_id):
                del owners[cid]

        remaining={cid for cid in affected if cid in self.clusters}
        while remaining:
            first=min(remaining)
            remaining.remove(first)
            group={first}
            pending=deque([first])
            while pending:
                for other in self._physical_contacts.get(pending.popleft(),{}).keys() & remaining:
                    remaining.remove(other)
                    group.add(other)
                    pending.append(other)
            if len(group)==1:
                task=self.clusters[first]
            else:
                members=set().union(*(self.clusters[cid]['members'] for cid in group))
                task=FrontierCluster(self,**self.describe_task(first,members))
            tasks[first]=task
            aliases[first]=group
            owners.update((cid,first) for cid in group)
            for tile in task['tiles']:
                spatial.setdefault(tile,set()).add(first)
        self.last_work['physical_columns_checked']=len(columns)
        self.last_work['physical_clusters_updated']=len(affected)
        self.last_work['physical_tasks_ms']=1000*(time.monotonic()-began)

    def _cluster_fragments(self, cid):
        owner=self.components.by_namespace[cid % SLICE_ID_STRIDE]
        return owner.cluster_fragments[cid]

    def describe_task(self, cid, members):
        s=self.map.settings
        points=np.asarray([point for point,_ in self.physical_targets(members)])
        at=int(np.argmin(np.linalg.norm(points-np.median(points,axis=0),axis=1)))
        point=points[at]
        return dict(id=cid,members=members,tiles={(k[1]//s.block_size,k[2]//s.block_size) for k in members},
                    revision=self.revision,frontier=point.tolist(),cell=self.map.indices(point).tolist(),
                    gain=len(points)*s.resolution,cells=len(points),
                    display=points[np.linspace(0,len(points)-1,min(len(points),256),dtype=int)].tolist())

    def query(self, base=None, minimum_distance=0., window_size=None, query_budget=0., continuation=None,
              floor=None, stairs=(), evaluate_observers=True, fast=False, prune_disconnected=False,
              floor_selection=None, attempt_scope=None, max_attempts=2, ascent_limits=None,
              ordinary_floor=True, review_remaining=None):
        began=time.monotonic()
        s=self.map.settings
        output=[]
        managed_cross=attempt_scope is not None
        seek_ascent=managed_cross and continuation is None
        bounded_floor=evaluate_observers and (managed_cross or (floor is not None and continuation is None))
        ordinary_floor=ordinary_floor and bounded_floor and not managed_cross
        review_enabled=not ordinary_floor or s.floor_global_review_enabled
        small_limit=s.floor_failed_small_cluster_max_cells if ordinary_floor else 0
        bounded_review=ordinary_floor and review_enabled and review_remaining is not None
        connectivity_deadline=began+.08
        task_scope=attempt_scope if managed_cross else floor
        floor_selection=floor_selection if bounded_floor else None
        prune_disconnected=prune_disconnected and bounded_floor and not managed_cross and base is not None
        connectivity=self.map.floor_connectivity(floor) if prune_disconnected else self.map.connectivity
        component=connectivity.component(self.map.indices(base)) if evaluate_observers and base is not None else None
        unreachable=0
        window=None
        clusters, spatial, _, aliases = self.physical_tasks()
        ids=clusters.keys()
        if window_size is not None:
            if base is None:raise ValueError('pct_local_window_requires_base')
            radius,above,below=map(float,window_size)
            if not np.isfinite([radius,above,below]).all() or min(radius,above,below)<=0:
                raise ValueError('pct_invalid_local_window')
            base=np.asarray(base,dtype=float)
            window=(base-[radius,radius,below],base+[radius,radius,above])
            s=self.map.settings;b=s.block_size
            lo=np.floor((window[0][:2]-[s.origin_x,s.origin_y])/(s.resolution*b)).astype(int)
            hi=np.floor((window[1][:2]-[s.origin_x,s.origin_y])/(s.resolution*b)).astype(int)
            ids=set()
            for bx in range(lo[0],hi[0]+1):
                for by in range(lo[1],hi[1]+1):ids.update(spatial.get((bx,by),()))
        built_before=self.observer_cache.built
        prepared=[]
        physical_entries={}
        queues=[[] for _ in range(max_attempts)]
        deferred_clusters=set()
        deferred_members=set()
        small_clusters=0
        small_members=set()
        disconnected_members=set()
        review_deferred=set()
        missing_connection=set()
        connection_reviews=set()
        def on_floor(z):
            return abs(z-floor[0]) <= floor[1]

        # Index only the boundary members supported by this floor. The same
        # cluster or XY column can also contain an independent upper boundary.
        floor_members=None
        if floor is not None:
            indices={}
            tread_tolerance=min(s.surface_normal_thickness,s.surface_noise_tolerance)
            for flight in stairs:
                if not flight.confirmed:
                    continue
                for xy in flight.entrance_cells():
                    indices.setdefault(xy,[]).append(flight)

            def belongs(key):
                node=self.nodes[key]
                if not on_floor(node.height) or not any(on_floor(src[2]) for src in node.sources):
                    return False
                x,y=key[1:3]
                for flight in indices.get((x,y),()):
                    # A measured first tread already explains elevated lip
                    # frontiers. Keep the stricter rule for floor-level ones.
                    if floor[0]+tread_tolerance < node.height <= flight.treads[0][2]+tread_tolerance:
                        return False
                    if flight.owns_boundary(x,y,node.sources,on_floor):
                        return False
                return True
            floor_members=belongs
        if seek_ascent:
            max_step,min_rise=ascent_limits

            def ascending_member(key):
                node=self.nodes[key]
                low,high=base[2]+min_rise,base[2]+max_step+1e-7
                return (low-s.surface_noise_tolerance<=node.height<=high and
                        any(low<=source[2]<=high for source in node.sources))

            floor_members=ascending_member
        elif managed_cross:
            level_filter=floor_members
            yaw,min_height,slack=continuation
            direction=(math.cos(yaw),math.sin(yaw))
            max_step,min_rise=ascent_limits if ascent_limits is not None else (math.inf,0.)

            def forward_member(key):
                node=self.nodes[key]
                dx=s.origin_x+key[1]*s.resolution-base[0]
                dy=s.origin_y+key[2]*s.resolution-base[1]
                ahead=dx*direction[0]+dy*direction[1]
                if ahead < -slack:
                    return False
                if level_filter is not None:
                    return level_filter(key) and abs(node.height-floor[0])<=floor[1]+s.surface_noise_tolerance
                return (abs(dx*direction[1]-dy*direction[0])<=1.5
                        and min_height-s.surface_noise_tolerance<=node.height<=base[2]+max_step+1e-7
                        and any(min_height<=source[2]<=base[2]+max_step+1e-7 for source in node.sources))

            floor_members=forward_member
        for cid in sorted(ids):
            cluster=clusters[cid]
            members=cluster['members']
            if window is not None:
                s=self.map.settings
                # A large component may cross the window. Read its local
                # fragments, not every distant member of that component.
                fragments=(fid for alias in aliases[cid]
                           for fid in self._cluster_fragments(alias))
                local_fragments=[fid for fid in fragments if
                    (lo<=self.components.tile(next(iter(self.components.fragments[fid])))[1:]).all() and
                    (hi>=self.components.tile(next(iter(self.components.fragments[fid])))[1:]).all()]
                members={key for fid in local_fragments for key in self.components.fragments[fid] if
                    window[0][0]-1e-8<=s.origin_x+key[1]*s.resolution<=window[1][0]+1e-8 and
                    window[0][1]-1e-8<=s.origin_y+key[2]*s.resolution<=window[1][1]+1e-8 and
                    window[0][2]-1e-8<=self.nodes[key].height<=window[1][2]+1e-8}
            if floor_members is not None:
                members={key for key in members if floor_members(key)}
            if not members:continue
            if bounded_floor:
                if prune_disconnected:
                    retired={key for key in members if self._disconnected_on_floor(task_scope,key)}
                    disconnected_members.update(retired)
                    members=members-retired
                blocked={key for key in members if self._floor_records(self.floor_blacklist,task_scope,key)}
                if blocked:
                    deferred_members.update(blocked)
                    deferred_clusters.add(cid)
                    members=members-blocked
                    if bounded_review:
                        failed_small={key for key in blocked if self._floor_records(self.floor_failed_small,task_scope,key)}
                        if failed_small:
                            small_clusters+=1
                            small_members.update(failed_small)
                            deferred_members.difference_update(failed_small)
                tasks={}
                for key in members:
                    failures=self._failure_count(task_scope,key)
                    if failures<max_attempts:
                        tasks.setdefault(failures,set()).add(key)
                failed_members={key for failures,keys in tasks.items() if failures for key in keys}
                if failed_members:
                    if bounded_review:
                        timed_out={key for key in failed_members if
                                   self._floor_records(self.floor_review_timeouts,task_scope,key)}
                        needs_connection=any(self.floor_failures[key[1:3]][record][2]=='floor_missing_connection'
                                             for key in failed_members
                                             for record in self._floor_records(self.floor_failures,task_scope,key))
                        if needs_connection:connection_reviews.add(cid)
                        waiting=needs_connection and (component is None or time.monotonic()>=connectivity_deadline or
                                 self.observer_cache.disconnected(failed_members,connectivity,component,floor))
                        if waiting:
                            missing_connection.add(cid)
                        if timed_out or (review_remaining<=0. and not waiting):
                            review_deferred.add(cid)
                        if waiting:
                            timed_out=failed_members
                        if timed_out or review_remaining<=0.:
                            deferred_members.update(timed_out if review_remaining>0. else failed_members)
                            for count in tasks:
                                if count:
                                    tasks[count]-=timed_out
                            if review_remaining<=0.:
                                tasks={0:tasks[0]} if 0 in tasks else {}
                    small=not bounded_review and small_limit and self._small_floor_members(cluster['members'],floor_members,small_limit)
                    if small:
                        small_clusters+=1
                        small_members.update(failed_members)
                    elif not review_enabled:
                        deferred_members.update(failed_members)
                        deferred_clusters.add(cid)
                    if small or not review_enabled:
                        tasks={0:tasks[0]} if 0 in tasks else {}
            else:
                tasks={0:members}
            for failures,task in tasks.items():
                if not task:continue
                physical=self.physical_targets(task)
                if base is None:
                    rank=0.
                else:
                    xyz=np.asarray([point for point,_ in physical])
                    rank=float(np.linalg.norm(xyz-base,axis=1).min())-math.log1p(len(xyz)*s.resolution)
                    if bounded_floor and not managed_cross:
                        from .pct_frontier_observers import observation_effort
                        rank=observation_effort(float(np.linalg.norm(xyz[:,:2]-base[:2],axis=1).min()),len(xyz))
                priority=int(failures>0 and cid in connection_reviews)
                if managed_cross:
                    if floor is None:
                        priority=int(not any(self.nodes[k].height-base[2]>=min_rise for k in task))
                    else:
                        rank-=max(0.,float(np.max((xyz[:,:2]-base[:2]) @ direction)))
                queues[failures].append(((priority,rank),cid,task))
                physical_entries[cid,failures]=physical
        reviews={cid for queue in queues[1:] for _,cid,_ in queue}
        allowed_rounds=len(queues) if window is None or managed_cross else 1
        failures=next((i for i in range(allowed_rounds) if queues[i]),0)
        prepared=queues[failures]
        reviewing=failures>0
        directional=ordinary_floor and floor_selection is not None and (
            bounded_review if reviewing else s.floor_four_direction_search)
        entries={}
        # Match suspended searches to current physical members, including
        # components rekeyed by incremental map updates.
        resume=None
        resumed_ms=0.
        resumed_slices=0
        if bounded_floor and self.floor_pending is not None:
            old_floor,old_failures,old_members,resumed_ms,resumed_slices=self.floor_pending
            if old_floor==task_scope and old_failures==failures:
                remaining=self.remaining_floor_members(old_members)
                resume=next((cid for _,cid,members in prepared if remaining & members),None)
            self.floor_pending=None
        search_scope=(task_scope,window_size is None,failures, floor is not None)
        rotate_local=floor_selection is not None and window is not None and not managed_cross and not reviewing
        if floor_selection is not None:
            active_searches={}
            for search in sorted(self.floor_view_searches.values(),key=lambda s:-s.elapsed):
                if search.scope!=search_scope:continue
                remaining=self.remaining_floor_members(search.members)
                if not remaining:continue
                overlap,cid=max(((len(remaining & members),cid) for _,cid,members in prepared),default=(0,None))
                if overlap and cid not in active_searches:
                    search.cluster=cid
                    active_searches[cid]=search
            self.floor_view_searches=active_searches
            route_retries=[cid for cid,search in active_searches.items()
                           if search.direction_center is not None and search.rejected]
            if route_retries:
                resume=max(route_retries,key=lambda cid:active_searches[cid].last_slice)
            elif not rotate_local and active_searches:
                resume=max(active_searches,key=lambda cid:active_searches[cid].last_slice)
        self.floor_query=(task_scope,floor_members,fast,failures,max_attempts,bounded_review) if bounded_floor else None
        self.floor_query_members={cid:members for _,cid,members in prepared} if bounded_floor else {}
        scoped_cells=0
        for _,cid,members in prepared:
            cluster=clusters[cid];gain=0.
            if window is not None or bounded_floor or floor is not None:
                xyz=np.asarray([point for point,_ in physical_entries[cid,failures]])
                scoped_cells+=len(xyz)
                at=0 if base is None else int(np.argmin(np.linalg.norm(xyz-base,axis=1)))
                frontier=xyz[at].tolist()
                output.append(dict(id=cid,revision=cluster['revision'],frontier=frontier,
                    cell=self.map.indices(frontier).astype(int).tolist(),gain=gain,cells=len(xyz),
                    display=xyz[np.linspace(0,len(xyz)-1,min(len(xyz),256),dtype=int)].tolist(),observations=[]))
            else:
                scoped_cells+=cluster['cells']
                output.append({name:cluster[name] for name in ('id','revision','frontier','cell','gain','cells','display')}|
                              dict(observations=[],gain=gain))
            output[-1]['attempt']=failures+1 if bounded_floor else 0
            output[-1]['observation_yaws']=[]
            output[-1]['observers_evaluated']=False
            output[-1]['observers_complete']=False
            entries[cid]=output[-1]
        scope=(None if base is None else tuple(np.rint(np.asarray(base)/self.map.settings.resolution).astype(int)),
               None if window_size is None else tuple(window_size),minimum_distance,continuation,task_scope,reviewing,fast)
        if evaluate_observers and (self.query_scope!=scope or not query_budget):
            self.query_visited.clear();self.query_scope=scope
        slice_budget=floor_selection[-1] if floor_selection is not None and reviewing else query_budget
        if bounded_review and reviewing:
            slice_budget=min(.20,slice_budget) if slice_budget else .20
        preparation_ms=1000*(time.monotonic()-began)
        deadline=(time.monotonic() if bounded_floor else began)+slice_budget if slice_budget else None
        if bounded_review and reviewing:
            deadline=min(deadline if deadline is not None else math.inf,began+review_remaining)
        no_view_clusters=[]
        timed_out_clusters=[]
        skipped=set()
        checked=0;geometry_ms=visibility_ms=0.;phase_ms={}
        ordered=sorted(prepared,key=lambda row:(row[1]!=resume,row[0],row[1]))
        if rotate_local:
            def last_slice(row):
                search=self.floor_view_searches.get(row[1])
                return search.last_slice if search is not None else 0

            ordered[:4]=sorted(ordered[:4],key=lambda row:(row[1]!=resume,last_slice(row)))
        for _,cid,members in (ordered if evaluate_observers else ()):
            revision=clusters[cid]['revision']
            prior=self.query_visited.get(cid)
            if not bounded_floor and query_budget and prior is not None and prior[:2]==(revision,members):
                continue
            self.query_visited.pop(cid,None)
            if deadline is not None and (checked or (bounded_review and reviewing)) and (
                    checked>=4 or time.monotonic()>=deadline):break
            cluster=clusters[cid]
            observer_started=time.monotonic() if directional else None
            if prune_disconnected and component is not None:
                start_geometry=time.monotonic()
                disconnected=self.observer_cache.disconnected
                if disconnected(members,connectivity,component,floor):
                    # The local window and the review queue may only hold a
                    # subset. Count a failure only with a full-task proof;
                    # the controller settles it through the same two attempts
                    # used for observation and execution failures.
                    whole={key for key in cluster['members'] if floor_members(key) and
                           not self._disconnected_on_floor(floor,key) and
                           not self._floor_records(self.floor_blacklist,floor,key) and
                           self._failure_count(task_scope,key)==failures}
                    proven=whole==members or disconnected(whole,connectivity,component,floor)
                    if proven:
                        reason='floor_disconnected'
                        if bounded_review and time.monotonic()<connectivity_deadline:
                            reason='floor_known_barrier' if connectivity.known_barrier(
                                self.map.indices(base),self.observer_cache.station_cells(whole,floor),
                                min(time.monotonic()+.04,connectivity_deadline)) else 'floor_missing_connection'
                        elif bounded_review:
                            reason='floor_missing_connection'
                        entries[cid].update(floor_failure=reason,
                            observers_evaluated=True,observers_complete=True)
                        no_view_clusters.append(cid)
                    elif window is not None:
                        # Other members outside this window may have a route.
                        # Let the global pass examine them without charging an attempt.
                        scoped_cells-=entries[cid]['cells']
                        skipped.add(cid)
                        self.floor_query_members.pop(cid,None)
                    if proven or window is not None:
                        self.clear_floor_view_search(cid)
                        self.observer_cache.discard_search(members)
                        checked+=1
                        geometry_ms+=1000*(time.monotonic()-start_geometry)
                        continue
                geometry_ms+=1000*(time.monotonic()-start_geometry)
            completed=self._completed_near(members)
            retry=self.retry_completion(cid,task_scope,members) if reviewing else None
            # A failed view search has no coverage record, but its old stand
            # position can still be avoided when choosing the review pose.
            if retry is not None and len(retry)==3 and retry not in completed and not (bounded_review and reviewing):
                completed.append(retry)
            observer_args=(members,base,minimum_distance,completed,component,window,deadline)
            observer_options={}
            if continuation is not None:observer_options['continuation']=continuation
            if ascent_limits is not None:observer_options['ascent_limits']=ascent_limits
            if floor is not None:observer_options['floor']=floor
            observer_options['physical']=physical_entries[cid,failures]
            if prune_disconnected:observer_options['connectivity']=connectivity
            if fast:observer_options['minimum_gain']=min(s.fast_min_visible_cells,entries[cid]['cells'])
            if observer_started is None:
                observer_started=time.monotonic()
            if floor_selection is not None:
                search=self.floor_view_searches.get(cid)
                if search is None:
                    search=self.floor_view_searches[cid]=FloorViewSearch(cid,search_scope,{})
                search.cluster=cid
                search.members={key:self.nodes[key] for key in members}
                self.floor_view_slice+=1
                search.last_slice=self.floor_view_slice
                admission,arrival_geometry,tolerance,first_budget,review_budget,_=floor_selection
                total_budget=review_budget if reviewing else first_budget
                if directional:
                    total_budget=min(1.,review_budget) if reviewing else s.floor_direction_search_timeout
                    if search.direction_center is None:
                        search.direction_center=tuple(cluster['frontier'][:2])
                        if reviewing:
                            records=[self.floor_failures[key[1:3]][record] for key in members
                                     for record in self._floor_records(self.floor_failures,task_scope,key)]
                            search.rejected.update(p for record in records for p in record[3])
                            search.resume_poses=tuple(dict.fromkeys(p for record in records for p in record[4]))
                            search.pose_limit=2
                remaining=max(0.,total_budget-search.elapsed)
                limit=observer_started+remaining
                if directional:
                    observer_options.update(direction_search=search,direction_deadline=limit)
                if deadline is not None:limit=min(deadline,limit)
                observer_args=observer_args[:6]+(limit,)
                observer_options.update(admission=(admission,arrival_geometry,tolerance,remaining>0.),
                                        rejected=search.rejected)
            observations=self._observers(*observer_args,**observer_options)
            # Prefer a new view. Allow one more look if the first failed
            # sweep's visit mask removes the last available option.
            if not observations and not self.observer_cache.pending and retry in completed and not (bounded_review and reviewing):
                retry_args=observer_args[:3]+([p for p in completed if p!=retry],)+observer_args[4:]
                observations=self._observers(*retry_args,**observer_options)
            checked+=1
            geometry_ms+=self.observer_cache.geometry_ms
            visibility_ms+=self.observer_cache.visibility_ms
            for phase,elapsed in self.observer_cache.phase_ms.items():
                phase_ms[phase]=phase_ms.get(phase,0.)+elapsed
            complete=not self.observer_cache.pending
            unreachable+=self.observer_cache.last_unreachable
            views=[self.observer_cache.last_views[tuple(p)] for p in observations]
            gain=max((v['gain'] for v in views),default=0)*self.map.settings.resolution
            entries[cid].update(observations=observations,gain=gain,observers_evaluated=True,
                                observers_complete=complete,observation_yaws=[v['yaw'] for v in views])
            if directional:
                entries[cid]['direction_limited']=True
            if bounded_floor:
                elapsed_ms=1000*(time.monotonic()-observer_started)+(resumed_ms if cid==resume else 0.)
                if floor_selection is not None:
                    search.elapsed+=time.monotonic()-observer_started
                    entries[cid]['observer_ms']=1000*search.elapsed
                    search.blockers=self.map.merge_unknown_blockers(
                        search.blockers+self.observer_cache.unknown_blockers,base)
                    if observations:
                        break
                    if not complete and search.elapsed<total_budget:
                        if rotate_local:continue
                        break
                    entries[cid]['floor_failure']=(
                        'floor_route_timeout' if search.route_timeout else
                        'floor_observer_timeout' if not complete else
                        'floor_no_route' if search.rejected else 'floor_no_observation')
                    entries[cid]['unknown_blockers']=search.blockers
                    if search.first_rejected is not None:
                        entries[cid]['first_rejected']=list(search.first_rejected)
                    if not bounded_review or reviewing:self.clear_floor_view_search(cid)
                    self.observer_cache.discard_search(members)
                    no_view_clusters.append(cid)
                    if search.blockers:break
                    continue
                entries[cid]['observer_ms']=elapsed_ms
                slices=1+(resumed_slices if cid==resume else 0)
                # The global review is the last chance to find a viewpoint.
                # Keep its progress across ticks with a finite per-cluster limit.
                if not complete and not observations and slices<(4 if reviewing else 2):
                    self.floor_pending=(task_scope,failures,{key:self.nodes[key] for key in members},elapsed_ms,slices)
                    break
                self.observer_cache.discard_search(members)
                if not observations:
                    (no_view_clusters if complete else timed_out_clusters).append(cid)
                else:
                    break
            elif complete or (observations and (window is not None or floor is None)):
                self.query_visited[cid]=(revision,members,not observations)
        if bounded_floor:
            pending=(self.floor_pending is not None or
                     (floor_selection is not None and bool(self.floor_view_searches)) or checked<len(prepared))
        else:
            pending=bool(evaluate_observers and query_budget and any(
                self.query_visited.get(cid,(None,None))[:2]!=(clusters[cid]['revision'],members)
                for _,cid,members in prepared))
        small_count=len(self.physical_targets(small_members))
        deferred_count=len(self.physical_targets(deferred_members))
        disconnected_count=len(self.physical_targets(disconnected_members))
        output=[row for row in output if row['id'] not in skipped]
        if evaluate_observers and not pending:self.query_visited.clear()
        return dict(frontier_cells=sum(c['cells'] for c in clusters.values()),frontier_layer_cells=len(self.nodes),
            scope='local' if window is not None else 'global',
            window=list(map(float,window_size)) if window is not None else [],
            window_center=list(map(float,base)) if window is not None else [],
            deferred_floor_frontier_cells=deferred_count+disconnected_count+small_count,
            disconnected_floor_frontier_cells=disconnected_count,
            small_skipped_clusters=small_clusters,small_skipped_cells=small_count,
            fast_floor=fast,fast_skipped_clusters=len(deferred_clusters) if fast else 0,
            fast_skipped_cells=deferred_count if fast else 0,
            pending_review_clusters=len(reviews),reviewing=reviewing,no_view_clusters=no_view_clusters,
            deferred_review_clusters=len(review_deferred),missing_connection_clusters=len(missing_connection),
            timed_out_clusters=timed_out_clusters,scoped_frontier_cells=scoped_cells,
            query_pending=pending,query_checked_clusters=checked,observer_prepare_ms=preparation_ms,
            observer_geometry_ms=geometry_ms,observer_visibility_ms=visibility_ms,
            observer_phase_ms=phase_ms,
            clusters=output,known_surface_cells=self.known_count,safe_cells=self.safe_count,
            no_observation_clusters=sum(c['observers_evaluated'] and not c['observations'] for c in output),
            unreachable_observers=unreachable,
            observer_cache_built=self.observer_cache.built-built_before,
            query_ms=1000*(time.monotonic()-began),
            extraction_ms=self.last_work['update_ms']+1000*(time.monotonic()-began),
            index_work=dict(self.last_work),index_revision=self.revision)


def rebuild_frontiers(mapping, observation_radius=2.):
    """Full rebuild oracle for regression tests, never called by production."""
    index=FrontierIndex(mapping,observation_radius)
    columns=set(mapping.column_layers)
    columns.update((x,y) for (_,x,y),_ in mapping.iter_cells(True))
    for k in mapping.layers:
        index.add_slice(k)
    index.update(columns)
    return index
