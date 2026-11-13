"""Exact frontier connectivity: relabel touched cut/XY tiles, then join fragments.

Fragments include height-compatible contacts within a cut. They are internal
index entries for the owning slice. Deleting a bridge can still require a
walk of its component, but that walk visits fragments rather than every cell.
"""
from collections import Counter, deque
import weakref

from .pct_frontier_index import SLICE_ID_STRIDE


class FrontierComponents:
    def __init__(self, index, namespace):
        self._index = weakref.ref(index)
        self.cells = {}
        self.fragments = {}
        self.tile_fragments = {}
        self.node_fragment = {}
        self.graph = {}
        self.cluster = {}
        self.cluster_fragments = {}
        self.next_fragment = namespace

    def tile(self, key):
        b = self._index().map.settings.block_size
        return key[0], key[1] // b, key[2] // b

    def replace(self, removed, added):
        for key in removed:
            tile = self.tile(key)
            self.cells[tile].remove(key)
            if not self.cells[tile]:
                del self.cells[tile]
        for key in added:
            self.cells.setdefault(self.tile(key), set()).add(key)

    def update(self, changed):
        index = self._index()
        dirty = {self.tile(key) for key in changed}
        affected = set()
        old_owner = {}
        new_fragments = set()
        relabelled = 0
        for tile in sorted(dirty):
            for fid in self.tile_fragments.pop(tile, ()):
                cid = self.cluster.pop(fid)
                affected.add(cid)
                self.cluster_fragments[cid].discard(fid)
                for key in self.fragments.pop(fid):
                    old_owner[key] = cid
                    self.node_fragment.pop(key)
                for other in self.graph.pop(fid):
                    self.graph[other].discard(fid)
            remaining = set(self.cells.get(tile, ()))
            relabelled += len(remaining)
            fragments = set()
            while remaining:
                first = min(remaining)
                remaining.remove(first)
                members = {first}
                queue = deque([first])
                while queue:
                    for other in index.neighbours(queue.popleft()):
                        if other in remaining:
                            remaining.remove(other)
                            members.add(other)
                            queue.append(other)
                fid = self.next_fragment
                self.next_fragment += SLICE_ID_STRIDE
                self.fragments[fid] = members
                self.graph[fid] = set()
                self.node_fragment.update((key, fid) for key in members)
                fragments.add(fid)
            if fragments:
                self.tile_fragments[tile] = fragments
                new_fragments.update(fragments)
        # All touched tiles now have their final labels. Join exact boundary
        # contacts, including diagonal tile corners.
        for fid in new_fragments:
            for key in self.fragments[fid]:
                for other in index.neighbours(key):
                    adjacent = self.node_fragment[other]
                    if adjacent == fid:
                        continue
                    self.graph[fid].add(adjacent)
                    self.graph[adjacent].add(fid)
                    if adjacent in self.cluster:
                        affected.add(self.cluster[adjacent])
        work = set(new_fragments)
        for cid in affected:
            work.update(self.cluster_fragments.pop(cid))
        components = []
        remaining = set(work)
        while remaining:
            first = min(remaining)
            remaining.remove(first)
            members = {first}
            queue = deque([first])
            while queue:
                for other in self.graph[queue.popleft()]:
                    if other in remaining:
                        remaining.remove(other)
                        members.add(other)
                        queue.append(other)
            components.append(members)
        old_clusters = {cid: index._drop_cluster(cid) for cid in affected}
        used = set()
        # Stable IDs: the largest surviving overlap keeps the old identity.
        components.sort(key=lambda f: (-sum(len(self.fragments[n]) for n in f), min(f)))
        for fragments in components:
            members = set().union(*(self.fragments[fid] for fid in fragments))
            overlaps = Counter()
            for fid in fragments:
                if fid in self.cluster:
                    overlaps[self.cluster[fid]] += len(self.fragments[fid])
                else:
                    overlaps.update(old_owner[k] for k in self.fragments[fid] if k in old_owner)
            candidates = sorted((cid for cid in overlaps if cid not in used),
                                key=lambda cid: (-overlaps[cid], cid))
            cid = candidates[0] if candidates else index.next_id
            if not candidates:
                index.next_id += SLICE_ID_STRIDE
            used.add(cid)
            self.cluster_fragments[cid] = fragments
            self.cluster.update((fid, cid) for fid in fragments)
            index._refresh(cid, members, old_clusters.get(cid))
        return dict(relabelled_tiles=len(dirty), reclustered_layers=relabelled,
                    component_fragments_visited=len(work), updated_clusters=len(components))
