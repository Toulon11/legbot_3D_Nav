"""Maintain clusters from changed members and their final neighbours.

Spatial fragments group a cluster's members by tile for observer queries.
Only affected clusters participate in connectivity updates.
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
        self.cluster_fragments = {}
        self.next_fragment = namespace

    def tile(self, key):
        size = self._index().map.settings.block_size
        return key[0], key[1] // size, key[2] // size

    def replace(self, removed, added):
        for key in removed:
            tile = self.tile(key)
            self.cells[tile].remove(key)
            if not self.cells[tile]:
                del self.cells[tile]
        for key in added:
            self.cells.setdefault(self.tile(key), set()).add(key)

    def _walk(self, first, allowed, targets=None):
        index = self._index()
        visited = {first}
        queue = deque([first])
        missing = set(targets or ()) - visited
        while queue:
            if targets is not None and not missing:
                break
            for other in index.neighbours(queue.popleft()):
                if other in allowed and other not in visited:
                    visited.add(other)
                    missing.discard(other)
                    queue.append(other)
        self.work['component_nodes_visited'] += len(visited)
        return visited

    def _remaining_groups(self, members, terminals, added, changed):
        if not members:
            return []
        if len(terminals) <= 1:
            return [members]
        index = self._index()
        self.work['local_connectivity_checks'] += 1
        area = {(key[1]+dx, key[2]+dy) for key in changed
                for dx in range(-2, 3) for dy in range(-2, 3)}
        local = {key for xy in area for key in index.by_column.get(xy, ())
                 if key in members or key in added}
        first = min(terminals)
        if terminals <= self._walk(first, local, terminals):
            self.work['local_connectivity_preserved'] += 1
            return [members]
        self.work['cluster_rebuilds'] += 1
        self.work['reclustered_layers'] += len(members)
        visited = self._walk(first, members, terminals)
        if terminals <= visited:
            return [members]
        groups = [visited]
        remaining = members - visited
        while remaining:
            group = self._walk(min(remaining), remaining)
            groups.append(group)
            remaining.difference_update(group)
        return groups

    def _update_fragments(self, old_ids, new_ids):
        """Retain unchanged spatial buckets and invalidate changed ones."""
        index = self._index()
        old = {}
        for cid in old_ids:
            for fid in self.cluster_fragments.pop(cid, ()):
                members = self.fragments[fid]
                old[self.tile(next(iter(members))), cid] = fid
        buckets = {}
        for cid in new_ids:
            for key in index.clusters[cid]['members']:
                buckets.setdefault((self.tile(key), cid), set()).add(key)
        touched = set()
        for (tile, cid), members in buckets.items():
            fid = old.get((tile, cid))
            if fid is not None and self.fragments[fid] == members:
                old.pop((tile, cid))
            else:
                fid = self.next_fragment
                self.next_fragment += SLICE_ID_STRIDE
                self.fragments[fid] = members
                self.tile_fragments.setdefault(tile, set()).add(fid)
                touched.add(tile)
            self.cluster_fragments.setdefault(cid, set()).add(fid)
            self.node_fragment.update((key, fid) for key in members)
        for (tile, _), fid in old.items():
            for key in self.fragments.pop(fid):
                if self.node_fragment.get(key) == fid:
                    del self.node_fragment[key]
            self.tile_fragments[tile].remove(fid)
            if not self.tile_fragments[tile]:
                del self.tile_fragments[tile]
            index.stale_observers.add((fid, index.layer))
            touched.add(tile)
        self.work['relabelled_tiles'] = len(touched)

    def update(self, changed, previous_edges):
        self.work = dict(relabelled_tiles=0, reclustered_layers=0,
                         component_fragments_visited=0, updated_clusters=0,
                         affected_clusters=0, component_nodes_visited=0,
                         local_connectivity_checks=0, local_connectivity_preserved=0,
                         cluster_rebuilds=0)
        if not changed:
            return self.work
        index = self._index()
        current = {key: set(index.neighbours(key)) if key in index.nodes else set()
                   for key in changed}
        affected = {index.membership[key] for key in changed if key in index.membership}
        affected.update(index.membership[other] for neighbours in current.values()
                        for other in neighbours if other in index.membership)
        removed = {key for key in changed if key not in index.nodes}
        added = {key for key in changed if key in index.nodes and key not in index.membership}
        terminals = {}
        for key in changed:
            for other in (previous_edges.get(key) or set()) - current[key]:
                cid = index.membership.get(key)
                if cid is None or cid != index.membership.get(other):
                    continue
                ends = terminals.setdefault(cid, set())
                ends.update(node for node in (key, other) if node in index.nodes)
        groups = []
        for cid in sorted(affected):
            members = index.clusters[cid]['members'] - removed
            groups.extend(self._remaining_groups(members, terminals.get(cid, set()), added, changed))
        pending = set(added)
        while pending:
            group = self._walk(min(pending), pending)
            groups.append(group)
            pending.difference_update(group)

        # Only changed edges can join the connected groups above. Reuse their
        # connectivity instead of walking every unchanged member's edges.
        owners = {key: group for group, members in enumerate(groups) for key in members}
        contacts = {group: set() for group in range(len(groups))}
        for key, neighbours in current.items():
            if key not in owners:
                continue
            first = owners[key]
            for other in neighbours:
                last = owners[other]
                if first != last:
                    contacts[first].add(last)
                    contacts[last].add(first)
        combined = []
        pending = set(contacts)
        while pending:
            first = min(pending)
            pending.remove(first)
            joined = {first}
            queue = deque([first])
            while queue:
                for other in contacts[queue.popleft()] & pending:
                    pending.remove(other)
                    joined.add(other)
                    queue.append(other)
            members = set().union(*(groups[group] for group in joined))
            overlap = Counter(index.membership[key] for key in members if key in index.membership)
            combined.append((members, overlap))
        old = {cid: index._drop_cluster(cid) for cid in affected}
        new_ids = set()
        combined.sort(key=lambda entry: (-len(entry[0]), min(entry[0])))
        for members, overlap in combined:
            available = sorted((cid for cid in overlap if cid not in new_ids),
                               key=lambda cid: (-overlap[cid], cid))
            if available:
                cid = available[0]
            else:
                cid = index.next_id
                index.next_id += SLICE_ID_STRIDE
            new_ids.add(cid)
            index._refresh(cid, members, old.get(cid))
        self._update_fragments(affected, new_ids)
        self.work.update(affected_clusters=len(affected), updated_clusters=len(combined))
        return self.work
