"""Batch connectivity and physical aliases against independent graph walks."""
from collections import deque
import unittest

from legbot_exploration.fixed_layer_map import FixedLayerMap, MapSettings
from legbot_exploration.pct_frontier_index import Boundary


class NoFullIteration(dict):
    def __iter__(self):
        raise AssertionError('full index iteration')

    def values(self):
        raise AssertionError('full index values')

    def items(self):
        raise AssertionError('full index items')


class FrontierComponentUpdatesTest(unittest.TestCase):
    def setUp(self):
        self.mapping = FixedLayerMap(MapSettings())
        self.index = self.mapping.frontier_index
        self.layer = self.index.add_slice(0)

    def apply(self, changes, layer=None):
        layer = self.layer if layer is None else layer
        layer.revision += 1
        layer.changed_clusters = set()
        layer.stale_observers = set()
        replacements = {}
        for (x, y), height in changes.items():
            key = layer.layer, x, y, (100, 100, 1)
            replacements[x, y] = {} if height is None else {
                key: Boundary(height, ((100, 100, height, 1),))}
        work, changed, _, _ = layer._replace_boundaries(replacements)
        layer.last_work.update(work)
        self.index.commit([(layer, (changed, set(), set()))])
        self.assert_graph(layer)
        return work

    def assert_graph(self, layer):
        remaining = set(layer.nodes)
        expected = set()
        while remaining:
            first = remaining.pop()
            found = {first}
            queue = deque([first])
            while queue:
                for key in layer.neighbours(queue.popleft()):
                    if key in remaining:
                        remaining.remove(key)
                        found.add(key)
                        queue.append(key)
            expected.add(frozenset(found))
        self.assertEqual(expected, {frozenset(c['members']) for c in layer.clusters.values()})
        self.assertEqual(set(layer.nodes), set(layer.membership))
        self.assertEqual(set(layer.nodes), set(layer.components.node_fragment))
        for cid, cluster in layer.clusters.items():
            self.assertEqual(cluster['members'], set().union(*(
                layer.components.fragments[fid] for fid in layer.components.cluster_fragments[cid])))
            self.assertTrue(all(layer.membership[key] == cid for key in cluster['members']))

    def test_growth_keeps_far_spatial_bucket_and_skips_old_edges(self):
        self.apply({(x, 0): 0. for x in range(100)})
        far = next(key for key in self.layer.nodes if key[1] == 0)
        fragment = self.layer.components.node_fragment[far]
        cluster = self.layer.membership[far]
        work = self.apply({(100, 0): 0.})
        self.assertEqual(work['cluster_rebuilds'], 0)
        self.assertEqual(work['component_nodes_visited'], 1)
        self.assertEqual(self.layer.components.node_fragment[far], fragment)
        self.assertEqual(self.layer.membership[far], cluster)

    def test_new_bridge_merges_old_clusters(self):
        self.apply({(0, 0): 0., (2, 0): 0.})
        self.assertEqual(len(self.layer.clusters), 2)
        work = self.apply({(1, 0): 0.})
        self.assertEqual(len(self.layer.clusters), 1)
        self.assertEqual(work['cluster_rebuilds'], 0)

    def test_interior_removal_uses_local_connection(self):
        self.apply({(x, y): 0. for x in range(80) for y in range(-2, 3)})
        work = self.apply({(40, 0): None})
        self.assertEqual(work['local_connectivity_preserved'], 1)
        self.assertEqual(work['cluster_rebuilds'], 0)
        self.assertLess(work['component_nodes_visited'], 40)

    def test_batch_removes_both_bridges_and_splits(self):
        self.apply({(x, y): 0. for x in range(9) for y in (0, 1)})
        work = self.apply({(4, 0): None, (4, 1): None})
        self.assertEqual(len(self.layer.clusters), 2)
        self.assertEqual(work['cluster_rebuilds'], 1)

    def test_added_detour_reconnects_removed_bridge_in_one_batch(self):
        self.apply({(x, 0): 0. for x in range(30)})
        work = self.apply({(15, 0): None, (15, 1): 0.})
        self.assertEqual(len(self.layer.clusters), 1)
        self.assertEqual(work['local_connectivity_preserved'], 1)
        self.assertEqual(work['cluster_rebuilds'], 0)

    def test_height_change_checks_lost_connections(self):
        self.apply({(x, 0): 0. for x in range(5)})
        self.apply({(2, 0): .4})
        self.assertEqual(len(self.layer.clusters), 3)

    def test_tail_and_complete_removal(self):
        self.apply({(x, 0): 0. for x in range(5)})
        work = self.apply({(4, 0): None})
        self.assertEqual(work['cluster_rebuilds'], 0)
        self.apply({(x, 0): None for x in range(4)})
        self.assertFalse(self.layer.clusters)
        self.assertFalse(self.layer.components.fragments)
        self.assertFalse(self.index.physical_tasks()[0])

    def test_unchanged_geometry_keeps_bucket_identity(self):
        self.apply({(0, 0): 0., (1, 0): 0.})
        before = dict(self.layer.components.node_fragment)
        work = self.apply({(0, 0): .01})
        self.assertEqual(work['affected_clusters'], 0)
        self.assertEqual(before, self.layer.components.node_fragment)

    def test_physical_aliases_split_and_merge_by_changed_columns(self):
        other = self.index.add_slice(1)
        self.apply({(x, 0): 0. for x in range(5)})
        self.apply({(x, 0): 0. for x in range(5)}, other)
        self.assertEqual(len(self.index.physical_tasks()[0]), 1)
        self.apply({(2, 0): None})
        self.assertEqual(len(self.index.physical_tasks()[0]), 1)
        self.apply({(2, 0): None}, other)
        self.assertEqual(len(self.index.physical_tasks()[0]), 2)
        self.apply({(2, 0): 0.})
        self.assertEqual(len(self.index.physical_tasks()[0]), 1)

    def test_remote_task_survives_without_full_index_iteration(self):
        self.apply({(0, 0): 0., (1000, 0): 0.})
        tasks = self.index.physical_tasks()[0]
        far = next(cid for cid, cluster in tasks.items() if cluster['frontier'][0] > 50.)
        old = tasks[far]
        self.index.by_column = NoFullIteration(self.index.by_column)
        self.index.clusters = NoFullIteration(self.index.clusters)
        self.apply({(1, 0): 0.})
        self.assertIs(self.index.physical_tasks()[0][far], old)
        self.assertLess(self.index.last_work['physical_columns_checked'], 5)

    def test_random_batch_updates_match_independent_connectivity(self):
        import random
        rng = random.Random(419)
        for _ in range(35):
            changes = {(rng.randrange(12), rng.randrange(5)): rng.choice((None, 0., .1, .4))
                       for _ in range(8)}
            self.apply(changes)

    def test_random_cut_updates_match_full_alias_graph(self):
        import random
        rng = random.Random(623)
        layers = [self.layer, self.index.add_slice(1), self.index.add_slice(2)]
        for _ in range(45):
            changes = {(rng.randrange(8), rng.randrange(4)): rng.choice((None, 0., .1, .4))
                       for _ in range(5)}
            self.apply(changes, rng.choice(layers))
            graph = {cid: set() for cid in self.index.clusters}
            for keys in self.index.by_column.values():
                for _, matches in self.index.physical_targets(keys):
                    aliases = {self.index.membership[key] for key in matches}
                    for cid in aliases:
                        graph[cid].update(aliases - {cid})
            remaining = set(graph)
            expected = set()
            while remaining:
                first = remaining.pop()
                connected = {first}
                pending = deque([first])
                while pending:
                    for other in graph[pending.popleft()] & remaining:
                        remaining.remove(other)
                        connected.add(other)
                        pending.append(other)
                expected.add(frozenset(connected))
            tasks, spatial, owners, aliases = self.index.physical_tasks()
            self.assertEqual(expected, {frozenset(group) for group in aliases.values()})
            self.assertEqual(set(owners), set(self.index.clusters))
            for tid, task in tasks.items():
                self.assertEqual(task['members'], set().union(*(
                    self.index.clusters[cid]['members'] for cid in aliases[tid])))
                self.assertTrue(all(tid in spatial[tile] for tile in task['tiles']))


if __name__ == '__main__':
    unittest.main()
