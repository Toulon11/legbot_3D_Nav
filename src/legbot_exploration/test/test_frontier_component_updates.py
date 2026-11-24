"""Batch connectivity and physical aliases against independent graph walks."""
from collections import deque
import unittest

from legbot_exploration.fixed_layer_map import FixedLayerMap, MapSettings
from legbot_exploration.pct_frontier_index import Boundary,FloorAttempt


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

    def apply(self, changes, layer=None, identity=1):
        layer = self.layer if layer is None else layer
        layer.revision += 1
        layer.changed_clusters = set()
        layer.stale_observers = set()
        replacements = {}
        for (x, y), height in changes.items():
            key = layer.layer, x, y, (100, 100, 1)
            replacements[x, y] = {} if height is None else {
                key: Boundary(height, ((100, 100, height, identity),))}
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

    def test_view_height_change_does_not_split_eight_connected_members(self):
        self.apply({(x, 0): 0. for x in range(5)})
        self.apply({(2, 0): .4})
        self.assertEqual(len(self.layer.clusters), 1)

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

    def test_slice_clusters_stay_independent_when_duplicate_contacts_change(self):
        other = self.index.add_slice(1)
        self.apply({(x, 0): 0. for x in range(5)})
        self.apply({(x, 0): 0. for x in range(5)}, other)
        self.assertEqual(len(self.index.physical_tasks()[0]), 2)
        self.apply({(2, 0): None})
        self.assertEqual(len(self.index.physical_tasks()[0]), 3)
        self.apply({(2, 0): None}, other)
        self.assertEqual(len(self.index.physical_tasks()[0]), 4)
        self.apply({(2, 0): 0.})
        self.assertEqual(len(self.index.physical_tasks()[0]), 3)

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

    def test_same_ground_adjacent_edges_are_alternatives_across_missing_cuts(self):
        upper=self.index.add_slice(4)
        self.apply({(0,0):0.})
        self.apply({(1,0):0.},upper)
        tasks=self.index.physical_tasks()[0]
        self.assertEqual(len(tasks),2)
        self.assertTrue(all(task['cells']==1 for task in tasks.values()))
        self.assertEqual(len(set(self.index.duplicate_groups.values())),1)
        self.apply({(10,0):0.},upper)
        self.assertEqual(len(self.index.physical_tasks()[0]),3)
        self.apply({(1,0):.13},upper)
        self.assertEqual(len(self.index.physical_tasks()[0]),3)

    def test_slice_alternatives_retain_members_and_share_physical_failure_history(self):
        upper=self.index.add_slice(4)
        self.apply({(0,0):0.,(1,0):0.})
        self.apply({(0,0):0.,(1,0):0.},upper)
        tasks=self.index.physical_tasks()[0]
        self.assertEqual(len(tasks),2)
        tid,task=next(iter(tasks.items()))
        self.assertEqual(task['cells'],2)
        self.assertEqual(len(task['members']),2)
        attempt=FloorAttempt(tid,(0.,.08),{k:self.index.nodes[k] for k in task['members']},set(task['members']))
        self.assertEqual(self.index.settle_floor_attempt(attempt,True),'first_no_progress')
        newest=self.index.add_slice(8)
        self.apply({(0,0):0.,(1,0):0.},newest)
        tid=next(iter(newest.clusters));task=self.index.physical_tasks()[0][tid]
        self.assertEqual(task['cells'],2)
        self.assertEqual(len(task['members']),2)
        self.assertEqual(len(self.index.physical_tasks()[0]),3)
        self.assertTrue(all(self.index._failure_count((0.,.08),k)==1 for k in task['members']))
        self.index.floor_query_members[tid]=set(task['members'])
        self.index.floor_query=((0.,.08),lambda key:True,False,1,2,False)
        second=self.index.begin_floor_attempt(next(iter(newest.clusters)))
        self.assertEqual(second.cluster,tid)
        self.assertEqual(second.failures,1)
        self.assertEqual(self.index.settle_floor_attempt(second,True),'blacklisted')
        self.assertTrue(all(self.index._floor_records(self.index.floor_blacklist,(0.,.08),k)
                            for k in task['members']))

    def test_same_height_different_source_identities_remain_separate(self):
        upper=self.index.add_slice(4)
        self.apply({(0,0):0.})
        self.apply({(0,0):0.},upper,identity=2)
        self.assertEqual(len(self.index.physical_tasks()[0]),2)

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
            # The sparse map also aliases adjacent XY across any cut gap
            # when their real source is the same. This fixture gives every
            # member the shared (100,100,1) source; include those edges in the
            # independent full graph, not just coincident-XY aliases.
            for key,node in self.index.nodes.items():
                for dx,dy in ((1,0),(0,1),(1,1),(1,-1)):
                    for other in self.index.by_column.get((key[1]+dx,key[2]+dy),()):
                        if (key[0]!=other[0] and
                                abs(node.height-self.index.nodes[other].height)<=.05+1e-7):
                            first,last=self.index.membership[key],self.index.membership[other]
                            if first!=last:
                                graph[first].add(last)
                                graph[last].add(first)
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
            self.assertEqual(expected, set(self.index.duplicate_groups.values()))
            self.assertTrue(all(group=={cid} for cid,group in aliases.items()))
            self.assertEqual(set(owners), set(self.index.clusters))
            for tid, task in tasks.items():
                self.assertEqual(task['members'], set().union(*(
                    self.index.clusters[cid]['members'] for cid in aliases[tid])))
                self.assertTrue(all(tid in spatial[tile] for tile in task['tiles']))

    def test_query_selects_one_best_cut_without_unioning_cluster_members(self):
        upper=self.index.add_slice(4)
        self.apply({(x,0):0. for x in range(5)})
        self.apply({(x,0):0. for x in range(3)},upper)
        tasks=self.index.physical_tasks()[0]
        self.assertEqual(sorted(task['cells'] for task in tasks.values()),[3,5])
        result=self.index.query(evaluate_observers=False)
        self.assertEqual(len(result['clusters']),1)
        self.assertEqual(result['clusters'][0]['cells'],5)
        winner=tasks[result['clusters'][0]['id']]
        self.assertEqual(len({key[0] for key in winner['members']}),1)


if __name__ == '__main__':
    unittest.main()
