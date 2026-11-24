"""Moving primary-LiDAR queries never manufacture or forget support nodes."""
import math
import unittest
from unittest.mock import patch

from legbot_exploration.fixed_layer_map import MapSettings
from legbot_exploration.interval_map import IntervalLayerMap


def mapping(native=False):
    return IntervalLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.,
        traversability_kernel_size=3,inflation_kernel_size=1),native_updates=native)


def room(heights=(0.,2.), missing=()):
    return [(x*.1,y*.1,z) for x in range(-3,4) for y in range(-3,4)
            if (x,y) not in missing for z in heights]


class RadarHeightFrontiersTest(unittest.TestCase):
    def test_shared_topology_and_column_queries_are_not_duplicated_per_cut(self):
        m=mapping();m.integrate(room(missing={(0,0)}));m.set_frontier_query_height(.6);m.build()
        active=[i for i in m.frontier_index.slices.values() if i.nodes]
        self.assertEqual(len(active),1)
        self.assertEqual({key[0] for key in m.frontier_index.nodes},{m.radar_frontier_layer})
        m._radar_known_cache.pop((0,1),None)
        with patch.object(m,'observed_at_height',wraps=m.observed_at_height) as query:
            for k in range(20):self.assertTrue(m.frontier_cell(k,(0,1)) is not None)
            self.assertEqual(query.call_count,1)
        low=m.frontier_index.query(selection_layer=1,evaluate_observers=False)
        high=m.frontier_index.query(selection_layer=7,evaluate_observers=False)
        self.assertEqual([c['id'] for c in low['clusters']],[c['id'] for c in high['clusters']])

    def test_upper_room_update_preserves_lower_node_and_planning_dependency(self):
        m=mapping();m.integrate(room((0.,1.5,3.,4.5)));m.set_frontier_query_height(.6);m.build()
        low=m._interval_nodes[0,0][1];low_cell=m.cell((1,0,0)).copy()
        version=m.planning_block_versions[1,0,0]
        m.integrate([(0.,0.,3.1)]);m.build()
        self.assertIs(m._interval_nodes[0,0][1],low)
        self.assertEqual(m.cell((1,0,0)).tobytes(),low_cell.tobytes())
        self.assertEqual(m.planning_block_versions[1,0,0],version)
        self.assertAlmostEqual(m._interval_nodes[0,0][13].ground,3.1,places=6)

    def test_exact_height_is_shared_by_all_cuts_without_allocating_cells(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=mapping(native);m.integrate(room((0.,1.03)));m.build()
                before=len(m.blocks),m.surface_records,m._interval_node_count
                m.set_frontier_query_height(.917);m.build()
                for k in (-3,1,4,20):
                    row=m.frontier_cell(k,(0,0))
                    self.assertAlmostEqual(float(row['G']),0.)
                    self.assertAlmostEqual(float(row['C']),1.03,places=6)
                m.set_frontier_query_height(1.04);m.build()
                self.assertIsNotNone(m.observed_cell((1,0,0)),'old node must remain valid')
                for k in (1,4,20):self.assertIsNone(m.frontier_cell(k,(0,0)))
                self.assertEqual(before,(len(m.blocks),m.surface_records,m._interval_node_count))

    def test_height_crossing_creates_unknown_boundary_and_lowering_clears_it(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=mapping(native)
                points=[(x*.1,y*.1,z) for x in range(-3,4) for y in range(-3,4)
                        for z in ((0.,.8) if (x,y)==(0,0) else (1.,2.5))]
                m.integrate(points);m.set_frontier_query_height(.4);m.build()
                self.assertTrue(m.frontier_observed(5,(0,0)))
                before=(len(m.blocks),m.surface_records,m._interval_node_count)
                m.set_frontier_query_height(1.4);m.build()
                self.assertFalse(m.frontier_observed(1,(0,0)))
                self.assertIsNotNone(m.observed_cell((1,0,0)))
                keys=[k for k in m.frontier_index.nodes if k[1:3]==(0,0)]
                self.assertTrue(keys,'new-height unknown must acquire a frontier')
                self.assertTrue(all(m.frontier_index.nodes[k].height==1. for k in keys))
                m.set_frontier_query_height(.4);m.build()
                self.assertFalse(any(k[1:3]==(0,0) for k in m.frontier_index.nodes))
                self.assertEqual(before,(len(m.blocks),m.surface_records,m._interval_node_count))

    def test_reuses_both_real_treads_without_new_ground_hits(self):
        m=mapping()
        m.integrate([(x*.1,y*.1,z) for x in range(-3,4) for y in range(-3,4)
                     for z in ((.12,2.) if x>0 else (0.,2.))])
        m.set_frontier_query_height(.7);m.build()
        count=m.accepted_frames
        m.set_frontier_query_height(.82);m.build()
        self.assertEqual(count,m.accepted_frames)
        self.assertAlmostEqual(float(m.frontier_cell(1,(-1,0))['G']),0.)
        self.assertAlmostEqual(float(m.frontier_cell(20,(1,0))['G']),.12,places=6)
        self.assertFalse(any(k[1:3] in ((-1,0),(1,0)) for k in m.frontier_index.nodes))

    def test_open_query_is_bounded_by_actual_clearance_and_strict_endpoints(self):
        m=mapping()
        m.integrate([(0.,0.,0.)],sensor_origin=(0.,0.,.9),body_height=.32)
        m.set_frontier_query_height(.8);m.build()
        node=next(iter(m._interval_nodes[0,0].values()))
        self.assertTrue(math.isnan(node.ceiling))
        self.assertIsNotNone(m.frontier_cell(10,(0,0)))
        m.set_frontier_query_height(node.clear_to);m.build()
        self.assertIsNone(m.frontier_cell(1,(0,0)))
        self.assertTrue(math.isnan(m.cell((1,0,0))['C']))

    def test_real_hit_clears_frontiers_on_all_cut_aliases_in_same_commit(self):
        m=mapping(True);m.integrate(room(missing={(0,0)}))
        m.set_frontier_query_height(.6);m.build()
        self.assertTrue(any(k[1:3]==(0,0) for k in m.frontier_index.nodes))
        before=len(m.blocks)
        self.assertTrue(m.cell((1,0,0))['unknown'],'allocated placeholder remains unknown')
        m.integrate([(0.,0.,0.),(0.,0.,2.)]);m.build()
        self.assertFalse(any(k[1:3]==(0,0) for k in m.frontier_index.nodes))
        self.assertEqual(before,len(m.blocks))

    def test_pending_surface_and_empty_allocated_cell_are_not_known(self):
        m=mapping();m.integrate(room(missing={(0,0)}))
        m.integrate([(0.,0.,2.82),(0.,0.,4.35)],sensor_origin=(0.,0.,2.),body_height=1.78)
        m.set_frontier_query_height(3.3);m.build()
        self.assertIsNone(m.frontier_cell(1,(0,0)))
        self.assertFalse(m._interval_nodes[0,0])
        self.assertTrue(m._pending_ground[0,0])

    def test_wall_and_its_eight_neighbors_cannot_be_frontiers(self):
        m=mapping(True);m.integrate(room(missing={(0,0),(1,0)}))
        m.integrate([(0.,0.,i*.1) for i in range(21)])
        m.set_frontier_query_height(.83);m.build()
        self.assertTrue(m.frontier_cell(20,(0,0))['solid'])
        self.assertFalse(any(k[1:3] in ((0,0),(1,0)) for k in m.frontier_index.nodes))

    def test_no_endpoint_crossing_does_not_scan_map_or_restart_clusters(self):
        m=mapping();m.integrate(room(missing={(0,0)}))
        m.set_frontier_query_height(.6);m.build()
        revision=m.frontier_index.revision;ids=set(m.frontier_index.clusters)
        with patch.object(m,'iter_cells',side_effect=AssertionError('whole map scan')):
            m.set_frontier_query_height(.601);m.build()
        self.assertEqual(m.stats['frontier_height_changed_columns'],0)
        self.assertEqual(m.frontier_index.revision,revision)
        self.assertEqual(ids,set(m.frontier_index.clusters))
        self.assertAlmostEqual(m.frontier_query_height,.601)


if __name__=='__main__':unittest.main()
