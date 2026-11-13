"""The borrowed-map native A* must match its original dense-map search."""
import unittest
import numpy as np
from test_online_pct import native_wrapper


class NativeAccessorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        native_wrapper()
        from lib import a_star
        cls.astar = a_star.Astar

    def test_obstacle_detour_matches_dense_native_result(self):
        n = 31
        cost = np.ones((n, n), dtype=float)
        cost[:25, 15] = 50
        height = np.zeros_like(cost)
        dense, sparse = self.astar(), self.astar()
        dense.init(20., 1, .1, 1., cost, height, height)
        sparse.init_accessor(20., 1, n, n, .1, 1., lambda k, x, y: (float(cost[x, y]), 0., 0.))
        for start, target in (([0, 2, 2], [0, 28, 28]), ([0, 28, 28], [0, 2, 2])):
            a, b = np.asarray(start, dtype='int32'), np.asarray(target, dtype='int32')
            self.assertEqual(dense.search(a, b), sparse.search(a, b))
            np.testing.assert_array_equal(dense.get_result_matrix(), sparse.get_result_matrix())
            self.assertLessEqual(sparse.get_search_node_count(), n*n)

    def test_large_sparse_extent_does_not_allocate_the_bounding_box(self):
        search = self.astar()
        search.init_accessor(20., 1, 1000000, 1000000, .1, 1.,
            lambda k, x, y: (0. if 1 <= x <= 5 and 1 <= y <= 5 else 50., 0., 0.))
        self.assertTrue(search.search(np.asarray([0, 1, 1], dtype='int32'), np.asarray([0, 5, 5], dtype='int32')))
        self.assertLess(search.get_search_node_count(), 100)

    def test_timeout_has_no_partial_route_and_unlimited_retry_still_works(self):
        search=self.astar()
        search.init_accessor(20.,1,31,31,.1,1.,lambda k,x,y:(0.,0.,0.))
        a,b=np.asarray([0,1,1],dtype='int32'),np.asarray([0,29,29],dtype='int32')
        search.set_time_limit(1e-9)
        self.assertFalse(search.search(a,b));self.assertTrue(search.timed_out())
        self.assertEqual(search.get_result_matrix().size,0)
        search.set_time_limit(0.)
        self.assertTrue(search.search(a,b));self.assertFalse(search.timed_out())

    def test_projection_rejects_island_without_allocating_native_nodes(self):
        from test_pct_connectivity import tile_fixture,set_cell
        from legbot_exploration.block_pct import BlockPlanner
        m=tile_fixture();keys={(0,x,0) for x in range(-5,6)}
        for key in keys:set_cell(m,key)
        set_cell(m,(0,0,0),False);m.connectivity.update(keys)
        planner=BlockPlanner(m,native_wrapper())
        self.assertIsNone(planner.plan((-.4,0.,0.),(.4,0.,0.)))
        self.assertEqual(planner.search_nodes,0)
        self.assertEqual(planner.search_diagnostics['prefilter'],'disconnected_xy_projection')

    def test_native_failure_cache_survives_unrelated_changes_but_not_a_new_bridge(self):
        from test_pct_connectivity import tile_fixture,set_cell
        from legbot_exploration.block_pct import BlockPlanner
        m=tile_fixture()
        for x in range(-5,0):set_cell(m,(0,x,0),height=0.)
        for x in range(0,6):set_cell(m,(1,x,0),height=1.)
        m.connectivity.update({(k,x,0) for k,x in [(0,x) for x in range(-5,0)]+[(1,x) for x in range(0,6)]})
        planner=BlockPlanner(m,native_wrapper())
        start,goal=(-.4,0.,0.),(.4,0.,1.)
        self.assertIsNone(planner.plan(start,goal))
        self.assertTrue(planner.search_diagnostics['attempts'])
        m.planning_block_versions[(1,20,20)]=99
        self.assertIsNone(planner.plan(start,goal))
        self.assertTrue(planner.search_diagnostics['cached_disconnection'])
        set_cell(m,(0,0,0),height=1.)
        m.cell((0,0,0))['inflated_terrain_cost']=2. # Native DecideLayer changes only for strictly lower cost.
        m.planning_block_versions[(0,0,0)]=1;m.connectivity.update({(0,0,0)})
        self.assertIsNotNone(planner.plan(start,goal))

    def test_unobserved_strip_blocks_native_astar_until_real_ground_arrives(self):
        from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
        from legbot_exploration.block_pct import BlockPlanner
        mapping=FixedLayerMap(MapSettings(maturity_min_points=3,robot_radius=0.,safety_margin=0.))
        points=[(x*.1,y*.1,0.) for x in range(-20,21) for y in range(-10,11) if x!=0]
        mapping.integrate(np.asarray(points*3,dtype='float32'));mapping.build()
        planner=BlockPlanner(mapping,native_wrapper())
        self.assertIsNone(planner.plan((-1.5,0.,0.),(1.5,0.,0.)))
        # The same borrowed map becomes connected only after actual returns.
        mapping.integrate(np.asarray([(0.,y*.1,0.) for y in range(-10,11)],dtype='float32'))
        mapping.build()
        self.assertFalse(mapping.cell((0,0,0))['g_mature'])
        self.assertIsNotNone(planner.plan((-1.5,0.,0.),(1.5,0.,0.)))

    def test_late_mature_ground_uses_existing_cut_for_admission_and_native_route(self):
        from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
        from legbot_exploration.block_pct import BlockPlanner
        mapping=FixedLayerMap(MapSettings(maturity_min_points=3,robot_radius=.35,safety_margin=.05))
        points=[(x*.1,y*.1,.01) for x in range(-12,13) for y in range(-12,13) if x or y]
        mapping.integrate(np.asarray(points*3,dtype='float32'));mapping.build()
        self.assertFalse(mapping.point_safe_check([.3,0.,.33])['ready'])
        mapping.integrate(np.asarray([[0.,0.,-.01]]*3,dtype='float32'));mapping.build()
        self.assertTrue(mapping.point_safe_check([.3,0.,.33])['ready'])
        planner=BlockPlanner(mapping,native_wrapper())
        route=planner.plan((-.7,0.,.01),(.3,0.,.01))
        self.assertIsNotNone(route)
        self.assertTrue(planner.search_diagnostics['attempts'])


if __name__ == '__main__':
    unittest.main()
