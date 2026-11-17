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

    def test_sparse_layer_switch_requires_the_same_ground_source(self):
        start = np.array([0, 0, 0], dtype=np.int32)
        goal = np.array([1, 2, 0], dtype=np.int32)
        for alias_id, alias_height, expect_route in ((3, 1.45, True), (4, 1.48, False)):
            with self.subTest(alias_id=alias_id):
                def read(layer, row, column):
                    if column == 0:
                        return (3., 1.45, 0., 3) if layer == 0 else (0., alias_height, 0., alias_id)
                    if layer == 1:
                        return (0., 1.58, 0., 1)
                    return (1e6, -100., 0., 0)

                search = self.astar()
                search.init_surface_accessor(20., 2, 1, 3, .1, 1., read)
                self.assertEqual(search.search(start, goal), expect_route)
                if not expect_route:
                    # Both heights are in the same native height bin. The
                    # lower tread must still not count as reaching the upper.
                    self.assertFalse(search.search(start, np.array([1, 0, 0], dtype=np.int32)))

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

    def test_new_floor_rejects_old_ground_until_the_same_surface_is_refreshed(self):
        from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
        from legbot_exploration.block_pct import BlockPlanner
        mapping=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.))
        points=np.asarray([(x*.1,y*.1,1.42) for x in range(-8,9)
                           for y in range(-4,5)],dtype='float32')
        mapping.integrate([(0.,y*.1,0.) for y in range(-4,5)])
        mapping.integrate(points);mapping.build()
        evidence=mapping.surface_observation
        evidence.begin_crossing()
        mapping.integrate(points[points[:,0]!=0])
        evidence.enter_frontier_floor((-.6,0.,1.42),.08);mapping.build()
        start,goal,band=(-.6,0.,1.42),(.6,0.,1.42),(1.42,.08)
        planner=BlockPlanner(mapping,native_wrapper())
        self.assertFalse(mapping.floor_connectivity(band).possible((-6,0),(6,0)))
        self.assertIsNone(planner.plan(start,goal,floor=band))
        self.assertIsNone(planner.plan(start,goal))
        self.assertFalse(mapping.point_safe_check((0.,0.,1.42),0.,floor=band)['ready'])

        # A new return in the same XY column at a different height changes C
        # but leaves the frame of the selected ground intact.
        strip=points[points[:,0]==0]
        mapping.integrate(strip+[0.,0.,1.]);mapping.build()
        reset=mapping.cell((3,0,0))
        self.assertTrue(reset['unknown'] and np.isnan(reset['G']) and np.isnan(reset['C']))
        self.assertTrue(any(abs(r.height-1.42)<1e-5 for r in mapping.columns[(0,0)].values()))
        self.assertIsNone(planner.plan(start,goal,floor=band))
        mapping.integrate(strip);mapping.build()
        self.assertTrue(mapping.floor_connectivity(band).possible((-6,0),(6,0)))
        self.assertIsNotNone(planner.plan(start,goal,floor=band))
        self.assertIsNotNone(planner.plan(start,goal))

    def test_after_arrival_a_lower_ground_hit_resolves_the_new_platform_cell(self):
        from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
        from legbot_exploration.block_pct import BlockPlanner
        mapping=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.))
        mapping.integrate(np.asarray([(.1*x,0.,1.45 if x==0 else 1.58)
                                      for x in range(-3,4)],dtype='float32'))
        mapping.build()
        evidence=mapping.surface_observation
        planner=BlockPlanner(mapping,native_wrapper())
        layer=4
        lower=mapping.cell((3,0,0))
        upper=mapping.cell((layer,0,0))
        self.assertAlmostEqual(lower['G'],1.45,places=5)
        self.assertAlmostEqual(upper['G'],1.45,places=5)
        self.assertEqual(lower['g_surface_id'],upper['g_surface_id'])
        self.assertNotEqual(int(lower['g_surface_id']),0)
        index=(layer-mapping.first_layer,-int(mapping.lower[0]),-int(mapping.lower[1]))
        self.assertAlmostEqual(planner._read(*index)[1],1.45,places=5)
        evidence.begin_crossing()
        mapping.integrate([(-.2,0.,1.58),(.2,0.,1.58),(0.,0.,1.45)])
        evidence.enter_frontier_floor((-.2,0.,1.58),.08)
        mapping.build()
        self.assertIsNotNone(evidence.floor_ground_band(1.58))
        self.assertIsNone(evidence.floor_ground_band(1.45))
        self.assertAlmostEqual(mapping.cell((layer,2,0))['G'],1.58,places=5)
        self.assertEqual(planner._read(*index),(1e6,-100.,0.,0))
        reset=mapping.cell((layer,0,0))
        self.assertTrue(reset['unknown'])
        self.assertTrue(np.isnan(reset['G']) and np.isnan(reset['C']))
        self.assertEqual((reset['g_surface_id'],reset['c_surface_id'],reset['inflated_terrain_cost']),(0,0,-1.))
        self.assertAlmostEqual(mapping.cell((3,0,0))['G'],1.45,places=5)
        mapping.integrate([(0.,0.,1.45)]);mapping.build()
        self.assertFalse(mapping.cell((layer,0,0))['unknown'])
        self.assertAlmostEqual(planner._read(*index)[1],1.45,places=5)
        self.assertEqual(planner._read(*index,floor=(1.58,.08)),(1e6,-100.,0.,0))
        mapping.integrate([(0.,0.,1.58)]);mapping.build()
        self.assertAlmostEqual(planner._read(*index)[1],1.58,places=5)

    def test_platform_retains_existing_ground_where_a_lower_cut_has_no_ground(self):
        from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
        mapping=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.))
        evidence=mapping.surface_observation
        evidence.enter_frontier_floor((0.,0.,0.),.08)
        mapping.integrate([(0.,0.,1.58),(.1,0.,1.58),(.1,0.,0.)])
        mapping.build()
        self.assertTrue(np.isnan(mapping.cell((3,0,0))['G']))
        ground_id=int(mapping.cell((4,0,0))['g_surface_id'])
        evidence.begin_crossing()
        mapping.integrate([(.2,0.,1.58)])
        evidence.enter_frontier_floor((.2,0.,1.58),.08)
        mapping.build()

        def check_cells():
            original=mapping.cell((4,0,0))
            self.assertFalse(original['unknown'])
            self.assertAlmostEqual(original['G'],1.58,places=5)
            self.assertEqual(original['g_surface_id'],ground_id)
            self.assertTrue(mapping.ground_pending((1,0),4))
            self.assertTrue(np.isnan(mapping.cell((4,1,0))['G']))
            self.assertTrue(mapping.cell((4,3,0))['unknown'])

        check_cells()
        mapping.integrate([(0.,0.,0.)])
        mapping.build()
        mapping._reset_slice_products()
        mapping.build()
        check_cells()

    def test_new_floor_freshness_is_limited_to_its_ground_slices(self):
        from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
        from legbot_exploration.block_pct import BlockPlanner
        mapping=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.))
        old_floor,new_floor,upper_floor=1.46,3.02,4.37
        points=[(x*.1,y*.1,z) for x in range(-4,5) for y in range(-4,5)
                for z in (old_floor,upper_floor)]
        points.extend([(0.,0.,2.82),(.1,0.,new_floor)])
        mapping.integrate(points);mapping.build()
        evidence=mapping.surface_observation
        self.assertTrue(evidence.enter_frontier_floor((0.,0.,old_floor),.08))
        evidence.begin_crossing()
        mapping.integrate([(0.,0.,upper_floor)])
        self.assertTrue(evidence.enter_frontier_floor((0.,0.,upper_floor),.08))
        mapping.build()
        old_upper_band=evidence.floor_ground_band(upper_floor)
        evidence.begin_crossing()
        self.assertTrue(evidence.enter_frontier_floor((.1,0.,new_floor),.08))
        mapping.build()
        band=evidence.floor_ground_band(new_floor)
        self.assertEqual(band.first_slice,7)
        self.assertEqual(evidence.floor_ground_band(upper_floor),old_upper_band)
        planner=BlockPlanner(mapping,native_wrapper())
        def read(k):
            return planner._read(k-mapping.first_layer,-int(mapping.lower[0]),
                                 -int(mapping.lower[1]))
        self.assertAlmostEqual(read(5)[1],old_floor,places=5)
        self.assertAlmostEqual(read(6)[1],2.82,places=5)
        self.assertEqual(read(7),(1e6,-100.,0.,0))
        self.assertEqual(read(8),(1e6,-100.,0.,0))
        self.assertAlmostEqual(read(9)[1],upper_floor,places=5)
        self.assertAlmostEqual(mapping.cell((5,0,0))['C'],2.82,places=5)
        self.assertFalse(mapping.point_safe_check((.1,0.,new_floor),0.)['ready'],
                         'the upper alias with an old same-height G stays unknown')
        mapping._reset_slice_products()
        mapping.build()
        self.assertEqual(read(7),(1e6,-100.,0.,0),
                         'rebuilding geometry must not restore an unobserved G from history')
        self.assertAlmostEqual(read(5)[1],old_floor,places=5)
        mapping.integrate([(0.,0.,new_floor),(.1,0.,new_floor),(-.1,0.,new_floor)])
        mapping.build()
        self.assertAlmostEqual(read(7)[1],new_floor,places=5)
        self.assertAlmostEqual(mapping.cell((5,0,0))['G'],old_floor,places=5)
        self.assertAlmostEqual(mapping.cell((5,0,0))['C'],2.82,places=5)
        self.assertAlmostEqual(mapping.cell((5,-1,0))['C'],new_floor,places=5)
        # A newly observed column can supply a measured G to multiple cuts.
        # Floor-constrained planning still filters the lower G by height.
        mapping.integrate([(.5,0.,old_floor)])
        mapping.build()
        old=mapping.cell((5,5,0))
        observed=mapping.cell((7,5,0))
        self.assertAlmostEqual(old['G'],old_floor,places=5)
        self.assertFalse(observed['unknown'])
        self.assertAlmostEqual(observed['G'],old_floor,places=5)
        index=(7-mapping.first_layer,5-int(mapping.lower[0]),-int(mapping.lower[1]))
        self.assertEqual(planner._read(*index,floor=(new_floor,.08)),(1e6,-100.,0.,0))
        mapping.integrate([(.5,0.,new_floor)])
        mapping.build()
        self.assertAlmostEqual(mapping.cell((7,5,0))['G'],new_floor,places=5)

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
