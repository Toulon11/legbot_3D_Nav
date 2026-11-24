"""The borrowed-map native A* must match its original dense-map search."""
import unittest
import numpy as np
from test_online_pct import native_wrapper
from legbot_exploration.fixed_layer_map import MapSettings as Settings


def open_settings(**settings):
    # Native footprint/route fixtures use exposed ground; endpoint/open-sky
    # policy itself is tested independently by test_slice_intervals.
    defaults=dict(allow_open_ceiling=True,origin_z=1e-5)
    defaults.update(settings)
    return Settings(**defaults)



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
        for x in range(0,6):set_cell(m,(1,x,0),height=.25)
        m.connectivity.update({(k,x,0) for k,x in [(0,x) for x in range(-5,0)]+[(1,x) for x in range(0,6)]})
        planner=BlockPlanner(m,native_wrapper())
        start,goal=(-.4,0.,0.),(.4,0.,.25)
        self.assertIsNone(planner.plan(start,goal))
        self.assertTrue(planner.search_diagnostics['attempts'])
        m.planning_block_versions[(1,20,20)]=99
        self.assertIsNone(planner.plan(start,goal))
        self.assertTrue(planner.search_diagnostics['cached_disconnection'])
        set_cell(m,(0,0,0),height=.12)
        m.cell((0,0,0))['inflated_terrain_cost']=2.
        m.planning_block_versions[(0,0,0)]=1;m.connectivity.update({(0,0,0)})
        self.assertIsNotNone(planner.plan(start,goal))

    def test_unobserved_strip_blocks_native_astar_until_real_ground_arrives(self):
        from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
        from legbot_exploration.block_pct import BlockPlanner
        mapping=FixedLayerMap(open_settings(maturity_min_points=3,robot_radius=0.,safety_margin=0.))
        points=[(x*.1,y*.1,0.) for x in range(-20,21) for y in range(-10,11) if x!=0]
        mapping.integrate(np.asarray(points*3,dtype='float32'));mapping.build()
        planner=BlockPlanner(mapping,native_wrapper())
        self.assertIsNone(planner.plan((-1.5,0.,0.),(1.5,0.,0.)))
        # The same borrowed map becomes connected only after actual returns.
        mapping.integrate(np.asarray([(0.,y*.1,0.) for y in range(-10,11)],dtype='float32'))
        mapping.build()
        self.assertFalse(mapping.cell((0,0,0))['g_mature'])
        self.assertIsNotNone(planner.plan((-1.5,0.,0.),(1.5,0.,0.)))

    def test_unobserved_upper_work_cut_blocks_native_route_until_real_ground_arrives(self):
        from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
        from legbot_exploration.block_pct import BlockPlanner
        mapping=FixedLayerMap(open_settings(maturity_min_points=1,robot_radius=0.,safety_margin=0.))
        points=np.asarray([(x*.1,y*.1,1.42) for x in range(-8,9)
                           for y in range(-4,5)],dtype='float32')
        lower=[(0.,y*.1,0.) for y in range(-4,5)]
        mapping.integrate(lower);mapping.build()
        evidence=mapping.surface_observation
        mapping.integrate(points[points[:,0]!=0])
        evidence.enter_frontier_floor((-.6,0.,1.42),.08);mapping.build()
        start,goal,band=(-.6,0.,1.42),(.6,0.,1.42),(1.42,.08)
        planner=BlockPlanner(mapping,native_wrapper())
        self.assertFalse(mapping.floor_connectivity(band).possible((-6,0),(6,0)))
        self.assertIsNone(planner.plan(start,goal,floor=band))
        self.assertIsNone(planner.plan(start,goal))
        self.assertFalse(mapping.point_safe_check((0.,0.,1.42),0.,floor=band)['ready'])

        mapping.integrate(lower);mapping.build()
        upper=mapping.cell((3,0,0))
        self.assertTrue(upper is None or (upper['unknown'] and not upper['measured']))
        self.assertEqual(mapping.cell((0,0,0))['G'],0.)
        self.assertIsNone(planner.plan(start,goal,floor=band))
        mapping.integrate(points[points[:,0]==0]);mapping.build()
        self.assertTrue(mapping.floor_connectivity(band).possible((-6,0),(6,0)))
        self.assertIsNotNone(planner.plan(start,goal,floor=band))
        self.assertIsNotNone(planner.plan(start,goal))

    def test_platform_arrival_preserves_the_measured_tread_across_cut_boundary(self):
        from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
        from legbot_exploration.block_pct import BlockPlanner
        mapping=FixedLayerMap(open_settings(maturity_min_points=1,robot_radius=.15,safety_margin=.05))
        points=[(x*.1,y*.1,1.45 if x<0 else 1.58)
                for x in range(-14,15) for y in range(-6,7)]
        mapping.integrate(points);mapping.build()
        evidence=mapping.surface_observation
        evidence.enter_frontier_floor((-.6,0.,1.45),.08)
        evidence.enter_frontier_floor((.6,0.,1.58),.08);mapping.build()
        lower=mapping.cell((3,-1,0))
        upper=mapping.cell((4,-1,0))
        self.assertAlmostEqual(lower['G'],1.45,places=5)
        self.assertTrue(mapping.frontier_observed(4,(-1,0)),'confirmed tread boundary')
        self.assertAlmostEqual(mapping.footprint_support(4,-1,0,1.58)['G'],1.45,places=5)
        self.assertTrue(upper is None or not upper['measured'])
        planner=BlockPlanner(mapping,native_wrapper())
        self.assertIsNotNone(planner.plan((-.6,0.,1.45),(.6,0.,1.58)))
        self.assertIsNotNone(planner.plan((.6,0.,1.58),(-.6,0.,1.45)))
        self.assertTrue(lower['unknown_hard'])
        poses=mapping.frontier_index.observation_model.pose_search
        stances=poses._stances(np.asarray((-.1,0.,1.77)),(0.,0.,0.),np.asarray(((.4,0.,1.58),)),
                              None,0.,None,None)
        self.assertTrue(any(np.allclose(position,(-.1,0.,1.77)) for position,_,_ in stances))
        # The shared lower tread and confirmed termination do not generate
        # a second task merely because the scheduling cut is higher.
        self.assertFalse(any(abs(mapping.frontier_index.nodes[k].height-1.58)<1e-5
                             for k in mapping.frontier_index.by_column.get((-1,0),())))

    def test_route_can_use_a_measured_tread_above_its_riser(self):
        from unittest.mock import patch
        from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
        from legbot_exploration.block_pct import BlockPlanner
        mapping=FixedLayerMap(open_settings(origin_z=.03,maturity_min_points=1,robot_radius=.15))
        points=[(x*.1,y*.1,-.002 if x<0 else .1)
                for x in range(-12,13) for y in range(-6,7)]
        faces=[(0.,y*.1,z) for y in range(-6,7) for z in (-.002,.056)]
        labels=[1]*len(points)+[1,-2]*(len(faces)//2)
        with patch('legbot_exploration.surface_orientation.classify',return_value=np.asarray(labels)):
            mapping.integrate(points+faces,sensor_origin=(0.,0.,.8));mapping.build()
        low=mapping.cell((0,0,0))
        self.assertEqual(low['passage_cost'],50.)
        self.assertTrue(mapping.cell((0,-1,0))['solid_hard'])
        planner=BlockPlanner(mapping,native_wrapper())
        self.assertIsNotNone(planner.plan((-.6,0.,-.002),(.6,0.,.1)))
        self.assertIsNotNone(planner.plan((.6,0.,.1),(-.6,0.,-.002)))

    def test_platform_retains_existing_ground_where_a_lower_cut_has_no_ground(self):
        from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
        mapping=FixedLayerMap(open_settings(maturity_min_points=1,robot_radius=0.,safety_margin=0.))
        evidence=mapping.surface_observation
        evidence.enter_frontier_floor((0.,0.,0.),.08)
        mapping.integrate([(0.,0.,1.58),(.1,0.,1.58),(.1,0.,0.)])
        mapping.build()
        self.assertIsNone(mapping.observed_cell((3,0,0)))
        ground_id=int(mapping.cell((4,0,0))['g_surface_id'])
        mapping.integrate([(.2,0.,1.58)])
        evidence.enter_frontier_floor((.2,0.,1.58),.08)
        mapping.build()

        def check_cells():
            original=mapping.cell((4,0,0))
            self.assertFalse(original['unknown'])
            self.assertAlmostEqual(original['G'],1.58,places=5)
            self.assertEqual(original['g_surface_id'],ground_id)
            self.assertFalse(mapping.ground_pending((1,0),4))
            self.assertAlmostEqual(mapping.cell((4,1,0))['G'],1.58,places=5)
            self.assertTrue(mapping.cell((4,3,0))['unknown'])

        check_cells()
        mapping.integrate([(0.,0.,0.)])
        mapping.build()
        mapping._reset_slice_products()
        mapping.build()
        check_cells()

    def test_new_floor_preserves_old_gc_below_an_observed_underside(self):
        from unittest.mock import patch
        from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
        from legbot_exploration.block_pct import BlockPlanner
        mapping=FixedLayerMap(open_settings(maturity_min_points=1,robot_radius=0.,safety_margin=0.))
        old_floor,new_floor,upper_floor=1.46,3.02,4.37
        points=[(x*.1,y*.1,z) for x in range(-4,5) for y in range(-4,5)
                for z in (old_floor,upper_floor)]
        mapping.integrate(points)
        with patch('legbot_exploration.surface_orientation.classify',return_value=np.asarray([-1])):
            mapping.integrate([(0.,0.,2.82)],sensor_origin=(0.,0.,2.))
        mapping.build()
        evidence=mapping.surface_observation
        self.assertTrue(evidence.enter_frontier_floor((0.,0.,old_floor),.08))
        mapping.integrate([(.1,0.,new_floor)])
        self.assertTrue(evidence.enter_frontier_floor((.1,0.,new_floor),.08))
        mapping.build()
        planner=BlockPlanner(mapping,native_wrapper())
        def read(k):
            return planner._read(k-mapping.first_layer,-int(mapping.lower[0]),-int(mapping.lower[1]))
        self.assertAlmostEqual(read(5)[1],old_floor,places=5)
        self.assertAlmostEqual(read(6)[1],2.82,places=5)
        self.assertAlmostEqual(read(7)[1],2.82,places=5)
        self.assertAlmostEqual(read(8)[1],2.82,places=5)
        self.assertAlmostEqual(read(9)[1],upper_floor,places=5)
        self.assertAlmostEqual(mapping.cell((5,0,0))['C'],2.82,places=5)
        mapping._reset_slice_products();mapping.build()
        self.assertAlmostEqual(read(7)[1],2.82,places=5)
        mapping.integrate([(0.,0.,new_floor)]);mapping.build()
        self.assertAlmostEqual(read(7)[1],new_floor,places=5)
        self.assertAlmostEqual(mapping.cell((5,0,0))['G'],old_floor,places=5)
        self.assertAlmostEqual(mapping.cell((5,0,0))['C'],2.82,places=5)

    def test_late_mature_ground_uses_existing_cut_for_admission_and_native_route(self):
        from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
        from legbot_exploration.block_pct import BlockPlanner
        mapping=FixedLayerMap(open_settings(maturity_min_points=3,robot_radius=.35,safety_margin=.05))
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
