"""Shared static surface evidence, cross-slice roles and local PCT costs."""
import unittest
from unittest.mock import patch
import numpy as np
from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
from legbot_exploration.map_evidence import SurfaceColumn
from legbot_exploration.map_costs import update_costs


def floor_grid(bound=1.,height=0.,hole=()):
    hole=set(hole)
    return np.asarray([(i*.1,j*.1,height) for i in range(-round(bound*10),round(bound*10)+1)
                       for j in range(-round(bound*10),round(bound*10)+1) if (i,j) not in hole],dtype='float32')


def open_settings(**settings):
    # Cost/fusion fixtures model exposed surfaces. Keep cuts off the measured
    # endpoints; strict indoor bounds are covered by test_slice_intervals.
    defaults=dict(allow_open_ceiling=True,origin_z=1e-5)
    defaults.update(settings)
    return MapSettings(**defaults)


def mapping(**settings):
    defaults=dict(maturity_min_points=1,robot_radius=.15,relief_update_threshold=0.,
                  terrain_update_threshold=0.,inflation_update_threshold=0.)
    defaults.update(settings)
    return FixedLayerMap(open_settings(**defaults))


class FixedMapTest(unittest.TestCase):
    def test_floor_arrival_keeps_measured_ceiling_until_this_column_changes(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=FixedLayerMap(open_settings(maturity_min_points=1),native_updates=native)
                below,new,upper=.98,1.40,2.10
                m.integrate(np.vstack((floor_grid(.5,below),floor_grid(.5,upper))))
                m.build()
                evidence=m.surface_observation
                evidence.enter_frontier_floor((0.,0.,upper),.08)
                m.integrate([[.1,0.,new]]);m.build()
                before=m.cell((2,0,0)).tobytes()
                self.assertTrue(evidence.enter_frontier_floor((.1,0.,new),.08))
                self.assertFalse(m.build())
                self.assertEqual(m.cell((2,0,0)).tobytes(),before)
                self.assertAlmostEqual(m.cell((2,0,0))['C'],upper,places=5)

                m.integrate([[0.,0.,new]]);m.build()
                row=m.cell((2,0,0))
                self.assertAlmostEqual(row['G'],below,places=5)
                self.assertAlmostEqual(row['C'],new,places=5)
                self.assertTrue(row['solid'])

    def test_surface_groups_reject_a_chain_beyond_the_full_noise_span(self):
        column=SurfaceColumn()
        for z in (0.,.049,.070):column.add(z,.05,100)
        self.assertEqual(len(column),2)
        self.assertAlmostEqual(column.surfaces[0].height,.0245)
        self.assertAlmostEqual(column.surfaces[1].height,.070)
        self.assertTrue(all(r.high-r.low<=.05+1e-7 for r in column.values()))

    def test_coalescing_cannot_bridge_two_overlapping_noise_bands(self):
        column=SurfaceColumn()
        for z in (0.,.07,.03,.04):column.add(z,.05,100)
        self.assertEqual(len(column),2)
        self.assertTrue(all(r.high-r.low<=.05+1e-7 for r in column.values()))
        self.assertEqual(sum(r.count for r in column.values()),4)

    def test_mature_mean_stays_fixed_but_tracks_the_full_accepted_span(self):
        column=SurfaceColumn()
        for z in (0.,.02,.04,-.02):column.add(z,.05,2)
        self.assertEqual(len(column),2)
        lower,higher=column.surfaces
        self.assertAlmostEqual(higher.height,.01)
        self.assertAlmostEqual(higher.high,.04)
        self.assertAlmostEqual(lower.height,-.02)
        self.assertTrue(all(r.high-r.low<=.05+1e-7 for r in column.values()))

    def test_upper_group_alone_supplies_g_above_two_mixed_treads(self):
        m=mapping(maturity_min_points=100,slice_dh=.1)
        m.integrate([(0.,0.,z) for z in (.91,.92,1.02,1.03)])
        m.build()
        self.assertEqual(len(m.columns[(0,0)]),2)
        middle,above=m.cell((10,0,0)),m.cell((11,0,0))
        self.assertAlmostEqual(middle['G'],.915,places=6)
        self.assertAlmostEqual(middle['C'],1.025,places=6)
        self.assertAlmostEqual(above['G'],1.025,places=6)
        self.assertTrue(np.isposinf(above['C']))
        m.integrate([(0.,0.,1.04)])
        m.build()
        self.assertAlmostEqual(m.cell((10,0,0))['G'],.915,places=6)
        self.assertAlmostEqual(m.cell((11,0,0))['G'],1.03,places=6)

    def test_mature_height_group_ignores_roles_and_keeps_full_span_noise_limit(self):
        m=mapping(maturity_min_points=3)
        m.integrate([(0,0,0.)]*3+[(0,0,.03)]*3,sensor_origin=(0,0,.8));m.build()
        self.assertAlmostEqual(m.cell((0,0,0))['G'],0.)
        m.integrate([(0,0,.07)],sensor_origin=(0,0,.8));m.build()
        self.assertAlmostEqual(m.cell((0,0,0))['G'],0.)
        self.assertAlmostEqual(m.cell((0,0,0))['C'],.07,places=6)
        self.assertAlmostEqual(m.observed_cell((1,0,0))['G'],.07,places=6)
        self.assertEqual(len(m.columns[0,0]),2)

    def test_sparse_measured_ground_immediately_fills_parking_hole(self):
        m=mapping(maturity_min_points=3,robot_radius=.35)
        m.integrate(np.repeat(floor_grid(1.,.30,hole=[(0,0)]),3,axis=0))
        m.integrate(np.repeat(floor_grid(1.,1.20),3,axis=0));m.build()
        pose=(.20,0.,.62)
        self.assertFalse(m.point_safe_check(pose)['ready'])
        with patch('legbot_exploration.surface_orientation.classify',side_effect=AssertionError('role gate')):
            m.integrate([(0.,0.,.306)]*3,sensor_origin=(0.,0.,.306));m.build()
        self.assertTrue(m.point_safe_check(pose)['ready'])
        self.assertAlmostEqual(m.cell((1,0,0))['G'],.306,places=6)
        self.assertAlmostEqual(m.cell((1,0,0))['C'],1.20,places=6)
        self.assertLess(m.cell((1,0,0))['terrain_cost'],20.)

    def test_opposite_views_of_close_heights_reuse_noise_group(self):
        m=mapping(maturity_min_points=3,robot_radius=0.,safety_margin=0.)
        m.integrate(np.repeat(floor_grid(.3,0.),3,axis=0),sensor_origin=(0.,0.,.4))
        m.integrate(np.repeat(floor_grid(.3,1.20),3,axis=0),sensor_origin=(0.,0.,.4));m.build()
        self.assertEqual(m.column_layers[0,0],{0,1,2,3})
        self.assertAlmostEqual(m.cell((2,0,0))['C'],1.20,places=5)
        m.integrate(floor_grid(.3,1.23),sensor_origin=(0.,0.,1.65));m.build()
        self.assertEqual(len(m.columns[0,0]),2)
        self.assertAlmostEqual(m.cell((3,0,0))['G'],1.20,places=5)
        self.assertAlmostEqual(m.cell((2,0,0))['C'],1.20,places=5)

    def test_close_height_reuses_frozen_g_then_distinct_ceiling_limits_support(self):
        m=mapping(maturity_min_points=1,robot_radius=0.,safety_margin=0.)
        m.integrate(floor_grid(.5,1.177),sensor_origin=(0.,0.,.8));m.build()
        self.assertTrue(m.point_safe_check((0.,0.,1.497))['ready'])
        m.integrate(floor_grid(.5,1.174),sensor_origin=(0.,0.,1.5));m.build()
        self.assertAlmostEqual(m.observed_cell((3,0,0))['G'],1.177,places=5)
        self.assertEqual(len(m.columns[0,0]),1)
        m.integrate(floor_grid(.5,1.30),sensor_origin=(0.,0.,1.5));m.build()
        self.assertFalse(m.point_safe_check((0.,0.,1.497),floor=(1.177,.05))['ready'])
        self.assertAlmostEqual(m.point_safe_check((0.,0.,1.62))['ground_z'],1.30,places=5)
        self.assertTrue(m.frontier_observed(3,(0,0)))
        self.assertFalse(m.surface_observation.has_frontier_ground((0,0),1.177,layer=3))

    def test_near_ground_noise_uses_the_configured_tolerance(self):
        samples=[(0.,0.,.091),(0.,0.,.097),(0.,0.,.099),(0.,0.,.30)]
        for tolerance,expected_ceiling in ((.05,.30),(.005,.097)):
            with self.subTest(tolerance=tolerance):
                m=mapping(origin_z=.093,surface_noise_tolerance=tolerance)
                with patch('legbot_exploration.surface_orientation.classify',
                           return_value=np.asarray([1,-2,0,-2])):
                    m.integrate(samples,sensor_origin=(0.,0.,.8))
                m.build()
                row=m.cell((0,0,0))
                self.assertAlmostEqual(row['G'],.091,places=6)
                self.assertAlmostEqual(row['C'],expected_ceiling,places=6)
                # A separate measured underside retains its own clearance.
                with patch('legbot_exploration.surface_orientation.classify',
                           return_value=np.asarray([-1])):
                    m.integrate([(0.,0.,.095)],sensor_origin=(0.,0.,0.))
                m.build()
                self.assertAlmostEqual(m.cell((0,0,0))['C'],expected_ceiling,places=6)

    def test_nearby_hits_share_frozen_ground_across_zero_cut(self):
        m=mapping(maturity_min_points=1,robot_radius=0.,safety_margin=0.)
        for h in (-.004,.002):m.integrate(floor_grid(.3,h),sensor_origin=(0.,0.,.8));m.build()
        self.assertAlmostEqual(m.observed_cell((0,0,0))['G'],-.004,places=6)
        self.assertAlmostEqual(m.observed_cell((1,0,0))['G'],-.004,places=6)
        self.assertTrue(np.isposinf(m.observed_cell((1,0,0))['C']))
        self.assertTrue(m.point_safe_check((0.,0.,.322))['ready'])
        self.assertFalse(any(key[1:3] in m.columns for key in m.frontier_index.nodes))

    def test_neighbor_tread_does_not_change_this_columns_measured_gc(self):
        m=mapping(maturity_min_points=3,slice_dh=.30)
        m.integrate([(-.025,0.,.23),(.008,0.,.335),(.009,0.,.329)]*3);m.build()
        original=tuple(float(m.cell((1,0,0))[f]) for f in ('G','C'))
        m.integrate([(.125,0.,.36)]*3);m.build()
        self.assertEqual(tuple(float(m.cell((1,0,0))[f]) for f in ('G','C')),original)
        self.assertEqual(m.cell((1,0,0))['passage_cost'],50.)
        self.assertAlmostEqual(m.cell((2,0,0))['G'],.333,places=5)
        self.assertAlmostEqual(m.cell((2,1,0))['G'],.36,places=5)

    def test_non_ground_barrier_keeps_clearance_without_supplying_g(self):
        m=mapping(maturity_min_points=3,slice_dh=.10)
        points=[(-.02,0,0.),(.002,0,.10),(.02,0,.16),(.02,0,.27)]*3
        with patch('legbot_exploration.surface_orientation.classify',return_value=np.asarray([1,-2,1,1]*3)):
            m.integrate(points,sensor_origin=(0.,0.,.8))
        m.build()
        self.assertAlmostEqual(m.cell((0,0,0))['G'],0.)
        self.assertAlmostEqual(m.cell((0,0,0))['C'],.10,places=5)
        self.assertEqual(m.cell((0,0,0))['passage_cost'],50.)
        blocked=m.observed_cell((1,0,0))
        self.assertFalse(blocked['unknown']);self.assertTrue(blocked['solid'])
        self.assertAlmostEqual(blocked['G'],.10,places=5)
        self.assertAlmostEqual(m.cell((2,0,0))['G'],.16,places=5)
        self.assertAlmostEqual(m.cell((3,0,0))['G'],.27,places=5)

    def test_step_boundary_uses_the_upper_tread_as_ground_in_higher_cuts(self):
        m=mapping(slice_dh=.10)
        lower=[(-.04,-.04,0.),(-.01,-.04,0.),(-.04,-.01,0.),(.01,-.04,0.)]
        upper=[(.01,-.01,.13),(.04,-.01,.13),(.04,.01,.13),(-.01,.01,.13)]
        other=[(-.1,0.,0.),(.1,0.,.13),(-.01,-.02,.09),(.01,-.02,.09),(0.,0.,.23)]
        labels=[1]*10+[-2,-2,-1]
        with patch('legbot_exploration.surface_orientation.classify',return_value=np.asarray(labels)):
            m.integrate(lower+upper+other,sensor_origin=(0.,0.,.8))
        m.build()
        low,high=m.cell((0,0,0)),m.cell((2,0,0))
        self.assertAlmostEqual(low['G'],0.)
        self.assertAlmostEqual(low['C'],.13,places=5)
        self.assertEqual(low['passage_cost'],50.)
        self.assertAlmostEqual(high['G'],.13,places=5)
        self.assertAlmostEqual(high['C'],.23,places=5)
        self.assertEqual(high['passage_cost'],50.)
        self.assertAlmostEqual(m.cell((2,1,0))['G'],.13,places=5)
        self.assertEqual(len(m.columns[(0,0)].surfaces),3)

    def test_step_boundary_keeps_g_and_c_when_samples_are_sparse_or_overlap(self):
        for upper_xy in [[(.035,.035)], [(-.035,-.035),(-.01,-.01),(.035,.035),(.01,.01)]]:
            with self.subTest(upper_xy=upper_xy):
                m=mapping(slice_dh=.10)
                samples=[(-.1,0.,0.),(.1,0.,.13),(-.035,-.035,0.),(-.01,-.01,0.)]
                samples.extend((x,y,.13) for x,y in upper_xy)
                with patch('legbot_exploration.surface_orientation.classify',return_value=np.ones(len(samples),int)):
                    m.integrate(samples,sensor_origin=(0.,0.,.8))
                m.build()
                self.assertAlmostEqual(m.cell((1,0,0))['C'],.13,places=5)
                self.assertEqual(m.cell((1,0,0))['passage_cost'],50.)

    def test_unmeasured_upper_tread_remains_blocked(self):
        m=mapping(slice_dh=.10)
        samples=[(-.035,0.,0.),(-.01,0.,0.),(.035,0.,.11),(0.,.1,0.),(.035,.1,.11)]
        with patch('legbot_exploration.surface_orientation.classify',return_value=np.asarray([1,1,-2,1,-1])):
            m.integrate(samples,sensor_origin=(0.,0.,.8))
        m.build()
        self.assertEqual(m.cell((1,0,0))['passage_cost'],50.)

    def test_three_samples_have_equal_weight_then_in_band_points_are_ignored(self):
        c=SurfaceColumn()
        for z in (0.,.01,.018):self.assertTrue(c.add(z,.02,3))
        record=c.surfaces[0]
        self.assertAlmostEqual(record.height,(0.+.01+.018)/3)
        self.assertEqual(record.count,3)
        self.assertTrue(record.mature)
        self.assertEqual(record.pending,[])
        self.assertFalse(c.add(.019,.02,3))
        self.assertEqual(record.count,3)
        self.assertAlmostEqual(record.height,.028/3)

    def test_same_frame_stops_at_num_and_does_not_average_all_later_returns(self):
        m=FixedLayerMap(open_settings())
        m.integrate(np.asarray([[0,0,z] for z in (0.,.01,.018,.02,.015)],dtype='float32'));m.build()
        self.assertAlmostEqual(m.columns[(0,0)].surfaces[0].height,.028/3,places=7)
        self.assertEqual(m.ignored_static_points,2)

    def test_count_is_per_surface_not_per_referencing_slice(self):
        m=FixedLayerMap(open_settings())
        m.integrate(np.asarray([[0,0,0.],[0,0,3.]],dtype='float32'));m.build()
        self.assertGreater(len(m.column_layers[(0,0)]),5)
        self.assertEqual([r.count for r in m.columns[(0,0)].surfaces],[1,1])
        self.assertFalse(any(row['g_mature'] for _,row in m.iter_cells()))

    def test_cross_cut_noise_is_one_shared_ground_instead_of_a_low_ceiling(self):
        m=FixedLayerMap(open_settings())
        heights=(-.0000256598,.0007749796,.008226186)
        m.integrate(np.vstack([floor_grid(height=z) for z in heights]));m.build()
        rows=[row for _,row in m.layers_at(0,0) if np.isfinite(row['G'])]
        self.assertTrue(rows)
        for row in rows:
            self.assertAlmostEqual(row['G'],sum(heights)/3,places=7)
            self.assertTrue(np.isposinf(row['C']))
            self.assertTrue(row['g_mature'])
        self.assertEqual(len({row['g_surface_id'] for row in rows}),1)
        self.assertTrue(m.point_safe_check([0,0,.32383])['ready'])

    def test_zero_tolerance_preserves_original_distinct_extrema(self):
        m=mapping(surface_noise_tolerance=0.)
        m.integrate(np.vstack((floor_grid(height=-.003),floor_grid(height=.004))));m.build()
        row=m.cell((0,0,0))
        self.assertAlmostEqual(row['G'],-.003,places=6)
        self.assertAlmostEqual(row['C'],.004,places=6)
        self.assertEqual(row['passage_cost'],50.)

    def test_shared_G_C_ids_are_distinct_and_match_the_authoritative_mean(self):
        m=FixedLayerMap(open_settings())
        points=[[0,0,z] for z in (0.,.004,.008,1.5,1.502,1.504)]
        m.integrate(np.asarray(points,dtype='float32'));m.build()
        records={r.id:r for r in m.columns[(0,0)].surfaces}
        for _,row in m.layers_at(0,0):
            for name in ('G','C'):
                sid=row[name.lower()+'_surface_id']
                if sid:self.assertEqual(row[name],records[sid].height)
            if row['g_surface_id'] and row['c_surface_id']:
                self.assertNotEqual(row['g_surface_id'],row['c_surface_id'])
                self.assertLess(row['G'],row['C'])

    def test_two_immature_groups_coalesce_using_unique_samples(self):
        c=SurfaceColumn()
        heights=(0.,.015,.008,.012,.013)
        # Separate initial records, then coalesce only within the full band.
        for at,z in enumerate(heights):c.add(z,0. if at<2 else .02,5)
        self.assertEqual(len(c),1)
        self.assertEqual(c.surfaces[0].count,5)
        self.assertAlmostEqual(c.surfaces[0].height,sum(heights)/5)
        self.assertTrue(c.surfaces[0].mature)

    def test_merge_prefers_the_already_mature_height(self):
        c=SurfaceColumn()
        for z in (0.,.012,.014):c.add(z,.02,3)
        c.add(.018,0.,3)
        self.assertEqual(len(c),2)
        c._coalesce(.02,3)
        self.assertEqual(len(c),1)
        self.assertAlmostEqual(c.surfaces[0].height,(0.+.012+.014)/3)
        self.assertEqual(c.surfaces[0].count,3)

    def test_out_of_band_new_obstacle_is_accepted_after_ground_matures(self):
        m=FixedLayerMap(open_settings())
        m.integrate(np.repeat(floor_grid(),3,axis=0));m.build()
        ground=m.columns[(0,0)].surfaces[0];height=ground.height
        m.integrate(np.asarray([[0,0,.3]],dtype='float32'));m.build()
        self.assertEqual(ground.height,height)
        self.assertEqual(ground.count,3)
        self.assertEqual(len(m.columns[(0,0)]),2)
        self.assertEqual(m.cell((0,0,0))['passage_cost'],50.)
        self.assertFalse(m.cell((0,0,0))['c_mature'])
        self.assertAlmostEqual(m.cell((0,0,0))['C'],.3,places=6)
        self.assertFalse(m.point_safe_check([0,0,.32])['ready'])
        m.integrate(np.asarray([[0,0,.301],[0,0,.302]],dtype='float32'));m.build()
        self.assertTrue(m.cell((0,0,0))['c_mature'])
        self.assertAlmostEqual(m.cell((0,0,0))['C'],.301,places=6)
        self.assertEqual(m.cell((0,0,0))['passage_cost'],50.)
        upper=m.cell((1,0,0))
        self.assertAlmostEqual(upper['G'],.301,places=6)
        self.assertTrue(np.isposinf(upper['C']))
        self.assertEqual(upper['passage_cost'],0.)
        self.assertFalse(m.point_safe_check([0,0,.32])['ready'])

    def test_new_stair_surface_patch_replaces_mature_ground_in_all_higher_cuts(self):
        m=mapping(maturity_min_points=3)
        # Heights recorded at the blocking stair cell (7.10, 1.90).
        x,y=7.10,1.90
        m.integrate([(x,y,z) for z in (3.828,5.880)*3]);m.build()
        higher=[m.cell((k,71,19)) for k in range(8,12)]
        self.assertTrue(all(row['g_mature'] for row in higher))
        m.integrate([(x,y,3.892)],sensor_origin=(7.1,1.1,4.2));m.build()
        self.assertAlmostEqual(higher[0]['G'],3.892,places=6)
        m.integrate([(x+dx*.1,y+dy*.1,3.892) for dx in range(-2,3) for dy in range(-2,3)
                     if dx or dy],sensor_origin=(7.1,1.1,4.2));m.build()
        # Reobserve the center using the now measured local surface geometry.
        m.integrate([(x,y,3.892)],sensor_origin=(7.1,1.1,4.2));m.build()
        for row in higher:
            self.assertAlmostEqual(row['G'],3.892,places=6)
            self.assertAlmostEqual(row['C'],5.880,places=6)
            self.assertAlmostEqual(row['C']-row['G'],1.988,places=6)
            self.assertFalse(row['g_mature'])
            self.assertEqual(row['passage_cost'],0.)
        self.assertAlmostEqual(m.cell((12,71,19))['G'],5.880,places=6)
        check=m.point_safe_check((x,y,3.892+.32))
        self.assertAlmostEqual(check['clearance'],1.988,places=6)

    def test_each_cut_uses_the_highest_below_and_lowest_above_in_one_batch(self):
        m=mapping(maturity_min_points=3,slice_dh=.05)
        m.integrate([(0,0,z) for z in (.43,.90)*3]);m.build()
        m.integrate([(0,0,.56),(0,0,.489)]);m.build()
        low,mid,high=[m.cell((k,0,0)) for k in (9,10,12)]
        for row,g,c in ((low,.43,.489),(mid,.489,.56),(high,.56,.90)):
            self.assertAlmostEqual(row['G'],g,places=6)
            self.assertAlmostEqual(row['C'],c,places=6)
        self.assertEqual(mid['g_surface_id'],low['c_surface_id'])
        self.assertEqual(high['g_surface_id'],mid['c_surface_id'])

    def test_recorded_four_centimetre_noise_does_not_close_mature_ground(self):
        for ground,noise in ((-.005091389,.036241353),(-.009104222,.031170696)):
            with self.subTest(ground=ground):
                m=FixedLayerMap(open_settings())
                m.integrate(np.repeat(floor_grid(height=ground),3,axis=0));m.build()
                version=m.revision
                m.integrate(np.asarray([[0,0,noise]],dtype='float32'));m.build()
                self.assertEqual(m.revision,version)
                self.assertEqual(len(m.columns[(0,0)]),1)
                self.assertTrue(np.isposinf(m.cell((0,0,0))['C']))
                self.assertTrue(m.point_safe_check([0,0,ground+.32])['ready'])

    def test_closer_C_and_higher_G_change_only_their_affected_layers(self):
        m=mapping()
        m.integrate(np.asarray([[0,0,z] for z in (0.,3.,6.)],dtype='float32'));m.build()
        far_before=m.cell((10,0,0)).copy()
        for _ in range(3):
            m.integrate(np.asarray([[0,0,2.]],dtype='float32'));m.build()
        self.assertEqual(m.cell((1,0,0))['C'],2.)
        self.assertEqual(m.cell((5,0,0))['G'],2.)
        self.assertEqual(m.cell((10,0,0)).tobytes(),far_before.tobytes())
        gs=[r['G'] for _,r in m.layers_at(0,0) if np.isfinite(r['G'])]
        cs=[r['C'] for _,r in m.layers_at(0,0) if np.isfinite(r['C'])]
        self.assertEqual(gs,sorted(gs));self.assertEqual(cs,sorted(cs))

    def test_missing_C_is_filled_even_after_G_matures(self):
        m=mapping();m.integrate(floor_grid());m.build()
        self.assertTrue(np.isposinf(m.cell((0,0,0))['C']))
        m.integrate(np.asarray([[0,0,.8]],dtype='float32'));m.build()
        self.assertAlmostEqual(m.cell((0,0,0))['C'],.8,places=6)
        self.assertEqual(m.cell((0,0,0))['G'],0.)

    def test_mature_repeats_do_not_change_geometry_or_store_more_samples(self):
        m=FixedLayerMap(open_settings());points=np.repeat(floor_grid(),3,axis=0)
        m.integrate(points);m.build();version=m.revision;blocks={k:v.tobytes() for k,v in m.blocks.items()}
        self.assertEqual(m.integrate(points),m.surface_records)
        self.assertFalse(m.build())
        self.assertEqual(m.revision,version)
        self.assertEqual({k:v.tobytes() for k,v in m.blocks.items()},blocks)
        self.assertTrue(all(not r.pending for c in m.columns.values() for r in c.values()))

    def test_closed_hole_stays_unknown_until_real_hits_arrive(self):
        m=FixedLayerMap(open_settings())
        m.integrate(np.repeat(floor_grid(hole=[(0,0)]),3,axis=0));m.build()
        self.assertFalse(m.point_safe_check([0,0,.32])['ready'])
        m.integrate(np.asarray([[0,0,0.]]*3,dtype='float32'));m.build()
        self.assertTrue(m.point_safe_check([0,0,.32])['ready'])

    def test_late_ground_across_cut_boundary_clears_unknown_disk(self):
        for cut in (0.,1.5):
            with self.subTest(cut=cut):
                m=mapping(maturity_min_points=3,robot_radius=.35,safety_margin=.05)
                m.integrate(np.repeat(floor_grid(bound=1.2,height=cut+.01,hole=[(0,0)]),3,axis=0));m.build()
                query=[.3,0.,cut+.33]
                self.assertEqual(m.point_safe_check(query)['reason'],'unknown_footprint')
                m.integrate(np.asarray([[0,0,cut-.01]],dtype='float32'));m.build()
                self.assertTrue(m.point_safe_check(query)['ready'])
                self.assertFalse(m.cell((round(cut/.5),0,0))['g_mature'])
                m.integrate(np.asarray([[0,0,cut-.01]]*2,dtype='float32'));m.build()
                check=m.point_safe_check(query)
                self.assertTrue(check['ready'],check)
                layer=round(cut/.5)
                row=m.cell((layer,0,0))
                self.assertAlmostEqual(row['G'],cut-.01,places=6)
                self.assertTrue(row['g_mature'])
                self.assertAlmostEqual(m.observed_cell((layer+1,0,0))['G'],cut-.01,places=6)
                self.assertEqual(m.columns[(0,0)].surfaces[0].count,3)

    def test_slice_coverage_is_independent_of_observation_batch_order(self):
        surrounding=floor_grid(bound=1.2,height=.01,hole=[(0,0)])
        center=np.asarray([[0,0,-.01]],dtype='float32')
        maps=[]
        for batches in ((surrounding,center),(center,surrounding),(np.vstack((surrounding,center)),)):
            m=mapping(robot_radius=.35,safety_margin=.05)
            for batch in batches:m.integrate(batch);m.build()
            self.assertTrue(m.point_safe_check([.3,0.,.33])['ready'])
            maps.append(m)
        expected=maps[0]
        for m in maps[1:]:
            self.assertEqual(m.column_layers,expected.column_layers)
            for key,_ in expected.iter_cells():
                for field in ('G','C','g_mature','passage_cost','terrain_cost','hard'):
                    a,b=m.cell(key)[field],expected.cell(key)[field]
                    if field in ('G','C') and np.isnan(a) and np.isnan(b):continue
                    self.assertAlmostEqual(float(a),float(b),places=7,msg=str((key,field)))

    def test_neighbor_hit_does_not_materialize_a_new_cut_in_another_column(self):
        m=mapping()
        m.integrate([(0.,0.,-.01)]);m.build()
        m.integrate([(.9,0.,.01)]);m.build()
        self.assertEqual(m.column_layers[(0,0)],{0})
        self.assertIsNotNone(m.observed_cell((1,0,0)))
        self.assertFalse(m.cell((1,0,0))['measured'])
        m.integrate([(0.,0.,.06)]);m.build()
        self.assertIn(1,m.column_layers[(0,0)])
        self.assertAlmostEqual(m.observed_cell((1,0,0))['G'],.06,places=6)

    def test_cut_move_retires_the_old_local_demand(self):
        m=mapping(maturity_min_points=3)
        m.integrate([(0.,0.,.51),(.1,0.,.15)]);m.build()
        self.assertEqual(m.column_layers[(0,0)],{2})
        self.assertAlmostEqual(m.cell((2,0,0))['G'],.51,places=5)
        self.assertFalse(m.ground_pending((1,0),2))
        # The mean of the same measured surface moves across the cut.
        m.integrate([(0.,0.,.47)]);m.build()
        self.assertEqual(m.column_layers[(0,0)],{1})
        self.assertEqual(m.column_layers[(1,0)],{1})
        self.assertTrue(m.cell((2,0,0))['unknown'])
        self.assertTrue(np.isnan(m.cell((2,0,0))['G']))
        self.assertAlmostEqual(m.observed_cell((2,1,0))['G'],.15,places=5)
        self.assertAlmostEqual(m.cell((1,0,0))['G'],.49,places=5)

    def test_inherited_slices_use_this_columns_true_surfaces_and_keep_floors_distinct(self):
        m=mapping(maturity_min_points=3,robot_radius=.35,safety_margin=.05)
        surroundings=np.vstack((floor_grid(height=.01,hole=[(0,0)]),
                                floor_grid(height=1.51,hole=[(0,0)])))
        m.integrate(np.repeat(surroundings,3,axis=0));m.build()
        m.integrate(np.asarray([[0.,0.,-.01]]*3,dtype='float32'));m.build()
        self.assertTrue(m.point_safe_check([0.,0.,.31])['ready'])
        self.assertFalse(m.point_safe_check([0.,0.,1.81])['ready'])
        for _,row in m.layers_at(0,0):
            if row['g_mature']:self.assertAlmostEqual(row['G'],-.01,places=6)
        for _ in range(3):
            m.integrate(np.asarray([[0.,0.,1.49]],dtype='float32'));m.build()
        self.assertTrue(m.point_safe_check([0.,0.,1.81])['ready'])
        lower=m.cell((1,0,0));upper=m.cell((3,0,0))
        self.assertAlmostEqual(lower['G'],-.01,places=6)
        self.assertAlmostEqual(lower['C'],1.49,places=6)
        self.assertAlmostEqual(upper['G'],1.49,places=6)
        self.assertAlmostEqual(m.observed_cell((4,0,0))['G'],1.49,places=5)
        self.assertNotEqual(lower['g_surface_id'],upper['g_surface_id'])

    def test_late_real_low_ceiling_remains_blocked_after_slice_inheritance(self):
        m=mapping(maturity_min_points=3,robot_radius=.35,safety_margin=.05)
        m.integrate(np.repeat(floor_grid(height=.01,hole=[(0,0)]),3,axis=0));m.build()
        m.integrate(np.asarray([[0.,0.,z] for z in (0.,.08)]*3,dtype='float32'));m.build()
        self.assertEqual(m.cell((0,0,0))['passage_cost'],50.)
        self.assertFalse(m.point_safe_check([0.,0.,.32],height_tolerance=.02)['ready'])
        self.assertAlmostEqual(m.cell((1,0,0))['G'],.08,places=6)

    def test_outdoor_zero_passage_does_not_make_missing_ground_safe(self):
        m=mapping();m.integrate(floor_grid());m.build()
        self.assertEqual(m.cell((0,0,0))['passage_cost'],0.)
        self.assertTrue(np.isposinf(m.cell((0,0,0))['C']))
        self.assertTrue(m.point_safe_check([0,0,.32])['ready'])
        self.assertFalse(m.point_safe_check([3,3,.32])['ready'])

    def test_isolated_first_hit_is_observed_with_unassessed_terrain(self):
        m=FixedLayerMap(open_settings())
        m.integrate([(0.,0.,0.)]);m.build()
        row=m.cell((0,0,0))
        self.assertEqual(row['G'],0.)
        self.assertFalse(row['g_mature'])
        self.assertFalse(row['unknown'])
        self.assertEqual(row['passage_cost'],0.)
        self.assertEqual(row['terrain_cost'],-1.)
        self.assertEqual(m.column_layers[(0,0)],{0})
        self.assertIsNone(m.cell((-1,0,0)))
        self.assertIsNone(m.cell((1,0,0)))

    def test_low_structure_outside_tolerance_remains_blocked(self):
        m=mapping();m.integrate(np.asarray([[0,0,0.],[0,0,.06]],dtype='float32'));m.build()
        self.assertEqual(m.cell((0,0,0))['passage_cost'],50.)

    def test_no_new_points_or_elapsed_time_cannot_change_static_map(self):
        m=mapping();m.integrate(floor_grid(),now=0.);m.build(now=0.)
        version=m.revision
        self.assertFalse(m.build(now=100000.))
        self.assertEqual(m.revision,version)

    def test_world_expansion_never_moves_existing_blocks(self):
        m=mapping(max_cells=100000);m.integrate(floor_grid());m.build()
        blocks=dict(m.blocks);before=m.cell((0,0,0)).copy()
        m.integrate(np.asarray([[10000,10000,2.1],[-10000,-10000,-1.2]],dtype='float32'));m.build()
        self.assertTrue(all(m.blocks[k] is v for k,v in blocks.items()))
        self.assertEqual(m.cell((0,0,0)).tobytes(),before.tobytes())
        self.assertLess(m.last_work['base_cells'],20)
        self.assertAlmostEqual(m.settings.origin_z+m.first_layer*m.settings.slice_dh,-1.+m.settings.origin_z)
        self.assertAlmostEqual(m.settings.origin_z+m.last_layer*m.settings.slice_dh,2.5+m.settings.origin_z)

    def test_incremental_costs_match_full_dependency_recompute(self):
        m=mapping();m.integrate(floor_grid(height=-.1));m.build()
        for point in ((0,0,-.06),(.3,.1,.3),(-.4,.3,-.02)):
            m.integrate(np.asarray([point],dtype='float32'));m.build()
            # Recompute costs on independent copies of the committed cell buffers.
            full=FixedLayerMap(m.settings,update_workers=1)
            full.blocks={key:block.copy() for key,block in m.blocks.items()}
            all_keys={key for key,_ in full.iter_cells()}
            for block in full.blocks.values():
                for name in ('passage_cost','relief_cost','terrain_cost','inflated_terrain_cost','terrain_inflation_reference'):block[name]=-1
                block['q']=block['m']=np.nan
                block['feature_valid']=block['flat']=block['solid']=block['solid_hard']=False
                block['unknown']=block['hard']=block['unknown_hard']=True
            update_costs(full,all_keys,all_keys)
            for key in all_keys:
                for name in ('passage_cost','relief_cost','terrain_cost','inflated_terrain_cost','hard'):
                    self.assertAlmostEqual(float(m.cell(key)[name]),float(full.cell(key)[name]),places=8,msg=str((point,key,name)))

    def test_weighted_inflation_matches_explicit_formula(self):
        m=mapping();m.integrate(floor_grid(height=-.1));m.build()
        m.integrate(np.asarray([[0,0,.3]],dtype='float32'));m.build()
        s=m.settings;key=(0,2,0);expected=0.;half=s.inflation_kernel_size//2
        for dx in range(-half,half+1):
            for dy in range(-half,half+1):
                row=m.cell((key[0],key[1]+dx,key[2]+dy))
                if row is None or row['terrain_cost']<0:continue
                weight=np.clip(1.-(np.hypot(dx,dy)*s.resolution-s.robot_radius)/(s.safety_margin+s.resolution),0.,1.)
                expected=max(expected,row['terrain_cost']*weight)
        self.assertAlmostEqual(m.cell(key)['inflated_terrain_cost'],expected)

    def test_capacity_rejection_is_atomic_for_mean_counts_and_columns(self):
        m=FixedLayerMap(open_settings(max_surface_records=1))
        m.integrate(np.asarray([[0,0,0.]],dtype='float32'))
        m.integrate(np.asarray([[0,0,.01],[1,0,0.]],dtype='float32'))
        self.assertEqual(m.surface_records,1)
        self.assertEqual(m.error,'pct_surface_record_limit')
        self.assertEqual(m.columns[(0,0)].surfaces[0].count,1)
        self.assertEqual(m.columns[(0,0)].surfaces[0].height,0.)
        self.assertNotIn((1,0),m.columns)

    def test_invalid_settings_and_unimplemented_dynamic_mode_are_rejected(self):
        for args in ({'mapping_mode':'dynamic'},{'inflation_kernel_size':12},
                     {'surface_noise_tolerance':-.1},{'maturity_min_points':0}):
            with self.assertRaises(ValueError):open_settings(**args)


if __name__=='__main__':unittest.main()
