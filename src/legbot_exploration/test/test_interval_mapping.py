"""Regression scenarios for interval mapping, real ray evidence and native A*."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np

from legbot_exploration.fixed_layer_map import MapSettings
from legbot_exploration.interval_map import IntervalLayerMap
from legbot_exploration.interval_rays import trace_free_columns


def mapping(native=True, **settings):
    defaults=dict(maturity_min_points=1,robot_radius=0.,safety_margin=0.,
                  traversability_kernel_size=3,inflation_kernel_size=3)
    defaults.update(settings)
    return IntervalLayerMap(MapSettings(**defaults),native_updates=native)


def plane(heights=(0.,1.5), bound=4, missing=()):
    return np.asarray([(i*.1,j*.1,z) for i in range(-bound,bound+1)
                       for j in range(-bound,bound+1) if (i,j) not in missing
                       for z in heights],dtype=np.float32)


def stored(m,xy=(0,0)):
    k=min(m.column_layers[xy])
    return m.cell((k,*xy))


class IntervalMappingTest(unittest.TestCase):
    def test_one_true_midpoint_per_finite_passage_and_shared_queries(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=mapping(native)
                m.integrate(plane((0.,3.)));m.build()
                self.assertEqual(m.column_layers[0,0],{1})
                row=m.cell((1,0,0))
                self.assertEqual((row['G'],row['C'],row['node_z']),(0.,3.,1.5))
                for k in range(1,6):
                    shared=m.observed_cell((k,0,0))
                    self.assertEqual((shared['G'],shared['C']),(0.,3.))
                    self.assertFalse(m.frontier_index.slices.get(k) and
                                     any(key[1:3]==(0,0) for key in m.frontier_index.slices[k].nodes))
                self.assertIsNone(m.observed_cell((math.ceil(3./m.settings.slice_dh),0,0)))
                self.assertTrue(m.point_safe_check((0.,0.,.32))['ready'])
                self.assertEqual(m.stats['interval_nodes'],81)

    def test_continuity_is_independent_of_clearance_and_inclusive(self):
        m=mapping()
        m.integrate([(0.,0.,z) for z in (1.5,.2,0.,.1)])
        m.build()
        walls=m.columns[0,0]._obstacle_samples
        self.assertEqual(len(walls),2)
        self.assertAlmostEqual(walls[0][0],0.)
        self.assertAlmostEqual(walls[0][1],.2,places=6)
        self.assertAlmostEqual(m.cell((1,0,0))['G'],.2,places=6)
        self.assertAlmostEqual(m.cell((1,0,0))['node_z'],.85,places=6)

    def test_low_clearance_is_known_unusable_not_an_unknown_boundary(self):
        m=mapping()
        m.integrate([(0.,0.,z) for z in (0.,.4)])
        m.build()
        self.assertFalse(m.column_layers[0,0])
        self.assertIsNotNone(m.obstacle_at((0,0,0)))
        self.assertFalse(m.point_safe_check((0.,0.,.32))['ready'])
        self.assertFalse(m.frontier_index.nodes)

    def test_exact_minimum_clearance_is_admitted(self):
        m=mapping();m.integrate(plane((0.,.5)));m.build()
        row=m.cell((1,0,0))
        self.assertIsNotNone(row)
        self.assertFalse(row['solid'])
        self.assertAlmostEqual(row['node_z'],.25)
        self.assertTrue(m.point_safe_check((0.,0.,.32))['ready'])

    def test_underside_alone_cannot_create_upper_ground(self):
        m=mapping()
        m.integrate(plane((2.82,)),sensor_origin=(0.,0.,2.),body_height=1.78)
        m.build()
        self.assertFalse(m.layers)
        self.assertFalse(m.frontier_index.nodes)
        self.assertTrue(m.columns)

    def test_range_and_pose_alone_do_not_prove_open_clearance(self):
        m=mapping()
        m.integrate([(0.,0.,0.)],sensor_origin=(.4,0.,.8),body_height=.32)
        m.build()
        self.assertFalse(m.column_layers[0,0])
        self.assertIn((0,0),m._pending_open)
        self.assertFalse(m.point_safe_check((0.,0.,.32))['ready'])

    def test_actual_vertical_ray_establishes_a_bound_not_an_exact_C(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=mapping(native)
                m.integrate([(0.,0.,1.)],sensor_origin=(0.,0.,1.8),body_height=1.32)
                m.build()
                row=stored(m)
                self.assertTrue(math.isnan(row['C']))
                self.assertAlmostEqual(row['C_clear'],1.8)
                self.assertAlmostEqual(row['node_z'],1.4)
                self.assertAlmostEqual(m.observed_cell((min(m.column_layers[0,0]),0,0))['C'],1.8)
                self.assertIsNone(m.observed_cell((math.ceil(1.8/m.settings.slice_dh),0,0)))

    def test_open_support_is_usable_by_parking_and_incremental_support_index(self):
        m=mapping()
        for x in range(-2,3):
            for y in range(-2,3):
                m.integrate([(x*.1,y*.1,0.)],sensor_origin=(x*.1,y*.1,.8),body_height=.32)
        m.build()
        checked=m.point_safe_check((0.,0.,.32))
        self.assertTrue(checked['ready'],checked)
        self.assertFalse(checked['clearance_observed'])
        self.assertAlmostEqual(checked['clearance_lower_bound'],.8)
        self.assertIn((0,0),m.support_targets.columns)
        k=min(m.column_layers[0,0])
        self.assertTrue(m.traversable_block((k,0,0),m.blocks[k,0,0])[0,0])

    def test_sensor_height_is_not_a_substitute_for_body_height(self):
        m=mapping()
        m.integrate([(0.,0.,0.)],sensor_origin=(0.,0.,.8))
        m.build()
        self.assertFalse(m.column_layers[0,0])

    def test_far_returns_supply_near_free_rays_without_becoming_surfaces(self):
        m=mapping()
        m.integrate([(0.,0.,0.)],sensor_origin=(0.,0.,.8),body_height=.32)
        m.build();before=m.revision
        m.integrate([],sensor_origin=(0.,0.,.8),free_rays=[(0.,0.,9.)])
        m.build()
        self.assertEqual(len(m.columns[0,0]),1)
        self.assertEqual(m.surface_records,1)
        self.assertTrue(math.isnan(stored(m)['C']))
        self.assertAlmostEqual(stored(m)['C_clear'],5.)
        self.assertAlmostEqual(stored(m)['node_z'],2.5)
        self.assertGreater(m.revision,before)
        self.assertIsNone(m.observed_cell((math.ceil(5./m.settings.slice_dh),0,0)))

    def test_late_noise_endpoint_changes_update_even_a_frozen_surface(self):
        m=mapping();m.integrate(plane());m.build();before=m.revision
        m.integrate([(0.,0.,.02)]);m.build()
        self.assertEqual(next(m.columns[0,0].values()).height,0.)
        self.assertAlmostEqual(stored(m)['G'],.02,places=6)
        self.assertGreater(m.revision,before)

    def test_wall_column_is_known_blocked_across_the_whole_occupied_span(self):
        m=mapping();m.integrate(plane((0.,3.),missing={(0,0)}))
        m.integrate([(0.,0.,z*.1) for z in range(31)]);m.build()
        for k in range(1,12):
            row=m.observed_cell((k,0,0))
            self.assertTrue(row['solid'])
        self.assertFalse(any(key[1:3]==(0,0) for key in m.frontier_index.nodes))

    def test_footprint_reads_measured_tread_above_a_query_plane_riser(self):
        m=mapping(robot_radius=.15)
        cloud=[(x*.1,y*.1,z) for x in range(-4,5) for y in range(-4,5)
               for z in ((.235,3.) if x<=0 else (0.,.37,3.))]
        m.integrate(cloud);m.build()
        layer=min(m.column_layers[0,0])
        self.assertTrue(m.observed_cell((layer,1,0))['solid'])  # query is below the tread
        support=m.footprint_support(layer,1,0,.235)
        self.assertAlmostEqual(support['G'],.37,places=6)
        self.assertFalse(support['solid'])
        self.assertTrue(m.point_safe_check((0.,0.,.555))['ready'])
        self.assertLessEqual(m.planning_cost((layer,0,0)),20.)
        # A genuine wall with no nearby usable upper support remains blocked.
        m.integrate([(.1,y*.1,z*.1) for y in range(-4,5) for z in range(11)])
        m.build()
        self.assertFalse(m.point_safe_check((0.,0.,.555))['ready'])

    def test_top_candidate_works_above_multiple_occupied_intervals(self):
        m=mapping()
        m.integrate([(0.,0.,z) for z in (0.,1.)],sensor_origin=(0.,0.,1.8),body_height=1.32)
        m.build()
        self.assertEqual(len(m._interval_nodes[0,0]),2)
        self.assertEqual(m._interval_nodes[0,0][1].ceiling,1.)
        self.assertTrue(math.isnan(m._interval_nodes[0,0][max(m.column_layers[0,0])].ceiling))

    def test_internal_hit_below_old_midpoint_rebuilds_the_interval(self):
        m=mapping()
        m.integrate(plane());m.build()
        before=m.revision
        m.integrate([(0.,0.,.3)]);m.build()
        row=stored(m)
        self.assertAlmostEqual(row['G'],.3,places=6)
        self.assertAlmostEqual(row['C'],1.5)
        self.assertAlmostEqual(row['node_z'],.9,places=6)
        self.assertGreater(m.revision,before)
        self.assertFalse(m.point_safe_check((0.,0.,.32))['ready'])

    def test_new_C_replaces_open_bound_and_preserves_ground(self):
        m=mapping();m.integrate([(0.,0.,0.)],sensor_origin=(0.,0.,1.),body_height=.32)
        m.build();before=m.revision
        self.assertTrue(math.isnan(m.cell((1,0,0))['C']))
        m.integrate([(0.,0.,.7)],sensor_origin=(0.,0.,.6),body_height=.32);m.build()
        row=m.cell((1,0,0))
        self.assertEqual(row['G'],0.)
        self.assertAlmostEqual(row['C'],.7,places=6)
        self.assertAlmostEqual(row['node_z'],.35,places=6)
        self.assertGreater(m.revision,before)
        self.assertIsNone(m.observed_cell((math.ceil(.7/m.settings.slice_dh),0,0)))

    def test_clearance_only_growth_commits_geometry_without_changing_G(self):
        m=mapping();m.integrate(plane((0.,)),sensor_origin=(0.,0.,.7),body_height=.32);m.build()
        before=m.revision
        m.integrate([(0.,0.,0.)],sensor_origin=(0.,0.,1.2),body_height=.32);m.build()
        row=m.cell((1,0,0))
        self.assertEqual(row['G'],0.)
        self.assertAlmostEqual(row['C_clear'],1.2)
        self.assertAlmostEqual(row['node_z'],.6)
        self.assertGreater(m.revision,before)

    def test_no_neighbor_ground_is_materialized_or_inferred_from_free_rays(self):
        m=mapping();m.integrate([(0.,0.,0.)],sensor_origin=(.3,0.,.8),body_height=.32);m.build()
        self.assertNotIn((1,0),m.columns)
        self.assertIsNone(m.observed_cell((1,1,0)))
        m.integrate(plane(bound=2,missing={(1,0)}));m.build()
        self.assertIsNone(m.observed_cell((1,1,0)))
        self.assertTrue(any(key[1:3]==(1,0) for key in m.frontier_index.nodes))

    def test_new_surface_resolves_frontiers_across_query_planes_in_one_commit(self):
        m=mapping();m.integrate(plane((.4,2.),missing={(0,0)}));m.build()
        self.assertTrue(any(key[1:3]==(0,0) for key in m.frontier_index.nodes))
        m.integrate([(0.,0.,.4),(0.,0.,2.)]);m.build()
        self.assertFalse(any(key[1:3]==(0,0) for key in m.frontier_index.nodes))
        self.assertEqual(len(m.column_layers[0,0]),1)
        self.assertAlmostEqual(m.observed_cell((math.ceil(1./m.settings.slice_dh),0,0))['G'],.4,places=6)

    def test_repeat_observation_does_not_rebuild_unchanged_intervals(self):
        m=mapping();cloud=plane();m.integrate(cloud);m.build();revision=m.revision
        m.integrate(cloud);m.build()
        self.assertEqual(m.revision,revision)
        self.assertEqual(m.last_work['geometry_update_cells'],0)

    def test_upper_hit_cannot_promote_an_old_underside_to_ground(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=mapping(native)
                m.integrate([(0.,0.,1.46),(0.,0.,2.87)],
                            sensor_origin=(0.,0.,1.8),body_height=1.78)
                m.build()
                self.assertFalse(m.frontier_observed(13,(0,0)))
                m.integrate([(0.,0.,4.36)],sensor_origin=(0.,0.,3.3),body_height=3.28)
                m.build()
                self.assertFalse(m.frontier_observed(13,(0,0)))
                self.assertEqual(m.stats['pending_ground_intervals'],1)
                self.assertTrue(any(abs(g-2.87)<1e-6 for g,_,_ in m._pending_ground[0,0]))

    def test_free_rays_over_old_underside_do_not_confirm_its_support(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=mapping(native)
                m.integrate(plane((2.96,4.36),bound=2,missing={(0,0)}));m.build()
                m.integrate([(0.,0.,2.87)],sensor_origin=(0.,0.,2.),body_height=1.98)
                m.integrate([(0.,0.,4.36)],sensor_origin=(0.,0.,3.3),body_height=3.28)
                # This real ray traverses the centre column, but its return
                # belongs to the next XY. Headroom cannot supply a G hit.
                m.integrate([(.051,0.,2.87)],sensor_origin=(0.,0.,3.7),body_height=3.28)
                m.build()
                self.assertGreaterEqual(m._open_clearance((0,0),2.87)-2.87,.5)
                self.assertIsNone(m.observed_cell((13,0,0)))
                self.assertFalse(m._interval_nodes[0,0])
                self.assertIsNone(m.columns[0,0].surfaces[0].support_hit_z)
                self.assertTrue(any(key[0]==13 and key[1:3]==(0,0)
                                    for key in m.frontier_index.nodes))
                before=m.revision
                m.integrate([(0.,0.,2.96)],sensor_origin=(0.,0.,3.7),body_height=3.28)
                m.build()
                node=next(iter(m._interval_nodes[0,0].values()))
                self.assertAlmostEqual(node.ground,2.96,places=6)
                self.assertAlmostEqual(node.midpoint,3.66,places=6)
                self.assertGreater(m.revision,before)
                self.assertFalse(any(key[1:3]==(0,0) for key in m.frontier_index.nodes))

    def test_direct_hit_can_confirm_same_frozen_surface_without_changing_G_C(self):
        m=mapping()
        m.integrate([(0.,0.,1.)],sensor_origin=(0.,0.,.5),body_height=.32)
        m.integrate([(0.,0.,2.)],sensor_origin=(0.,0.,1.8),body_height=1.32)
        m.integrate([(.051,0.,1.)],sensor_origin=(0.,0.,1.8),body_height=1.32)
        m.build()
        self.assertFalse(m._interval_nodes[0,0])
        record=m.columns[0,0].surfaces[0]
        self.assertTrue(record.mature)
        self.assertIsNone(record.support_hit_z)
        before=m.revision
        m.integrate([(0.,0.,1.)],sensor_origin=(0.,0.,1.8),body_height=1.32)
        m.build()
        self.assertEqual(record.height,1.)
        self.assertEqual(record.support_hit_z,1.)
        self.assertEqual(record.orientation,1)
        self.assertEqual(stored(m)['C'],2.)
        self.assertGreater(m.revision,before)

    def test_open_exception_needs_direct_ground_hit_as_well_as_free_headroom(self):
        m=mapping()
        m.integrate([(0.,0.,1.)],sensor_origin=(0.,0.,.5),body_height=.32)
        # Seeing another surface in the same XY from above makes the pose
        # eligible, but cannot turn the untouched old top into support.
        m.integrate([(0.,0.,0.)],sensor_origin=(1.,0.,1.8),body_height=1.32)
        m.integrate([(.051,0.,1.)],sensor_origin=(0.,0.,1.8),body_height=1.32)
        m.build()
        self.assertGreaterEqual(m._open_clearance((0,0),1.)-1.,.5)
        self.assertFalse(any(n.ground==1. for n in m._interval_nodes[0,0].values()))
        self.assertIn((0,0),m._pending_open)
        m.integrate([(0.,0.,1.)],sensor_origin=(0.,0.,1.8),body_height=1.32)
        m.build()
        row=m.cell((5,0,0))
        self.assertEqual(row['G'],1.)
        self.assertTrue(math.isnan(row['C']))
        self.assertAlmostEqual(row['C_clear'],1.8)
        self.assertAlmostEqual(row['node_z'],1.4)

    def test_direct_support_evidence_survives_capacity_staging_clone(self):
        m=mapping(max_surface_records=2)
        m.integrate([(0.,0.,0.),(0.,0.,1.)],sensor_origin=(0.,0.,.8),body_height=.32)
        m.build()
        # At capacity, fusion stages a clone of the existing column.
        m.integrate([(0.,0.,0.)],sensor_origin=(0.,0.,.8),body_height=.32)
        m.build()
        self.assertEqual(m.columns[0,0].surfaces[0].support_hit_z,0.)
        self.assertEqual(stored(m)['G'],0.)

    def test_oblique_direct_hit_with_measured_C_admits_finite_ground(self):
        m=mapping()
        # Finite intervals use both measured endpoints and a direct lower
        # hit, not the open-ceiling requirement for a full free-ray column.
        m.integrate([(0.,0.,2.96),(0.,0.,4.36)],
                    sensor_origin=(1.,0.,3.3),body_height=3.28)
        m.build()
        clear=m._open_clearance((0,0),2.96)
        self.assertTrue(not math.isfinite(clear) or clear-2.96<.5)
        row=m.observed_cell((13,0,0))
        self.assertAlmostEqual(row['G'],2.96,places=6)
        self.assertAlmostEqual(row['C'],4.36,places=6)
        self.assertAlmostEqual(row['node_z'],3.66,places=6)
        self.assertNotIn((0,0),m._pending_ground)

    def test_confirmed_ground_survives_new_C_without_new_lower_rays(self):
        m=mapping()
        m.integrate([(0.,0.,0.)],sensor_origin=(0.,0.,.8),body_height=.32)
        m.build()
        old_id=int(stored(m)['g_surface_id'])
        m.integrate([(0.,0.,3.)],sensor_origin=(1.,0.,2.8),body_height=2.78)
        m.build()
        self.assertEqual(int(stored(m)['g_surface_id']),old_id)
        self.assertAlmostEqual(stored(m)['C'],3.)
        self.assertNotIn((0,0),m._pending_ground)
        self.assertAlmostEqual(m.observed_cell((5,0,0))['G'],0.)

    def test_open_interval_cannot_bridge_disconnected_free_evidence(self):
        m=mapping()
        m.integrate([(0.,0.,2.96)],
                    sensor_origin=(1.,0.,3.3),body_height=3.28)
        m.integrate([],sensor_origin=(0.,0.,3.2),free_rays=[(0.,0.,2.96)])
        m.integrate([],sensor_origin=(0.,0.,3.7),free_rays=[(0.,0.,4.)])
        m.build()
        self.assertFalse(m.frontier_observed(13,(0,0)))
        self.assertIn((0,0),m._pending_open)
        m.integrate([],sensor_origin=(0.,0.,3.2),free_rays=[(0.,0.,3.7)])
        m.build()
        self.assertTrue(m.frontier_observed(13,(0,0)))
        self.assertNotIn((0,0),m._pending_open)

    def test_pending_ground_preserves_frontier_until_new_interior_hit_confirms_it(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=mapping(native)
                # Explicit finite-surface import establishes the surrounding
                # true third-floor supports, but leaves the centre unobserved.
                m.integrate(plane((2.96,4.36),bound=2,missing={(0,0)}));m.build()
                m.integrate([(0.,0.,2.87)],sensor_origin=(0.,0.,2.),body_height=1.98)
                m.integrate([(0.,0.,4.36)],sensor_origin=(0.,0.,3.3),body_height=3.28)
                m.build()
                self.assertFalse(m.frontier_observed(13,(0,0)))
                self.assertTrue(any(key[0]==13 and key[1:3]==(0,0)
                                    for key in m.frontier_index.nodes))
                # 2.96 lies inside the old (2.87,4.36) candidate. It must
                # update/merge the occupied span instead of being discarded.
                m.integrate([(0.,0.,2.96)],sensor_origin=(0.,0.,3.7),body_height=3.28)
                m.build()
                self.assertAlmostEqual(m.observed_cell((13,0,0))['G'],2.96,places=6)
                self.assertFalse(any(key[1:3]==(0,0) for key in m.frontier_index.nodes))
                self.assertNotIn((0,0),m._pending_ground)

    def test_prelabelled_import_does_not_confirm_other_online_endpoints(self):
        m=mapping()
        m.integrate([(0.,0.,2.87)],sensor_origin=(0.,0.,2.),body_height=1.98)
        m.build()
        m.integrate([(0.,0.,1.46)])  # import only this lower support
        m.integrate([(0.,0.,4.36)],sensor_origin=(1.,0.,3.3),body_height=3.28)
        m.build()
        self.assertFalse(m.frontier_observed(13,(0,0)))

    def test_explicit_import_can_confirm_pending_support_without_changing_heights(self):
        m=mapping()
        m.integrate([(0.,0.,2.96),(0.,0.,4.36)],
                    sensor_origin=(1.,0.,2.5),body_height=2.48)
        m.build()
        self.assertFalse(m.frontier_observed(13,(0,0)))
        before=m.revision
        m.integrate([(0.,0.,2.96)])  # explicitly prelabelled offline support
        m.build()
        self.assertTrue(m.frontier_observed(13,(0,0)))
        self.assertGreater(m.revision,before)

    def test_interior_hit_revokes_old_shared_knownness_until_new_support_is_confirmed(self):
        m=mapping()
        m.integrate([(0.,0.,0.),(0.,0.,3.)])  # finite prelabelled import
        m.build()
        self.assertTrue(m.frontier_observed(5,(0,0)))
        m.integrate([(0.,0.,1.)],sensor_origin=(1.,0.,.8),body_height=.78)
        m.build()
        # The old interval must split even though the hit was below its
        # midpoint. Its valid lower support is kept; its upper alias is not.
        self.assertAlmostEqual(stored(m)['C'],1.)
        self.assertFalse(m.frontier_observed(5,(0,0)))
        self.assertEqual(m.stats['pending_ground_intervals'],1)
        m.integrate([(0.,0.,1.)],sensor_origin=(0.,0.,1.8),body_height=1.32)
        m.build()
        self.assertAlmostEqual(m.observed_cell((5,0,0))['G'],1.)
        self.assertEqual(m.stats['pending_ground_intervals'],0)


class BlockerNavigationEvidenceTest(unittest.TestCase):
    def test_thin_surface_at_discrete_cut_is_not_a_known_wall(self):
        m=mapping()
        m.integrate([(0.,0.,2.94),(0.,0.,3.04)],sensor_origin=(1.,0.,2.7),body_height=2.7);m.build()
        point=(0.,0.,2.956)
        # This was the mismatched obstacle query in the recovery path.
        cut=m.surface_observation.first_ground_slice(point[2])
        self.assertIsNotNone(m.obstacle_at((cut,0,0)))
        status=m.frontier_index.observation_model.blocker_review._target_status(point)
        self.assertFalse(status['resolved'])
        self.assertEqual(status['resolution_reason'],'missing_ground_or_headroom')

    def test_unconfirmed_ground_cannot_settle_a_parking_requirement(self):
        from unittest.mock import patch
        m=mapping();m.integrate(plane());m.build()
        b=m.frontier_index.observation_model.blocker_review
        self.assertTrue(b._target_status((0.,0.,0.))['resolved'])
        with patch.object(m.surface_observation,'ground_observed',return_value=False):
            self.assertFalse(b._target_status((0.,0.,0.))['resolved'])

    def test_known_wall_is_not_a_missing_support_in_any_recovery_entry(self):
        m=mapping()
        m.integrate([(0.,0.,z) for z in (.1,2.8)],sensor_origin=(1.,0.,.7));m.build()
        m.integrate([(0.,0.,i*.05) for i in range(7,45)],sensor_origin=(1.,0.,.7));m.build()
        point=(0.,0.,.22)
        b=m.frontier_index.observation_model.blocker_review
        self.assertEqual(b._target_status(point)['resolution_reason'],'known_obstacle')
        self.assertFalse(b.available(point))
        self.assertFalse(b.begin([point])['ready'])
        for layer in (1,2,3):self.assertEqual(list(m.unknown_blockers(layer,[(0,0)],.22)),[])

    def test_obstacle_known_before_arm_settles_without_a_new_target_return(self):
        m=mapping();m.integrate([(1.,0.,.1)],sensor_origin=(1.,0.,.7));m.build()
        b=m.frontier_index.observation_model.blocker_review
        self.assertTrue(b.begin([(0.,0.,.22)])['ready'])
        m.integrate([(0.,0.,i*.05) for i in range(7,45)],sensor_origin=(1.,0.,.7));m.build()
        # It becomes known after selection, before a turn or a fresh matching hit.
        early=b.progress()
        self.assertTrue(early['ready']);self.assertFalse(early['fresh']);self.assertEqual(early['gain'],0)
        b.arm()
        settled=b.finish()
        self.assertTrue(settled['ready']);self.assertEqual(settled['observed_count'],0)
        self.assertEqual(settled['gain'],0);self.assertEqual(settled['known_obstacle_count'],1)
        self.assertFalse(b.available((0.,0.,.22)))

    def test_matching_return_without_open_clearance_does_not_consume_gap(self):
        m=mapping();m.integrate([(0.,0.,1.)],sensor_origin=(1.,0.,1.3),body_height=1.32);m.build()
        b=m.frontier_index.observation_model.blocker_review
        b.begin([(0.,0.,1.)]);b.arm()
        m.integrate([(0.,0.,1.)],sensor_origin=(1.,0.,1.3),body_height=1.32);m.build()
        p=b.progress()
        self.assertEqual(p['matched_count'],1)
        self.assertEqual(p['observed_count'],1)
        self.assertEqual(p['gain'],0)
        self.assertFalse(p['ready'])
        self.assertEqual(p['surface_evidence'][0]['resolution_reason'],'missing_continuous_clearance')
        self.assertFalse(b.finish()['ready'])
        self.assertTrue(b.available((0.,0.,1.)))
        self.assertTrue(b.begin([(0.,0.,1.)])['ready'])

    def test_new_measured_C_resolves_gap_and_only_retires_its_own_height(self):
        m=mapping();m.integrate([(0.,0.,1.)],sensor_origin=(1.,0.,1.3),body_height=1.32);m.build()
        b=m.frontier_index.observation_model.blocker_review
        b.begin([(0.,0.,1.)]);b.arm()
        m.integrate([(0.,0.,2.)],sensor_origin=(1.,0.,1.3),body_height=1.32);m.build()
        p=b.finish()
        self.assertTrue(p['ready']);self.assertEqual(p['resolved_count'],1)
        self.assertEqual(p['gain'],1)
        self.assertFalse(b.available((0.,0.,1.)))
        self.assertTrue(b.available((0.,0.,2.)))  # a different surface is independent

    def test_free_only_update_can_resolve_an_already_hit_open_support(self):
        m=mapping();m.integrate([(0.,0.,1.)],sensor_origin=(1.,0.,1.3),body_height=1.32);m.build()
        b=m.frontier_index.observation_model.blocker_review
        b.begin([(0.,0.,1.)]);b.arm()
        m.integrate([],sensor_origin=(0.,0.,1.8),free_rays=[(0.,0.,1.)]);m.build()
        p=b.finish()
        self.assertTrue(p['ready']);self.assertEqual(p['gain'],1)
        self.assertFalse(b.available((0.,0.,1.)))

    def test_support_hit_missing_is_not_resolved_by_an_upper_return(self):
        m=mapping();m.integrate([(0.,0.,1.)],sensor_origin=(0.,0.,.5),body_height=.32);m.build()
        b=m.frontier_index.observation_model.blocker_review
        b.begin([(0.,0.,1.)]);b.arm()
        m.integrate([(0.,0.,2.)],sensor_origin=(0.,0.,1.8),body_height=1.32);m.build()
        p=b.finish()
        self.assertFalse(p['ready']);self.assertEqual(p['gain'],0)
        self.assertEqual(p['surface_evidence'][0]['resolution_reason'],'missing_support_hit')
        self.assertTrue(b.available((0.,0.,1.)))


class RayEvidenceTest(unittest.TestCase):
    def test_native_matches_reference_and_never_clears_hit_voxel(self):
        from _legbot_layer_kernels import trace_free_columns as native
        points=np.asarray([(0.,0.,0.),(.4,.2,.1),(-.2,-.3,1.1),(.0,0.,9.)],dtype=np.float32)
        args=(points,(0.,0.,.8),.1,0.,0.,0.,1.,10000)
        actual=native(*args)
        self.assertEqual(actual,trace_free_columns(*args))
        vertical=native(np.asarray([(0.,0.,0.)],dtype=np.float32),(0.,0.,.8),.1,0.,0.,0.,5.,1000)
        self.assertEqual(vertical,{(0,0):[(1,7)]})
        with self.assertRaises((ValueError,RuntimeError)):
            native(*args[:-1],1)


class IntervalPlanningTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from legbot_exploration.online_pct import OnlinePct
        root=Path(__file__).resolve().parents[3]/'install/pct_planner/share/pct_planner'
        cls.bootstrap=OnlinePct(str(root))

    def test_online_adapter_uses_interval_map_and_plans_across_unequal_centres(self):
        from legbot_exploration.online_pct import OnlinePct
        root=Path(__file__).resolve().parents[3]/'install/pct_planner/share/pct_planner'
        adapter=OnlinePct(str(root));adapter.configure(dict(robot_radius=.05,safety_margin=0.,
            maturity_min_points=1,traversability_kernel_size=3,inflation_kernel_size=3))
        cloud=np.asarray([(i*.1,j*.1,z) for i in range(-5,6) for j in range(-5,6)
                          for z in (0.,1.2 if i<0 else 2.)],dtype=np.float32)
        adapter.integrate(cloud.tobytes());adapter.prepare(b'',1)
        self.assertIsInstance(adapter.map,IntervalLayerMap)
        self.assertAlmostEqual(adapter.map.cell((1,-2,0))['node_z'],.6,places=6)
        self.assertAlmostEqual(adapter.map.cell((1,2,0))['node_z'],1.)
        result=adapter.plan(b'',1,(-.2,0.,.32),(.2,0.,.32))
        self.assertEqual(result[0],'ok',result)

    def test_native_route_uses_known_tread_above_lower_cut_obstacle(self):
        from legbot_exploration.online_pct import OnlinePct
        root=Path(__file__).resolve().parents[3]/'install/pct_planner/share/pct_planner'
        a=OnlinePct(str(root));a.configure(dict(robot_radius=.15,safety_margin=0.,
            maturity_min_points=1,traversability_kernel_size=3,inflation_kernel_size=3))
        cloud=np.asarray([(x*.1,y*.1,z) for x in range(-6,7) for y in range(-4,5)
                          for z in ((.235,3.) if x<=0 else (0.,.37,3.))],dtype=np.float32)
        a.integrate(cloud.tobytes());a.prepare(b'',1)
        result=a.plan(b'',1,(-.2,0.,.555),(.2,0.,.69))
        self.assertEqual(result[0],'ok',result)

    def test_oblique_stair_returns_with_finite_ceiling_allow_next_tread(self):
        from legbot_exploration.online_pct import OnlinePct
        root=Path(__file__).resolve().parents[3]/'install/pct_planner/share/pct_planner'
        a=OnlinePct(str(root));a.configure(dict(robot_radius=.15,safety_margin=0.,
            maturity_min_points=1,traversability_kernel_size=3,inflation_kernel_size=3))
        # Same scale as the stalled final treads of the second flight.
        cloud=np.asarray([(7.+x*.1,3.6+y*.1,z) for x in range(-5,6) for y in range(-5,16)
                          for z in ((2.69,4.25) if y<6 else (1.415,2.777,2.829,4.25))],dtype=np.float32)
        a.integrate(cloud.tobytes(),sensor_origin=(7.,3.6,3.35),body_height=3.01);a.prepare(b'',1)
        self.assertAlmostEqual(a.map.observed_cell((12,70,47))['G'],2.829,places=5)
        result=a.plan(b'',1,(7.,3.6,3.01),(7.,4.8,3.149))
        self.assertEqual(result[0],'ok',result)

    def test_blind_disk_does_not_relabel_separately_observed_ground_as_unknown(self):
        from legbot_exploration.online_pct import OnlinePct
        root=Path(__file__).resolve().parents[3]/'install/pct_planner/share/pct_planner'
        a=OnlinePct(str(root));a.configure(dict(robot_radius=.1,safety_margin=0.,
            maturity_min_points=1,traversability_kernel_size=3,inflation_kernel_size=3))
        a.integrate(plane(bound=10,missing={(0,y) for y in range(-10,11)}).tobytes())
        a.prepare(b'',1)
        search=a.map.frontier_index.observation_model.pose_search
        _,layers,check=search._arrival_stop((-.4,0.,.32),.1)
        self.assertTrue(check['ready'],check)
        area={(x,0) for x in range(-4,5)}
        coverage,_=search._unobserved_area((-.4,0.,.32),layers,area)
        self.assertTrue(coverage)
        self.assertEqual(coverage[0][1],{(0,0)})
        # The missing stripe is still forbidden inside a real stopping disk.
        self.assertFalse(search._arrival_stop((-.1,0.,.32),.2)[2]['ready'])

    def test_native_edges_require_common_vertical_clearance(self):
        # Both nodes individually have enough clearance and a 10 cm step.
        # Their common free interval must still fit the required robot height.
        from lib import a_star
        planner=a_star.Astar();planner.set_surface_step_limit(.16)
        def run(first_ceiling):
            reader=lambda k,x,y:(0.,0. if y==0 else .1,0.,1,
                                 first_ceiling if y==0 else 1.5,.25 if y==0 else .8,0)
            planner.init_interval_accessor(20.,1,1,2,.1,1.,reader)
            planner.set_passage_clearance(.5)
            return planner.search(np.array([0,0,0],dtype=np.int32),np.array([0,1,0],dtype=np.int32))
        self.assertFalse(run(.5))
        self.assertTrue(run(.7))

    def test_query_aliases_search_and_export_only_the_physical_owner(self):
        from lib import a_star
        planner=a_star.Astar();planner.set_surface_step_limit(.16)
        # Four query addresses all reference the same real interval node.
        planner.init_interval_accessor(20.,4,1,5,.1,1.,
            lambda k,x,y:(0.,0.,0.,1,2.,1.,0))
        planner.set_passage_clearance(.5)
        self.assertTrue(planner.search(np.array([3,0,0],dtype=np.int32),
                                       np.array([2,4,0],dtype=np.int32)))
        route=planner.get_result_matrix()
        self.assertTrue((route[:,0]==0).all())
        self.assertLessEqual(planner.get_search_node_count(),7)


if __name__=='__main__':unittest.main()
