"""Physical surface identity, height epochs and bounded observation proofs."""
import unittest
from unittest.mock import patch
import numpy as np

from legbot_exploration.fixed_layer_map import FixedLayerMap, MapSettings


class SurfaceObservationTest(unittest.TestCase):
    def setUp(self):
        self.map=FixedLayerMap(MapSettings(maturity_min_points=1,allow_open_ceiling=True,origin_z=1e-5),update_workers=1,native_updates=False)
        self.evidence=self.map.surface_observation

    def frame(self, points, origin=None):
        self.map.integrate(np.asarray(points,dtype='float32'),sensor_origin=origin)
        self.map.build()

    def test_known_step_resolves_expectation_and_reopens_for_overlapping_sheets(self):
        points=[(x*.1,y*.1,0. if y<=0 else .13)
                for x in range(-4,5) for y in range(-4,5) if (x,y)!=(1,0)]
        self.frame(points)
        with patch('legbot_exploration.surface_orientation.classify',return_value=np.full(len(points),-1)):
            self.frame([(x,y,2.) for x,y,_ in points],origin=(0.,0.,.8))
        record=self.evidence.expect((0.,0.,.13),slices=[1],review=True)
        self.assertEqual(record.state,'pending')
        self.assertFalse(self.evidence.ground_observed((0,0),0.,1))
        with patch.object(self.map,'iter_cells',side_effect=AssertionError('whole-map scan')):
            self.frame([(.1,0.,0.)])
        self.assertEqual(record.state,'step_boundary')
        self.assertTrue(self.evidence.ground_observed((0,0),0.,1))
        self.assertEqual(self.evidence.query((0.,0.,.13),expectation=record)['state'],'refuted')
        self.assertAlmostEqual(self.map.cell((0,0,0))['G'],0.)
        self.assertFalse(any(abs(self.map.frontier_index.nodes[k].height-.13)<1e-5
                             for k in self.map.frontier_index.by_column.get((0,0),())))
        self.frame([(0.,.1,0.)])
        self.assertEqual(record.state,'pending')
        self.assertEqual(self.evidence.query((0.,0.,.13),expectation=record)['state'],'unresolved')
        self.assertTrue(self.evidence.matching((0,1),0.))
        self.assertTrue(self.evidence.matching((0,1),.13))

    def test_new_height_does_not_invent_upper_tread_over_a_known_red_lower_tread(self):
        self.frame([(x*.1,y*.1,0. if y<=0 else .13)
                    for x in range(-7,8) for y in range(-7,8)])
        for _,row in self.map.layers_at(0,0):
            row['relief_cost']=row['terrain_cost']=row['inflated_terrain_cost']=50.
        self.evidence.visit_height((0.,-.2,0.))
        with patch.object(self.map,'iter_cells',side_effect=AssertionError('whole-map scan')):
            self.assertTrue(self.evidence.visit_height((0.,.2,.13))['triggered'])
        self.assertFalse(any(r.xy==(0,0) and abs(r.height-.13)<1e-5 for r in self.evidence.records.values()))
        self.assertTrue(all(row['inflated_terrain_cost']==50 for _,row in self.map.layers_at(0,0)))
        self.assertFalse(any(abs(z-.13)<1e-5 and abs(y)<1e-5
                             for _,y,z,_ in self.map.unknown_blockers(1,[(0,0)],.13)))

    def test_frontier_coverage_settles_inferred_work_without_confirming_a_blocker(self):
        point=(0.,0.,1.425)
        inferred=self.evidence.expect(point,slices=[3],review=True,neighbor_inferred=True)
        frontier=self.evidence.expect(point,slices=[3],kind='frontier')
        blocker=self.evidence.expect(point,slices=[3],review=True)
        self.assertIsNot(inferred,blocker)
        self.frame([(0.,0.,1.315),(0.,0.,2.1)])
        self.frame([(0.,0.,1.315),(0.,0.,2.1)])
        self.assertEqual(self.evidence.query(point,expectation=inferred)['state'],'frontier_covered')
        self.assertEqual(self.evidence.query(point,expectation=frontier)['state'],'frontier_covered')
        self.assertEqual(blocker.state,'pending')
        self.assertFalse(self.evidence.matching((0,0),point[2]))
        self.assertFalse(self.evidence.refuted((0,0),point[2]))
        self.assertTrue(self.evidence.query(point,kind='frontier')['ready'])
        self.assertFalse(self.evidence.query(point,expectation=blocker)['ready'])

    def test_partial_clearance_evidence_does_not_invent_a_full_column_wall(self):
        self.frame([(.6,0.,z) for z in (0.,.3,1.5)])
        ceiling=next(r for r in self.map.columns[6,0].values() if abs(r.height-.3)<.01)
        ceiling.orientation=-1
        self.map.dirty.add((6,0));self.map.surface_relations.invalidate({(6,0)})
        self.map.build()
        self.evidence.height_serial=2
        relation=self.map.surface_relations
        self.frame([(.6,0.,0.)])
        with patch('legbot_exploration.surface_orientation.classify',return_value=np.array([-1])):
            self.frame([(.6,0.,.3)],origin=(0.,0.,.1))
        self.assertGreaterEqual(self.map.observed_cell((0,6,0))['passage_cost'],50.)
        self.assertAlmostEqual(self.map.observed_cell((1,6,0))['G'],.3,places=5)
        self.assertIsNone(relation.fresh_clearance_barrier((6,0),0.),
                          'a cut outside the measured interval remains unknown')
        # Red terrain is not extra physical evidence or a finite G/C interval.
        for _,row in self.map.layers_at(6,0):
            row['terrain_cost']=row['inflated_terrain_cost']=50.
        self.assertIsNone(relation.fresh_clearance_barrier((6,0),0.))

    @staticmethod
    def rays(z=0.):
        # From a single sensor above the cell, these rays cover all 16
        # subcells on each of the three expected-height planes around z=1.
        return [(x,y,z) for x in (-.075,-.025,.025,.075) for y in (-.075,-.025,.025,.075)]

    def test_expectation_never_creates_a_ground_or_propagates_itself(self):
        self.frame([(0.,0.,1.45)])
        before={(k,x,y):float(r['G']) for (k,x,y),r in self.map.iter_cells(True)}
        record=self.evidence.expect((0.,0.,2.98),sources=[(1,0,1)],slices=[6])
        self.assertEqual(record.state,'pending')
        self.assertEqual(self.evidence.query((0.,0.,2.98))['state'],'unresolved')
        self.assertEqual(before,{(k,x,y):float(r['G']) for (k,x,y),r in self.map.iter_cells(True)})
        self.assertEqual(len(self.evidence.records),1)

    def test_fresh_lower_return_is_separate_from_high_surface_matching(self):
        self.frame([(0.,0.,1.45)])
        record=self.evidence.expect((0.,0.,2.98),review=True)
        self.frame([(0.,0.,1.45)])
        state=self.evidence.query((0.,0.,2.98),after=record.serial)
        self.assertEqual(state['state'],'unresolved')
        self.assertTrue(state['lower_observed'])
        self.assertFalse(state['surfaces'])
        self.assertTrue(self.evidence.matching((0,0),1.45,after=record.serial))

    def test_low_return_does_not_overwrite_an_existing_higher_ground(self):
        self.frame([(0.,0.,1.45),(0.,0.,2.98)])
        source=dict(self.map.layers_at(0,0))[6]['g_surface_id']
        self.frame([(0.,0.,1.45)])
        row=dict(self.map.layers_at(0,0))[6]
        self.assertAlmostEqual(row['G'],2.98,places=5)
        self.assertEqual(row['g_surface_id'],source)

    def test_one_point_can_supply_a_different_actual_ground_without_matching_prediction(self):
        self.frame([(0.,0.,1.45)])
        record=self.evidence.expect((0.,0.,3.10),review=True)
        self.frame([(0.,0.,2.97)])
        self.assertAlmostEqual(dict(self.map.layers_at(0,0))[6]['G'],2.97,places=5)
        self.assertEqual(self.evidence.query((0.,0.,3.10),after=record.serial)['state'],'unresolved')
        self.assertTrue(self.evidence.query((0.,0.,2.97),after=record.serial)['ready'])

    def test_static_matching_evidence_is_reusable_without_a_new_frame(self):
        self.frame([(0.,0.,2.98)])
        record=self.evidence.expect((0.,0.,2.98))
        self.assertEqual(record.state,'matched')
        self.assertEqual(self.evidence.query((0.,0.,2.98))['state'],'matched')
        self.assertFalse(self.evidence.query((0.,0.,2.98),after=self.map.integrated_serial)['ready'])

    def test_distinct_height_and_source_hypotheses_remain_separate(self):
        low=self.evidence.expect((0.,0.,1.45),sources=[(1,0,1)])
        high=self.evidence.expect((0.,0.,2.98),sources=[(1,0,2)])
        other=self.evidence.expect((0.,0.,3.00),sources=[(1,0,3)])
        self.assertEqual(len({low.id,high.id,other.id}),3)
        same=self.evidence.expect((0.,0.,2.99),sources=[(1,0,2)],slices=[7])
        self.assertEqual(same.id,high.id)
        self.assertEqual(high.slices,{7})

    def test_one_ray_and_geometric_field_of_view_cannot_refute_a_whole_cell(self):
        record=self.evidence.expect((0.,0.,1.),review=True)
        self.frame([(.025,.025,0.)],origin=np.array([0.,0.,2.]))
        self.assertEqual(record.state,'pending')
        self.assertFalse(self.evidence.refuted((0,0),1.))

    def test_merging_nearby_heights_widens_proof_and_cannot_chain_across_surfaces(self):
        record=self.evidence.expect((0.,0.,1.),sources=[(1,0,1)],review=True)
        record.free_masks=[0xffff,0xffff,0xffff]
        self.assertIs(self.evidence.expect((0.,0.,1.04),sources=[(1,0,1)],review=True),record)
        self.assertEqual(record.free_masks,[0,0,0])
        self.assertEqual(record.center_range,(1.,1.04))
        other=self.evidence.expect((0.,0.,.96),sources=[(1,0,1)],review=True)
        self.assertNotEqual(other.id,record.id)
        self.frame(self.rays(),origin=np.array([0.,0.,2.]))
        self.assertEqual(record.state,'refuted')
        self.assertTrue(self.evidence.refuted((0,0),1.04))

    def test_query_matches_requested_height_and_settled_proof_cannot_expand(self):
        record=self.evidence.expect((0.,0.,1.),review=True)
        self.evidence.expect((0.,0.,1.04),review=True)
        self.frame([(0.,0.,.96)])
        self.assertEqual(record.state,'pending')
        self.assertEqual(self.evidence.query((0.,0.,1.),expectation=record)['state'],'matched')
        self.assertEqual(self.evidence.query((0.,0.,1.04),expectation=record)['state'],'unresolved')
        self.frame(self.rays(),origin=np.array([0.,0.,2.]))
        self.assertEqual(record.state,'pending', 'free rays cannot erase an existing measured sheet')
        other=self.evidence.expect((0.,0.,2.),review=True)
        self.frame(self.rays(),origin=np.array([0.,0.,4.]))
        self.assertEqual(other.state,'refuted')
        shifted=self.evidence.expect((0.,0.,2.04),review=True)
        self.assertNotEqual(shifted.id,other.id)
        self.assertEqual(shifted.state,'pending')
        self.assertEqual(self.evidence.query((0.,0.,1.10),expectation=record)['state'],'unresolved')

    def test_actual_rays_covering_the_band_can_refute_a_missing_sheet(self):
        record=self.evidence.expect((0.,0.,1.),review=True)
        self.frame(self.rays(),origin=np.array([0.,0.,2.]))
        self.assertEqual(record.state,'refuted')
        self.assertTrue(self.evidence.refuted((0,0),1.))
        self.assertTrue(all(mask==0xffff for mask in record.free_masks))
        self.assertFalse(self.evidence.matching((0,0),1.))
        self.assertFalse(self.evidence.refuted((0,0),1.04), 'a shifted height band needs its own coverage')

    def test_a_return_before_the_expected_sheet_does_not_clear_behind_it(self):
        record=self.evidence.expect((0.,0.,1.),review=True)
        self.frame([(.025,.025,1.4)],origin=np.array([0.,0.,2.]))
        self.assertEqual(record.state,'pending')
        self.assertFalse(any(record.free_masks))

    def test_full_higher_surface_supersedes_hypothesis_but_never_deletes_real_groups(self):
        self.frame([(0.,0.,0.)])
        record=self.evidence.expect((0.,0.,1.),review=True)
        points=[(x,y,1.13) for x in (-.0375,-.0125,.0125,.0375) for y in (-.0375,-.0125,.0125,.0375)]
        self.frame(points,origin=np.array([0.,0.,2.]))
        self.assertEqual(record.state,'superseded')
        self.assertTrue(self.evidence.matching((0,0),0.))
        self.assertTrue(self.evidence.matching((0,0),1.13))

    def test_repeated_review_registration_does_not_rearm_a_completed_physical_target(self):
        record=self.evidence.expect((0.,0.,1.),review=True)
        self.frame([(0.,0.,1.)])
        serial=record.serial
        self.assertEqual(record.state,'matched')
        self.frame([(0.,0.,1.)])
        self.assertIs(self.evidence.expect((0.,0.,1.),review=True),record)
        self.assertEqual(record.state,'matched')
        self.assertEqual(record.serial,serial)

    def test_next_step_is_a_search_band_not_an_asserted_surface_height(self):
        record=self.evidence.expect((0.,0.,.105),kind='search',height_range=(.05,.16),review=True)
        self.frame([(0.,0.,.13)])
        check=self.evidence.query((0.,0.,.105),after=record.serial,kind='search',expectation=record)
        self.assertEqual(check['state'],'matched')
        self.assertAlmostEqual(check['surfaces'][0]['height'],.13,places=5)
        self.assertTrue(self.evidence.matching((0,0),.105,kind='geometry'))
        self.assertTrue(all(abs(row.height-.13)<1e-5 for row in self.map.columns[0,0].values()))

    def test_pending_upper_surface_does_not_invalidate_a_different_floor(self):
        self.frame([(0.,0.,1.45)])
        self.evidence.expect((0.,0.,2.98),review=True)
        self.assertTrue(self.evidence.ground_observed((0,0),1.45))

    def test_pending_nearby_high_surface_cannot_borrow_a_stale_low_return(self):
        self.frame([(0.,0.,1.)])
        self.evidence.expect((0.,0.,1.13),review=True)
        geometry=self.map.revision;revision=self.evidence.revision
        self.assertFalse(self.evidence.ground_observed((0,0),1.))
        self.frame([(0.,0.,1.)])
        self.assertTrue(self.evidence.ground_observed((0,0),1.))
        self.assertEqual(self.map.revision,geometry)
        self.assertGreater(self.evidence.revision,revision)
        self.assertEqual(self.evidence.query((0.,0.,1.13))['state'],'unresolved')

    def test_ceiling_hits_do_not_supersede_ground_hypothesis(self):
        record=self.evidence.expect((0.,0.,1.),review=True)
        points=[(x,y,2.5) for x in (-.0375,-.0125,.0125,.0375) for y in (-.0375,-.0125,.0125,.0375)]
        self.frame(points,origin=np.array([0.,0.,2.]))
        self.assertEqual(record.state,'pending')
        self.assertEqual(record.higher_mask,0)

    def test_height_threshold_is_once_per_new_measured_height_and_local(self):
        self.assertFalse(self.evidence.visit_height((0.,0.,0.))['triggered'])
        self.assertFalse(self.evidence.visit_height((0.,0.,.05))['triggered'])
        self.assertFalse(self.evidence.visit_height((0.,0.,.10))['triggered'])
        self.frame([(x*.1,y*.1,.13) for x in range(-5,6) for y in range(-5,6)])
        with patch.object(self.map,'iter_cells',side_effect=AssertionError('global traversal forbidden')):
            result=self.evidence.visit_height((0.,0.,.13))
        self.assertTrue(result['triggered'])
        self.assertEqual(result['registered_targets'],0)
        count=len(self.evidence.records)
        self.assertFalse(self.evidence.visit_height((0.,0.,.13))['triggered'])
        self.assertFalse(self.evidence.visit_height((0.,0.,0.))['triggered'])
        self.assertFalse(self.evidence.visit_height((0.,0.,.13))['triggered'])
        self.assertEqual(len(self.evidence.records),count)
        self.assertTrue(all(np.linalg.norm(np.array(r.xy)*.1)<=.93 for r in self.evidence.records.values()))

    def test_unvisited_lower_and_intermediate_heights_trigger_and_revisits_do_not(self):
        self.assertFalse(self.evidence.visit_height((0.,0.,1.5))['triggered'])
        for height in (3.,2.2,.8):
            self.assertTrue(self.evidence.visit_height((0.,0.,height))['triggered'])
        for height in (1.5,3.,2.2,.8,2.25):
            self.assertFalse(self.evidence.visit_height((0.,0.,height))['triggered'])
        self.assertEqual(self.evidence.visited_heights,[.8,1.5,2.2,3.])
        self.assertEqual(self.evidence.status()['height_triggers'],3)

    def test_new_height_registers_context_without_inferred_targets_or_clearing(self):
        self.frame([(x*.1,y*.1,z) for x in range(-3,4) for y in range(-3,4) for z in (0.,.63)])
        self.evidence.visit_height((0.,0.,0.))
        before={(k,x,y):float(r['G']) for (k,x,y),r in self.map.iter_cells(True)}
        self.assertTrue(self.evidence.visit_height((0.,0.,.63))['triggered'])
        self.assertFalse(self.evidence.records)
        self.assertEqual(before,{(k,x,y):float(r['G']) for (k,x,y),r in self.map.iter_cells(True)})

    def test_high_cut_pending_review_does_not_invalidate_lower_cut_support(self):
        self.frame([(0.,0.,.45),(.1,0.,.9)])
        record=self.evidence.expect((0.,0.,.58),slices=[2],review=True)
        self.assertTrue(self.evidence.ground_observed((0,0),.45,1))
        self.assertFalse(self.evidence.ground_observed((0,0),.45,2))
        self.assertEqual(self.evidence.pending_at((0,0),.45,1),())
        check=self.evidence.describe_ground((0,0),.45,2)
        self.assertEqual(check['reason'],'ground_unobserved')
        self.assertEqual(check['layers'],[])
        self.assertEqual(self.evidence.pending_at((0,0),.45,2),(record,))
        self.frame([(0.,0.,.45)])
        self.assertTrue(self.evidence.ground_observed((0,0),.45,2))

    def test_new_height_review_does_not_stop_lower_cut_cloud_updates(self):
        self.frame([(0.,0.,0.),(.1,0.,.63)])
        self.evidence.visit_height((0.,0.,0.))
        self.assertTrue(self.evidence.visit_height((.1,0.,.63))['triggered'])
        self.frame([(0.,0.,.08)])
        self.assertAlmostEqual(self.map.cell((1,0,0))['G'],.08,places=5)
        self.assertFalse(self.evidence.records)

    def test_diagnostics_distinguish_missing_ground_and_invalid_surface_identity(self):
        self.assertEqual(self.evidence.describe_ground((0,0),1.)['reason'],'ground_unobserved')
        self.frame([(0.,0.,1.)])
        record=next(iter(self.map.columns[0,0].values()))
        record.orientation=-1
        check=self.evidence.describe_ground((0,0),1.)
        self.assertEqual(check['reason'],'ground_source_invalid')
        self.assertEqual(check['layers'][0]['surface_id'],record.id)
        self.assertEqual(check['layers'][0]['orientation'],-1)
        self.assertEqual(check['layers'][0]['last_seen'],self.map.integrated_serial)

    def test_later_frame_can_resolve_frontier_without_changing_frozen_G(self):
        self.check_evidence_only_frontier_update()

    def test_native_frontier_update_uses_the_same_interval_evidence(self):
        self.map=FixedLayerMap(MapSettings(maturity_min_points=1,allow_open_ceiling=True,origin_z=1e-5),update_workers=1)
        if self.map._boundary_kernel is None:
            self.skipTest('native boundary kernel unavailable')
        self.evidence=self.map.surface_observation
        self.check_evidence_only_frontier_update()

    def check_evidence_only_frontier_update(self):
        from legbot_exploration.pct_frontier_index import Boundary
        points=[(x*.1,y*.1,z) for x in range(-3,4) for y in range(-3,4) for z in (.1,1.6)]
        self.frame(points)
        k=self.evidence.first_ground_slice(.1)
        self.assertNotIn((0,0),self.map.layers[k].frontiers.by_column)
        boundary=Boundary(.1,((1,0,.1,1),),k)
        record=self.evidence.expect((0.,0.,.1),kind='frontier',slices=[k])
        version=self.map.revision
        self.frame(points)
        self.assertEqual(record.state,'matched')
        self.assertEqual(self.evidence.query((0.,0.,.1),kind='frontier',layer=k)['state'],'frontier_covered')
        self.assertEqual(self.map.revision,version)
        self.assertEqual(self.map.last_work['geometry_update_cells'],0)
        self.assertNotIn((0,0),self.map.layers[k].frontiers.by_column)
        self.assertEqual(self.map.frontier_index.floor_observation_counts({((0,0),boundary):set()}),(1,1))


if __name__=='__main__':
    unittest.main()
