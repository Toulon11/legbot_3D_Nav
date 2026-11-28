"""Shared frontier/support scheduling and level-area landing, without stair recognition."""
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from legbot_exploration.fixed_layer_map import FixedLayerMap, MapSettings
from legbot_exploration.online_pct import OnlinePct
from legbot_exploration.pct_platform import platform_cells

PCT=Path(__file__).resolve().parents[3]/'install/pct_planner/share/pct_planner'
LIMITS=(.15,.10,.10,False,4,.16,.08)


def adapter(points):
    result=OnlinePct(str(PCT))
    result.configure(dict(robot_radius=0.,safety_margin=0.,maturity_min_points=1,
                          traversability_kernel_size=3,inflation_kernel_size=1))
    # These are offline navigation fixtures, without timestamped LiDAR rays.
    # Bound exposed support columns with an explicit measured upper surface.
    points=list(points);columns={}
    for x,y,z in points:columns.setdefault((x,y),[]).append(z)
    points += [(x,y,max(zs)+1.5) for (x,y),zs in columns.items()
               if max(zs)-min(zs)<.5]
    result.integrate(np.asarray(points,dtype='float32').tobytes())
    result.prepare(b'',1)
    return result


def stepped_ground():
    return [(x*.1,y*.1,0. if x<0 else .13 if x<8 else .26)
            for x in range(-15,16) for y in range(-10,11)]


class DirectAscentTest(unittest.TestCase):
    def test_current_known_intervals_remove_cached_markers_and_finish_resolved_route_work(self):
        a=adapter([(x*.1,y*.1,z) for x in range(-15,16) for y in range(-12,13)
                   if (x,y) not in ((2,0),(3,0)) for z in (.1,2.8)])
        m=a.map;index=m.frontier_index
        key=next(iter(index.by_column[(2,0)]));cid=index.task_id(key)
        task=index.task_cluster(cid);points=task['display']
        self.assertEqual(len(points),2)
        index.floor_query_members[cid]=set(task['members'])
        index.floor_query=((.1,.08),lambda key:True,False,0,2,False)
        index.observation_model.floor_attempt=index.begin_floor_attempt(cid)
        self.assertEqual(len(a.refresh_frontier_displays(1,[(cid,points)])[0]),2)
        for version,x in ((2,.2),(3,.3)):
            a.integrate(np.asarray([(x,0.,z) for z in (.1,2.8)],dtype='float32').tobytes())
            a.prepare(b'',version)
            with patch.object(m,'iter_cells',side_effect=AssertionError('whole map scan')):
                display=a.refresh_frontier_displays(version,[(cid,points)])[0]
                pending=a.observation_task(b'',version,'floor_task_pending',cid,(0.,0.,.42),0.,0.)
            self.assertEqual(len(display),3-version)
            self.assertEqual(pending['ready'],version==2)
        self.assertEqual(pending['reason'],'frontier_resolved_in_transit')
        self.assertIsNone(index.observation_model.floor_attempt)
        self.assertFalse(index.floor_failures or index.floor_blacklist)

    def test_observation_uses_measured_ground_despite_body_height_offset(self):
        a=adapter([(x*.1,y*.1,0.) for x in range(-20,21) for y in range(-20,21)
                   if (x,y)!=(15,0)])
        cid=a.map.frontier_index.task_id(next(iter(a.map.frontier_index.by_column[(15,0)])))
        for dz in (-.06,.06):
            with self.subTest(body_offset=dz):
                result=a.observation_task(b'',1,'begin_flat',cid,(0.,0.,a.reference_height+dz),0.,0.)
                self.assertTrue(result['ready'],result)
                self.assertAlmostEqual(a.map.frontier_index.observation_model.task['point'][2],0.)
        with patch.object(a,'_ground_support',return_value=dict(ready=False)):
            result=a.observation_task(b'',1,'begin_flat',cid,(0.,0.,a.reference_height+.06),0.,0.)
        self.assertFalse(result['ready'])
        self.assertEqual(result['reason'],'arrival_surface_height_mismatch')

    def test_default_missing_ceiling_keeps_one_strict_upper_cut(self):
        m=FixedLayerMap(MapSettings(maturity_min_points=1))
        self.assertTrue(m.settings.allow_open_ceiling)
        m.integrate([(0.,0.,0.)]);m.build()
        self.assertEqual(m.column_layers[0,0],{1})
        self.assertTrue(np.isposinf(m.cell((1,0,0))['C']))
        self.assertIsNotNone(m.observed_cell((2,0,0)))
        self.assertIsNone(m.cell((2,0,0)))
        self.assertFalse(hasattr(m,'stair_index'))
        self.assertFalse(hasattr(m,'stair_candidates'))

    def test_support_query_reads_incremental_cache_and_updates_only_changed_columns(self):
        a=adapter(stepped_ground());m=a.map;index=m.support_targets
        before=index.columns[-10,0]
        with patch.object(m,'iter_cells',side_effect=AssertionError('whole map scan')), \
             patch.object(m,'layers_at',side_effect=AssertionError('rebuild supports during query')):
            candidates=index.candidates((-.7,0.,0.),.08,.16,limit=20)
        self.assertTrue(candidates)
        self.assertTrue(all(abs(entry[3][2]-.13)<1e-5 for entry in candidates))
        with patch('legbot_exploration.surface_orientation.classify',return_value=np.asarray([-1])):
            a.integrate(np.asarray([(.3,0.,.20)],dtype='float32').tobytes(),sensor_origin=(.3,0.,.1))
        a.prepare(b'',2)
        self.assertIs(index.columns[-10,0],before)
        # The new endpoint must not inherit the old tread's confirmation.
        # Below-surface observation alone did not measure its headroom.
        self.assertNotIn((3,0),index.columns)
        with patch('legbot_exploration.surface_orientation.classify',return_value=np.asarray([-1])):
            a.integrate(np.asarray([(.3,0.,.20)],dtype='float32').tobytes(),sensor_origin=(.3,0.,1.))
        a.prepare(b'',3)
        self.assertIs(index.columns[-10,0],before)
        self.assertIn((3,0),index.columns)
        self.assertAlmostEqual(max(r[1] for r in index.columns[3,0]),.20,places=5)

    def test_completed_frontiers_can_continue_on_measured_upward_support(self):
        a=adapter(stepped_ground())
        with patch.object(a.map.frontier_index,'_observers',return_value=[]):
            result=a.frontiers(b'',1,(-.7,0.,.32),.25,query_budget=.20,
                floor=(0.,.08),floor_selection=LIMITS,seek_ascent=True)
        self.assertTrue(result['upward_support_targets'])
        self.assertTrue(all(p[2]>.08 for p in result['upward_support_targets']))
        self.assertTrue(any(p[2]>.16 for p in result['upward_support_targets']))
        self.assertNotIn('stair_flights',result)

    def test_ascent_looks_ahead_but_returns_one_cut_and_farthest_measured_support(self):
        points=[(x*.1,y*.1,.13*max(0,min(8,(x+8)//5)))
                for x in range(-15,36) for y in range(-10,11)]
        a=adapter(points);m=a.map
        result=a.frontiers(b'',1,(-1.,0.,.32),.25,query_budget=.2,
                           floor=(0.,.08),floor_selection=LIMITS,seek_ascent=True)
        layer=result['selection_layer']
        self.assertGreater(layer,m.frontier_selection_layer((-1.,0.,0.))[0])
        self.assertGreater(result['upward_support_targets'][0][2],.5)
        for view in result['upward_support_targets']:
            row=m.observed_cell((layer,*m.indices(view)))
            self.assertAlmostEqual(float(row['G']),view[2])
        for cluster in result['clusters']:
            self.assertTrue(all(key[0]==layer for key in m.frontier_index.floor_query_members[cluster['id']]))
        self.assertEqual(result['map']['ascent_selection_slices']['selected'],layer)
        reason,route,metadata=a.plan(b'',1,(-1.,0.,.32),
                                    tuple(np.asarray(result['upward_support_targets'][0][:3])+[0.,0.,.32]))
        self.assertEqual(reason,'ok',metadata)
        self.assertGreater(route[-1][2]-route[0][2],.5)
        # Returned RDP waypoints may span several legal native graph edges.
        # A genuinely unbroken >16 cm step must still be rejected.
        blocked=adapter([(x*.1,y*.1,0. if x<0 else .30)
                         for x in range(-15,16) for y in range(-10,11)])
        reason,_,_=blocked.plan(b'',1,(-.7,0.,.32),(1.1,0.,.62))
        self.assertNotEqual(reason,'ok')

    def test_rejected_ascent_cut_falls_back_without_mixing_or_immediate_repeat(self):
        a=adapter([(x*.1,y*.1,.13*max(0,min(8,(x+8)//5)))
                   for x in range(-15,36) for y in range(-10,11)])
        seen=[]
        for _ in range(3):
            layer=a._ascent_layer((-1.,0.,0.),.08)
            if layer is None:break
            self.assertNotIn(layer,seen);seen.append(layer)
            a.observation_task(b'',1,'advance_ascent_slice',0,(-1.,0.,.32),0.,0.)
        self.assertGreater(len(seen),1)
        self.assertIsNone(a._ascent_layer((-1.,0.,0.),.08))
        # Ordinary work remains on the valid main cut.
        normal=a.frontiers(b'',1,(-1.,0.,.32),.25,query_budget=.1,
                           floor=(0.,.08),floor_selection=LIMITS)
        self.assertEqual(normal['selection_layer'],a.map.frontier_selection_layer((-1.,0.,0.))[0])

    def test_new_returns_do_not_restart_far_cut_before_nearer_fallback(self):
        a=adapter([(x*.1,y*.1,.13*max(0,min(8,(x+8)//5)))
                   for x in range(-15,36) for y in range(-10,11)])
        far=a._ascent_layer((-1.,0.,0.),.08)
        a.observation_task(b'',1,'advance_ascent_slice',0,(-1.,0.,.32),0.,0.)
        a.map.support_targets.revision+=1
        near=a._ascent_layer((-1.,0.,0.),.08)
        self.assertNotEqual(far,near)
        self.assertIn(far,a._ascent_selection['failed'])

    def test_exhausted_ascent_pass_keeps_failures_and_near_blockers_on_cost_updates(self):
        a=adapter([(x*.1,y*.1,.13*max(0,min(8,(x+8)//5)))
                   for x in range(-15,36) for y in range(-10,11)])
        anchor=(-1.,0.,0.)
        while (layer:=a._ascent_layer(anchor,.08)) is not None:
            a._ascent_selection['blockers'][layer]=[[.2,0.,.26,layer]]
            a.observation_task(b'',1,'advance_ascent_slice',0,(-1.,0.,.32),0.,0.)
        a.map.support_targets.revision+=1
        self.assertIsNone(a._ascent_layer(anchor,.08))
        result=a.frontiers(b'',1,(-1.,0.,.32),.25,query_budget=.1,
            floor=(0.,.08),floor_selection=LIMITS,seek_ascent=True)
        self.assertTrue(result['unknown_blockers'])

    def test_support_search_resumes_past_early_rejections_despite_new_geometry(self):
        a=adapter(stepped_ground())
        seen=[];clock=[0.]
        def now():
            clock[0]+=.004
            return clock[0]
        def reject(body,*args):
            seen.append(tuple(body))
            return dict(ready=False,reason='arrival_unknown_footprint',unknown_blockers=[body],
                        unsafe_cells=0,unobserved_cells=1)
        with patch('legbot_exploration.online_pct.time.monotonic',side_effect=now), \
             patch.object(a,'_arrival_area',side_effect=reject), \
             patch.object(a,'_arrival_geometry',side_effect=lambda *args:(0,0,object())):
            pending=True;batches=0
            while pending and batches<100:
                _,_,pending=a._support_views((-.7,0.,0.),.25,.08,budget=.005,
                                             global_search=True,selection_layer=1)
                batches+=1
                a.map.support_targets.revision+=1
        self.assertFalse(pending)
        self.assertGreater(batches,1)
        self.assertEqual(len(seen),len(set(seen)))
        self.assertEqual(a.metadata['support_search']['checked'],a.metadata['support_search']['candidates'])

    def test_dense_high_surface_pool_keeps_lower_near_supports(self):
        a=adapter([(x*.1,y*.1,.39 if x>=0 else .13)
                   for x in range(-15,20) for y in range(-15,16)])
        rows=a.map.support_targets.candidates((-.7,0.,0.),.08,.5,.25,None,limit=16,prefer_ascent=True)
        heights={round(row[3][2],2) for row in rows}
        self.assertEqual(heights,{.13,.39})
        self.assertLessEqual(len(rows),16)
        self.assertAlmostEqual(rows[0][3][2],.39,places=5)

    def test_map_commit_settles_resolved_floor_task_for_native_handoff_cache(self):
        from types import SimpleNamespace
        a=adapter(stepped_ground());index=a.map.frontier_index
        task=SimpleNamespace(cluster=321,members=((1,0,0),))
        index.observation_model.floor_attempt=task
        with patch.object(index,'remaining_floor_members',return_value=[]), \
             patch.object(index,'settle_floor_attempt') as settle:
            self.assertEqual(a.commit_pending(b'',1),321)
            settle.assert_called_once_with(task,False,sufficient_progress=True)
            self.assertIsNone(index.observation_model.floor_attempt)
            self.assertEqual(a.commit_pending(b'',1),0)
        index.observation_model.floor_attempt=task
        with patch.object(index,'remaining_floor_members',return_value=[(1,0,0)]), \
             patch.object(index,'settle_floor_attempt') as settle:
            self.assertEqual(a.commit_pending(b'',1),0)
            self.assertIs(index.observation_model.floor_attempt,task)
            settle.assert_not_called()

    def test_resolved_navigation_evidence_reopens_failed_cut_search(self):
        a=adapter(stepped_ground());review=a.map.frontier_index.observation_model.blocker_review
        for fresh,gain in ((False,0),(True,0),(True,1)):
            a._ascent_selection={'failed':{1,2,3}}
            a._support_passes={'pass':{'cursor':96}}
            with patch.object(review,'finish',return_value=dict(ready=fresh,fresh=fresh,gain=gain)):
                a.observation_task(b'',1,'finish_blocker_review',0,(-.7,0.,.32),0.,0.)
            if fresh and gain:
                self.assertIsNone(a._ascent_selection);self.assertFalse(a._support_passes)
            else:
                self.assertIsNotNone(a._ascent_selection);self.assertTrue(a._support_passes)

    def test_pending_ascent_frontier_search_does_not_starve_known_support_or_blockers(self):
        a=adapter(stepped_ground())
        for views,pending in (([[.4,0.,.13,0.]],False),([],True)):
            with self.subTest(has_safe_support=bool(views)), \
                 patch.object(a.map.frontier_index,'query',return_value=dict(clusters=[],query_pending=True)), \
                 patch.object(a,'_support_views',return_value=(views,[[.3,0.,.13,1]],pending)) as search:
                result=a.frontiers(b'',1,(-.7,0.,.32),.25,query_budget=.20,
                    floor=(0.,.08),floor_selection=LIMITS,seek_ascent=True)
                self.assertTrue(result['query_pending']);search.assert_called_once()
                self.assertEqual(result['upward_support_targets'],views)
                self.assertTrue(result['unknown_blockers'])

    def test_known_support_is_offered_even_while_a_lower_observation_is_available(self):
        a=adapter(stepped_ground())
        query=dict(clusters=[dict(observations=[[.1,0.,0.]])],query_pending=False)
        with patch.object(a.map.frontier_index,'query',return_value=query), \
             patch.object(a,'_support_views',return_value=([[.4,0.,.13,0.]],[],False)) as supports:
            result=a.frontiers(b'',1,(-.7,0.,.32),.25,query_budget=.20,
                floor=(0.,.08),floor_selection=LIMITS,seek_ascent=True)
        supports.assert_called_once();self.assertTrue(result['upward_support_targets'])
        self.assertFalse(hasattr(a,'_local_stair_task'))

    def test_ascent_query_distinguishes_admitted_supports_from_progress_only_hints(self):
        a=adapter(stepped_ground())
        for progress_only in (False,True):
            def support(*args,**kwargs):
                a.metadata['support_search']={'progress_only_candidates':1 if progress_only else 0}
                return [[.4,0.,.13,0.]],[],False
            with patch.object(a.map.frontier_index,'query',return_value=dict(clusters=[],query_pending=False)), \
                 patch.object(a,'_support_views',side_effect=support):
                result=a.frontiers(b'',1,(-.7,0.,.32),.25,query_budget=.20,
                    floor=(0.,.08),floor_selection=LIMITS,seek_ascent=True)
            self.assertEqual(result['support_progress_only'],progress_only)

    def test_known_next_support_is_available_during_pending_frontier_search(self):
        a = adapter(stepped_ground())
        ground = (.4, 0., .13)
        for platform_candidate in (False, True):
            with self.subTest(platform_candidate=platform_candidate):
                pending = dict(clusters=[], query_pending=True)
                with patch.object(a.map.frontier_index, 'query', return_value=pending), \
                     patch.object(a, '_platform_seed', return_value=platform_candidate):
                    result = a.frontiers(b'', 1, (.4, 0., .45), .25, query_budget=.20,
                                         continuation=(0., ground[2] + .005, .15),
                                         floor=(0., .08), floor_selection=LIMITS)
                self.assertTrue(result['query_pending'])
                self.assertTrue(result['upward_support_targets'])
                self.assertTrue(all(p[2] > ground[2] for p in result['upward_support_targets']))

    def test_eight_cm_priority_is_relative_to_floor_and_has_no_total_rise_cap(self):
        a=adapter(stepped_ground());index=a.map.frontier_index
        def no_views(*args,**kwargs):return []
        with patch.object(index,'_observers',side_effect=no_views):
            ordinary=index.query((-.7,0.,0.),floor=(0.,.08))
            ascent=index.query((-.7,0.,.06),attempt_scope=('floor_ascent',(0.,.08)),
                               ascent_limits=(.16,.08),max_attempts=4)
        self.assertTrue(ordinary['clusters'])
        self.assertTrue(all(c['frontier'][2]<=.08 for c in ordinary['clusters']))
        heights={round(index.nodes[key].height,2) for c in ascent['clusters']
                 for key in index.floor_query_members[c['id']]}
        self.assertIn(.13,heights)
        self.assertIn(.26,heights)

    def test_native_path_can_accumulate_two_legal_steps(self):
        a=adapter(stepped_ground())
        reason,route,metadata=a.plan(b'',1,(-.7,0.,.32),(1.1,0.,.58))
        self.assertEqual(reason,'ok',metadata)
        self.assertGreater(route[-1][2]-route[0][2],.16)

    def test_continuous_ramp_connects_to_a_new_exploration_platform(self):
        def ground_height(x):
            return .025 * min(12, max(0, x))

        a = adapter([(x * .1, y * .1, ground_height(x))
                     for x in range(-12, 33) for y in range(-12, 13)])
        reference_floor = (0., .08)
        slope_position = (.6, 0., .47)
        plateau_position = (2., 0., .62)
        with patch.object(a.map.frontier_index, '_observers', return_value=[]):
            ascent = a.frontiers(b'', 1, (-.5, 0., .32), .25, query_budget=.20,
                                 floor=reference_floor, floor_selection=LIMITS, seek_ascent=True)
        self.assertTrue(ascent['upward_support_targets'])
        reason, route, details = a.plan(b'', 1, (-.5, 0., .32), plateau_position)
        self.assertEqual(reason, 'ok', details)
        self.assertGreater(route[-1][2] - route[0][2], .16)
        self.assertEqual(a.landing_check(b'', 1, slope_position, reference_floor)['reason'],
                         'platform_not_confirmed')
        plateau = a.landing_check(b'', 1, plateau_position, reference_floor)
        self.assertTrue(plateau['ready'], plateau)
        self.assertAlmostEqual(plateau['platform_height'], .30, places=5)

    def test_area_confirmation_accepts_an_irregular_platform_in_any_heading(self):
        cells={(x,y) for x in range(15) for y in range(5)}|{
               (x,y) for x in range(5) for y in range(20)}
        a=adapter([(x*.1,y*.1,.13) for x,y in cells])
        point=(.2,.2,.45)
        for heading in (-2.5,0.,2.5):
            result=a.landing_check(b'',1,point,(0.,.08),heading)
            self.assertTrue(result['ready'],result)
            self.assertGreaterEqual(result['platform_area'],1.5-1e-7)
            self.assertAlmostEqual(result['platform_height'],.13,places=5)

    def test_level_area_uses_one_fixed_height_and_deduplicates_xy(self):
        a=adapter([(x*.1,y*.1,.13*(1+x//4)) for x in range(24) for y in range(-4,5)])
        point=(.1,0.,.45)
        result=a.landing_check(b'',1,point,(0.,.08))
        self.assertFalse(result['ready'])
        self.assertEqual(result['reason'],'platform_not_confirmed')
        self.assertLess(result['platform_area'],1.5)
        before=a._platform_patch((.1,0.,.13))[0]
        a.integrate(np.asarray([(x*.1,y*.1,.13*(1+x//4)) for x in range(24)
                               for y in range(-4,5)],dtype='float32').tobytes());a.prepare(b'',2)
        self.assertEqual(a._platform_patch((.1,0.,.13))[0],before)

    def test_platform_area_excludes_measured_cells_rejected_by_public_planning(self):
        a = adapter([(x * .1, y * .1, .13) for x in range(16) for y in range(10)])
        point = (.4, .4, .45)
        self.assertTrue(a.landing_check(b'', 1, point, (0., .08))['ready'])
        for x in range(12, 16):
            for y in range(10):
                for _, row in a.map.layers_at(x, y):
                    row['inflated_terrain_cost'] = 50.
        # This fixture edits committed rows directly; emulate the writer's
        # dependency invalidation that build() performs for changed costs.
        a.map._invalidate_auxiliary({(k,x,y) for x in range(12,16) for y in range(10)
                                   for k,_ in a.map.layers_at(x,y)})
        result = a.landing_check(b'', 1, point, (0., .08))
        self.assertEqual(result['reason'], 'platform_not_confirmed')
        self.assertLess(result['platform_area'], 1.5)

    def test_area_below_departure_threshold_does_not_register_a_floor(self):
        a=adapter([(x*.1,y*.1,.06) for x in range(-8,9) for y in range(-8,9)])
        result=a.landing_check(b'',1,(0.,0.,.38),(0.,.08))
        self.assertFalse(result['ready'])
        self.assertEqual(result['reason'],'still_on_departure_floor')

    def test_upward_observation_candidates_can_turn_around(self):
        a=adapter([(x*.1,y*.1,.13) for x in range(-12,13) for y in range(-12,13)])
        views,_,_=a._support_views((0.,0.,0.),.25,.08,budget=1.)
        self.assertTrue(views)
        # No fixed ascent direction is required to generate safe positions.
        self.assertTrue(any(p[0]<0 for p in views))

    def test_nearby_same_surface_positions_do_not_hide_the_next_higher_support(self):
        a = adapter([(x * .1, y * .1, .15 if x < 8 else .26)
                     for x in range(-15, 16) for y in range(-10, 11)])
        views, _, _ = a._support_views((0., 0., .13), .25, .135, budget=1.)
        self.assertTrue(views)
        self.assertGreater(views[0][2] - .13, a.map.settings.surface_noise_tolerance)

    def test_same_failed_stopping_poses_are_not_revalidated_without_dependency_changes(self):
        a=adapter(stepped_ground())
        with patch.object(a,'_arrival_area',return_value=dict(ready=False)) as stop:
            a._support_views((-.7,0.,0.),.25,.08,budget=1.)
            calls=stop.call_count
            self.assertGreater(calls,0)
            a._support_views((-.7,0.,0.),.25,.08,budget=1.)
            self.assertEqual(stop.call_count,calls)


if __name__=='__main__':unittest.main()
