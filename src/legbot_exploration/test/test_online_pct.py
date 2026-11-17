"""CPU boundary/native-A* tests; the original GPU mapper is a separate gate."""
import ctypes
from dataclasses import replace
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from legbot_exploration.online_pct import OnlinePct, _load
from legbot_exploration.pct_stairs import StairFlight, _find_stairs, level_platform

PROJECT = Path(__file__).resolve().parents[3]
PCT = PROJECT / 'install/pct_planner/share/pct_planner'


def native_wrapper():
    libs = PROJECT / 'third_party/pct/install/lib/x86_64-linux-gnu'
    for name in ('libmetis.so.5','libgtsam.so.4'):
        ctypes.CDLL(str(libs/name),mode=ctypes.RTLD_GLOBAL)
    sys.path[:0] = [str(PCT/'planner/scripts'), str(PCT/'planner')]
    return _load('_test_pct_wrapper',PCT/'planner/scripts/planner_wrapper.py').TomogramPlanner


def stair_probe(adapter, *args, **kwargs):
    """Drive the resumable recognition task through its ordinary tick boundary."""
    for _ in range(200):
        result=adapter.floor_stair_probe(*args,**kwargs)
        if result.get('reason')!='stair_index_pending':return result
    raise AssertionError('stair index did not finish its incremental batches')


class PctAdapterTest(unittest.TestCase):
    def test_local_stair_heading_uses_measured_treads_instead_of_approach_bearing(self):
        adapter=OnlinePct(str(PCT));adapter.configure({'robot_radius':.15})
        cloud=np.asarray([(x*.1,y*.1,z) for x in range(-15,16) for y in range(-15,19)
                          for z in (0. if y<3 else .13 if y<9 else .26,1.5)]*3,dtype='float32')
        adapter.integrate(cloud.tobytes());adapter.prepare(b'',1)
        with patch.object(adapter.map,'iter_cells',side_effect=AssertionError('whole-map direction scan')):
            result=adapter.observation_task(b'',1,'stair_heading',0,(0.,0.,.32),-.65,0.)
            self.assertTrue(result['ready'],result)
            self.assertAlmostEqual(result['yaw'],np.pi/2)
            result=adapter.observation_task(b'',1,'stair_heading',0,(0.,.5,.45),-.65,0.)
            self.assertTrue(result['ready'],result)
            self.assertAlmostEqual(result['yaw'],np.pi/2)
            # Reusing a pose does not recompute unchanged local evidence.
            with patch.object(adapter.map,'support_samples',side_effect=AssertionError('cache missed')):
                self.assertEqual(result,adapter.observation_task(b'',1,'stair_heading',0,(0.,.5,.45),-.65,0.))
        flat=OnlinePct(str(PCT))
        flat.integrate(np.asarray([(x*.1,y*.1,z) for x in range(-12,13) for y in range(-12,13)
                                   for z in (0.,1.5)]*3,dtype='float32').tobytes())
        self.assertFalse(flat.observation_task(b'',1,'stair_heading',0,(0.,0.,.32),-.65,0.)['ready'])

    def test_stair_treads_require_matching_traversable_slices_on_every_review(self):
        adapter=OnlinePct(str(PCT));m=adapter.map;floor=(0.,.08)
        flight=StairFlight((0,0,0.),(1,0),((3,0,.13),(6,0,.26),(9,0,.39)),3)
        for k,(x,y,height) in enumerate((flight.entry,*flight.treads),1):
            row=m.cell((k,x,y),True)
            row['G']=height;row['measured']=True;row['unknown']=row['solid']=row['hard']=False
            row['inflated_terrain_cost']=0.
            m.column_layers.setdefault((x,y),set()).add(k)
            m.columns[x,y]={0:SimpleNamespace(orientation=1,height=height)}
        self.assertTrue(adapter._upward_stair_check(flight,floor)['ready'])
        adapter._pending_stairs[floor]={flight:None}
        row=m.cell((3,6,0))
        for field,value in [('inflated_terrain_cost',50.),('G',.12)]:
            original=row[field];row[field]=value
            checked=adapter._upward_stair_check(flight,floor)
            self.assertTrue(checked['ready'])
            self.assertEqual(checked['usable_count'],1)
            self.assertEqual(checked['tail_reason'],'stair_tread_no_traversable_slice')
            with patch.object(m.stair_index,'query',return_value=()), \
                 patch.object(m.stair_scout_index,'query',return_value=()):
                candidate=adapter._stair_candidates(floor,(0.,0.,0.),0.)[0]
                self.assertEqual(candidate.usable_treads,flight.treads[:1])
                self.assertFalse(candidate.confirmed)
            row[field]=original
        # Another cut may express the same tread safely; each tread is independent.
        row['inflated_terrain_cost']=50.
        alternate=m.cell((4,6,0),True)
        for field in row.dtype.names:alternate[field]=row[field]
        alternate['inflated_terrain_cost']=10.;m.column_layers[6,0].add(4)
        self.assertTrue(adapter._upward_stair_check(flight,floor)['ready'])
        m.cell((4,9,0))['inflated_terrain_cost']=50.
        prefix=adapter._qualified_stair(flight,floor)
        self.assertTrue(prefix.confirmed)
        self.assertEqual(prefix.usable_treads,flight.treads[:2])
        self.assertEqual(prefix.pending_treads,flight.treads[2:])
        m.settings=replace(m.settings,slice_dh=.05)
        self.assertEqual(adapter._upward_stair_check(flight,floor)['tail_reason'],
                         'stair_tread_no_traversable_slice')

    def test_floor_handoff_finds_ascending_frontiers_without_a_stair_heading(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.20})
        flat=[(i*.1,j*.1,0.) for i in range(-20,10) for j in range(-15,16)]
        higher=[(i*.1,j*.1,.13) for i in range(10,36) for j in range(-15,16) if (i,j)!=(25,0)]
        adapter.integrate(np.asarray((flat+higher)*3,dtype='float32').tobytes())
        scope=(0.,.08)
        ordinary=adapter.frontiers(b'',1,(0.,0.,.32),.25,floor=scope)
        heights=[p[2] for cluster in ordinary['clusters'] for p in cluster['display']]
        self.assertTrue(heights)
        self.assertTrue(all(abs(height)<=scope[1] for height in heights))
        result=adapter.frontiers(b'',1,(0.,0.,.32),.25,floor=scope,
            floor_selection=(.15,2.,4.,.1,4,.16,.05),stair_heading=np.pi,seek_ascent=True)
        observations=[p for cluster in result['clusters'] for p in cluster['observations']]
        self.assertTrue(observations,result)
        self.assertTrue(all(p[0]>=1. and .05<p[2]<=.16 for p in observations),observations)
        self.assertEqual(adapter.map.frontier_index.floor_query[0],('floor_ascent',scope))

    def test_stair_tread_margin_prefers_interior_with_arrival_tolerance(self):
        adapter=OnlinePct(str(PCT));m=adapter.map
        for x in range(10):
            row=m.cell((1,x,0),True)
            row['G']=.13;row['measured']=True;row['unknown']=row['solid']=row['hard']=False
            row['inflated_terrain_cost']=0.;m.column_layers[x,0]={1}
        self.assertEqual(adapter._tread_margin((0.,0.,.13),0.),0.)
        self.assertGreater(adapter._tread_margin((.4,0.,.13),0.),.25)
        self.assertLess(adapter._tread_margin((.1,0.,.13),0.,.15),
                        adapter._tread_margin((.1,0.,.13),0.,0.))

    def test_same_floor_platform_offset_is_not_an_upward_step(self):
        adapter=OnlinePct(str(PCT));floor=(2.96595,.08)
        flight=StairFlight((0,0,2.984),(1,0),((3,0,3.018),),3)
        check=adapter._upward_stair_check(flight,floor)
        self.assertFalse(check['ready'])
        self.assertFalse(check['reviewable'])
        self.assertEqual(check['reason'],'stair_first_tread_within_entry_noise')

    def test_cross_layer_rise_uses_measured_ground_with_body_height_error(self):
        adapter=OnlinePct(str(PCT));m=adapter.map
        for x,height in ((0,1.853), (1,2.001)):
            row=m.cell((5,x,0),True)
            row['G']=height;row['measured']=True;row['unknown']=row['solid']=False
            row['inflated_terrain_cost']=0.;m.column_layers[x,0]={5}
        ground=adapter._measured_ground((0.,0.,2.134))
        self.assertAlmostEqual(ground[2],1.853)
        self.assertLess(2.001-ground[2],.16)
        self.assertGreater(2.001-(2.134-.32),.16)

    def test_new_height_cannot_be_registered_from_a_floor_underside(self):
        adapter=OnlinePct(str(PCT))
        points=np.asarray([(x*.1,y*.1,1.) for x in range(-10,11) for y in range(-10,11)]*3,dtype='float32')
        adapter.integrate(points.tobytes());adapter.prepare(b'',1)
        self.assertTrue(adapter._ground_support((0.,0.,1.32))['ready'])
        for record in adapter.map.columns[0,0].values():record.orientation=-1
        result=adapter.observation_task(b'',1,'observe_height',0,(0.,0.,1.32),0.,0.)
        self.assertFalse(result['ready'])
        self.assertEqual(result['reason'],'height_support_unobserved')
        self.assertEqual(adapter.map.surface_observation.visited_heights,[])

    def test_fresh_different_height_releases_arrival_cache_and_recovery_wait_without_changing_G(self):
        adapter=OnlinePct(str(PCT))
        points=np.asarray([(x*.1,y*.1,0.) for x in range(-12,13) for y in range(-12,13)]*3,dtype='float32')
        adapter.integrate(points.tobytes());adapter.prepare(b'',1)
        base=(0.,0.,.32)
        self.assertTrue(adapter._arrival_area(base,.15,0.)['ready'])
        record=adapter.map.surface_observation.expect((0.,0.,.13),review=True)
        self.assertFalse(adapter._arrival_area(base,.15,0.)['ready'])
        before=adapter.recovery_evidence(b'',1,base,(.6,0.,.32));geometry=adapter.map.revision
        adapter.integrate(points.tobytes());adapter.prepare(b'',2)
        self.assertEqual(adapter.map.revision,geometry)
        self.assertNotEqual(adapter.recovery_evidence(b'',2,base,(.6,0.,.32)),before)
        self.assertTrue(adapter._arrival_area(base,.15,0.)['ready'])
        self.assertEqual(adapter.map.surface_observation.query((0.,0.,.13))['state'],'unresolved')
        # Alignment starts a new review boundary; neither stale successful
        # arrival geometry nor a pre-alignment hit can verify that boundary.
        adapter.map.surface_observation.arm_review(record,adapter.map.integrated_serial)
        self.assertFalse(adapter._arrival_area(base,.15,0.)['ready'])

    def test_stair_stop_timeout_replans_locally_only_from_verified_arrival_area(self):
        adapter=OnlinePct(str(PCT));task=dict(tolerance=.15,scans=1,finished=False,verified=True)
        adapter._local_stair_task=task
        # No succeeded receipt, no reset of the physical scan count.
        with patch.object(adapter,'prepare'), \
             patch.object(adapter,'_measured_ground',return_value=np.array([0.,0.,.13])), \
             patch.object(adapter,'_arrival_area',return_value=dict(ready=True)):
            result=adapter.observation_task(b'',1,'recover_stair_stop',0,(0.,0.,.411),0.,0.)
            self.assertTrue(result['ready']);self.assertFalse(task['finished'])
            self.assertFalse(task['verified']);self.assertEqual(task['scans'],1)
            self.assertEqual(task['rejected'],[(0.,0.,.13)])
            self.assertTrue(adapter.observation_task(b'',1,'recover_stair_stop',0,(0.,0.,.411),0.,0.)['ready'])
            self.assertFalse(adapter.observation_task(b'',1,'recover_stair_stop',0,(0.,0.,.411),0.,0.)['ready'])
            self.assertEqual(task['stop_recoveries'],2)
        with patch.object(adapter,'prepare'), \
             patch.object(adapter,'_measured_ground',return_value=np.array([0.,0.,.13])), \
             patch.object(adapter,'_arrival_area',return_value=dict(ready=False,reason='arrival_unsafe_stop_area')):
            self.assertFalse(adapter.observation_task(b'',1,'recover_stair_stop',0,(0.,0.,.411),0.,0.)['ready'])
        task.update(approach_pending=True,stop_recoveries=0)
        with patch.object(adapter,'prepare'), \
             patch.object(adapter,'_measured_ground',return_value=np.array([-.5,0.,0.])), \
             patch.object(adapter,'_arrival_area',return_value=dict(ready=True)):
            self.assertTrue(adapter.observation_task(b'',1,'recover_stair_stop',0,(-.5,0.,.32),0.,0.)['ready'])
            self.assertEqual(task['rejected_approaches'],[(-.5,0.,0.)])

    def test_passive_stair_candidates_keep_the_observed_approach_without_actions(self):
        adapter=OnlinePct(str(PCT));floor=(0.,.08)
        single=StairFlight((0,0,0.),(1,0),((3,0,.13),),3)
        extended=StairFlight((0,0,0.),(1,0),((3,0,.13),(6,0,.26),(9,0,.39)),3)
        with patch.object(adapter,'_upward_stair_check',return_value={'ready':True}), \
             patch.object(adapter,'observation_poses',side_effect=AssertionError('passive recording must not search')):
            adapter._record_stair_candidates(floor,(single,),(-.5,0.,0.),0.,True)
            adapter._record_stair_candidates(floor,(extended,single),(8.,0.,0.),0.,True)
            self.assertIsNone(adapter._local_stair_task)
            self.assertEqual(adapter._saved_stairs,{})
            self.assertEqual(adapter._pending_stairs[floor],{extended:(-.5,0.,0.,0.)})
            # Fresh local indexing can be empty after the robot walks away.
            with patch.object(adapter.map.stair_index,'query',return_value=()), \
                 patch.object(adapter.map.stair_scout_index,'query',return_value=()):
                self.assertIn(extended,adapter._stair_candidates(floor,(8.,0.,0.),0.))

    def test_passive_stair_qualification_invalidates_only_affected_columns(self):
        adapter=OnlinePct(str(PCT));m=adapter.map;floor=(0.,.08)
        cache=m.stair_candidates;cache.set_floor(floor)
        near=StairFlight((0,0,0.),(1,0),((3,0,.13),),3)
        far=StairFlight((24,0,0.),(1,0),((27,0,.13),),3)
        cache.replace(('test',0),(near,far))
        publish=lambda group,flight:None
        with patch.object(adapter,'_qualified_stair',side_effect=lambda flight,_:flight) as qualify:
            cache.process(qualify,(0.,0.,0.),float('inf'),publish)
            self.assertEqual(set(cache.candidates.values()),{near,far})
            cache.process(qualify,(0.,0.,0.),float('inf'),publish)
            self.assertEqual(qualify.call_count,2)
            cache.invalidate({(8,0)})
            cache.process(qualify,(0.,0.,0.),float('inf'),publish)
            self.assertEqual(qualify.call_count,2)
            cache.invalidate({(3,0)})
            cache.process(qualify,(0.,0.,0.),float('inf'),publish)
            self.assertEqual(qualify.call_count,3)
            qualify.assert_called_with(near,floor)
            cache.replace(('test',1),(far,))
            cache.replace(('test',0),())
            cache.process(qualify,(0.,0.,0.),float('inf'),publish)
            self.assertEqual(set(cache.candidates.values()),{far})
            cache.replace(('test',1),())
            cache.process(qualify,(0.,0.,0.),float('inf'),publish)
            self.assertFalse(cache.candidates)
            self.assertFalse(cache.dependents)

    def test_passive_stair_budget_keeps_unfinished_qualification_across_ticks(self):
        adapter=OnlinePct(str(PCT));m=adapter.map;floor=(0.,.08)
        cache=m.stair_candidates;cache.set_floor(floor)
        flights=[StairFlight((i*20,0,0.),(1,0),((i*20+3,0,.13),),3) for i in range(100)]
        cache.replace(('test',0),flights)
        m.stair_index.floor=m.stair_scout_index.floor=floor
        now=100.

        def qualify(flight,_):
            nonlocal now
            now+=.005
            return flight

        with (patch('legbot_exploration.pct_stairs.time.monotonic',side_effect=lambda:now),
              patch.object(adapter,'_qualified_stair',side_effect=qualify) as check):
            work=adapter._maintain_stair_candidates(floor,(0.,0.,0.),0.,True)
            self.assertLessEqual(check.call_count,4)
            self.assertTrue(cache.pending)
            self.assertLessEqual(work['stair_maintenance_ms'],20.001)
            first=check.call_count
            adapter._maintain_stair_candidates(floor,(0.,0.,0.),0.,True)
            self.assertGreater(check.call_count,first)
            self.assertEqual(len({call.args[0] for call in check.call_args_list}),check.call_count)

    def test_static_stair_maintenance_and_duplicate_hits_do_not_requalify(self):
        adapter=OnlinePct(str(PCT));adapter.configure({'robot_radius':.20})
        scope=(0.,.08);ground=(-.9,0.,0.)
        points=np.asarray([(x*.1,y*.1,0.) for x in range(-20,0) for y in range(-10,11)]+
                          [(x*.1,y*.1,.13+.13*(x//6)) for x in range(18) for y in range(-10,11)],dtype='float32')
        adapter.integrate(np.tile(points,(3,1)).tobytes());adapter.prepare(b'',1)
        m=adapter.map
        for _ in range(200):
            adapter._maintain_stair_candidates(scope,ground,0.,True)
            if (m.stair_index.floor==m.stair_scout_index.floor==scope and
                    not(m.stair_candidates.needs_work() or m.stair_index.pending or m.stair_scout_index.pending)):
                break
        else:self.fail('stair maintenance did not drain')
        self.assertTrue(m.stair_candidates.confirmed)
        with (patch.object(adapter,'_qualified_stair',side_effect=AssertionError('unchanged candidate rechecked')),
              patch.object(m.stair_index,'cached_flights',side_effect=AssertionError('full candidate list read')),
              patch.object(m.stair_scout_index,'cached_flights',side_effect=AssertionError('full candidate list read'))):
            for _ in range(10):adapter._maintain_stair_candidates(scope,ground,0.,True)
            adapter.integrate(np.tile(points,(3,1)).tobytes());adapter.prepare(b'',2)
            adapter._maintain_stair_candidates(scope,ground,0.,True)

    def test_blocker_review_requires_target_xy_height_and_a_post_arm_return(self):
        adapter=OnlinePct(str(PCT))
        adapter.integrate(np.asarray([(0.,0.,0.)]*3,dtype='float32').tobytes())
        pose=(0.,0.,.32);target=[[0.,0.,1.46,3],[0.,0.,1.47,4]]
        op=lambda version,name:adapter.observation_task(b'',version,name,0,pose,0.,0.,target)
        self.assertTrue(op(1,'begin_blocker_review')['ready'])
        self.assertTrue(op(1,'arm_blocker_review')['ready'])
        for version,point in ((2,(.1,0.,1.46)),(3,(0.,0.,0.))):
            adapter.integrate(np.asarray([point],dtype='float32').tobytes())
            progress=op(version,'blocker_review_progress')
            self.assertEqual(progress['ready'],version==3)
            self.assertEqual(progress['matched_count'],0)
            self.assertEqual(progress['unresolved_count'],1)
        adapter.integrate(np.asarray([(0.,0.,1.46)],dtype='float32').tobytes())
        self.assertTrue(op(4,'blocker_review_progress')['ready'])
        self.assertEqual(op(4,'blocker_review_progress')['target_count'],1)
        rows=dict(adapter.map.layers_at(0,0))
        self.assertAlmostEqual(rows[0]['G'],0.)
        self.assertAlmostEqual(rows[0]['C'],1.46,places=6)
        self.assertAlmostEqual(rows[3]['G'],1.46,places=6)
        self.assertTrue(op(4,'finish_blocker_review')['ready'])
        self.assertFalse(op(5,'begin_blocker_review')['ready'])
        review=adapter.map.frontier_index.observation_model.blocker_review
        self.assertFalse(review.available((0.,0.,1.47)))
        self.assertTrue(review.available((0.,0.,3.0)))

    def test_blocker_review_accepts_the_actual_tread_without_inventing_expected_height(self):
        adapter=OnlinePct(str(PCT))
        adapter.integrate(np.asarray([(0.,0.,.65)]*3,dtype='float32').tobytes())
        adapter.prepare(b'',1)
        review=adapter.map.frontier_index.observation_model.blocker_review
        review.begin([(0.,0.,.78)]);review.arm()
        adapter.integrate(np.asarray([(0.,0.,.65)],dtype='float32').tobytes())
        adapter.prepare(b'',2)
        self.assertTrue(review.progress()['ready'])
        self.assertAlmostEqual(adapter.map.cell((2,0,0))['G'],.65,places=6)
        review.finish()
        self.assertFalse(review.available((0.,0.,.65)))
        self.assertFalse(review.available((0.,0.,.78)))

    def test_blocker_review_can_confirm_an_unchanged_obstacle_once(self):
        adapter=OnlinePct(str(PCT))
        points=[(i*.1,j*.1,1.46) for i in range(-12,13) for j in range(-12,13)]
        points.append((0.,0.,1.60))
        adapter.integrate(np.asarray(points*3,dtype='float32').tobytes())
        pose=(0.,0.,1.78);targets=[[0.,0.,1.46,3]]
        def op(version,name):return adapter.observation_task(b'',version,name,0,pose,0.,0.,targets)
        op(1,'begin_blocker_review');op(1,'arm_blocker_review')
        self.assertFalse(op(1,'blocker_review_progress')['ready'])
        revision=adapter.map.revision
        adapter.integrate(np.asarray([(0.,0.,1.46)],dtype='float32').tobytes())
        self.assertTrue(op(2,'blocker_review_progress')['ready'])
        self.assertEqual(adapter.map.revision,revision)
        self.assertFalse(adapter.map.point_safe_check(pose,floor=(1.46,.05))['ready'])
        op(2,'finish_blocker_review')
        self.assertFalse(op(3,'begin_blocker_review')['ready'])

    def test_blocker_review_without_target_returns_can_change_view_before_consuming_observation(self):
        adapter=OnlinePct(str(PCT))
        adapter.integrate(np.asarray([(0.,0.,0.)],dtype='float32').tobytes())
        review=adapter.map.frontier_index.observation_model.blocker_review
        review.begin([(0.,0.,.13)]);review.arm()
        self.assertFalse(review.finish()['ready'])
        adapter.integrate(np.asarray([(.1,0.,.13)],dtype='float32').tobytes())
        self.assertTrue(review.begin([(0.,0.,.13)])['ready'])
        review.arm()
        adapter.integrate(np.asarray([(0.,0.,.13)],dtype='float32').tobytes())
        self.assertTrue(review.finish()['ready'])
        self.assertFalse(review.begin([(0.,0.,.13)])['ready'])

    def test_safe_parking_slice_discards_other_slices_relief_blockers(self):
        adapter=OnlinePct(str(PCT))
        adapter.integrate(np.asarray([(i*.1,j*.1,0.) for i in range(-12,13)
                                     for j in range(-12,13)],dtype='float32').tobytes())
        adapter.prepare(b'',1)
        m=adapter.map
        row=m.cell((1,5,0))
        row['relief_cost']=50.;row['solid']=True
        row['terrain_cost']=row['inflated_terrain_cost']=50.
        check=adapter._arrival_area((0.,0.,.32),.15,None)
        self.assertTrue(check['ready'],check)
        self.assertEqual(check['unknown_blockers'],[])

    def test_platform_query_uses_flat_connectivity_and_two_attempts(self):
        adapter=OnlinePct(str(PCT))
        index=adapter.map.frontier_index
        row=adapter.map.cell((4,0,0),True)
        row['G']=1.5;row['measured']=True;row['unknown']=False
        with (patch.object(adapter,'prepare'),
              patch.object(adapter,'_admission',return_value=dict(ready=True,ground_z=1.5,layer=4)),
              patch.object(index,'query',return_value={}) as query):
            adapter.frontiers(b'',1,(0.,0.,1.82),.25,(3.,2.,2.),.2,
                continuation=(0.,1.45,.15),floor=(0.,.08),platform_floor=(1.5,.05),
                floor_selection=(.15,2.,4.,.4,4,.16,.05))
        self.assertTrue(query.call_args.kwargs['prune_disconnected'])
        self.assertEqual(query.call_args.kwargs['max_attempts'],2)
        self.assertEqual(query.call_args.kwargs['floor'],(1.5,.05))
        self.assertNotIn('continuation',query.call_args.kwargs)
        self.assertNotIn('attempt_scope',query.call_args.kwargs)

    def test_platform_confirmation_uses_fixed_height_instead_of_following_each_stop(self):
        adapter=OnlinePct(str(PCT))
        rows=lambda xy:{.41}
        with (patch.object(adapter,'prepare'),
              patch.object(adapter,'_admission',return_value=dict(ready=True,ground_z=.41,reason='ready')),
              patch.object(adapter,'_landing_heights',side_effect=rows)):
            point=(0.,0.,.73)
            self.assertTrue(adapter.landing_check(b'',1,point,(0.,.08))['ready'])
            rejected=adapter.landing_check(b'',1,point,(0.,.08),0.,.33)
            self.assertEqual(rejected['reason'],'platform_height_changed')

    def test_arrival_area_requires_measured_ground_in_the_near_blind_area(self):
        adapter = OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.20})
        target = (0.,0.,.32)
        hole = (.7,0.,0.)
        cloud = [(x/10,y/10,0.) for x in range(-13,14) for y in range(-13,14)]
        adapter.integrate(np.asarray([p for p in cloud if p!=hole],dtype='float32').tobytes())
        self.assertTrue(adapter.point_check(b'',1,target)['ready'])
        engine=adapter.map.frontier_index.observation_model.pose_search
        with patch.object(engine,'_arrival_stop',wraps=engine._arrival_stop) as stop:
            self.assertTrue(adapter._arrival_area(target,.15,None)['ready'])
            rejected = adapter.observation_task(b'',1,'arrival_area',0,target,0.,.15)
            self.assertFalse(adapter.observation_task(b'',1,'arrival_area',0,target,np.pi,.15)['ready'])
            stop.assert_called_once()
        self.assertEqual(rejected['reason'],'arrival_unobserved_blind_area',rejected)
        self.assertAlmostEqual(rejected['observation_radius'],.85)
        self.assertAlmostEqual(rejected['stop_radius'],.40)
        self.assertTrue(any(abs(x-.7)<.001 and abs(y)<.001 for x,y,_,_ in rejected['unknown_blockers']))
        adapter.integrate(np.asarray([hole],dtype='float32').tobytes())
        self.assertTrue(adapter.observation_task(b'',2,'arrival_area',0,target,0.,.15)['ready'])

    def test_floor_search_uses_the_next_safe_batch_after_three_failed_routes(self):
        adapter=OnlinePct(str(PCT))
        cloud=np.asarray([(i*.1,j*.1,0.) for i in range(-22,23) for j in range(-22,23)],dtype='float32')
        adapter.integrate(cloud.tobytes())
        pose, band=(0.,0.,.32),(0.,.08)

        def batch():
            output=adapter.frontiers(b'',1,pose,.5,(4.,2.,2.),1.,floor=band,
                                     floor_selection=(.15,8.,8.,1.))
            self.assertFalse(output['no_view_clusters'])
            return next(c for c in output['clusters'] if c['observations'])

        first=batch()
        self.assertEqual(len(first['observations']),3)
        for i,(point,yaw) in enumerate(zip(first['observations'],first['observation_yaws'])):
            self.assertTrue(adapter._arrival_area((*point[:2],point[2]+adapter.reference_height),.15,yaw)['ready'])
            operation='floor_reject_view_timeout' if i==0 else 'floor_reject_view'
            self.assertTrue(adapter.observation_task(b'',1,operation,first['id'],point,0.,0.)['ready'])
        second=batch()
        self.assertEqual(first['id'],second['id'])
        self.assertFalse(any(point in first['observations'] for point in second['observations']))

    def test_common_pose_search_applies_near_filter_to_clusters_and_recovery(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.20,'ground_lidar_elevation_min':-1.,
                           'ground_lidar_elevation_max':1.})
        points=[(x*.1,y*.1,0.) for x in range(-20,31) for y in range(-20,21)]
        adapter.integrate(np.asarray(points*3,dtype='float32').tobytes())
        adapter.prepare(b'',1)
        base=np.asarray((0.,0.,.32));model=adapter.map.frontier_index.observation_model
        for target,expected in (((.75,0.,0.),False),((1.4,0.,0.),True)):
            normal=adapter.observation_poses(base,(0.,0.,0.),[target],stances=[(base,0,True)])
            recovery=adapter.observation_poses(base,(0.,0.,0.),[target],motions=False)
            cluster=model._admitted_view((0.,0.,0.),[target])
            self.assertEqual(bool(normal['poses']),expected)
            self.assertEqual(bool(recovery['poses']),expected)
            self.assertEqual(cluster is not None,expected)
            if expected:
                self.assertAlmostEqual(normal['poses'][0][6],recovery['poses'][0][6])

    def test_recovery_cannot_bypass_an_unobserved_arrival_disc(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.20})
        hole=(.7,0.,0.)
        points=[(x*.1,y*.1,0.) for x in range(-20,31) for y in range(-20,21)
                if (x,y)!=(7,0)]
        adapter.integrate(np.asarray(points*3,dtype='float32').tobytes())
        adapter.prepare(b'',1)
        base=np.asarray((0.,0.,.32));target=(1.4,0.,0.)
        for mode in (False,True):
            query=adapter.observation_poses(base,(0.,0.,0.),[target],motions=mode)
            for view in query['poses']:
                self.assertTrue(adapter._arrival_area(tuple(view[3:6]),.15,view[6])['ready'])
            if not mode:self.assertFalse(query['poses'])
        adapter.integrate(np.asarray([hole]*3,dtype='float32').tobytes())
        adapter.prepare(b'',2)
        self.assertTrue(adapter.observation_poses(base,(0.,0.,0.),[target],motions=False)['poses'])

    def test_current_scan_requires_actual_support_but_not_unobserved_outer_disc(self):
        adapter=OnlinePct(str(PCT));adapter.configure({'robot_radius':.20})
        # Known support under the body, one unobserved point in the outer
        # arrival disc. Looking from here must not require moving here again.
        points=[(x*.1,y*.1,0.) for x in range(-20,31) for y in range(-20,21)
                if (x,y)!=(7,0)]
        adapter.integrate(np.asarray(points*3,dtype='float32').tobytes());adapter.prepare(b'',1)
        base=(0.,0.,.32);target=(1.4,0.,0.)
        def search(**kw):
            return adapter.observation_poses(base,(0.,0.,0.),[target],**kw)
        self.assertFalse(search(motions=False)['poses'])
        current=dict(motions=False,purpose='current_observation')
        self.assertTrue(search(**current)['poses'])
        self.assertFalse(adapter.observation_poses(base,(0.,.4,0.),[target],**current)['poses'])
        for invalid in ({'motions':True},{'stances':[(base,0,True)]},{'minimum_gain':0}):
            with self.subTest(invalid=invalid),self.assertRaises(ValueError):
                search(**dict(current,**invalid))
        # The mode cannot turn a blind-region target into a visible target.
        self.assertFalse(adapter.observation_poses(base,(0.,0.,0.),[(.5,0.,0.)],**current)['poses'])
        wall=[(.6,y*.1,z*.1) for y in range(-10,11) for z in range(1,9)]
        with patch('legbot_exploration.surface_orientation.classify',return_value=np.full(len(wall)*3,-2)):
            adapter.integrate(np.asarray(wall*3,dtype='float32').tobytes())
        adapter.prepare(b'',2)
        self.assertFalse(search(**current)['poses'])
        # Missing ground within the actual body remains a hard refusal.
        adapter=OnlinePct(str(PCT));adapter.configure({'robot_radius':.20})
        points=[(x*.1,y*.1,0.) for x in range(-20,31) for y in range(-20,21)
                if (x,y)!=(1,0)]
        adapter.integrate(np.asarray(points*3,dtype='float32').tobytes());adapter.prepare(b'',1)
        self.assertFalse(search(**current)['poses'])

    def test_common_pose_search_uses_sensor_pitch_and_obstacle_occlusion(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.20})
        points=[(x*.1,y*.1,0.) for x in range(-20,31) for y in range(-20,21)]
        adapter.integrate(np.asarray(points*3,dtype='float32').tobytes())
        adapter.prepare(b'',1)
        base=(0.,0.,.32);target=(1.4,0.,0.)
        self.assertTrue(adapter.observation_poses(base,(0.,0.,0.),[target],motions=False)['poses'])
        self.assertFalse(adapter.observation_poses(base,(0.,.4,0.),[target],motions=False)['poses'])
        wall=[(.6,y*.1,z*.1) for y in range(-10,11) for z in range(1,9)]
        with patch('legbot_exploration.surface_orientation.classify',return_value=np.full(len(wall)*3,-2)):
            adapter.integrate(np.asarray(wall*3,dtype='float32').tobytes())
        adapter.prepare(b'',2)
        self.assertFalse(adapter.observation_poses(base,(0.,0.,0.),[target],motions=False)['poses'])

    def test_arrival_disc_exempts_only_observed_obstacle_shadows_outside_body(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.20})
        pose=(0.,0.,.32)
        floor=[(x*.1,y*.1,0.) for x in range(-13,14) for y in range(-13,14) if (x,y)!=(7,0)]
        adapter.integrate(np.asarray(floor*3,dtype='float32').tobytes())
        adapter.prepare(b'',1)
        self.assertFalse(adapter._arrival_area(pose,.15,0.)['ready'])
        wall=[(.5,y*.1,z*.1) for y in range(-4,5) for z in range(1,9)]
        with patch('legbot_exploration.surface_orientation.classify',return_value=np.full(len(wall)*3,-2)):
            adapter.integrate(np.asarray(wall*3,dtype='float32').tobytes())
        adapter.prepare(b'',2)
        check=adapter._arrival_area(pose,.15,0.)
        self.assertTrue(check['ready'],check)
        self.assertEqual(check['occluded_cells'],1)
        # Moving the body disc onto the obstacle cannot be excused by its shadow.
        self.assertFalse(adapter._arrival_area((.2,0.,.32),.15,0.)['ready'])

    def test_stair_scan_needs_new_returns_ahead_at_tread_height(self):
        adapter = OnlinePct(str(PCT))
        pose = (0.,0.,.32)
        check = lambda version: adapter.observation_task(b'',version,'stair_view_progress',0,pose,0.,0.)
        adapter.integrate(np.asarray([[.6,0.,.13]],dtype='float32').tobytes())
        adapter.observation_task(b'',1,'arm_stair_view',0,pose,0.,0.)
        self.assertFalse(check(1)['fresh'])
        adapter.integrate(np.asarray([[-.6,0.,0.],[.6,0.,0.],[.6,0.,1.0]],dtype='float32').tobytes())
        self.assertFalse(check(2)['fresh'])
        adapter.integrate(np.asarray([[.6,0.,.13]],dtype='float32').tobytes())
        self.assertTrue(check(3)['fresh'])

    def test_side_stair_scan_ignores_returns_from_the_other_side(self):
        adapter=OnlinePct(str(PCT));pose=(0.,0.,.32)
        adapter.integrate(np.asarray([[.6,-.3,.13]],dtype='float32').tobytes())
        adapter.observation_task(b'',1,'arm_stair_view',0,pose,0.,.30)
        adapter.integrate(np.asarray([[.6,-.3,.13]],dtype='float32').tobytes())
        self.assertFalse(adapter.observation_task(b'',2,'stair_view_progress',0,pose,0.,0.)['ready'])
        adapter.integrate(np.asarray([[.6,.3,.13]],dtype='float32').tobytes())
        self.assertTrue(adapter.observation_task(b'',3,'stair_view_progress',0,pose,0.,0.)['ready'])

    def test_stair_recognition_stops_on_a_verified_stance_and_reuses_it_for_departure(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.20})
        home,floor=(0.,0.,.32),(0.,.08)
        flat=[(i*.1,j*.1,0.) for i in range(-18,12) for j in range(-14,15)]
        first=[(i*.1,j*.1,.13) for i in range(12,18) for j in range(-9,10)]
        higher=[(i*.1,j*.1,.26 if i<24 else .39) for i in range(18,31) for j in range(-9,10)]
        adapter.integrate(np.asarray((flat+first)*3,dtype='float32').tobytes())
        initial=stair_probe(adapter,b'',1,home,floor,0.)
        self.assertTrue(initial['ready'])
        stance=initial['observations'][0]
        pose=(*stance[:2],stance[2]+.32)
        adapter.observation_task(b'',1,'arm_floor_stair_probe',0,pose,0.,0.)
        adapter.integrate(np.asarray(first*3,dtype='float32').tobytes())
        progress=adapter.observation_task(b'',2,'floor_stair_probe_progress',0,pose,0.,0.)
        self.assertTrue(progress['fresh'])
        self.assertFalse(progress['ready'])
        adapter.integrate(np.asarray(higher*3,dtype='float32').tobytes())
        progress=adapter.observation_task(b'',3,'floor_stair_probe_progress',0,pose,0.,0.)
        self.assertTrue(progress['ready'],progress)
        adapter.observation_task(b'',3,'finish_floor_stair_probe',0,pose,0.,0.)
        departure=stair_probe(adapter,b'',3,pose,floor,0.,depart=True)
        self.assertTrue(departure['climbing'],departure)
        target=departure['observations'][0]
        self.assertGreater(target[2],stance[2]+.05)
        self.assertLessEqual(target[2]-stance[2],.16)
        target_pose=(*target[:2],target[2]+.32)
        self.assertTrue(adapter._arrival_area(target_pose,.15,departure['yaws'][0])['ready'])
        self.assertEqual(adapter.plan_surface(b'',3,pose,target_pose)[0],'ok')
        self.assertTrue(any(f.confirmed for f in adapter._saved_stairs[floor]))

    def test_single_safe_tread_advances_before_three_treads_are_visible(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.20})
        flat=[(i*.1,j*.1,0.) for i in range(-18,10) for j in range(-14,15)]
        tread=[(i*.1,j*.1,.13) for i in range(10,34) for j in range(-12,13)]
        adapter.integrate(np.asarray((flat+tread)*3,dtype='float32').tobytes())
        task=stair_probe(adapter,b'',1,(0.,0.,.32),(0.,.08),0.,depart=True)
        self.assertTrue(task['ready'],task)
        self.assertFalse(task['climbing'],task)
        stance=task['observations'][0]
        self.assertAlmostEqual(stance[2],.13,places=5)
        self.assertFalse(adapter._saved_stairs.get((0.,.08)))
        pose=(*stance[:2],stance[2]+.32)
        self.assertTrue(adapter._arrival_area(pose,.15,task['yaws'][0])['ready'])
        self.assertEqual(adapter.plan_surface(b'',1,(0.,0.,.32),pose)[0],'ok')
        # Once on the tread, a fully measured level continuation excludes an
        # isolated step. Unknown continuation must never be classified this way.
        result=stair_probe(adapter,b'',1,pose,(0.,.08),0.,depart=True)
        self.assertTrue(result['returning'],result)
        self.assertEqual(adapter._local_stair_task['reason'],'stair_known_level_continuation')

    def test_stair_scan_limit_survives_slice_and_task_refresh(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.20})
        flat=[(i*.1,j*.1,0.) for i in range(-18,12) for j in range(-14,15)]
        tread=[(i*.1,j*.1,.13) for i in range(12,18) for j in range(-9,10)]
        adapter.integrate(np.asarray((flat+tread)*3,dtype='float32').tobytes())
        home=(0.,0.,.32);floor=(0.,.08)
        initial=stair_probe(adapter,b'',1,home,floor,0.,depart=True)
        task=adapter._local_stair_task
        for _ in range(2):
            adapter.observation_task(b'',1,'cancel_local_stair_probe_scan',0,home,0.,0.)
        self.assertEqual(task['scans'],2)
        result=stair_probe(adapter,b'',1,home,floor,0.,depart=True)
        self.assertFalse(result['ready'],result)
        adapter.integrate(np.asarray(tread*3,dtype='float32').tobytes())
        self.assertFalse(stair_probe(adapter,b'',2,home,floor,0.,depart=True)['ready'])
        self.assertEqual(len(adapter._stair_recognitions[floor]),1)

    def test_failed_stair_stances_cannot_restart_an_exhausted_task(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.20})
        flat=[(i*.1,j*.1,0.) for i in range(-18,10) for j in range(-14,15)]
        tread=[(i*.1,j*.1,.13) for i in range(10,34) for j in range(-12,13)]
        adapter.integrate(np.asarray((flat+tread)*3,dtype='float32').tobytes())
        home,floor=(0.,0.,.32),(0.,.08)
        initial=stair_probe(adapter,b'',1,home,floor,0.,depart=True)
        point=initial['observations'][0];pose=(*point[:2],point[2]+.32)
        adapter.observation_task(b'',1,'reject_stair_stance',0,pose,0.,0.)
        stair_probe(adapter,b'',1,home,floor,0.,depart=True)
        self.assertFalse(stair_probe(adapter,b'',1,home,floor,0.,depart=True)['ready'])
        review=stair_probe(adapter,b'',1,home,floor,0.,depart=True,review=True)
        if review['ready']:
            point=review['observations'][0]
            adapter.observation_task(b'',1,'reject_stair_stance',0,(*point[:2],point[2]+.32),0.,0.)
        result=stair_probe(adapter,b'',1,home,floor,0.,depart=True)
        self.assertFalse(result['ready'],result)
        self.assertEqual(adapter._stair_recognitions[floor][0]['failures'],2)
        self.assertFalse(stair_probe(adapter,b'',1,home,floor,0.,depart=True)['ready'])

    def test_pending_distant_stair_tiles_do_not_block_a_local_scout(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.15})
        floor=(0.,.08)
        ground=np.asarray((-1.6,0.,0.))
        flat=[(x*.1,y*.1,0.) for x in range(-30,0) for y in range(-12,13)]
        tread=[(x*.1,y*.1,.06) for x in range(5) for y in range(-8,9)]
        adapter.integrate(np.asarray((flat+tread)*3,dtype='float32').tobytes())
        adapter.prepare(b'',1)
        m=adapter.map
        hints=m.stair_scout_index.query(m,*floor)
        self.assertTrue(hints)
        m.stair_index.dirty.add((99,99))
        m.stair_scout_index.dirty.add((99,99))
        result=stair_probe(adapter,b'',1,(*ground[:2],.32),floor,0.)
        self.assertTrue(result['ready'],result)
        self.assertTrue(adapter._local_stair_task)
        self.assertTrue(adapter._arrival_area((*result['observations'][0][:2],
            result['observations'][0][2]+.32),.15,result['yaws'][0])['ready'])

    def test_local_stair_probe_uses_safe_views_then_saves_and_returns(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.20})
        floor=(0.,.08)
        home=(0.,0.,.32)
        flat=[(x*.1,y*.1,0.) for x in range(-18,12) for y in range(-14,15)]
        first=[(x*.1,y*.1,.13) for x in range(12,18) for y in range(-9,10)]
        higher=[(x*.1,y*.1,.26 if x<24 else .39) for x in range(18,31) for y in range(-9,10)]
        adapter.integrate(np.asarray((flat+first)*3,dtype='float32').tobytes())
        adapter.prepare(b'',1)
        adapter.map.stair_index.query(adapter.map,*floor)
        selected=stair_probe(adapter,b'',1,home,floor,0.)
        self.assertTrue(selected['ready'])
        x,y,z=selected['observations'][0]
        pose=(x,y,z+.32)
        self.assertTrue(adapter._arrival_area(pose,.15,selected['yaw'])['ready'])
        self.assertEqual(adapter.plan_surface(b'',1,home,pose)[0],'ok')
        adapter.observation_task(b'',1,'arm_floor_stair_probe',0,pose,selected['yaw'],0.)
        adapter.integrate(np.asarray(higher*3,dtype='float32').tobytes())
        adapter.observation_task(b'',2,'finish_floor_stair_probe',0,pose,selected['yaw'],0.)
        returned=stair_probe(adapter,b'',2,pose,floor,selected['yaw'])
        if returned.get('finished'):
            self.assertFalse(returned.get('returning',False))
            self.assertTrue(any(f.confirmed for f in adapter._saved_stairs[floor]))
            self.assertIsNone(adapter._local_stair_task)
            return
        if not returned.get('returning'):
            next_stance=returned['observations'][0]
            pose=(*next_stance[:2],next_stance[2]+.32)
            returned=stair_probe(adapter,b'',2,pose,floor,selected['yaw'])
        self.assertTrue(returned['returning'],returned)
        self.assertTrue(any(f.confirmed for f in adapter._saved_stairs[floor]))
        self.assertFalse(adapter.observation_task(b'',2,'finish_local_stair_return',0,pose,0.,.15)['ready'])
        self.assertTrue(adapter.observation_task(b'',2,'finish_local_stair_return',0,home,0.,.15)['ready'])
        self.assertIsNone(adapter._local_stair_task)

    def test_stair_qualification_rejects_low_cached_flights_and_disconnected_entries(self):
        adapter=OnlinePct(str(PCT));adapter.configure({'robot_radius':.15})
        floor=(1.46,.08);base=(-1.,0.,1.78)
        points=[(x*.1,y*.1,1.46) for lo,hi in ((-20,20),(70,100))
                for x in range(lo,hi) for y in range(-12,13)]
        points.extend(((.5,0.,1.59),(8.5,0.,1.59)))
        adapter.integrate(np.asarray(points*3,dtype='float32').tobytes());adapter.prepare(b'',1)
        valid=StairFlight((0,0,1.46),(1,0),((5,0,1.59),),2)
        self.assertTrue(adapter._upward_stair_check(valid,floor,base)['ready'])
        low_entry=StairFlight((0,0,1.39),(1,0),((5,0,1.52),),2)
        low_tread=StairFlight((0,0,1.46),(1,0),((5,0,1.48),),2)
        self.assertEqual(adapter._upward_stair_check(low_entry,floor)['reason'],'stair_entry_outside_floor')
        self.assertEqual(adapter._upward_stair_check(low_tread,floor)['reason'],'stair_first_tread_not_above_floor')
        remote=StairFlight((80,0,1.46),(1,0),((85,0,1.59),),2)
        self.assertEqual(adapter._upward_stair_check(remote,floor,base)['reason'],'stair_entry_disconnected')
        adapter._saved_stairs[floor]=dict.fromkeys((low_entry,low_tread),(*base[:2],1.46,0.))
        with (patch.object(adapter.map.stair_index,'query',return_value=[]),
              patch.object(adapter.map.stair_scout_index,'query',return_value=[])):
            self.assertFalse(adapter._stair_candidates(floor,np.asarray((-1.,0.,1.46)),0.))

    def test_saved_stair_handoff_uses_safe_approach_outside_local_window(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.15})
        floor=(1.46,.08)
        home=(-.9,0.,1.78)
        remote=(-8.5,0.,1.78)
        flat=[(x*.1,y*.1,floor[0]) for x in range(-95,0) for y in range(-14,15)]
        steps=[(x*.1,y*.1,1.59+.13*(x//6)) for x in range(18) for y in range(-11,12)]
        adapter.integrate(np.asarray((flat+steps)*3,dtype='float32').tobytes())
        adapter.prepare(b'',1)
        flight=StairFlight((-3,14,1.46),(1,0),((2,0,1.59),(8,0,1.72),(14,0,1.85)),11)
        self.assertEqual(adapter._upward_stair_check(flight,floor,home)['reason'],
                         'stair_entry_disconnected')
        with patch.object(adapter,'_stair_candidates',return_value=[flight]):
            stair_probe(adapter,b'',1,home,floor,0.)
            saved=adapter._saved_stairs[floor][flight]
            self.assertEqual(saved[:2],home[:2])
            self.assertFalse(stair_probe(adapter,b'',1,remote,floor,0.)['ready'])

            approach=stair_probe(adapter,b'',1,remote,floor,0.,depart=True,review=True)
            self.assertTrue(approach.get('approaching'),approach)
            self.assertFalse(approach.get('climbing',False))
            self.assertEqual(approach['observations'][0][:2],list(home[:2]))
            self.assertEqual(adapter.plan_surface(b'',1,remote,home,floor)[0],'ok')

            adapter.observation_task(b'',1,'reject_stair_stance',0,home,saved[3],.15)
            replacement=stair_probe(adapter,b'',1,remote,floor,0.,depart=True,review=True)
            self.assertTrue(replacement.get('approaching'),replacement)
            x,y,z=replacement['observations'][0]
            self.assertGreater(np.linalg.norm(np.subtract((x,y),home[:2])),.15)
            arrived=(x,y,z+.32)
            self.assertTrue(adapter._arrival_area(arrived,.15,replacement['yaws'][0])['ready'])
            self.assertEqual(adapter.plan_surface(b'',1,remote,arrived,floor)[0],'ok')

            climb=stair_probe(adapter,b'',1,arrived,floor,0.,depart=True,review=True)
            self.assertTrue(climb.get('climbing'),climb)
            self.assertGreater(climb['observations'][0][2],floor[0])

    def test_stair_retry_does_not_reset_for_sideways_motion_or_duplicate_points(self):
        adapter=OnlinePct(str(PCT));adapter.configure({'robot_radius':.20})
        home,floor=(0.,0.,.32),(0.,.08)
        flat=[(x*.1,y*.1,0.) for x in range(-18,12) for y in range(-14,15)]
        first=[(x*.1,y*.1,.13) for x in range(12,18) for y in range(-9,10)]
        adapter.integrate(np.asarray((flat+first)*3,dtype='float32').tobytes())
        probe=stair_probe(adapter,b'',1,home,floor,0.)
        self.assertTrue(probe['ready'])
        adapter.observation_task(b'',1,'cancel_local_stair_probe_scan',0,home,0.,0.)
        stair_probe(adapter,b'',1,home,floor,0.)
        adapter.integrate(np.asarray(first*3,dtype='float32').tobytes())
        self.assertFalse(stair_probe(adapter,b'',2,(0.,.4,.32),floor,0.)['ready'])
        task=adapter._stair_recognitions[floor][0]
        self.assertEqual(task['failures'],1);self.assertEqual(task['scans'],1)
        retry=stair_probe(adapter,b'',2,home,floor,0.,review=True)
        if retry['ready']:
            adapter.observation_task(b'',2,'cancel_local_stair_probe_scan',0,home,0.,0.)
            stair_probe(adapter,b'',2,home,floor,0.)
        self.assertEqual(task['failures'],2)
        self.assertFalse(stair_probe(adapter,b'',2,home,floor,0.,review=True)['ready'])
        higher=[(x*.1,y*.1,.26 if x<24 else .39) for x in range(18,31) for y in range(-9,10)]
        adapter.integrate(np.asarray(higher*3,dtype='float32').tobytes())
        reopened=stair_probe(adapter,b'',3,home,floor,0.,depart=True)
        self.assertTrue(reopened['ready'],reopened)
        self.assertEqual(reopened['diagnostic']['reopen_reason'],'new_upward_tread')
        self.assertEqual(reopened['diagnostic']['failures'],0)

    def test_platform_geometry_permits_only_a_verified_same_level_escape(self):
        adapter = OnlinePct(str(PCT))
        adapter.configure({'robot_radius': .20})
        points = [(i*.1,j*.1,.52) for i in range(-20,21) for j in range(-14,15)]
        adapter.integrate(np.asarray(points*3,dtype='float32').tobytes())
        with patch('legbot_exploration.surface_orientation.classify', return_value=np.full(3,-1,dtype=int)):
            adapter.map.integrate([(.20,0.,.83)]*3,sensor_origin=(0.,0.,.9))
        start, target, floor = (0.,0.,.84), (-.4,0.,.84), (0.,.08)
        self.assertEqual(adapter.landing_check(b'',1,start,floor)['reason'], 'obstacle_footprint')
        self.assertTrue(adapter.landing_recovery_check(b'',1,start,floor)['ready'])
        self.assertTrue(adapter.landing_check(b'',1,target,floor)['ready'])
        self.assertEqual(adapter.plan_shift(b'',1,start,target)[0], 'ok')
        self.assertNotEqual(adapter.plan_shift(b'',1,start,(.4,0.,.84))[0], 'ok')

    def test_landing_requires_level_support_in_a_two_dimensional_area(self):
        adapter=OnlinePct(str(PCT))
        point=np.asarray([0.,0.,.65])
        floor=(0.,.08)
        surfaces=adapter.map.stair_index.surfaces
        def measured_rows(x,y):
            return [(0,dict(measured=True,g_mature=True,solid=False,G=z,terrain_cost=0.))
                    for z in surfaces.get((x,y),())]
        with (patch.object(adapter,'prepare'),
              patch.object(adapter.map,'point_safe_check',
                           return_value=dict(ready=True,ground_z=.33,reason='ready')),
              patch.object(adapter.map,'layers_at',side_effect=measured_rows),
              patch.object(adapter.map.stair_index,'query',return_value=[]) as stairs):
            # A tread is long enough across the stairs, but too shallow.
            surfaces.update({(x,y):(.33,) for x in range(-2,3) for y in range(-12,13)})
            ground=point-[0.,0.,.32]
            self.assertFalse(adapter._stair_platform_visible(ground,floor,0.))
            self.assertEqual(adapter.landing_check(b'',1,point,floor)['reason'],'platform_not_confirmed')
            surfaces.clear()
            surfaces.update({(x,y):(.33,) for x in range(-5,6) for y in range(-4,5)})
            self.assertTrue(adapter._stair_platform_visible(ground,floor,0.))
            self.assertFalse(adapter.landing_check(b'',1,point,floor)['ready'])
            surfaces.update({(x,y):(.33,) for x in range(-8,9) for y in range(-8,9)})
            result=adapter.landing_check(b'',1,point,floor)
            self.assertTrue(result['ready'],result)
            self.assertEqual(result['reason'],'safe_platform')
            del surfaces[(2,2)]
            self.assertTrue(adapter.landing_check(b'',1,point,floor)['ready'])
            surfaces[(2,2)]=(.40,)
            self.assertTrue(adapter.landing_check(b'',1,point,floor)['ready'])
            for y in range(-12,13):
                surfaces[1,y]=(.40,)
                surfaces[-1,y]=(.40,)
            self.assertFalse(adapter.landing_check(b'',1,point,floor)['ready'])
            for y in range(-8,9):
                surfaces[1,y]=surfaces[-1,y]=(.33,)
            surfaces[(2,2)]=(.33,)
            surfaces.update({xy:(.36,) for xy in surfaces if xy!=(0,0)})
            # The center G remains available, while the floor reference is
            # taken from the connected platform. Slice aliases count once.
            surfaces[(0,0)]=(.33,)*400
            result=adapter.landing_check(b'',1,point,floor)
            self.assertAlmostEqual(result['ground_z'],.33)
            self.assertAlmostEqual(result['platform_height'],.36)
            surfaces[(0,0)]=(.40,)
            with patch.object(adapter.map,'point_safe_check',
                              return_value=dict(ready=True,ground_z=.40,reason='ready')):
                self.assertEqual(adapter.landing_check(b'',1,(0.,0.,.72),(.37,.08))['reason'],
                                 'still_on_departure_floor')
            surfaces[(0,0)]=(.33,)
            stairs.return_value=[StairFlight((-5,0,0.),(1,0),
                                             ((-3,0,.20),(0,0,.33),(5,0,.46)),3)]
            # A previously recognized flight must not override the measured
            # platform or trigger staircase indexing during floor confirmation.
            self.assertEqual(adapter.landing_check(b'',1,point,floor)['reason'],
                             'safe_platform')
            stairs.assert_not_called()
            self.assertEqual(adapter.landing_check(b'',1,point,(.33,.08))['reason'],
                             'still_on_departure_floor')

    def test_stair_exit_edge_selects_an_interior_landing_route(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.15})
        point=(.3,0.,.65)
        floor=(0.,.08)
        cloud=np.asarray([(x*.1,y*.1,.33 if x>=0 else .20)
                          for x in range(-12,26) for y in range(-12,13)]*3,dtype='float32')
        adapter.integrate(cloud.tobytes())
        # A connected platform may already qualify at the edge.
        self.assertEqual(adapter.landing_check(b'',1,point,floor)['reason'],'safe_platform')
        result=adapter.frontiers(b'',1,point,continuation=(0.,.28,.15),floor=floor)
        candidates=result['stair_landing_targets']
        self.assertTrue(candidates)
        x,y,z,_=candidates[0]
        self.assertGreaterEqual(x,.6)
        goal=(x,y,z+.32)
        self.assertTrue(adapter.landing_check(b'',1,goal,floor)['ready'])
        reason,route,_=adapter.plan_surface(b'',1,point,goal)
        self.assertEqual(reason,'ok')
        self.assertTrue(route)
        inside=adapter.frontiers(b'',1,goal,continuation=(0.,.28,.15),floor=floor)
        self.assertEqual(inside['stair_landing_targets'],[])

    def test_partial_landing_rejects_stations_with_unobserved_blind_area(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.15})
        points=[(x*.1,y*.1,.52) for x in range(9) for y in range(-11,12)]
        points.extend((x*.1,y*.1,z) for lo,hi,z in ((12,16,.65),(16,20,.78))
                      for x in range(lo,hi) for y in range(-8,9))
        adapter.integrate(np.asarray(points*3,dtype='float32').tobytes())
        pose=(.3,0.,.84)
        departure=(0.,.08)
        floor=(.535,.035)
        route=adapter.frontiers(b'',1,pose,.23,window_size=(4.,1.,1.),
                                continuation=(0.,.50,.10),platform_floor=floor)
        self.assertFalse(adapter._stair_platform_visible(np.asarray((.3,0.,.52)),departure,0.),
                         'a shallow landing must not switch to platform exploration')
        self.assertEqual(adapter.landing_check(b'',1,pose,departure)['reason'],'platform_not_confirmed')
        observers=[p for cluster in route['clusters'] for p in cluster['observations']]
        self.assertTrue(route['clusters'])
        self.assertFalse(observers)
        self.assertEqual(adapter._arrival_area((.55,0.,.84),.15,0.)['reason'],
                         'arrival_unobserved_blind_area')

    def test_stair_flight_ends_at_a_platform_before_next_rising_run(self):
        adapter=OnlinePct(str(PCT))
        def heights(x,y):
            if abs(y)>7 or not -8<=x<52:
                return ()
            if x<0:return (0.,)
            if x<6:return (.13,)
            if x<12:return (.26,)
            if x<25:return (.39,)
            return (.52+.13*((x-25)//6),)
        seeds={(x,y):heights(x,y) for x in range(6) for y in range(-7,8)}
        flights=_find_stairs(adapter.map.settings,0.,.08,seeds,heights)
        aligned=[f for f in flights if f.confirmed and f.direction==(1,0) and f.entry[1]==0]
        self.assertTrue(aligned)
        self.assertTrue(all(f.treads[-1][2]==.39 for f in aligned),aligned)
        self.assertTrue(any(f.continues_from((8,0),.26,.1,.05) for f in aligned))
        self.assertFalse(any(f.continues_from((20,0),.39,.1,.05) for f in aligned))

    def test_next_tread_is_selected_without_a_recognized_flight(self):
        adapter = OnlinePct(str(PCT))
        adapter.configure({'robot_radius': .15})
        cloud = np.asarray([(x*.1, y*.1, .25 if x < 3 else .38 if x < 9 else .51)
                            for x in range(-12, 18) for y in range(-9, 10)]*3, dtype='float32')
        adapter.integrate(cloud.tobytes())
        start = (0., 0., .57)
        result = adapter.frontiers(b'', 1, start, .25, continuation=(0., .2, .15), floor=(0., .08))
        self.assertEqual(result['stair_targets'], [])
        targets = result['stair_step_targets']
        self.assertTrue(targets)
        self.assertTrue(all(np.hypot(x,y) >= .25 and min(abs(z-.38),abs(z-.25)) < .001
                            for x, y, z, _ in targets))
        self.assertAlmostEqual(targets[0][2],.38,places=3)
        x, y, z, _ = targets[0]
        reason, route, _ = adapter.plan_surface(b'', 1, start, (x, y, z+.32))
        self.assertEqual(reason, 'ok')
        self.assertTrue(route)

    def test_stair_recognition_and_handoff_use_departure_floor_height(self):
        adapter=OnlinePct(str(PCT))
        def heights(x,y):
            if abs(y)>6 or not -8<=x<18:
                return ()
            if x<0:
                return (.025,)
            return ((.05,.09,.13)[x//6],)

        seeds={(x,y):heights(x,y) for x in range(6) for y in range(-6,7)}
        flights=_find_stairs(adapter.map.settings,0.,.08,seeds,heights)
        flight=next(f for f in flights if f.confirmed and f.direction==(1,0) and f.entry[1]==0)
        self.assertEqual([t[2] for t in flight.treads],[.05,.09,.13])
        with patch.object(adapter,'_admission',side_effect=lambda p,**_:dict(ready=True,ground_z=p[2]-.32)):
            targets=adapter._stair_targets([flight],(.0,0.,.05),departure_height=0.)
        self.assertTrue(any(x>0. and abs(z-.05)<1e-7 for x,y,z,yaw in targets))

    def test_landing_dimensions_follow_a_diagonal_staircase(self):
        settings=OnlinePct(str(PCT)).map.settings
        self.assertTrue(level_platform(settings,(0,0),(1,1),
                                       lambda x,y: abs(x+y)<=9 and abs(x-y)<=8))
        self.assertFalse(level_platform(settings,(0,0),(1,1),
                                        lambda x,y: abs(x+y)<=5 and abs(x-y)<=17))

    def test_partial_platform_can_be_explored_before_floor_confirmation(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.15})
        def add(points):
            adapter.integrate(np.asarray(points*3,dtype='float32').tobytes())

        add([(x*.1,y*.1,.52 if x>=0 else .39)
             for x in range(-20,9) for y in range(-12,13)])
        start=(.4,0.,.84)
        old_floor=(0.,.08)
        platform=(.535,.035)
        self.assertEqual(adapter.landing_check(b'',1,start,old_floor)['reason'],'platform_not_confirmed')
        result=adapter.frontiers(b'',1,start,.17,(3.,1.,1.),
                                 continuation=(0.,.50,.15),platform_floor=platform)
        self.assertTrue(all(abs(c['frontier'][2]-.52)<.001 for c in result['clusters']))
        observers=[p for c in result['clusters'] for p in c['observations']]
        self.assertFalse(observers)
        x,y,z=.60,0.,.52
        reason,route,_=adapter.plan_surface(b'',1,start,(x,y,z+.32),platform)
        self.assertEqual(reason,'ok')
        self.assertTrue(all(abs(p[2]-.84)<.001 for p in route))
        self.assertEqual(adapter.plan_surface(b'',1,start,(-.6,0.,.71),platform)[0],
                         'pct_route_outside_current_floor')
        add([(x*.1,y*.1,.52) for x in range(9,17) for y in range(-12,13)])
        self.assertTrue(adapter.landing_check(b'',2,start,old_floor)['ready'])

    def test_sparse_height_returns_do_not_trigger_a_global_stair_station_search(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.15})
        floor=[(i*.1,j*.1,0.) for i in range(-30,7) for j in range(-12,13)]
        start=(-2.5,0.,.32)
        scope=(0.,.08)
        for version,points in enumerate((floor,[(.8,0.,.10)],[(1.2,0.,.23)]),1):
            pts=np.asarray(points*3,dtype='float32')
            with patch('legbot_exploration.surface_orientation.classify',
                       return_value=np.ones(len(pts),dtype=int)):
                adapter.integrate(pts.tobytes(),(0.,0.,1.3))
            result=adapter.frontiers(b'',version,start,minimum_distance=.5,
                                     floor=scope,geometry_only=True)
            self.assertEqual(result['stair_flights'],0)
            self.assertEqual(result['stair_targets'],[])
            with patch.object(adapter,'observation_poses',wraps=adapter.observation_poses) as poses:
                probe=stair_probe(adapter,b'',version,start,scope,0.)
                self.assertFalse(probe.get('ready'),probe)
                poses.assert_not_called()

    def test_upper_floor_records_stairs_and_scouts_unconfirmed_treads(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.15})
        def integrate(points, orientation):
            cloud=np.asarray(points*3,dtype='float32')
            with patch('legbot_exploration.surface_orientation.classify',
                       return_value=np.full(len(cloud),orientation)):
                adapter.integrate(cloud.tobytes(),(-1.,0.,2.1))

        floor=(1.46,.08)
        start=(-2.,0.,1.78)
        integrate([(x*.1,y*.1,floor[0]) for x in range(-30,0) for y in range(-12,13)],1)
        def tread(level):
            return [(x*.1,y*.1,floor[0]+.13*(level+1))
                    for x in range(level*5,(level+1)*5) for y in range(-9,10)]

        for level in range(3):
            integrate(tread(level),0)
        scout=adapter.frontiers(b'',1,start,minimum_distance=.5,floor=floor,geometry_only=True)
        self.assertFalse(scout['saved_stair_entrances'])
        self.assertEqual(scout['stair_targets'],[])
        probe=stair_probe(adapter,b'',1,start,floor,0.,depart=True)
        self.assertTrue(probe['ready'],probe)
        x,y,z=probe['observations'][0]
        self.assertTrue(adapter._arrival_area((x,y,z+.32),.15,probe['yaws'][0])['ready'])
        self.assertEqual(adapter.plan_surface(b'',1,start,(x,y,z+.32))[0],'ok')
        for level in range(3):
            integrate(tread(level),1)
        pose=(x,y,z+.32)
        for _ in range(4):
            probe=stair_probe(adapter,b'',2,pose,floor,0.,depart=True)
            if probe.get('climbing'):
                break
            self.assertTrue(probe['ready'],probe)
            x,y,z=probe['observations'][0]
            pose=(x,y,z+.32)
        self.assertTrue(probe['climbing'],probe)
        self.assertTrue(adapter._saved_stairs[floor])
        other=adapter.frontiers(b'',2,start,minimum_distance=.5,floor=(0.,.08),geometry_only=True)
        self.assertFalse(other['saved_stair_entrances'])

    def test_stair_scout_uses_supported_treads_with_inconsistent_normal_labels(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.15})
        floor=(1.46,.08)
        base=(-1.4,0.,1.78)
        def integrate(points, orientation):
            cloud=np.asarray(points*3,dtype='float32')
            with patch('legbot_exploration.surface_orientation.classify',
                       return_value=np.full(len(cloud),orientation)):
                adapter.integrate(cloud.tobytes(),(-1.4,0.,2.1))

        integrate([(x*.1,y*.1,floor[0]) for x in range(-30,0) for y in range(-11,12)],1)
        integrate([(x*.1,y*.1,1.59+(x//6)*.13) for x in range(24) for y in range(-10,11)],-2)
        result=stair_probe(adapter,b'',1,base,floor,0.,depart=True)
        self.assertTrue(adapter.map.stair_index.query(adapter.map,*floor))
        self.assertTrue(result.get('climbing'),result)
        x,y,z=result['observations'][0]
        target=(x,y,z+.32)
        self.assertTrue(adapter._arrival_area(target,.15,result['yaws'][0])['ready'])
        self.assertEqual(adapter.plan_surface(b'',1,base,target)[0],'ok')

    def test_stair_index_preserves_partial_work_when_an_unrelated_column_in_its_tile_changes(self):
        from collections import defaultdict
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':0.,'safety_margin':0.})
        points=[(x*.1,y*.1,0.) for x in range(12) for y in range(12)]
        adapter.integrate(np.asarray(points*3,dtype='float32').tobytes())
        adapter.prepare(b'',1)
        index=adapter.map.stair_index
        index.floor=(0.,.08)
        key=((2,2),(1,0))
        result=[]
        index._building_tile=(0,0)
        index._building={key:result}
        index._building_reads={(2,2),(3,2)}
        index._building_dependencies={key:{(2,2),(3,2)}}
        index._building_users=defaultdict(set,{(2,2):{key},(3,2):{key}})
        index.surfaces[(2,2)]=(.1,)
        index.update(adapter.map,[(10,10)])
        self.assertIs(index._building[key],result)
        index.update(adapter.map,[(2,2)])
        self.assertNotIn(key,index._building)
        self.assertNotIn(key,index._building_dependencies)
        self.assertFalse(index._building_users)

    def test_traversable_stair_index_uses_working_ground_above_a_lower_raw_return(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.15})
        points=[(x*.1,y*.1,0.) for x in range(-20,0) for y in range(-10,11)]
        for x in range(16):
            height=.13+.13*(x//8)
            points.extend((x*.1,y*.1,z) for y in range(-10,11) for z in (height-.07,height))
        adapter.integrate(np.asarray(points*3,dtype='float32').tobytes())
        adapter.prepare(b'',1)
        flights=adapter.map.stair_index.query(adapter.map,0.,.08)
        self.assertTrue(any(f.confirmed for f in flights))
        for flight in flights:
            self.assertGreater(flight.treads[0][2],.1)
            self.assertTrue(adapter._tread_layers(*flight.treads[0]))

    def test_single_six_centimetre_tread_starts_shared_recognition(self):
        ground=[(x*.1,y*.1,0.) for x in range(-30,0) for y in range(-12,13)]
        for rise in (-.10,.04,.06,.17):
            with self.subTest(rise=rise):
                stair=[(x*.1,y*.1,rise) for x in range(0,5) for y in range(-8,9)]
                # Each rise is a separate static map.
                adapter=OnlinePct(str(PCT))
                adapter.configure({'robot_radius':.15})
                adapter.integrate(np.asarray((ground+stair)*3,dtype='float32').tobytes())
                result=adapter.frontiers(b'',1,(-1.6,0.,.32),.5,floor=(0.,.08),geometry_only=True)
                self.assertEqual(result['stair_flights'],0)
                self.assertEqual(result['stair_targets'],[])
                self.assertFalse(result['saved_stair_entrances'])
                probe=stair_probe(adapter,b'',1,(-1.6,0.,.32),(0.,.08),0.)
                self.assertEqual(bool(adapter._stair_recognitions.get((0.,.08))),rise==.06)
                if rise==.06:
                    self.assertTrue(probe['ready'],probe)

    def test_stair_priority_prefers_more_treads_then_nearer_entry(self):
        adapter=OnlinePct(str(PCT))
        def flight(x, count):
            return StairFlight((x,0,0.),(1,0),tuple((x+3*i,0,.06*i) for i in range(1,count+1)),3)
        single, double, near, far=flight(1,1),flight(12,2),flight(23,3),flight(34,3)
        ranks=adapter._rank_stair_entrances((single,far,double,near),(0.,0.,0.))
        self.assertEqual(list(ranks),[near,far,double,single])

    def test_stair_route_uses_high_cut_geometry_and_preserves_low_cut_clearance(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure(dict(origin_z=-.108, robot_radius=.15))

        def integrate(points, orientation):
            pts=np.asarray(points,dtype='float32')
            with patch('legbot_exploration.surface_orientation.classify',
                       return_value=np.full(len(pts),orientation)):
                adapter.integrate(pts.tobytes(),(0.,0.,1.3))

        cross_section=range(-10,11)
        integrate([(i*.1,j*.1,.77637 if i<0 else .8906 if i==0 else .9132)
                   for i in range(-15,16) for j in cross_section]*3,1)
        integrate([(0.,j*.1,.8944) for j in cross_section]*3,-1)
        start,target=(-.7,0.,1.09637),(.7,0.,1.2332)
        reason,_,metadata=adapter.plan_surface(b'',1,start,target)
        self.assertEqual(reason,'ok',metadata)
        high=adapter.map.cell((3,0,0))
        self.assertAlmostEqual(high['G'],.8906,places=5)
        # The separated upper face enters the high cut immediately.
        upper=[(0.,j*.1,.9005) for j in cross_section]
        integrate(upper*2,1)
        self.assertEqual(adapter.plan_surface(b'',2,start,target)[0],'ok')
        self.assertAlmostEqual(high['G'],.9005,places=5)
        self.assertFalse(high['g_mature'])
        integrate(upper,1)
        reason,route,metadata=adapter.plan_surface(b'',3,start,target)
        self.assertEqual(reason,'ok',metadata)
        search=metadata['anchor_search']
        self.assertEqual(search['selected_start_layer'],search['selected_goal_layer'],search)
        self.assertGreater(search['selected_goal_layer']*.5-.108,.9132)
        low=adapter.map.cell((2,0,0))
        self.assertAlmostEqual(low['G'],.8906,places=5)
        self.assertAlmostEqual(low['C'],.8944,places=5)
        self.assertEqual(low['passage_cost'],50.)
        self.assertAlmostEqual(adapter.map.cell((3,0,0))['G'],.9005,places=5)
        self.assertGreater(route[-1][2],route[0][2]+.12)

    def test_stair_planner_reports_unknown_cells_from_endpoint_and_native_search(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'maturity_min_points':1})
        floor=np.asarray([(i*.1,j*.1,0.) for i in range(-20,21) for j in range(-9,10)
                          if not 8<=i<=10]+[(.8,0.,2.98)],dtype='float32')
        adapter.integrate(floor.tobytes())
        adapter.prepare(b'',1)
        start=(0.,0.,.32)
        reason,_,details=adapter.plan_surface_with_blockers(b'',1,start,(.8,0.,.32))
        self.assertEqual(reason,'pct_unsafe_goal',details)
        self.assertTrue(any(abs(x-.8)<1e-7 and abs(y)<1e-7 and abs(z)<1e-7
                            for x,y,z,_ in details['unknown_blockers']),details)
        reason,_,details=adapter.plan_surface_with_blockers(b'',1,start,(1.6,0.,.32))
        self.assertEqual(reason,'pct_route_disconnected',details)
        self.assertTrue(details['anchor_search']['attempts'],details)
        self.assertTrue(any(.79<x<1.01 and abs(z)<1e-7
                            for x,_,z,_ in details['unknown_blockers']),details)

    @staticmethod
    def duplicate_floor_fixture(island_layer=1, island_at='start', other_height=0.):
        """Equal-cost duplicate cuts; one is an island, the other spans the hall."""
        from legbot_exploration.fixed_layer_map import FixedLayerMap, MapSettings
        adapter=OnlinePct(str(PCT))
        m=FixedLayerMap(MapSettings(robot_radius=0.,safety_margin=0.,maturity_min_points=1))
        for k in (0,1):
            for x in range(-6,7):
                island=(x<=-2 if island_at=='start' else x>=2 if island_at=='goal' else abs(x)>=2)
                if k==island_layer and not island:
                    continue
                for y in range(-3,4):
                    row=m.cell((k,x,y),True)
                    row['G']=0. if k==island_layer else other_height
                    # Imported planner-only fixture: no physical column records.
                    row['g_surface_id']=0
                    row['g_mature']=row['measured']=row['feature_valid']=True
                    row['unknown']=row['solid']=row['hard']=row['unknown_hard']=row['solid_hard']=False
                    row['passage_cost']=0.;row['relief_cost']=row['terrain_cost']=row['inflated_terrain_cost']=1.
                    m.column_layers.setdefault((x,y),set()).add(k)
        m.first_layer=0;m.last_layer=1;m.lower=np.array([-6,-3]);m.upper=np.array([7,4])
        m.counts={}
        adapter.map=m
        return adapter

    def test_native_search_falls_back_after_preferred_upper_cut_fails(self):
        adapter=self.duplicate_floor_fixture(island_layer=1,island_at='both')
        reason,route,metadata=adapter.plan(b'',1,(-.4,0,.32),(.4,0,.32))
        self.assertEqual(reason,'ok',metadata)
        search=metadata['anchor_search']
        self.assertEqual(metadata['start_check']['layer'],0)
        self.assertEqual(search['admitted_start_layer'],0)
        self.assertEqual(search['selected_start_layer'],0)
        self.assertEqual([(a['start_layer'],a['goal_layer'],a['found']) for a in search['attempts']],
                         [(1,1,False),(0,0,True)])
        self.assertTrue(route)

    def test_native_search_prefers_upper_common_cut_over_admitted_lower_cut(self):
        for island_at in ('start','both'):
            with self.subTest(island_at=island_at):
                adapter=self.duplicate_floor_fixture(island_layer=0,island_at=island_at)
                reason,route,metadata=adapter.plan(b'',1,(-.4,0,.32),(.4,0,.32))
                self.assertEqual(reason,'ok',metadata)
                search=metadata['anchor_search']
                self.assertEqual((search['selected_start_layer'],search['selected_goal_layer']),(1,1))
                self.assertEqual(len(search['attempts']),1)
                self.assertTrue(route)

    def test_native_search_tries_next_common_upper_cut_before_lower_cut(self):
        adapter=self.duplicate_floor_fixture(island_layer=1,island_at='both')
        m=adapter.map
        for (k,bx,by),block in list(m.blocks.items()):
            if k==0:
                m.blocks[(2,bx,by)]=block.copy()
        for layers in m.column_layers.values():
            layers.add(2)
        m.last_layer=2
        reason,_,metadata=adapter.plan(b'',1,(-.4,0,.32),(.4,0,.32))
        self.assertEqual(reason,'ok',metadata)
        search=metadata['anchor_search']
        self.assertEqual([(a['start_layer'],a['goal_layer'],a['found']) for a in search['attempts']],
                         [(1,1,False),(2,2,True)])

    def test_native_search_can_leave_and_return_to_preferred_upper_cut(self):
        adapter=self.duplicate_floor_fixture(island_layer=1,island_at='both')
        for y in range(-3,4):
            adapter.map.cell((1,-2,y))['inflated_terrain_cost']=2.
            adapter.map.cell((0,2,y))['inflated_terrain_cost']=2.
        reason,_,metadata=adapter.plan(b'',1,(-.4,0,.32),(.4,0,.32))
        self.assertEqual(reason,'ok',metadata)
        search=metadata['anchor_search']
        self.assertEqual((search['selected_start_layer'],search['selected_goal_layer']),(1,1))
        native_cells=adapter.planner.planner.get_result_matrix()
        self.assertIn(0,native_cells[:,0])

    def test_native_search_keeps_a_successful_mixed_slice_anchor_pair(self):
        adapter=self.duplicate_floor_fixture(island_layer=0,island_at='goal')
        adapter.map.cell((1,4,0))['unknown']=True
        for y in range(-3,4):
            adapter.map.cell((1,3,y))['inflated_terrain_cost']=2.
        reason,route,metadata=adapter.plan(b'',1,(-.4,0,.32),(.4,0,.32))
        self.assertEqual(reason,'ok',metadata)
        # Neither cut admits both endpoints. Native A* must switch cuts.
        search=metadata['anchor_search']
        self.assertEqual((search['selected_start_layer'],search['selected_goal_layer']),(1,0))
        self.assertEqual(len(search['attempts']),1)
        self.assertTrue(route)

    def test_anchor_retry_excludes_an_unsafe_equivalent_slice(self):
        adapter=self.duplicate_floor_fixture(island_layer=0)
        adapter.map.cell((1,-4,0))['unknown']=True
        reason,route,metadata=adapter.plan(b'',1,(-.4,0,.32),(.4,0,.32))
        self.assertEqual(reason,'pct_route_disconnected',metadata)
        self.assertEqual(metadata['anchor_search']['start_layers'],[0])
        self.assertFalse(route)

    def test_anchor_retry_respects_floor_band(self):
        adapter=self.duplicate_floor_fixture(island_layer=0,island_at='both',other_height=.1)
        reason,route,metadata=adapter.plan(b'',1,(-.4,0,.32),(.4,0,.32),floor=(0.,.08))
        self.assertEqual(reason,'pct_route_disconnected',metadata)
        self.assertEqual(metadata['anchor_search']['start_layers'],[0])
        self.assertEqual(metadata['anchor_search']['goal_layers'],[0])
        self.assertEqual(len(metadata['anchor_search']['attempts']),1)
        self.assertFalse(route)

    def test_stair_start_uses_safe_higher_slice_when_nearest_ground_is_blocked(self):
        lower, upper = 2.478, 2.535
        adapter = self.duplicate_floor_fixture(island_layer=0, other_height=upper)
        lower_start = adapter.map.cell((0, -4, 0))
        lower_start['G'], lower_start['C'] = lower, upper
        lower_start['solid'] = True
        lower_start['terrain_cost'] = lower_start['inflated_terrain_cost'] = 50.
        adapter.map.cell((1, -4, 0))['inflated_terrain_cost'] = 16.91
        start, goal = (-.4, 0., 2.459+.32), (.4, 0., upper+.32)

        self.assertTrue(adapter.start_ready(b'', 1, start))
        check = adapter.point_check(b'', 1, start)
        self.assertEqual(check['safe_layers'], [1])
        self.assertAlmostEqual(check['ground_z'], upper, places=5)
        self.assertFalse(adapter.start_ready(b'', 1, start, (lower, .04)))
        reason, route, metadata = adapter.plan_surface(b'', 1, start, goal)
        self.assertEqual(reason, 'ok', metadata)
        self.assertEqual(metadata['anchor_search']['selected_start_layer'], 1)
        self.assertTrue(route)

    def test_lowest_cost_selection_and_explicit_goal_use_the_same_height_filter(self):
        adapter = self.duplicate_floor_fixture(island_layer=0, island_at='both', other_height=.10)
        adapter.map.cell((1, -4, 0))['inflated_terrain_cost'] = 0.
        adapter.map.cell((0, 4, 0))['inflated_terrain_cost'] = 0.
        start, goal = (-.4, 0., .32), (.4, 0., .42)

        self.assertEqual(adapter.point_check(b'', 1, start)['safe_layers'], [1, 0])
        self.assertEqual(adapter.point_check(b'', 1, goal)['safe_layers'], [0, 1])
        reason, route, metadata = adapter.plan_surface(b'', 1, start, goal)
        self.assertEqual(reason, 'ok', metadata)
        self.assertEqual(metadata['anchor_search']['goal_layers'], [1])
        self.assertTrue(route)

    def test_native_rdp_output_is_returned_without_post_simplification_map_checks(self):
        adapter=OnlinePct(str(PCT),reference_height=.38)
        adapter.configure({'robot_radius':.2,'safety_margin':.05,'maturity_min_points':1})
        floor=np.asarray([(i*.1,j*.1,0.) for i in range(-20,21) for j in range(-20,21)],dtype='float32')
        adapter.integrate(memoryview(floor));adapter.prepare(b'',1)
        start=np.asarray([-.8,0.,.38]);target=np.asarray([.8,0.,.38])
        expected=np.asarray(adapter.planner.plan(start-[0,0,.38],target-[0,0,.38],optimize=False)).copy()
        expected[:,2]+=.38-.5
        point_check=adapter.map.point_safe_check

        def endpoints_only(point, *args, **kwargs):
            if np.allclose(point,start) or np.allclose(point,target):
                return point_check(point,*args,**kwargs)
            # A separate online edge gate would reject this route. Native RDP
            # output must be passed through without consulting such a gate.
            return {'ready':False,'reason':'unknown_footprint'}

        with patch.object(adapter.map,'point_safe_check',side_effect=endpoints_only) as checks:
            reason,route,metadata=adapter.plan(b'',1,start,target)
        self.assertEqual(reason,'ok',metadata)
        np.testing.assert_allclose(route,expected)
        self.assertEqual(checks.call_count,2)

    def test_display_and_ascent_preview_read_the_actual_planning_map(self):
        from legbot_exploration.pct_inspection import SURFACE_DTYPE
        adapter=OnlinePct(str(PCT));adapter.configure({'robot_radius':.2,'maturity_min_points':1})
        floor=np.asarray([(i*.1,j*.1,.1*i*.1) for i in range(-20,21) for j in range(-15,16)],dtype='float32')
        adapter.integrate(memoryview(np.vstack((floor,[[1.9,1.4,2.0]])).astype('float32')));adapter.prepare(b'',1)
        blocks={key:block.tobytes() for key,block in adapter.map.blocks.items()};revision=adapter.map.revision
        inspected=adapter.inspect(1,(-.8,0,.24),(True,100,True,3.,.1,8,200))
        displayed=np.frombuffer(inspected['surfaces'],dtype=SURFACE_DTYPE)
        self.assertEqual(len(displayed),sum(1 for _ in adapter.map.iter_cells(True)))
        for record,(key,row) in zip(displayed,adapter.map.iter_cells(True)):
            self.assertAlmostEqual(float(record['cost']),adapter.map.planning_cost(key),places=4)
            self.assertEqual(record['source'],1)
            flags=sum(int(row[name])<<bit for bit,name in enumerate(
                ('solid','unknown','solid_hard','unknown_hard')))
            flags|=int(np.isfinite(row['C']))<<4|int(row['g_mature'])<<5|int(row['c_mature'])<<6
            self.assertEqual(record['flags'],flags)
        repeated=adapter.inspect(1,(-.8,0,.24),(True,100,True,3.,.1,8,200))
        self.assertIs(inspected['surfaces'],repeated['surfaces'])
        self.assertIs(inspected['returns'],repeated['returns'])
        self.assertEqual(inspected['path'],repeated['path'])
        self.assertLessEqual(len(inspected['display'])//SURFACE_DTYPE.itemsize,100)
        plane=displayed[displayed['z']<1]
        np.testing.assert_allclose(plane['z'],.1*plane['x'],atol=1e-6)
        self.assertTrue((displayed['z']==2.).any())  # Highest cut is retained, too.
        self.assertEqual(inspected['preview_state'],'ok',inspected['preview_state'])
        self.assertTrue(inspected['path'])
        self.assertGreater(inspected['path'][-1][2],inspected['path'][0][2]+.07)
        self.assertEqual(adapter.map.revision,revision)
        self.assertEqual({key:block.tobytes() for key,block in adapter.map.blocks.items()},blocks)
        disabled=adapter.inspect(1,(-.8,0,.24),(False,100,True,3.,.1,8,200))
        self.assertFalse(disabled['surfaces']);self.assertFalse(disabled['path'])

    def test_static_surface_native_astar_then_new_obstacle(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'maturity_min_points':1})
        x,y=np.meshgrid(np.arange(-2,2.01,.1),np.arange(-2,2.01,.1))
        floor=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,-.02)))
        points=np.vstack((floor+[0,0,-.003],floor+[0,0,.004]))
        adapter.integrate(memoryview(points.astype('float32')));adapter.prepare(b'',1)
        reason,route,metadata=adapter.plan(b'',1,(-.8,0,.32),(.8,0,.32))
        self.assertEqual(reason,'ok',metadata)
        np.testing.assert_allclose(np.asarray(route)[:,2],.297,atol=1e-6)
        self.assertTrue(adapter.recovery_targets(b'',1,(0,0,.32)))
        adapter.integrate(memoryview(np.asarray([[.8,0,.3]],dtype='float32')))
        reason,_,metadata=adapter.plan(b'',2,(-.8,0,.32),(.8,0,.32))
        self.assertEqual(reason,'pct_unsafe_goal',metadata)
        self.assertFalse(metadata['goal_check']['ready'])
        self.assertAlmostEqual(metadata['goal_check']['clearance'],.323,places=5)

    def test_display_deduplicates_cuts_without_hiding_full_planning_data(self):
        from legbot_exploration.pct_inspection import SURFACE_DTYPE
        adapter=OnlinePct(str(PCT));adapter.configure({'robot_radius':0.,'safety_margin':0.,'maturity_min_points':1})
        plane=np.asarray([(x*.1,y*.1,0.) for x in range(-10,11) for y in range(-10,11)],dtype='float32')
        adapter.integrate(memoryview(plane));adapter.prepare(b'',1)
        cuts=[k for k,row in adapter.map.layers_at(0,0) if np.isfinite(row['G'])]
        self.assertGreater(len(cuts),1)
        adapter.map.cell((cuts[-1],0,0))['hard']=True
        adapter.map.cell((cuts[-1],0,0))['unknown_hard']=True
        adapter.map.revision+=1
        result=adapter.inspect(1,(0.,0.,.32),(True,100000,False,3.,.1,8,200))
        full=np.frombuffer(result['surfaces'],dtype=SURFACE_DTYPE)
        display=np.frombuffer(result['display'],dtype=SURFACE_DTYPE)
        xyz=np.column_stack([display[name] for name in ('x','y','z')])
        self.assertEqual(len(np.unique(xyz,axis=0)),len(display))
        at=lambda data:data[(data['x']==0)&(data['y']==0)&(data['z']==0)]
        self.assertGreater(len(at(full)),1)
        self.assertIn(0x808080,at(full)['rgb'].view('<u4'))
        self.assertEqual(len(at(display)),1)
        self.assertEqual(int(at(display)['rgb'].view('<u4')[0]),0x20c860)

    def test_real_native_astar_and_reference_height(self):
        adapter=OnlinePct(str(PCT),reference_height=.38)
        adapter.configure({'robot_radius':.2,'safety_margin':.05,'maturity_min_points':1})
        x,y=np.meshgrid(np.arange(-2,2.01,.1),np.arange(-2,2.01,.1))
        floor=np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)))
        ceiling=floor.copy();ceiling[:,2]=1.5
        adapter.integrate(memoryview(np.vstack((floor,ceiling)).astype('float32')))
        adapter.prepare(b'',1)
        reason,route,metadata=adapter.plan(b'',1,(-.8,-.2,.38),(.8,.4,.38))
        self.assertEqual(reason,'ok',metadata)
        np.testing.assert_allclose(np.asarray(route)[:,2],.38)
        self.assertEqual(adapter.plan(b'',1,(20,0,.38),(.8,.4,.38))[0],'pct_unsafe_start')
        self.assertEqual(adapter.plan(b'',1,(-.8,-.2,.38),(100,100,.38))[0],'pct_unsafe_goal')
        self.assertEqual(adapter.plan_surface(b'',1,(-.8,-.2,.38),(.8,.4,1.88))[0],'pct_route_disconnected')
        self.assertTrue(adapter.start_ready(b'',1,(-.8,-.2,.38)))
        self.assertEqual(adapter.plan_stair_reverse(b'',1,(0,0,.38),(-.3,0,.38))[0],'ok')

    def test_stair_reverse_uses_step_capability_for_the_rear_footprint(self):
        for rise,capability in ((.14,.16),(.17,.16),(.14,.12)):
            with self.subTest(rise=rise,capability=capability):
                adapter=OnlinePct(str(PCT))
                adapter.configure({'maturity_min_points':1,'robot_radius':.35,
                                   'safety_margin':.05,'step_max':capability})
                points=np.asarray([(i*.1,j*.1,0. if i<-6 else rise)
                                   for i in range(-20,21) for j in range(-15,16)],dtype='float32')
                adapter.integrate(points.tobytes())
                base=(0.,0.,rise+.32)
                reason,_,meta=adapter.plan_stair_reverse(b'',1,base,(-.45,0.,base[2]))
                if rise<=capability:
                    self.assertEqual(reason,'ok',meta)
                else:
                    self.assertIn(reason,('pct_unsafe_stair_reverse_stop',
                                         'pct_stair_reverse_rear_support_unverified'),meta)

    def test_old_lower_ground_becomes_a_relief_scan_target_until_new_upper_returns(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'robot_radius':.15})
        points=[(i*.1,j*.1,0. if (i,j)==(0,0) else 1.46)
                for i in range(-15,16) for j in range(-15,16)]
        adapter.integrate(np.asarray(points*3,dtype='float32').tobytes())
        target=(.2,0.,1.78)
        blocked=adapter.observation_task(b'',1,'arrival_area',0,target,0.,.15)
        self.assertFalse(blocked['ready'],blocked)
        self.assertTrue(any(abs(x)<1e-7 and abs(y)<1e-7 and abs(z-1.46)<1e-6
                            for x,y,z,_ in blocked['unknown_blockers']),blocked)
        self.assertTrue(adapter.observation_task(b'',1,'blocker_pending',0,(0.,0.,1.78),0.,0.)['ready'])
        self.assertTrue(adapter.map.cell((3,0,0))['solid'])
        adapter.integrate(np.asarray([(0.,0.,1.46)]*3,dtype='float32').tobytes())
        self.assertTrue(adapter.observation_task(b'',2,'arrival_area',0,target,0.,.15)['ready'])
        self.assertFalse(adapter.observation_task(b'',2,'blocker_pending',0,(0.,0.,1.78),0.,0.)['ready'])

    def test_low_clearance_recheck_targets_the_blocking_surface_without_clearing_it(self):
        adapter=OnlinePct(str(PCT))
        points=[(i*.1,j*.1,1.46) for i in range(-15,16) for j in range(-15,16)]
        points.append((0.,0.,1.60))
        adapter.integrate(np.asarray(points*3,dtype='float32').tobytes())
        adapter.prepare(b'',1)
        self.assertFalse(adapter.map.point_safe_check((0.,0.,1.78),floor=(1.46,.05))['ready'])
        targets=list(adapter.map.unknown_blockers(3,[(0,0)],1.46))
        self.assertEqual(len(targets),1)
        self.assertAlmostEqual(targets[0][2],1.60,places=6)
        self.assertFalse(adapter.map.point_safe_check((0.,0.,1.78),floor=(1.46,.05))['ready'])

    def test_sparse_blind_start_is_not_fabricated_as_safe(self):
        adapter=OnlinePct(str(PCT))
        adapter.configure({'maturity_min_points':1})
        points=np.array([[x,y,0] for x in (2.5,2.6,2.7) for y in (-.1,0,.1)],dtype='float32')
        adapter.integrate(memoryview(points));adapter.prepare(b'',1)
        self.assertFalse(adapter.start_ready(b'',1,(0,0,.32)))
        targets=adapter.recovery_targets(b'',1,(0,0,.32))
        self.assertFalse(targets)  # A sparse strip is not a safe parking region.
        full=np.array([[x*.1+4,y*.1,0] for x in range(-20,21) for y in range(-20,21)],dtype='float32')
        adapter.integrate(memoryview(full));adapter.prepare(b'',2)
        targets=adapter.recovery_targets(b'',2,(0,0,.32))
        self.assertTrue(targets)
        self.assertGreater(np.linalg.norm(np.array(targets[0])[:2]),2.4)
        for p in targets:
            check=adapter.point_check(b'',2,(p[0],p[1],p[2]+.32))
            self.assertTrue(check['ready'],check)
        self.assertGreater(adapter.frontiers(b'',2)['safe_cells'],0)


if __name__=='__main__':
    unittest.main()
