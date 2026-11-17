"""Three-dimensional target scope, global fallback and completed-view lifecycle."""
import unittest
from unittest.mock import patch
import numpy as np
from test_pct_frontier_index import create,floor
from legbot_exploration.pct_stairs import StairFlight,find_stairs


def closed_floor(heights=(0.,),hole=(),bound=16):
    edge=bound+1
    wall=[(x*.1,y*.1,.30) for x in range(-edge,edge+1) for y in range(-edge,edge+1)
          if abs(x)==edge or abs(y)==edge]
    return np.vstack((floor(heights,hole,bound),wall))


class LocalWindowTest(unittest.TestCase):
    def test_cross_layer_xy_minimum_rejects_higher_point_already_in_arrival_circle(self):
        m=create(floor(hole={(0,0)}))
        index=m.frontier_index
        key=next(iter(index.by_column[(0,0)]))
        # Reproduce the failed stair target relative to the measured stop.
        near=[.01086,-.1007,.07179]
        farther=[.01086,-.30,.07179]
        poses=np.asarray([near+[1.5,0.],farther+[1.5,0.]])
        with (patch.object(index.observer_cache,'candidates',return_value=poses),
              patch.object(index.observer_cache,'_safe_views',side_effect=lambda ordered,*args:[list(p) for p in ordered])):
            points=index._observers({key},base=np.zeros(3),minimum_distance=.25,
                                    continuation=(-np.pi/2,-.05,.15))
        self.assertEqual(points,[farther])

    def test_floor_filter_uses_each_boundary_source_height(self):
        m=create(floor((0.,1.5,3.)))
        index=m.frontier_index
        lower=index.query(np.zeros(3),.5,floor=(0.,.08))
        upper=index.query(np.array([0.,0.,1.5]),.5,floor=(1.5,.08))
        for result,height in ((lower,0.),(upper,1.5)):
            self.assertGreater(result['scoped_frontier_cells'],0)
            self.assertLess(result['scoped_frontier_cells'],result['frontier_cells'])
            points=[p for c in result['clusters'] for p in c['display']+c['observations']]
            self.assertTrue(points)
            self.assertTrue(all(abs(p[2]-height)<.08 for p in points))

    def test_stair_requires_continuous_supported_treads_and_preserves_side_boundaries(self):
        # A measured sloped nosing is allowed between the flat plateaus.
        m=create(np.asarray([(x*.1,y*.1,.13*max(0,x//5)-(.06 if x>=5 and x%5==0 else 0))
                             for x in range(-10,25) for y in range(-6,7)]))
        flights=[f for f in find_stairs(m,0.,.08) if len(f.treads)>=3]
        flight=next(f for f in flights if f.entry[:2]==(4,0) and f.direction==(1,0))
        self.assertEqual(len(flight.treads),4)
        self.assertTrue(flight.owns_boundary(5,0,[(4,0,0.,1)],lambda z: abs(z)<.08))
        self.assertFalse(flight.owns_boundary(5,0,[(5,-1,0.,1)],lambda z: abs(z)<.08))
        # Replace the third tread with a mature underside while leaving all
        # its raw height evidence and the higher tread in the map.
        for (x,_),column in m.columns.items():
            if 15<=x<20:
                for surface in column.values():
                    surface.orientation=-1
        self.assertFalse(any(len(f.treads)>=3 for f in find_stairs(m,0.,.08)))

    def test_first_measured_tread_excludes_elevated_entrance_from_floor_count(self):
        stair=StairFlight((0,0,0.),(1,0),((3,0,.10),(8,0,.23),(13,0,.36)),1)
        for height in (0.,.06):
            m=create(floor((height,),hole={(0,0),(5,0)}))
            index=m.frontier_index
            for window in (None,(1.,1.,1.)):
                unscoped=index.query(np.zeros(3),.5,window,stairs=(stair,))
                scoped=index.query(np.zeros(3),.5,window,floor=(0.,.08),stairs=(stair,))
                self.assertEqual(scoped['scoped_frontier_cells'],
                                 unscoped['scoped_frontier_cells']-int(height!=0.))
                self.assertEqual(any(abs(p[0])<1e-6 and abs(p[1])<1e-6
                                     for c in scoped['clusters'] for p in c['display']),height==0.)
                self.assertTrue(any(abs(p[0]-.5)<1e-6 and abs(p[1])<1e-6
                                    for c in scoped['clusters'] for p in c['display']))

    def test_stair_index_registers_and_invalidates_with_local_support_changes(self):
        lower=np.asarray([(x*.1,y*.1,0.) for x in range(-10,0) for y in range(-5,6)])
        def tread(start,height):
            return [(x*.1,y*.1,height) for x in range(start,start+4) for y in range(-5,6)]
        m=create(np.vstack((lower,tread(1,.094))))
        first=m.stair_index.query(m,0.,.08)
        self.assertTrue(first)
        self.assertFalse(any(f.confirmed for f in first))
        m.integrate(tread(7,.23)+tread(13,.366));m.build()
        confirmed=m.stair_index.query(m,0.,.08)
        self.assertTrue(any(f.confirmed and f.direction==(1,0) for f in confirmed))
        with patch('legbot_exploration.pct_stairs._find_stairs',side_effect=AssertionError('rescanned')):
            m.integrate([(100.,100.,0.)]);m.build()
            self.assertEqual(m.stair_index.query(m,0.,.08),confirmed)
        for xy,column in m.columns.items():
            if 13<=xy[0]<=16:
                for surface in column.values():surface.orientation=-1
                m.dirty.add(xy)
        m.build()
        remaining=m.stair_index.query(m,0.,.08)
        self.assertTrue(any(f.confirmed for f in remaining))
        self.assertTrue(all(len(f.treads)<=2 and all(z<.3 for _,_,z in f.treads) for f in remaining))

    def test_two_failed_views_remove_whole_floor_cluster_outside_local_window(self):
        hole={(x,0) for x in range(10)}
        m=create(closed_floor((0.,1.5),hole=hole))
        index=m.frontier_index
        base=np.zeros(3)
        key=next(k for k in index.by_column[(0,0)] if index.nodes[k].height==0.)
        cid=index.task_id(key)
        lower={k for k in index.task_cluster(cid)['members'] if index.nodes[k].height==0.}
        upper={k for k in index.nodes if index.nodes[k].height==1.5}
        self.assertTrue(any(k[1]==9 for k in lower))
        model=index.observation_model
        with patch.object(model,'_admitted_view',return_value=None):
            for at,point in enumerate(([-.6,0.,0.],[0.,.6,0.])):
                query=index.query(base,.1,(.30,1.,1.) if at==0 else None,floor=(0.,.08))
                self.assertEqual(query['reviewing'],bool(at))
                model.floor_attempt=index.begin_floor_attempt(cid)
                if at==0:self.assertLess(len(model.floor_attempt.local),len(lower))
                outcome=model.begin(cid,np.asarray(point),floor_retry=True)
                self.assertEqual(outcome['reason'],'observation_pose_unavailable')
                self.assertEqual(outcome['floor_attempt'],('first_no_progress','blacklisted')[at])
        self.assertTrue(lower <= index.nodes.keys())
        self.assertTrue(upper <= index.nodes.keys())
        scoped=index.query(base,.5,floor=(0.,.08))
        self.assertEqual(scoped['deferred_floor_frontier_cells'],len(hole))
        self.assertNotIn(cid,[c['id'] for c in scoped['clusters']])

        # An unrelated map update and new evidence on the upper sheet
        # cannot restore the suppressed lower boundary.
        m.integrate([(40.,40.,0.),(0.,0.,1.5)]);m.build()
        self.assertNotIn(cid,[c['id'] for c in index.query(base,.5,floor=(0.,.08))['clusters']])
        m.integrate([(0.,0.,0.)]);m.build()
        self.assertFalse(any(index.nodes[k].height==0. for k in index.by_column.get((0,0),())))
        self.assertTrue(any(index.nodes[k].height==0. for k in index.by_column[(1,0)]))
        self.assertTrue(any(index.nodes[k].height==0. for k in index.by_column[(9,0)]))
        scoped=index.query(base,.5,floor=(0.,.08))
        self.assertEqual(scoped['deferred_floor_frontier_cells'],len(hole)-1)
        self.assertFalse(any((1,0) in {k[1:3] for k in keys} for keys in index.floor_query_members.values()))

    def test_review_waits_for_ordinary_clusters_outside_the_local_window(self):
        m=create(closed_floor(hole={(0,0),(15,0)},bound=20))
        index=m.frontier_index
        base=np.asarray([-.5,0.,0.])
        scope=(0.,.08)
        window=(.7,1.,1.)
        local=index.query(base,.1,window,floor=scope)
        cid=local['clusters'][0]['id']
        first=index.begin_floor_attempt(cid)
        self.assertEqual(index.settle_floor_attempt(first,True),'first_no_progress')
        local=index.query(base,.1,window,floor=scope)
        self.assertEqual(local['scoped_frontier_cells'],0)
        self.assertEqual(local['pending_review_clusters'],1)
        self.assertTrue(first.members.keys() <= index.nodes.keys())

        # An ordinary cluster whose observer search yields no pose still
        # takes priority over a waiting review.
        with patch.object(index,'_observers',return_value=[]):
            ordinary=index.query(base,.1,floor=scope)
        self.assertFalse(ordinary['reviewing'])
        self.assertEqual(ordinary['scoped_frontier_cells'],1)
        self.assertNotIn(cid,[c['id'] for c in ordinary['clusters']])

        m.integrate([(1.5,0.,0.)]);m.build()
        review=index.query(base,.1,floor=scope)
        self.assertTrue(review['reviewing'])
        self.assertEqual(review['scoped_frontier_cells'],1)
        self.assertEqual([c['id'] for c in review['clusters']],[cid])
        self.assertIsNotNone(index.begin_floor_attempt(cid))

    def test_new_mature_support_resets_consecutive_failures(self):
        m=create(closed_floor(hole={(x,0) for x in range(6)}),num=3)
        index=m.frontier_index
        base=np.zeros(3)
        cid=index.task_id(next(iter(index.by_column[(5,0)])))
        index.query(base,.5,floor=(0.,.08))
        first=index.begin_floor_attempt(cid)
        self.assertEqual(index.settle_floor_attempt(first,True),'first_no_progress')
        self.assertTrue(index.query(base,.5,floor=(0.,.08))['reviewing'])
        self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(cid),False),'inconclusive')
        self.assertTrue(index.query(base,.5,floor=(0.,.08))['reviewing'])
        second=index.begin_floor_attempt(cid)
        m.integrate([[0.,0.,0.]]*3);m.build()
        self.assertEqual(index.settle_floor_attempt(second,True,sufficient_progress=True),'progress')
        self.assertFalse(index.query(base,.5,floor=(0.,.08))['reviewing'])
        third=index.begin_floor_attempt(cid)
        self.assertEqual(index.settle_floor_attempt(third,True),'first_no_progress')
        self.assertFalse(index.floor_blacklist)

    def test_completed_sweep_requires_new_frames_and_counts_once(self):
        m=create(closed_floor(hole={(0,0)}))
        index=m.frontier_index
        model=index.observation_model
        point=np.asarray([-1.3,0.,0.])
        cid=index.task_id(next(iter(index.by_column[(0,0)])))
        index.query(np.zeros(3),.5,floor=(0.,.08))
        with patch.object(model,'_admitted_view',side_effect=lambda p,t,*args,**kwargs:dict(yaw=0.,gain=len(t),mask=np.ones(len(t),bool))):
            for fresh,expected in ((False,'inconclusive'),(True,'first_no_progress'),(True,'blacklisted')):
                index.query(np.zeros(3),.5,floor=(0.,.08))
                model.floor_attempt=index.begin_floor_attempt(cid)
                view=model.begin(cid,point,floor_retry=True)
                self.assertTrue(view['ready'],view)
                model.arm(point,0.)
                model.arm(point,.3)
                if fresh:m.integrate([[-.6,0.,0.]]);m.build()
                result=model.finish(point,.3,.3)
                self.assertEqual(result['floor_attempt'],expected)
                if not fresh:self.assertFalse(index.completed_views)
                self.assertIsNone(model.finish(point,.3,.3))
        scoped=index.query(np.zeros(3),.5,floor=(0.,.08))
        self.assertNotIn(cid,[c['id'] for c in scoped['clusters']])

    def test_each_unavailable_batch_settles_before_the_next_batch(self):
        holes={(x,0) for x in (-10,-5,0,5,10)}
        m=create(closed_floor(hole=holes))
        index=m.frontier_index
        base=np.zeros(3);floor_scope=(0.,.08)
        ids={index.task_id(next(iter(index.by_column[xy]))) for xy in holes}
        with patch.object(index,'_observers',return_value=[]):
            checked=set()
            for _ in ids:
                result=index.query(base,.1,(.7,1.,1.),.0001,floor=floor_scope)
                for cid in result['no_view_clusters']:
                    self.assertNotIn(cid,checked)
                    checked.add(cid)
                    self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(cid),True),
                                     'first_no_progress')
            self.assertEqual(len(checked),3)
            for _ in range(2*len(ids)):
                result=index.query(base,.1,query_budget=.0001,floor=floor_scope)
                if not result['no_view_clusters']:break
                for cid in result['no_view_clusters']:
                    expected='blacklisted' if result['reviewing'] else 'first_no_progress'
                    self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(cid),True),expected)
        self.assertEqual(result['scoped_frontier_cells'],0)
        self.assertEqual(result['deferred_floor_frontier_cells'],len(holes))
        self.assertTrue(all(xy in index.by_column for xy in holes))

    def test_split_cluster_keeps_the_attempt_of_each_physical_boundary(self):
        m=create(closed_floor(hole={(x,0) for x in range(5)}))
        index=m.frontier_index
        base=np.zeros(3);scope=(0.,.08)
        with patch.object(index,'_observers',return_value=[]):
            first=index.query(base,.1,floor=scope)
            self.assertEqual(len(first['no_view_clusters']),1)
            cid=first['no_view_clusters'][0]
            self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(cid),True),'first_no_progress')
            m.integrate([(0.2,0.,0.)]);m.build()
            review=index.query(base,.1,floor=scope)
            self.assertTrue(review['reviewing'])
            self.assertEqual(len(review['no_view_clusters']),2)
            for part in review['no_view_clusters']:
                self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(part),True),'blacklisted')
            self.assertEqual(index.query(base,.1,floor=scope)['scoped_frontier_cells'],0)
            cross=index.query(base,.1,continuation=(0.,0.,0.),floor=scope,evaluate_observers=False)
            self.assertEqual(cross['scoped_frontier_cells'],4)

    def test_pending_observer_cannot_restart_attempt_after_map_update(self):
        m=create(closed_floor(hole={(0,0),(14,0)},bound=20))
        index=m.frontier_index
        base=np.asarray([-.5,0.,0.]);scope=(0.,.08);window=(.7,1.,1.)
        def always_yield(*args,**kwargs):
            index.observer_cache.pending=True
            return []
        with patch.object(index,'_observers',side_effect=always_yield):
            first=index.query(base,.1,window,.001,floor=scope)
            self.assertTrue(first['query_pending'])
            self.assertEqual(first['timed_out_clusters'],[])
            m.integrate([(40.,40.,0.)]);m.build()
            second=index.query(base,.1,window,.001,floor=scope)
            self.assertFalse(second['query_pending'])
            self.assertEqual(len(second['timed_out_clusters']),1)
            cid=second['timed_out_clusters'][0]
            self.assertEqual(index.settle_floor_attempt(index.begin_floor_attempt(cid),True),'first_no_progress')
            self.assertEqual(index.query(base,.1,window,.001,floor=scope)['scoped_frontier_cells'],0)
            global_result=index.query(base,.1,query_budget=.001,floor=scope)
            self.assertFalse(global_result['reviewing'])
            self.assertNotIn(cid,[c['id'] for c in global_result['clusters']])

    def test_floor_settlement_requires_five_resolved_cells_or_all_of_a_small_view(self):
        for size,resolved,expected in ((6,4,'first_no_progress'),(8,4,'first_no_progress'),
                                       (6,5,'progress'),(2,2,'progress')):
            with self.subTest(size=size,resolved=resolved):
                before_alignment=resolved==4
                hole={(15,y) for y in range(size+int(before_alignment))}
                m=create(closed_floor((0.,1.5),hole=hole,bound=20))
                index=m.frontier_index;model=index.observation_model
                key=next(k for k in index.by_column[(15,0)] if index.nodes[k].height==0.)
                cid=index.task_id(key)
                point=np.zeros(3)
                index.query(point,.1,floor=(0.,.08))
                model.floor_attempt=index.begin_floor_attempt(cid)
                self.assertGreater(len(model.floor_attempt.members),len(hole),'fixture must span duplicate cuts')
                view=model.begin(cid,point,floor_retry=True)
                self.assertTrue(view['ready'])
                if before_alignment:
                    m.integrate([(1.5,size*.1,0.)]);m.build()
                model.arm(point,view['yaw'])
                m.integrate([(1.5,y*.1,0.) for y in range(resolved)]);m.build()
                model.progress()
                result=model.finish(point,view['yaw'],0.)
                self.assertEqual(result['covered_frontier_cells'],size)
                self.assertEqual(result['resolved_frontier_cells'],resolved)
                self.assertEqual(result['minimum_resolved_frontier_cells'],min(5,size))
                self.assertEqual(result['floor_attempt'],expected)

    def test_large_view_requires_ten_percent_reduction_before_resetting_failure(self):
        for resolved,outcome in ((5,'first_no_progress'),(10,'progress')):
            with self.subTest(resolved=resolved):
                m=create(closed_floor(hole={(15,y) for y in range(6)},bound=20))
                index=m.frontier_index;model=index.observation_model
                key=next(k for k in index.by_column[(15,0)] if index.nodes[k].height==0.)
                cid=index.task_id(key);point=np.zeros(3)
                index.query(point,.1,floor=(0.,.08))
                model.floor_attempt=index.begin_floor_attempt(cid)
                view=model.begin(cid,point,floor_retry=True)
                self.assertTrue(view['ready'])
                # Isolate accounting after the visibility/physical-alias
                # pipeline supplies a large view's measured reduction.
                model.arm(point,view['yaw'])
                model.task['floor_frame_received']=True
                with (patch.object(model,'_record_floor_view'),
                      patch.object(index,'floor_observation_counts',return_value=(100,resolved))):
                    result=model.finish(point,view['yaw'],0.)
                self.assertEqual(result['minimum_resolved_frontier_cells'],10)
                self.assertEqual(result['floor_attempt'],outcome)

    def test_budget_resumes_until_every_local_cluster_has_been_considered(self):
        import time
        m=create(floor((0.,1.5,3.)));index=m.frontier_index
        full=index.query(np.zeros(3),.5,(2.,4.,.5))
        expected={c['id'] for c in full['clusters']}
        seen=set()
        original=index._observers
        def slow(*args,**kwargs):
            result=original(*args[:-1],None)
            time.sleep(.002)
            return result
        with patch.object(index,'_observers',side_effect=slow):
            for _ in range(len(expected)+2):
                result=index.query(np.zeros(3),.5,(2.,4.,.5),.0001)
                seen.update(c['id'] for c in result['clusters'] if c['observers_evaluated'])
                self.assertEqual(result['scoped_frontier_cells'],full['scoped_frontier_cells'])
                if not result['query_pending']:break
        self.assertFalse(result['query_pending'])
        self.assertEqual(seen,expected)

    def test_actual_surface_heights_and_both_observers_and_frontiers_are_clipped(self):
        m=create(floor((0.,1.5,3.),bound=15))
        result=m.frontier_index.query(np.zeros(3),.5,(2.,2.,.5))
        self.assertEqual(result['scope'],'local')
        self.assertGreater(result['frontier_cells'],result['scoped_frontier_cells'])
        heights=set()
        for c in result['clusters']:
            for p in c['display']+c['observations']+[c['frontier']]:
                self.assertLessEqual(max(abs(p[0]),abs(p[1])),2.+1e-7)
                self.assertTrue(-.5<=p[2]<=2.)
            heights.update(round(p[2],1) for p in c['observations'])
        self.assertIn(0.,heights);self.assertIn(1.5,heights)
        self.assertNotIn(3.,heights)

    def test_empty_local_does_not_claim_global_complete_and_global_finds_target(self):
        m=create(floor(bound=15)+[10,0,0]);index=m.frontier_index
        local=index.query(np.zeros(3),.5,(2.,2.,1.))
        self.assertEqual(local['clusters'],[])
        self.assertGreater(local['frontier_cells'],0)
        global_=index.query(np.zeros(3),.5)
        self.assertEqual(global_['scope'],'global')
        self.assertTrue(any(c['observations'] for c in global_['clusters']))
        self.assertEqual(local['frontier_cells'],global_['frontier_cells'])

    def test_local_observers_regenerated_from_large_cluster_without_whole_map_scan(self):
        m=create(floor(bound=30));index=m.frontier_index;base=np.array([2.5,0.,0.])
        with patch.object(m,'iter_cells',side_effect=AssertionError('map scan')):
            result=index.query(base,.5,(1.,.5,.5))
        candidates=[p for c in result['clusters'] for p in c['observations']]
        self.assertTrue(candidates)
        self.assertTrue(all(1.5-1e-7<=p[0]<=3.5+1e-7 and abs(p[1])<=1.+1e-7 for p in candidates))
        full_gain=sum(c['gain'] for c in index.physical_tasks()[0].values())
        self.assertLess(sum(c['gain'] for c in result['clusters']),full_gain)

    def test_soft_cost_change_keeps_completed_view_but_safety_change_reopens_it(self):
        m=create(floor(hole={(0,0)},bound=20));index=m.frontier_index
        cid=index.task_id(next(iter(index.by_column[(0,0)])));c=index.task_cluster(cid)
        p=tuple(c['observations'][0]);index.complete(cid,c['revision'],p)
        x,y=map(int,m.indices(p))
        for k,row in m.layers_at(x,y):
            if row['g_mature'] and not row['hard']:row['inflated_terrain_cost']+=.5
        index.update({(x,y)},set())
        self.assertIn(p,index.completed_points)
        for k,row in m.layers_at(x,y):row['hard']=True;row['unknown']=True
        index.update({(x,y)},set())
        self.assertNotIn(p,index.completed_points)

    def test_learning_a_boundary_is_known_does_not_erase_completed_view(self):
        m=create(floor(hole={(0,0)},bound=20));index=m.frontier_index
        cid=index.task_id(next(iter(index.by_column[(0,0)])));c=index.task_cluster(cid)
        p=tuple(c['observations'][0]);index.complete(cid,c['revision'],p)
        m.integrate(np.asarray([[0.,0.,0.]],dtype='float32'));m.build()
        self.assertIn(p,index.completed_points)
        self.assertNotIn((0,0),index.by_column)

    def test_nearby_new_static_boundary_keeps_a_still_safe_completed_view(self):
        m=create(floor(bound=15));index=m.frontier_index
        cid=next(iter(index.clusters));c=index.task_cluster(cid)
        p=tuple(c['observations'][0]);index.complete(cid,c['revision'],p)
        # Extend actual observations at the edge, creating a new unknown
        # boundary nearby. That does not undo arrival at the same safe pose.
        m.integrate(floor(bound=18));m.build()
        self.assertIn(p,index.completed_points)

    def test_completed_local_cache_is_invalidated_and_nearby_views_suppressed(self):
        m=create(floor(hole={(0,0)},bound=20));index=m.frontier_index;base=np.zeros(3)
        before=index.query(base,.5,(2.,1.,1.))
        c=next(c for c in before['clusters'] if c['observations']);p=np.asarray(c['observations'][0])
        index.complete(c['id'],c['revision'],p)
        after=index.query(base,.5,(2.,1.,1.))
        points=[np.asarray(point) for c in after['clusters'] for point in c['observations']]
        self.assertTrue(all(np.linalg.norm(point[:2]-p[:2])>m.settings.observation_revisit_radius or
                            abs(point[2]-p[2])>m.settings.step_max for point in points))

    def test_completion_is_shared_with_a_neighbouring_clusters_old_cache(self):
        m=create(floor(hole={(0,0),(5,0)},bound=25));index=m.frontier_index;base=np.zeros(3)
        cid=index.task_id(next(iter(index.by_column[(0,0)])))
        other=index.task_id(next(iter(index.by_column[(5,0)])))
        self.assertNotEqual(cid,other)
        # Populate the other cluster's local cache before completion.
        local=index.query(base,.5,(3.,1.,1.))
        selected=next(c for c in local['clusters'] if c['id']==cid)
        p=np.asarray(selected['observations'][0])
        index.complete(cid,selected['revision'],p)
        for result in (index.query(base,.5,(3.,1.,1.)),index.query(base,.5)):
            for cluster in result['clusters']:
                for point in cluster['observations']:
                    self.assertTrue(np.linalg.norm(np.asarray(point[:2])-p[:2])>m.settings.observation_revisit_radius or
                                    abs(point[2]-p[2])>m.settings.step_max,(p,cluster))

    def test_completed_lookup_includes_revisit_halo_across_tile_boundary(self):
        m=create(floor(hole={(20,0)},bound=30));index=m.frontier_index
        key=next(iter(index.by_column[(20,0)]));cid=index.task_id(key)
        # This view can come from a different nearby frontier. Its tile is
        # outside this frontier's +/-2m box but overlaps the candidate halo.
        point=(-.5,0.,0.)
        index.complete(cid,index.task_cluster(cid)['revision'],point)
        self.assertIn(point,index._completed_near({key}))


if __name__=='__main__':unittest.main()
