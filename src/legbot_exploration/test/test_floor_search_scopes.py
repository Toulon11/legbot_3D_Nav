"""A failed local window must not consume a large task's global first pass."""
import unittest
from dataclasses import replace
from unittest.mock import patch

from legbot_exploration.fixed_layer_map import FixedLayerMap, MapSettings


FLOOR=(.1,.08)
BASE=(0.,-.4,.1)
HOLES={(0,y) for y in range(-2,3)}


def mapping(holes=HOLES):
    m=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.,
        traversability_kernel_size=3,inflation_kernel_size=1,
        floor_global_review_enabled=False,fast_exploration=True))
    m.integrate([(x*.1,y*.1,z) for x in range(-18,19) for y in range(-18,19)
                 if (x,y) not in holes for z in (.1,2.8)],sensor_origin=(0.,0.,.42))
    m.build()
    return m


def query(index, local=False, **options):
    return index.query(BASE,window_size=(.7,.5,.5) if local else None,
                       floor=FLOOR,selection_layer=3,fast=True,**options)


def target_cluster(index,result,holes=HOLES):
    return next(c for c in result['clusters'] if
                any(k[1:3] in holes for k in index.floor_query_members[c['id']]))


def fail_local(index,reason='floor_no_observation'):
    with patch.object(index,'_observers',return_value=[]):
        local=query(index,True)
    cid=target_cluster(index,local)['id']
    attempt=index.begin_floor_attempt(cid)
    assert attempt.local_search and not attempt.review
    outcome=index.settle_floor_attempt(attempt,True,failure_reason=reason)
    return cid,attempt,outcome


class FloorSearchScopeTest(unittest.TestCase):
    def test_local_failure_still_gets_an_executable_global_first_pose_outside_window(self):
        m=mapping();index=m.frontier_index
        cid,_,outcome=fail_local(index)
        self.assertEqual(outcome,'first_no_progress')
        self.assertTrue(index.floor_work_available(FLOOR,3))
        point=(1.5,0.,.1);windows=[]
        def global_pose(members,base,distance,completed,component,window,deadline,**kwargs):
            if not any(k[1:3] in HOLES for k in members):return []
            windows.append(window)
            index.observer_cache.last_views={point:dict(gain=5,yaw=0.)}
            return [list(point)]
        with patch.object(index,'_observers',side_effect=global_pose):
            result=query(index)
        cluster=target_cluster(index,result)
        self.assertEqual(cluster['id'],cid)
        self.assertEqual(cluster['observations'],[list(point)])
        self.assertEqual(windows,[None])
        self.assertEqual(cluster['attempt'],1)
        self.assertFalse(result['reviewing'])
        self.assertEqual(result['global_first_after_local_failure_cells'],5)
        self.assertFalse(index.begin_floor_attempt(cid).local_search)

    def test_static_hits_and_another_local_query_cannot_repeat_failed_local_search(self):
        m=mapping();index=m.frontier_index
        fail_local(index)
        m.integrate([(.1,0.,.1)],sensor_origin=(0.,0.,.42));m.build()
        calls=[]
        def probe(members,*args,**kwargs):
            if any(k[1:3] in HOLES for k in members):calls.append(members)
            return []
        with patch.object(index,'_observers',side_effect=probe):
            query(index,True)
        self.assertFalse(calls)
        self.assertTrue(index.floor_work_available(FLOOR,3))

    def test_global_first_failure_defers_the_task_without_enabling_second_review(self):
        m=mapping();index=m.frontier_index
        fail_local(index)
        with patch.object(index,'_observers',return_value=[]):result=query(index)
        cluster=target_cluster(index,result)
        attempt=index.begin_floor_attempt(cluster['id'])
        self.assertFalse(attempt.local_search or attempt.review)
        index.settle_floor_attempt(attempt,True,failure_reason='floor_no_observation')
        calls=[]
        with patch.object(index,'_observers',side_effect=lambda members,*a,**k:calls.append(members) or []):
            done=query(index)
        self.assertFalse(any(k[1:3] in HOLES for members in calls for k in members))
        self.assertFalse(done['reviewing'] or done['pending_review_clusters'])
        self.assertTrue(all(xy in index.by_column for xy in HOLES))

    def test_all_local_rejection_reasons_leave_a_large_task_for_global_first(self):
        for reason in ('floor_known_barrier','floor_disconnected','floor_observer_timeout',
                       'floor_route_timeout','floor_no_route'):
            with self.subTest(reason=reason):
                m=mapping();index=m.frontier_index
                fail_local(index,reason)
                with patch.object(index,'_observers',return_value=[]):result=query(index)
                c=target_cluster(index,result)
                self.assertEqual(c['attempt'],1)
                self.assertFalse(result['reviewing'])

    def test_tiny_fragment_of_a_large_cluster_does_not_defer_its_global_first_pass(self):
        m=mapping();index=m.frontier_index
        with patch.object(index,'_observers',return_value=[]):
            local=index.query((0.,-.3,.1),window_size=(.11,.5,.5),
                              floor=FLOOR,selection_layer=3,fast=True)
        self.assertEqual(local['low_yield_deferred_cells'],0)
        with patch.object(index,'_observers',return_value=[]):result=query(index)
        self.assertEqual(target_cluster(index,result)['cells'],5)
        self.assertFalse(result['reviewing'])

    def test_globally_small_cluster_is_offered_in_both_first_pass_windows(self):
        holes={(0,0),(0,1)};m=mapping(holes);index=m.frontier_index;calls=[]
        def probe(members,*args,**kwargs):
            if any(k[1:3] in holes for k in members):calls.append(members)
            return []
        with patch.object(index,'_observers',side_effect=probe):
            query(index,True);result=query(index)
        self.assertEqual(len(calls),2)
        self.assertEqual(result['low_yield_deferred_cells'],0)
        self.assertTrue(all(xy in index.by_column for xy in holes))

    def test_attempt_keeps_its_local_scope_while_other_queries_run(self):
        m=mapping();index=m.frontier_index
        with patch.object(index,'_observers',return_value=[]):
            local=query(index,True)
            cid=target_cluster(index,local)['id'];attempt=index.begin_floor_attempt(cid)
            query(index)
        index.settle_floor_attempt(attempt,True,failure_reason='floor_no_observation')
        self.assertTrue(index.floor_local_failures)
        self.assertFalse(index.floor_failures)

    def test_local_failure_does_not_mark_members_outside_the_local_window(self):
        m=mapping();index=m.frontier_index
        with patch.object(index,'_observers',return_value=[]):
            result=index.query((0.,-.2,.1),window_size=(.21,.5,.5),
                               floor=FLOOR,selection_layer=3,fast=True)
        cid=target_cluster(index,result)['id'];attempt=index.begin_floor_attempt(cid)
        self.assertEqual({k[1:3] for k in attempt.members},{(0,-2),(0,-1),(0,0)})
        index.settle_floor_attempt(attempt,True,failure_reason='floor_no_observation')
        self.assertNotIn((0,2),index.floor_local_failures)

    def test_rekeyed_cluster_is_classified_by_its_current_full_members(self):
        m=mapping();index=m.frontier_index
        with patch.object(index,'_observers',return_value=[]):
            result=index.query((0.,-.2,.1),window_size=(.21,.5,.5),
                               floor=FLOOR,selection_layer=3,fast=True)
        cid=target_cluster(index,result)['id'];attempt=index.begin_floor_attempt(cid)
        # Two local members resolve while the dog travels. Three members of
        # the full boundary remain, including two outside the original view.
        m.integrate([(0.,y,z) for y in (-.2,-.1) for z in (.1,2.8)],
                    sensor_origin=(.2,0.,.42));m.build()
        attempt=replace(attempt,cluster=0)  # The old component ID is gone.
        index.settle_floor_attempt(attempt,True,failure_reason='floor_no_observation')
        self.assertIn((0,0),index.floor_local_failures)
        self.assertNotIn((0,0),index.floor_failures)

    def test_new_measured_safe_ground_reopens_local_failures(self):
        m=mapping();index=m.frontier_index
        fail_local(index)
        self.assertIn((0,0),index.floor_local_failures)
        m.integrate([(0.,y,z) for y in (.1,.2) for z in (.1,2.8)],
                    sensor_origin=(.2,0.,.42));m.build()
        self.assertNotIn((0,0),index.floor_local_failures)

    def test_global_search_shares_route_rejections_but_reconsiders_partial_views(self):
        for route_rejected in (True,False):
            with self.subTest(route_rejected=route_rejected):
                m=mapping();m.settings=replace(m.settings,floor_four_direction_search=True)
                index=m.frontier_index;tried=(-.4,0.,.1);other=(1.5,0.,.1)
                def local_rejection(members,*args,**kwargs):
                    search=kwargs['direction_search'];search.tested_poses.add(tried)
                    if route_rejected:search.rejected.add(tried)
                    return []
                options=(None,None,.15,1.5,1.,.2)
                with patch.object(index,'_observers',side_effect=local_rejection):
                    local=query(index,True,floor_selection=options)
                cid=target_cluster(index,local)['id'];attempt=index.begin_floor_attempt(cid)
                self.assertIn(tried,attempt.tested_poses)
                index.settle_floor_attempt(attempt,True,failure_reason=(
                    'floor_no_route' if route_rejected else 'floor_no_observation'))
                checked=[];chosen=other if route_rejected else tried
                def global_pose(members,*args,**kwargs):
                    if not any(k[1:3] in HOLES for k in members):return []
                    checked.append(tried in kwargs['rejected'])
                    index.observer_cache.last_views={chosen:dict(gain=5,yaw=0.)}
                    return [list(chosen)]
                with patch.object(index,'_observers',side_effect=global_pose):
                    result=query(index,floor_selection=options)
                self.assertEqual(checked,[route_rejected])
                self.assertEqual(target_cluster(index,result)['observations'],[list(chosen)])



if __name__=='__main__':unittest.main()
