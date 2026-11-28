"""Incomplete maps may yield safe navigation segments, never invented support."""
import unittest
import numpy as np
from unittest.mock import patch
from test_online_pct import native_wrapper,PCT
from legbot_exploration.online_pct import OnlinePct


class GoalProgressTest(unittest.TestCase):
    def test_ascent_stop_constraint_filters_during_search_without_restricting_full_routes(self):
        a=self.adapter()
        cloud=np.asarray([(x*.1,y*.1,z) for x in range(-20,13) for y in range(-12,13)
                          for z in ((0. if x<0 else -.1),1.5)],dtype=np.float32)
        a.integrate(cloud.tobytes());a.prepare(b'',1)
        start=(-1.,0.,.32);unknown_goal=(3.,0.,.82)
        reason,path,meta=a.plan_progress_surface(b'',1,start,unknown_goal)
        self.assertEqual(reason,'pct_partial_route',meta)
        self.assertLess(path[-1][2]-.32,-.05)
        reason,path,meta=a.plan_progress_surface(b'',1,start,unknown_goal,minimum_stop_ground=-.05)
        self.assertEqual(reason,'pct_partial_route',meta)
        self.assertGreaterEqual(path[-1][2]-.32,-.05-1e-7)
        # A valid complete route may traverse different heights. The constraint
        # limits fallback stops, not the complete navigation route.
        reason,path,meta=a.plan_progress_surface(b'',1,start,(.7,0.,.22),minimum_stop_ground=-.05)
        self.assertEqual(reason,'ok',meta)
        self.assertFalse(meta['partial'])

    def test_absent_review_candidates_do_not_settle_an_explicit_parking_gap(self):
        a=self.adapter();self.patch(a,-20,12,1)
        with patch.object(a.map,'unknown_blockers',return_value=[]):
            result=a.observation_task(b'',1,'blocker_state',0,(3.,0.,.32),0.,0.)
        self.assertTrue(result['ready']);self.assertFalse(result['resolved'])
        self.assertEqual(result['reason'],'observation_pending')

    def adapter(self):
        native_wrapper()
        a=OnlinePct(str(PCT))
        a.configure(dict(robot_radius=.15,safety_margin=0.,maturity_min_points=1))
        return a

    @staticmethod
    def patch(a,lo,hi,version,ground=0.):
        cloud=np.asarray([(x*.1,y*.1,z) for x in range(lo,hi+1) for y in range(-12,13)
                          for z in (ground,ground+1.5)],dtype=np.float32)
        a.integrate(cloud.tobytes());a.prepare(b'',version)

    def test_unknown_goal_progress_then_complete_after_measured_extension(self):
        a=self.adapter();self.patch(a,-20,12,1)
        start=(-1.,0.,.32);target=(3.,0.,.32)
        self.assertEqual(a.plan_surface(b'',1,start,target)[0],'pct_unsafe_goal')
        reason,path,meta=a.plan_progress_surface(b'',1,start,target)
        self.assertEqual(reason,'pct_partial_route',meta)
        self.assertTrue(meta['partial']);self.assertEqual(meta['final_target'],list(target))
        self.assertGreater(path[-1][0],start[0]+.3);self.assertLess(path[-1][0],1.2)
        self.assertTrue(a.observation_task(b'',1,'navigation_arrival',0,path[-1],0.,.15)['ready'])
        self.assertFalse(a.point_check(b'',1,target)['ready'])
        self.patch(a,13,40,2)
        reason,path,meta=a.plan_progress_surface(b'',2,tuple(path[-1]),target)
        self.assertEqual(reason,'ok',meta);self.assertFalse(meta['partial'])
        np.testing.assert_allclose(path[-1],target,atol=.02)

    def test_unknown_start_is_not_replaced_by_a_virtual_anchor(self):
        a=self.adapter();self.patch(a,-20,12,1)
        reason,path,_=a.plan_progress_surface(b'',1,(3.,0.,.32),(0.,0.,.32))
        self.assertEqual(reason,'pct_unsafe_start');self.assertEqual(path,[])

    def test_provisional_segments_do_not_reserve_unexecuted_stops(self):
        a=self.adapter();self.patch(a,-20,12,1)
        start=(-1.,0.,.32);target=(3.,0.,.32)
        reason,first,_=a.plan_progress_surface(b'',1,start,target)
        self.assertEqual(reason,'pct_partial_route')
        reason,second,_=a.plan_progress_surface(b'',1,start,target)
        self.assertEqual(reason,'pct_partial_route')
        np.testing.assert_allclose(first[-1],second[-1])
        self.assertFalse(a.planner._progress_stops)
        self.assertTrue(a.commit_progress_surface(b'',1,target,first[-1]))
        self.assertEqual(sum(map(len,a.planner._progress_stops.values())),1)
        reason,next_path,_=a.plan_progress_surface(b'',1,start,target)
        if reason=='pct_partial_route':
            self.assertGreater(np.linalg.norm(np.asarray(next_path[-1])-first[-1]),.01)

    def test_navigation_wait_ignores_repeated_hits_and_other_height_intervals(self):
        a=self.adapter();self.patch(a,-20,12,1)
        target=(3.,0.,.32);blockers=[[1.3,0.,0.,1]]
        dependency=lambda version:a.observation_task(b'',version,'navigation_dependency',0,target,0.,0.,blockers)
        first=dependency(1)['evidence']
        self.patch(a,-20,12,2)
        self.assertEqual(dependency(2)['evidence'],first)
        # A genuinely new interval, but on another floor, is irrelevant.
        a.integrate(np.asarray([(x*.1,y*.1,z) for x in range(-5,6) for y in range(-5,6)
                                for z in (3.,4.5)],dtype=np.float32).tobytes())
        a.prepare(b'',3)
        self.assertEqual(dependency(3)['evidence'],first)
        self.patch(a,13,40,4)
        self.assertNotEqual(dependency(4)['evidence'],first)

    def test_new_related_observer_support_reopens_missing_goal(self):
        a=self.adapter();self.patch(a,-20,12,1)
        target=(3.,0.,.32)
        first=a.observation_task(b'',1,'navigation_dependency',0,target,0.,0.)['evidence']
        a.integrate(np.asarray([(x*.1,y*.1,z) for x in range(20,23) for y in range(10,13)
                                for z in (0.,1.5)],dtype=np.float32).tobytes());a.prepare(b'',2)
        second=a.observation_task(b'',2,'navigation_dependency',0,target,0.,0.)['evidence']
        self.assertNotEqual(first,second)

    def test_known_wall_is_distinct_from_missing_target_support(self):
        a=self.adapter();self.patch(a,-20,40,1)
        target=(3.,0.,.32)
        before=a.observation_task(b'',1,'navigation_dependency',0,target,0.,0.)
        self.assertEqual(before['reason'],'navigation_dependency')
        a.integrate(np.asarray([(3.,0.,i*.05) for i in range(31)],dtype=np.float32).tobytes())
        a.prepare(b'',2)
        after=a.observation_task(b'',2,'navigation_dependency',0,target,0.,0.)
        self.assertEqual(after['reason'],'navigation_known_obstacle')
        self.assertNotEqual(before['evidence'],after['evidence'])

    def multiple_anchor_search(self, outcomes, expire=False):
        """Real admitted geometry with deterministic native search outcomes."""
        a=self.adapter();self.patch(a,-20,40,1)
        start=np.asarray((-1.,0.,0.));target=np.asarray((3.,0.,0.))
        check=a._admission((*start[:2],.32));goal=a._admission((*target[:2],.32))
        p=a.planner
        self.assertIsNotNone(p.plan(start,target,start_check=check,goal_check=goal,dense=True))
        cells=np.asarray(p.planner.get_result_matrix()).copy()
        self.assertGreater(len(cells),3)
        native=p.planner
        class Search:
            calls=0
            def __getattr__(self,name):return getattr(native,name)
            def search_towards(self,*args):
                result=outcomes[self.calls];self.calls+=1;return result
            def get_result_matrix(self):return cells
            def get_partial_result_matrix(self):return cells[:len(cells)//2]
            def get_blocked_edges(self):return {}
            def timed_out(self):return False
            def get_search_node_count(self):return 1
        search=Search();p.planner=search
        check=dict(check,safe_layers=[check['layer'],check['layer']+1])
        def clock():return 100. if expire and search.calls else 0.
        with patch('legbot_exploration.block_pct.time.monotonic',side_effect=clock):
            path=p.plan(start,target,start_check=check,goal_check=goal,dense=True,
                        allow_partial=True,stop_check=lambda p:(True,1),deadline=1.)
        return p,search,path,cells

    def test_later_complete_anchor_wins_over_earlier_segment(self):
        p,search,path,cells=self.multiple_anchor_search([False,True])
        self.assertEqual(search.calls,2)
        self.assertFalse(p.search_diagnostics.get('partial',False))
        np.testing.assert_allclose(path[-1][:2],(3.,0.),atol=.01)

    def test_segment_is_used_only_after_all_complete_anchors_fail(self):
        p,search,path,_=self.multiple_anchor_search([False,False])
        self.assertEqual(search.calls,2)
        self.assertTrue(p.search_diagnostics['partial'])
        self.assertEqual(p.search_diagnostics['complete_failure'],'disconnected')
        self.assertLess(path[-1][0],3.)

    def test_budget_expiry_preserves_already_found_safe_segment(self):
        p,search,path,_=self.multiple_anchor_search([False],expire=True)
        self.assertEqual(search.calls,1)
        self.assertIsNotNone(path)
        self.assertEqual(p.search_diagnostics['complete_failure'],'timeout')
        self.assertFalse(p._failures,'budget expiry is not a permanent disconnection')

    def test_descending_segment_limits_each_step_not_the_entire_height_change(self):
        a=self.adapter()
        ground=lambda x:.13*((12-x)//6)
        cloud=np.asarray([(x*.1,y*.1,z) for x in range(-30,13) for y in range(-12,13)
                          for z in (ground(x),2.)],dtype=np.float32)
        a.integrate(cloud.tobytes());a.prepare(b'',1)
        reason,path,meta=a.plan_progress_surface(b'',1,(-2.,0.,ground(-20)+.32),(3.,0.,.32))
        self.assertEqual(reason,'pct_partial_route',meta)
        heights=np.asarray(path)[:,2]
        self.assertGreater(heights[0]-heights[-1],.16)
        self.assertTrue((np.abs(np.diff(heights))<=.16+1e-6).all())

    def test_navigation_parking_does_not_require_the_observation_blind_disc(self):
        a=self.adapter();self.patch(a,-20,12,1)
        target=(.7,0.,.32)
        # This fixture's narrow edge is sufficient for parking, while its
        # 0.85 m observation disc reaches unobserved space.
        self.assertTrue(a.observation_task(b'',1,'navigation_arrival',0,target,0.,.15)['ready'])
        self.assertFalse(a.observation_task(b'',1,'arrival_area',0,target,0.,.15)['ready'])
        self.assertEqual(a.plan_progress_surface(b'',1,(-1.,0.,.32),target)[0],'ok')

    def test_measured_upper_tread_can_nominate_a_partial_route_before_parking_is_known(self):
        a=self.adapter()
        a.configure(dict(robot_radius=.15,safety_margin=0.,maturity_min_points=1,
                         traversability_kernel_size=3,inflation_kernel_size=1))
        low,high=4.027,4.166
        cloud=np.asarray([(x*.1,y*.1,z) for x in range(-20,7) for y in range(-12,13)
                          for z in (low if x<4 else high,6.)],dtype=np.float32)
        a.integrate(cloud.tobytes());a.prepare(b'',1)
        start=(-1.,0.,low+.32)
        views=[];pending=True
        for _ in range(10):
            views,_,pending=a._support_views((-1.,0.,low),.25,low+.08,budget=.1,allow_partial=True)
            if views or not pending:break
        self.assertTrue(views,'measured supports were discarded before the common planner')
        target=(*views[0][:2],views[0][2]+.32)
        self.assertFalse(a.observation_task(b'',1,'navigation_arrival',0,target,0.,.15)['ready'])
        reason,path,meta=a.plan_progress_surface(b'',1,start,target)
        self.assertEqual(reason,'pct_partial_route',meta)
        self.assertTrue(a.observation_task(b'',1,'navigation_arrival',0,path[-1],0.,.15)['ready'])
        self.assertGreater(path[-1][0],start[0]+.3)

    def test_resolved_blocker_is_distinct_from_exhausted_observation(self):
        a=self.adapter()
        cloud=np.asarray([(x*.1,y*.1,z) for x in range(-12,13) for y in range(-12,13)
                          for z in (0.,1.5) if not(x==0 and y==0 and z==0.)],dtype=np.float32)
        a.integrate(cloud.tobytes());a.prepare(b'',1)
        query=lambda version:a.observation_task(b'',version,'blocker_state',0,(0.,0.,.32),0.,0.)
        self.assertTrue(query(1)['ready'])
        review=a.map.frontier_index.observation_model.blocker_review
        with patch.object(review,'available',return_value=False):
            self.assertEqual(query(1)['reason'],'blocker_review_exhausted')
            self.assertTrue(query(1)['ready'],'exhausted review must not erase missing evidence')
        a.integrate(np.asarray([(0.,0.,0.)],dtype=np.float32).tobytes());a.prepare(b'',2)
        self.assertEqual(query(2)['reason'],'blocker_resolved')
        self.assertFalse(query(2)['ready'])
        count=a.map.surface_records
        a._ascent_selection={'selected':3};a._support_passes['stale']={}
        self.assertTrue(a.observation_task(b'',2,'refresh_ascent_search',0,(0.,0.,.32),0.,0.)['ready'])
        self.assertIsNone(a._ascent_selection);self.assertFalse(a._support_passes)
        self.assertEqual(a.map.surface_records,count)

    def test_partial_route_reports_the_tread_gap_before_side_boundaries(self):
        a=self.adapter()
        a.configure(dict(robot_radius=.35,safety_margin=.05,maturity_min_points=1))
        points=[(i*.1,j*.1,z) for i in range(-20,8) for j in range(-18,19) for z in (0.,1.5)]
        points += [(i*.1,j*.1,z) for i in range(8,60) for j in range(-10,11) if i!=20
                   for z in (.13*min(4,1+(i-8)//8),1.5)]
        a.integrate(np.asarray(points,dtype=np.float32).tobytes());a.prepare(b'',1)
        reason,path,meta=a.plan_progress_surface(b'',1,(0.,0.,.32),(3.2,0.,.84))
        self.assertEqual(reason,'pct_partial_route',meta)
        self.assertLess(path[-1][0],2.)
        blockers=meta['unknown_blockers']
        self.assertTrue(any(abs(p[0]-2.)<.05 for p in blockers),blockers)
        self.assertTrue(all(abs(p[1])<=.65 for p in blockers),blockers)


if __name__=='__main__':unittest.main()
