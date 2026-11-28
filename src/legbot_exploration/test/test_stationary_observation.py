"""Current-stance safety and dispatch-bound physical frontier settlement."""
from pathlib import Path
import unittest
from unittest.mock import patch

from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
from legbot_exploration.online_pct import OnlinePct

PCT=Path(__file__).resolve().parents[3]/'install/pct_planner/share/pct_planner'
BASE=(0.,0.,.1)
FLOOR=(.1,.08)


def room(holes=((13,0),)):
    m=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=.1,safety_margin=0.,
        traversability_kernel_size=3,inflation_kernel_size=1,floor_global_review_enabled=False),native_updates=True)
    m.integrate([(x*.1,y*.1,z) for x in range(-22,36) for y in range(-22,23)
                 if (x,y) not in holes for z in (.1,2.8)],sensor_origin=(0.,0.,.42))
    m.build();return m


def members(m,xy=(13,0)):
    return {k for k in m.frontier_index.by_column[xy] if k[0]==3}


def select(m,keys,completed=()):
    engine=m.frontier_index.observation_model.pose_search
    return m.frontier_index.observer_cache.select(keys,BASE,.5,completed,floor=FLOOR,
        admission=(engine.observation_poses,engine._arrival_geometry,.15,True))


def dispatched(m,keys):
    a=OnlinePct(str(PCT));a.map=m;a.prepare(b'',1)
    index=m.frontier_index;cid=index.task_id(next(iter(keys)))
    index.floor_query_members[cid]=keys
    index.floor_query=(FLOOR,lambda key:True,True,0,2,False)
    index.floor_query_local=True
    record=a.observation_task(b'',1,'floor_attempt',cid,(0.,0.,.42),0.,0.)
    assert record['ready'],record
    return a,cid


class StationaryObservationTest(unittest.TestCase):
    def test_saved_route_keeps_its_task_scope_when_global_query_replaces_local_cursor(self):
        m=room();keys=members(m);a=OnlinePct(str(PCT));a.map=m;a.prepare(b'',1)
        index=m.frontier_index;cid=index.task_id(next(iter(keys)))
        index.floor_query_members[cid]=keys
        index.floor_query=(FLOOR,lambda key:True,True,0,2,False)
        index.floor_query_local=True
        preview=a.observation_task(b'',1,'floor_preview_attempt',cid,(0.,0.,.42),0.,0.)
        self.assertTrue(preview['ready'])
        self.assertIsNone(index.observation_model.floor_attempt)
        index.floor_query=None;index.floor_query_members={}
        committed=a.observation_task(b'',1,'floor_attempt',cid,(0.,0.,.42),0.,0.)
        self.assertTrue(committed['ready'])
        attempt=index.observation_model.floor_attempt
        self.assertTrue(attempt.local_search);self.assertEqual(set(attempt.members),keys)
        self.assertFalse(index.observation_model.preview_attempts)

    def test_comparison_direction_does_not_reject_a_feasible_view_or_consume_attempt(self):
        from legbot_exploration.pct_frontier_index import FloorViewSearch
        m=room();keys=members(m);index=m.frontier_index
        a=OnlinePct(str(PCT));a.map=m;a.prepare(b'',1)
        cid=index.task_id(next(iter(keys)))
        search=FloorViewSearch(cid,FLOOR,{k:index.nodes[k] for k in keys})
        search.direction_poses=((0.,0.,.1),(-.2,0.,.1))
        index.floor_view_searches[cid]=search
        target=(*m.position(next(iter(keys))[:3]),index.nodes[next(iter(keys))].height)
        blocked=[[*target,0],[0.,0.,.1,0]]
        result=a.observation_task(b'',1,'floor_compare_view',cid,(0.,0.,.42),0.,.5,blocked)
        self.assertTrue(result['ready'],result)
        self.assertEqual(result['region'],[-.2,0.,.1])
        self.assertGreater(result['gain'],0)
        self.assertFalse(search.rejected);self.assertFalse(search.tested_poses)
        self.assertEqual(search.elapsed,0.)
        self.assertIsNone(index.observation_model.floor_attempt)
        with patch.object(a,'observation_poses',side_effect=AssertionError('expired comparison did work')):
            self.assertFalse(a.observation_task(b'',1,'floor_compare_view',cid,(0.,0.,.42),0.,0.,blocked)['ready'])

    def test_safe_current_stance_precedes_any_moving_candidate_extraction(self):
        m=room();keys=members(m);cache=m.frontier_index.observer_cache
        with patch.object(cache,'candidates',side_effect=AssertionError('moving poses searched before safe current pose')):
            self.assertEqual(select(m,keys),[list(BASE)])
        self.assertGreaterEqual(cache.last_views[BASE]['gain'],1)

    def test_current_stance_with_known_occlusion_is_rejected(self):
        m=room()
        m.integrate([(.6,y*.1,z*.1) for y in range(-2,3) for z in range(2,16)],sensor_origin=(0.,0.,.42));m.build()
        views=select(m,members(m))
        self.assertNotIn(list(BASE),views)
        self.assertTrue(views,'a rejected current stance must still allow safe moving observers')

    def test_blind_area_unknown_cannot_be_bypassed_by_already_standing_there(self):
        m=room(((13,0),(6,0)))
        stop=m.frontier_index.observation_model.pose_search._arrival_area((0.,0.,.42),.15,None)
        self.assertTrue(stop['ready'])
        blind=m.frontier_index.observation_model.pose_search._arrival_area((0.,0.,.42),.15,0.)
        self.assertFalse(blind['ready'])
        views=select(m,members(m))
        self.assertNotIn(list(BASE),views)
        self.assertTrue(views,'a rejected current stance must still allow safe moving observers')

    def test_same_station_may_observe_a_different_physical_boundary(self):
        m=room(((13,0),(14,0)));index=m.frontier_index
        previous=next(iter(members(m,(14,0))))
        record=(*BASE,0.,-3.1416,3.1416)
        index.completed_views[record]=((previous,.1),)
        self.assertEqual(select(m,members(m),(record,)),[list(BASE)])
        self.assertNotIn(list(BASE),select(m,{previous},(record,)))

    def test_resolved_dispatch_does_not_retarget_surviving_cluster_members(self):
        m=room(((13,0),(14,0)));original=members(m)
        a,cid=dispatched(m,original);model=m.frontier_index.observation_model
        pin=model.dispatched_attempt
        self.assertEqual(set(pin.members),original)
        m.integrate([(1.3,0.,z) for z in (.1,2.8)],sensor_origin=(0.,0.,.42))
        self.assertEqual(a.commit_pending(b'',2),cid)
        self.assertIsNone(model.floor_attempt)
        self.assertIs(model.dispatched_attempt,pin)
        self.assertEqual(a.commit_pending(b'',2),0)
        remaining=members(m,(14,0));self.assertTrue(remaining)
        m.frontier_index.floor_query_members[cid]=remaining
        answer=a.observation_task(b'',2,'begin_floor',cid,(0.,0.,.42),0.,0.)
        self.assertFalse(answer['ready']);self.assertEqual(answer['reason'],'frontier_resolved')
        self.assertEqual(answer['floor_attempt'],'progress')
        self.assertFalse(m.frontier_index.floor_failures)
        self.assertTrue(remaining<=m.frontier_index.nodes.keys())

    def test_mutable_query_range_cannot_replace_unresolved_dispatch(self):
        m=room(((13,0),(30,0)));original=members(m)
        a,cid=dispatched(m,original)
        m.frontier_index.floor_query_members[cid]=members(m,(30,0))
        answer=a.observation_task(b'',1,'begin_floor',cid,(0.,0.,.42),0.,0.)
        self.assertTrue(answer['ready'],answer)
        self.assertEqual({k[1:3] for k in m.frontier_index.observation_model.task['members']},
                         {k[1:3] for k in original})


if __name__=='__main__':unittest.main()
