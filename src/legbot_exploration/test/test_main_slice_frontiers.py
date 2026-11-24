"""One scheduling cut, shared physical ground and strict obstacle adjacency."""
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
from legbot_exploration.online_pct import OnlinePct
from legbot_exploration.pct_frontier_index import rebuild_frontiers


def room(g=.1,c=2.8,holes=(),native=True,**settings):
    m=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.,
        traversability_kernel_size=3,inflation_kernel_size=1,**settings),native_updates=native)
    points=[(x*.1,y*.1,z) for x in range(-8,9) for y in range(-8,9)
            if (x,y) not in holes for z in (g,c)]
    m.integrate(points,sensor_origin=(0.,0.,g+.32));m.build()
    return m


class MainSliceFrontierTest(unittest.TestCase):
    def test_observer_reserves_arrival_error_inside_two_meter_boundary(self):
        m=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.,
            traversability_kernel_size=3,inflation_kernel_size=1),native_updates=True)
        m.integrate([(x*.1,y*.1,z) for x in range(-25,26) for y in range(-25,26)
                     if (x,y)!=(0,0) for z in (.1,2.8)],sensor_origin=(0.,0.,.42));m.build()
        index=m.frontier_index;key=next(k for k in index.by_column[0,0] if k[0]==1)
        poses=index._observers({key},base=(0.,-.5,.1))
        self.assertTrue(poses)
        reserve=.15+m.settings.resolution/np.sqrt(2.)
        for pose in poses:
            target=index.observer_cache.last_views[tuple(pose)]['target']
            distance=np.linalg.norm(np.asarray(pose[:2])-target[:2])
            self.assertLessEqual(distance,index.radius-reserve+1e-7)
            self.assertLessEqual(distance+.15,index.radius)

    def test_observing_the_only_member_completes_instead_of_deferring_small_gain(self):
        m=room(holes={(0,0)},floor_global_review_enabled=False)
        index=m.frontier_index;point=(0.,-.2,.1)
        def one_visible(members,*args,**kwargs):
            index.observer_cache.last_views={point:dict(gain=1,yaw=0.)}
            return [list(point)]
        with patch.object(index,'_observers',side_effect=one_visible):
            result=index.query(point,window_size=(.5,.5,.5),floor=(.1,.08),selection_layer=3,fast=True)
        attempt=index.begin_floor_attempt(result['clusters'][0]['id'])
        model=index.observation_model
        model.task=dict(keys=list(attempt.members),heights=[b.height for b in attempt.members.values()],
                        floor_attempt=attempt,floor_frame_received=True,view_yaws=[0.],
                        floor_covered={(key[1:3],boundary):set() for key,boundary in attempt.members.items()})
        m.integrate([(0.,0.,.1),(0.,0.,2.8)]);m.build()
        result=model.finish(point,0.,0.)
        self.assertEqual(result['floor_attempt'],'progress')
        self.assertEqual(result['resolved_frontier_cells'],1)
        self.assertEqual(result['minimum_resolved_frontier_cells'],1)
        self.assertFalse(index.floor_failures or index.floor_blacklist)

    def test_single_cell_cluster_gets_its_first_view_then_low_progress_is_deferred(self):
        m=room(holes={(0,0)},floor_global_review_enabled=False)
        index=m.frontier_index;point=(0.,-.2,.1)
        def one_visible(members,*args,**kwargs):
            self.assertEqual(kwargs['minimum_gain'],1)
            index.observer_cache.last_views={point:dict(gain=1,yaw=0.)}
            return [list(point)]
        with patch.object(index,'_observers',side_effect=one_visible):
            first=index.query(point,window_size=(.5,.5,.5),floor=(.1,.08),selection_layer=3,fast=True)
        self.assertEqual(first['low_yield_deferred_cells'],0)
        cid=first['clusters'][0]['id'];attempt=index.begin_floor_attempt(cid)
        outcome=index.settle_floor_attempt(attempt,True,(*point,0.,-.5,.5),failure_reason='floor_low_progress')
        self.assertEqual(outcome,'first_no_progress')
        with patch.object(index,'_observers',side_effect=AssertionError('second review disabled')):
            again=index.query(point,window_size=(.5,.5,.5),floor=(.1,.08),selection_layer=3,fast=True)
        self.assertFalse(again['clusters'])
        self.assertIn((0,0),index.by_column)

    def test_large_cluster_with_one_visible_member_is_admitted(self):
        m=room(holes={(0,0),(0,1),(0,2)},floor_global_review_enabled=False)
        index=m.frontier_index;point=(0.,-.2,.1)
        def one_visible(members,*args,**kwargs):
            index.observer_cache.last_views={point:dict(gain=1,yaw=0.)}
            return [list(point)]
        with patch.object(index,'_observers',side_effect=one_visible):
            result=index.query(point,window_size=(.5,.5,.5),floor=(.1,.08),selection_layer=3,fast=True)
        self.assertTrue(result['clusters'])
        self.assertEqual(result['low_yield_deferred_cells'],0)
        attempt=index.begin_floor_attempt(result['clusters'][0]['id'])
        index.settle_floor_attempt(attempt,True,(*point,0.,-.5,.5),failure_reason='floor_low_progress')
        self.assertFalse(index.floor_local_failures)
        self.assertTrue(index.floor_failures,'actual low progress enters review directly')

    def test_high_cut_frontier_can_use_open_ground_observer_on_lower_cut(self):
        m=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=.2,safety_margin=0.,
            traversability_kernel_size=3,inflation_kernel_size=1),native_updates=True)
        points=[(x*.1,y*.1,.1) for x in range(-25,26) for y in range(-25,26) if (x,y)!=(0,8)]
        points += [(x*.1,y*.1,2.8) for x in range(-25,26) for y in range(4,26) if (x,y)!=(0,8)]
        m.integrate(points,sensor_origin=(0.,0.,.42));m.build()
        index=m.frontier_index
        key=next(n for n in index.nodes if n[0]==3 and n[1:3]==(0,8))
        fid=index.components.node_fragment[key]
        cache=index.observer_cache
        data=cache.candidates(fid,3,{key})
        self.assertTrue(any(p[1]<0 for p in data),data)
        # The query borrowed a real stance; it did not fill its high-cut G.
        self.assertIsNotNone(m.observed_cell((3,0,-5)))
        self.assertIsNone(m.cell((3,0,-5)))
        self.assertIsNotNone(m.observed_cell((1,0,-5)))
        parking=index.observation_model.pose_search._arrival_area((0.,-.5,.42),.15,None)
        self.assertTrue(parking['ready'],parking)
        cache.invalidate_surfaces([(1,0,-5,.1)])
        self.assertNotIn((fid,3),cache.cache)

    def test_primary_cut_uses_measured_height_plus_sixteen_centimeters(self):
        m=room(g=1.46,c=2.8)
        layer,info=m.frontier_selection_layer((0.,0.,1.46))
        self.assertEqual(layer,4);self.assertEqual(info['height'],2.)
        self.assertEqual(info['reason'],'preferred')

    def test_low_ceiling_uses_a_valid_internal_cut_instead_of_losing_the_room(self):
        m=room(g=1.35,c=1.95)
        layer,info=m.frontier_selection_layer((0.,0.,1.35))
        self.assertEqual(info['preferred'],4);self.assertEqual(layer,3)
        self.assertEqual(info['reason'],'ceiling_limited')
        self.assertLess(m.observed_cell((layer,0,0))['G'],info['height'])
        self.assertLess(info['height'],m.observed_cell((layer,0,0))['C'])

    def test_exact_grid_threshold_uses_a_strictly_higher_cut(self):
        m=room(g=.34,c=2.8)
        self.assertEqual(m.frontier_selection_layer((0.,0.,.34))[0],2)

    def test_query_and_work_check_only_use_selected_cut_without_merging(self):
        m=room(holes={(0,0)});index=m.frontier_index
        result=index.query((-.4,0.,.1),floor=(.1,.08),evaluate_observers=False,selection_layer=3)
        self.assertTrue(result['clusters'])
        for c in result['clusters']:
            self.assertEqual({key[0] for key in index.task_cluster(c['id'])['members']},{3})
        self.assertTrue(index.floor_work_available((.1,.08),3))
        self.assertFalse(index.floor_work_available((.1,.08),100))
        self.assertTrue(any(key[0]!=3 for key in index.nodes))

    def test_online_queries_report_and_use_the_main_cut(self):
        root=Path(__file__).resolve().parents[3]/'install/pct_planner/share/pct_planner'
        a=OnlinePct(str(root));a.map=room(g=1.46,c=2.8,holes={(0,0)})
        a.prepare(b'',1)
        result=a.frontiers(b'',1,(-.4,0.,1.78),floor=(1.46,.08),query_budget=.01,
            floor_selection=(.15,.1,.1,.01,2,.16,.08))
        self.assertEqual(result['selection_layer'],4)
        self.assertEqual(result['map']['frontier_selection_slice']['selected'],4)
        self.assertTrue(all({key[0] for key in a.map.frontier_index.task_cluster(c['id'])['members']}=={4}
                            for c in result['clusters']))

    def test_open_ground_on_both_sides_of_grid_zero_does_not_make_false_frontiers(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.,
                    traversability_kernel_size=3,inflation_kernel_size=1),native_updates=native)
                m.integrate([(x*.1,y*.1,-.002 if x<0 else .002)
                    for x in range(-8,9) for y in range(-8,9)],sensor_origin=(0.,0.,.5));m.build()
                self.assertIsNotNone(m.observed_cell((1,-1,0)))
                self.assertTrue(m.frontier_index.nodes,'unobserved outer boundary must remain')
                self.assertFalse(any(key[1:3] in m.columns for key in m.frontier_index.nodes))
                self.assertEqual(dict(m.frontier_index.nodes),dict(rebuild_frontiers(m).nodes))

    def test_old_open_ground_remains_known_without_confirming_a_target_height(self):
        m=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.,
            traversability_kernel_size=3,inflation_kernel_size=1))
        m.integrate([(x*.1,y*.1,0.) for x in range(-8,9) for y in range(-8,9)],
                    sensor_origin=(0.,0.,.5));m.build()
        m.integrate([(x*.1,y*.1,1.) for x in range(3) for y in range(-2,3)],
                    sensor_origin=(0.,0.,1.5));m.build()
        self.assertNotIn((-1,0),m.layers[3].frontiers.by_column)
        self.assertIsNotNone(m.observed_cell((3,-1,0)))
        self.assertTrue(m.frontier_observed(3,(-1,0)))
        self.assertFalse(m.surface_observation.has_frontier_ground((-1,0),1.,layer=3))

    def test_a_new_same_ground_hit_removes_an_existing_frontier_on_another_cut(self):
        m=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.,
            traversability_kernel_size=3,inflation_kernel_size=1))
        m.integrate([(x*.1,y*.1,.002) for x in range(9) for y in range(-8,9)],
                    sensor_origin=(0.,0.,.5));m.build()
        index=m.frontier_index
        self.assertIn((-1,0),m.layers[1].frontiers.by_column)
        m.integrate([(-.1,0.,-.002)],sensor_origin=(0.,0.,.5));m.build()
        self.assertIsNotNone(m.observed_cell((1,-1,0)))
        self.assertIsNone(m.cell((1,-1,0)))
        self.assertIsNone(m.observed_cell((11,-1,0)),'bounded to G+5 m')
        self.assertNotIn((-1,0),m.layers[1].frontiers.by_column)
        self.assertEqual(dict(index.nodes),dict(rebuild_frontiers(m).nodes))

    def test_unknown_touching_both_free_ground_and_known_wall_is_excluded_on_every_cut(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=room(holes={(0,0),(0,1),(-4,0)},native=native)
                self.assertIn((0,0),m.layers[2].frontiers.by_column)
                m.integrate([(.1,0.,float(z)) for z in np.arange(.1,3.01,.025)],
                            sensor_origin=(0.,0.,.42));m.build()
                for k in (1,2,3,4,5):
                    self.assertTrue(m.observed_cell((k,1,0))['solid'])
                    self.assertNotIn((0,0),m.layers[k].frontiers.by_column)
                    self.assertNotIn((0,1),m.layers[k].frontiers.by_column)
                    self.assertNotIn((1,0),m.layers[k].frontiers.by_column)
                    self.assertIn((-4,0),m.layers[k].frontiers.by_column)
                self.assertEqual(dict(m.frontier_index.nodes),dict(rebuild_frontiers(m).nodes))

    def test_unknown_inflation_is_not_mistaken_for_an_observed_wall(self):
        m=room(holes={(0,0)})
        row=m.cell((2,1,0));row['hard']=row['unknown_hard']=True
        row['inflated_terrain_cost']=50.
        m.frontier_index.update({(1,0)})
        self.assertIn((0,0),m.layers[2].frontiers.by_column)

    def test_finite_interval_alias_remains_known_without_allocating_a_neighbor(self):
        m=room();old=m.cell((3,0,0)).copy()
        m.cell((3,0,0))['G']=np.nan;m.cell((3,0,0))['unknown']=True
        m._index_column_intervals((0,0));before=len(m.blocks)
        with patch.object(m,'iter_cells',side_effect=AssertionError('whole map scan')):
            row=m.observed_cell((3,0,0))
        self.assertEqual(float(row['G']),float(old['G']))
        self.assertEqual(float(row['C']),float(old['C']))
        self.assertEqual(len(m.blocks),before)


if __name__=='__main__':unittest.main()
