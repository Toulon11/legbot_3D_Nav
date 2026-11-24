"""Views must fit the physical sensor; arrival/version changes cannot complete them."""
import unittest
from unittest.mock import patch
import numpy as np
from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
from legbot_exploration.pct_observation import ObservationModel, ray_columns


class ObservationTests(unittest.TestCase):
    def mapping(self,holes=((15,0),),**settings):
        m=FixedLayerMap(MapSettings(maturity_min_points=3,robot_radius=0.,safety_margin=0.,**settings))
        points=np.array([(x*.1,y*.1,0.) for x in range(-20,21) for y in range(-20,21)
                         if (x,y) not in holes],np.float32)
        m.integrate(np.repeat(points,3,axis=0));m.build()
        return m

    def test_near_blind_region_and_facing(self):
        m=self.mapping();v=m.frontier_index.observation_model
        points=np.array([[.5,0.,0.],[1.5,0.,0.],[-1.5,0.,0.],[1.5,0.,1.5]])
        np.testing.assert_array_equal(v.visible([0,0,0],0,points,False),[False,True,False,False])
        np.testing.assert_array_equal(v.visible([0,0,0],np.pi,points,False),[False,False,True,False])

    def test_arrival_view_reserves_stop_tolerance_outside_lidar_blind_range(self):
        m=self.mapping();v=m.frontier_index.observation_model;search=v.pose_search
        base=np.asarray((0.,0.,.32));target=[(1.,0.,0.)]
        self.assertTrue(v.visible((0.,0.,0.),0.,target)[0])
        self.assertFalse(v.visible((0.,0.,0.),0.,target,range_margin=.22)[0])
        # The same ray cached without a margin cannot admit a future arrival.
        near=search.observation_poses(base,(0.,0.,0.),target,
            stances=[(base,0,True)],headings=[0.])
        self.assertFalse(near['poses'])
        far=base-np.asarray((.3,0.,0.))
        admitted=search.observation_poses(base,(0.,0.,0.),target,
            stances=[(far,0,True)],headings=[0.])
        self.assertTrue(admitted['poses'],admitted)

    def test_blocker_stations_include_farther_observed_ground_before_candidate_cap(self):
        m=self.mapping();search=m.frontier_index.observation_model.pose_search
        target=np.asarray(((0.,0.,0.),))
        stations=search._stances(np.asarray((0.,0.,.32)),(0.,0.,0.),target,None,None,None,None)
        self.assertLessEqual(len(stations),128)
        self.assertGreater(max(np.linalg.norm(stand[:2]) for stand,_,_ in stations),1.6)

    def test_wrong_floor_cannot_be_credited(self):
        m=self.mapping();v=m.frontier_index.observation_model
        v.task=dict(keys=[(1,15,0)],heights=[0.],serial=m.integrated_serial,armed=True)
        m.integrate(np.array([[1.5,0.,3.]]*3));m.build()
        p=v.progress();self.assertFalse(p['ready']);self.assertFalse(p['fresh']);self.assertEqual(p['gain'],0)

    def test_arrival_surface_mismatch_is_not_reported_as_invisible_frontier(self):
        m=self.mapping();index=m.frontier_index;model=index.observation_model
        cid=index.task_id(next(iter(index.by_column[(15,0)])))
        result=model.begin(cid,[0.,0.,.03],current_yaw=0.)
        self.assertFalse(result['ready'])
        self.assertEqual(result['reason'],'arrival_surface_height_mismatch')
        self.assertGreater(result['diagnostic']['near_frontiers'],0)
        check=result['diagnostic']['checks'][0]
        self.assertEqual(check['rejections']['arrival_surface_height_mismatch'],1)
        self.assertAlmostEqual(check['rejected_poses'][0]['stance'][2],.35)

    def test_cross_layer_sweep_retains_forward_direction_without_visible_targets(self):
        m=self.mapping();v=m.frontier_index.observation_model
        # Its ordinary frontier is behind the ascent direction.
        cid=m.frontier_index.task_id(next(iter(m.frontier_index.by_column[(15,0)])))
        result=v.begin(cid,[0.,0.,0.],forward=np.pi,half_angle=.35)
        self.assertTrue(result['ready'])
        self.assertEqual(result['yaw'],np.pi)
        self.assertEqual(result['gain'],0)
        self.assertEqual(v.task['keys'],[])

    def test_flat_view_keeps_current_yaw_only_if_it_covers_the_frontier(self):
        m=self.mapping();index=m.frontier_index;v=index.observation_model
        cid=index.task_id(next(iter(index.by_column[(15,0)])))
        result=v.begin(cid,[0.,0.,0.],half_angle=.35,current_yaw=.1)
        self.assertAlmostEqual(result['yaw'],.1)
        self.assertIsNone(result['side_yaw'])
        result=v.begin(cid,[0.,0.,0.],half_angle=.35,current_yaw=np.pi)
        self.assertAlmostEqual(result['yaw'],0.)
        self.assertIsNone(result['side_yaw'])
        v.arm([0,0,0],0.)
        v.finish([0,0,0],0.,0.)
        result=v.begin(cid,[0.,0.,0.],half_angle=.35,current_yaw=.1)
        self.assertFalse(result['ready'],'keeping the current yaw must respect completed views')

    def test_flat_view_selects_one_side_and_only_records_it_after_arming(self):
        m=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.))
        m.integrate(np.array([(x*.1,y*.1,0.) for x in range(-20,21) for y in range(-20,21)
                              if x!=11 or abs(y)>14],np.float32))
        m.build()
        index=m.frontier_index;v=index.observation_model
        cid=index.task_id(next(iter(index.by_column[(11,0)])))
        point=[0.,0.,0.]
        result=v.begin(cid,point,half_angle=.35,current_yaw=0.)
        self.assertAlmostEqual(result['yaw'],0.)
        self.assertAlmostEqual(abs(result['side_yaw']),.35)
        self.assertGreaterEqual(result['side_gain'],4)
        primary=set(v.task['keys'])
        side={key for key,_ in v.task['side_keys']}
        self.assertTrue(side.isdisjoint(primary))
        v.arm(point,0.)
        self.assertEqual(set(v.task['keys']),primary)
        v.arm(point,result['side_yaw'])
        self.assertEqual(set(v.task['keys']),primary|side)

    def test_known_ordinary_cell_does_not_confirm_a_fresh_target_height(self):
        m=self.mapping();v=m.frontier_index.observation_model
        v.task=dict(keys=[(1,15,0,(-1,0,1))],heights=[.14],serial=m.integrated_serial,armed=True)
        with patch('legbot_exploration.surface_orientation.classify',return_value=np.full(3,-1)):
            m.integrate([[1.5,0.,0.]]*3,sensor_origin=(0.,0.,.5))
        m.build()
        result=v.progress()
        self.assertEqual(result['gain'],1)
        self.assertFalse(result['fresh'])
        self.assertFalse(result['ready'])
        self.assertFalse(m.surface_observation.has_frontier_ground((15,0),.14,layer=1))
        m.integrate([[1.5,0.,.14]]);m.build()
        self.assertTrue(v.progress()['ready'])

    def test_points_above_and_below_do_not_imply_a_fresh_middle_observation(self):
        m=self.mapping();v=m.frontier_index.observation_model
        m.integrate(np.array([[1.5,0.,0.]]*3));m.build()
        v.task=dict(keys=[(1,15,0)],heights=[0.],serial=m.integrated_serial,armed=True)
        m.integrate(np.array([[1.5,0.,-3.],[1.5,0.,3.]]*3));m.build()
        self.assertFalse(v.progress()['fresh'])
        self.assertFalse(v.progress()['ready'])

    def test_first_integrated_local_hit_completes(self):
        m=self.mapping();v=m.frontier_index.observation_model
        v.task=dict(keys=[(1,15,0)],heights=[0.],serial=m.integrated_serial,armed=True)
        m.integrate(np.array([[8.,8.,0.]]*3));m.build()
        self.assertFalse(v.progress()['ready'])
        m.integrate(np.array([[1.5,0.,0.]]));m.build()
        self.assertTrue(v.progress()['fresh']);self.assertTrue(v.progress()['ready'])
        m.integrate(np.array([[1.5,0.,0.]]*2));m.build()
        self.assertTrue(v.progress()['ready'])
        v.arm([0,0,0],0.)
        self.assertFalse(v.progress()['ready'],'old frames must not satisfy a newly armed view')
        m.integrate(np.array([[1.5,0.,0.]]));m.build()
        self.assertTrue(v.progress()['ready'],'ignored static repeats still prove fresh local integration')

    def test_completed_direction_does_not_hide_opposite_view(self):
        m=self.mapping();v=m.frontier_index.observation_model
        targets=np.array([[1.5,0,0],[-1.5,0,0]])
        view=v._visibility_view([0,0,0],targets,[(0,0,0,0,.8)],False)
        self.assertIsNotNone(view);self.assertAlmostEqual(abs(view['yaw']),np.pi)

    def test_completed_sweep_keeps_actual_asymmetric_angular_coverage(self):
        m=self.mapping();v=m.frontier_index.observation_model
        v.task=dict(keys=[(1,15,0)],heights=[0.],view_yaws=[0.,-.35])
        v.finish([0,0,0],-.35,.35)
        record=next(iter(m.frontier_index.completed_views))
        self.assertAlmostEqual(record[3],-.35)
        self.assertAlmostEqual(record[4],-m.settings.ground_lidar_horizontal_half_angle)
        self.assertAlmostEqual(record[5],m.settings.ground_lidar_horizontal_half_angle+.35)

    def test_known_wall_occludes_expected_ground(self):
        m=self.mapping();v=m.frontier_index.observation_model
        m.integrate(np.array([[.8,0,.3]]*3));m.build()
        self.assertFalse(v.visible([0,0,0],0,[[1.5,0,0]])[0])

    def test_cached_open_ray_is_invalidated_by_a_first_wall_hit(self):
        m=self.mapping();v=m.frontier_index.observation_model
        self.assertTrue(v.visible([0,0,0],0,[[1.5,0,0]])[0])
        self.assertTrue(v.visible([0,0,0],0,[[1.5,0,0]])[0])
        m.integrate(np.array([[.8,0,.3]]));m.build()
        self.assertFalse(v.visible([0,0,0],0,[[1.5,0,0]])[0])

    def grid_barrier(self,mapping,key):
        original=mapping.observed_cell
        row=original((1,8,0)).copy()
        row['solid']=True;row['terrain_cost']=50.
        return patch.object(mapping,'observed_cell',side_effect=lambda candidate:
                            row if tuple(candidate)==key else original(candidate))

    def test_known_obstacle_grid_blocks_even_when_lidar_ray_passes_above_returns(self):
        m=self.mapping();v=m.frontier_index.observation_model
        sensor=np.array([m.settings.ground_lidar_x,0.,.52]);target=[1.5,0.,0.]
        self.assertFalse(v.ray_blocked(sensor,target))
        self.assertTrue(v.visible([0.,0.,0.],0.,[target])[0])
        with self.grid_barrier(m,(1,8,0)):
            self.assertFalse(v.ray_blocked(sensor,target),'optical ray alone cannot reject this view')
            self.assertFalse(v.visible([0.,0.,0.],0.,[target])[0])
        self.assertTrue(v.visible([0.,0.,0.],0.,[target])[0],
                        'removing a grid obstacle must invalidate the rejected visibility cache too')

    def test_lower_cut_wall_band_blocks_high_cut_view_and_invalidates_its_cache(self):
        m=self.mapping();v=m.frontier_index.observation_model;target=[[1.5,0.,0.]]
        self.assertTrue(v.visible([0.,0.,0.],0.,target)[0])
        column=m.columns[(4,0)]
        column.remember_obstacles([.18,.4],m.settings.minimum_clearance)
        self.assertIsNone(column.obstacle_at(.5),'wall stops below the selected cut')
        self.assertTrue(m.ground_band_obstacle((4,0),0.))
        self.assertFalse(v.visible([0.,0.,0.],0.,target)[0])

    def test_ascending_view_does_not_treat_cumulative_tread_rise_as_a_wall(self):
        m=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.))
        m.integrate([(x*.1,y*.1,.13*sum(x>=edge for edge in (4,8,12)))
                     for x in range(-20,21) for y in range(-20,21)]);m.build()
        v=m.frontier_index.observation_model
        # Individual risers are below 16 cm. The final tread's cumulative
        # rise exceeds it; other measured treads are not same-floor walls.
        for x,lo,hi in [(4,.0,.13),(8,.13,.26),(12,.26,.39)]:
            m.columns[(x,0)].remember_obstacles([lo,hi],m.settings.minimum_clearance)
        self.assertFalse(v.grid_view_blocked([0.,0.,0.],[1.5,0.,.39],1,{}))

    def test_obstacle_in_an_independent_other_slice_does_not_hide_this_floor(self):
        m=self.mapping();v=m.frontier_index.observation_model
        with self.grid_barrier(m,(6,8,0)):
            self.assertTrue(v.visible([0.,0.,0.],0.,[[1.5,0.,0.]])[0])

    def test_unknown_footprint_inflation_is_not_a_known_intervening_obstacle(self):
        m=self.mapping();v=m.frontier_index.observation_model;original=m.observed_cell
        row=original((1,8,0)).copy()
        row['hard']=row['solid_hard']=True;row['inflated_terrain_cost']=50.
        self.assertFalse(row['solid']);self.assertLess(row['terrain_cost'],50.)
        with patch.object(m,'observed_cell',side_effect=lambda key:
                          row if tuple(key)==(1,8,0) else original(key)):
            self.assertTrue(v.visible([0.,0.,0.],0.,[[1.5,0.,0.]])[0])

    def test_grid_barrier_at_a_diagonal_corner_cannot_be_skipped(self):
        m=self.mapping();v=m.frontier_index.observation_model
        target=[[1.5,1.5,0.]]
        self.assertTrue(v.visible([0.,0.,0.],np.pi/4,target)[0])
        with self.grid_barrier(m,(1,8,7)):
            self.assertFalse(v.visible([0.,0.,0.],np.pi/4,target)[0])

    def test_pose_search_rejects_target_across_known_grid_obstacle(self):
        m=self.mapping();v=m.frontier_index.observation_model
        base=np.array([0.,0.,.32])
        with self.grid_barrier(m,(1,8,0)):
            result=v.pose_search.observation_poses(base,(0.,0.,0.),[[1.5,0.,0.]],
                stances=[(base,0,True)],headings=[0.],diagnostics=True)
            self.assertFalse(result['poses'],result)
            self.assertEqual(result['rejections'],{'lidar_occluded':1})

    def test_partial_cluster_only_credits_unblocked_targets(self):
        m=self.mapping();v=m.frontier_index.observation_model
        targets=[[1.5,0.,0.],[1.5,.7,0.]];base=np.array([0.,0.,.32])
        with self.grid_barrier(m,(1,8,0)):
            np.testing.assert_array_equal(v.visible([0.,0.,0.],0.,targets),[False,True])
            result=v.pose_search.observation_poses(base,(0.,0.,0.),targets,
                stances=[(base,0,True)],headings=[0.])
            view=result['views'][tuple(base)]
            self.assertEqual(view['gain'],1)
            self.assertEqual(view['target'],targets[1])

    def test_dispatch_uses_visible_member_instead_of_hidden_cluster_representative(self):
        m=self.mapping(holes={(15,0),(15,1),(15,2)},fast_min_visible_cells=1);index=m.frontier_index
        v=index.observation_model;base=np.array([0.,0.,.32]);point=(0.,0.,0.)
        cid=index.task_id(next(iter(index.by_column[(15,0)])))
        with self.grid_barrier(m,(1,8,0)):
            found=v.pose_search.observation_poses(base,(0.,0.,0.),
                [[1.5,0.,0.],[1.5,.2,0.]],stances=[(base,0,True)],headings=[0.])
            view=found['views'][tuple(base)]
            self.assertEqual(view['target'],[1.5,.2,0.])
            def observers(members,*args,**kwargs):
                if not any(k[1:3]==(15,0) for k in members):return []
                index.observer_cache.last_views={point:view}
                return [list(point)]
            with patch.object(index,'_observers',side_effect=observers):
                result=index.query(point,window_size=(2.,.5,.5),floor=(0.,.08))
            cluster=next(c for c in result['clusters'] if c['id']==cid)
            self.assertEqual(cluster['frontier'],[1.5,.2,0.])
            self.assertEqual(cluster['cell'],[15,2])
            self.assertEqual(cluster['observations'],[list(point)])

    def test_vertical_wall_band_blocks_between_sample_heights(self):
        m=self.mapping();v=m.frontier_index.observation_model
        self.assertTrue(v.visible([0,0,0],0,[[1.5,0,0]])[0])
        column=m.columns[(8,0)]
        for _ in range(3):column.add(.26,.05,3,orientation=-2)
        column.remember_obstacles([.26],m.settings.minimum_clearance)
        self.assertFalse(v.visible([0,0,0],0,[[1.5,0,0]])[0])

    def test_ray_checks_cells_touched_at_grid_corners(self):
        start=np.array([0.,0.,.5]);target=np.array([.2,.2,0.])
        columns={xy:(first,last) for xy,first,last in ray_columns(start,target,MapSettings())}
        self.assertEqual(columns[(1,0)],(.25,.25))
        self.assertEqual(columns[(0,1)],(.25,.25))

    def test_fast_floor_admits_tiny_clusters_and_defers_actual_low_progress(self):
        m=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.,
                                   floor_global_review_enabled=False))
        m.integrate(np.array([(x*.1,y*.1,0.) for x in range(-20,21) for y in range(-20,21)
                              if (x,y)!=(15,0) and (x!=11 or abs(y)>9)],np.float32))
        m.build()
        index=m.frontier_index;v=index.observation_model
        tiny=index.task_id(next(iter(index.by_column[(15,0)])))
        cid=index.task_id(next(iter(index.by_column[(11,0)])))
        base=np.array([0.,0.,0.])
        result=index.query(base,floor=(0.,.08),fast=True)
        self.assertIn(tiny,{c['id'] for c in result['clusters']})
        self.assertEqual(result['low_yield_deferred_clusters'],0)
        self.assertTrue(index.by_column[(15,0)],'size filtering must preserve unknown evidence')
        v.floor_attempt=index.begin_floor_attempt(cid)
        view=v.begin(cid,base,half_angle=.35,current_yaw=0.,floor_retry=True)
        self.assertTrue(view['ready'],view)
        self.assertIsNone(view['side_yaw'])
        v.arm(base,view['yaw'])
        m.integrate(np.array([[0.,0.,0.]],np.float32));m.build()
        result=v.finish(base,view['yaw'],0.)
        self.assertEqual(result['floor_attempt'],'first_no_progress')
        self.assertTrue(index.by_column.get((11,0)))

if __name__=='__main__':unittest.main()
