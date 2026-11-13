"""Views must fit the physical sensor; arrival/version changes cannot complete them."""
import unittest
import numpy as np
from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
from legbot_exploration.pct_observation import ObservationModel, ray_columns


class ObservationTests(unittest.TestCase):
    def mapping(self):
        m=FixedLayerMap(MapSettings(maturity_min_points=3,robot_radius=0.,safety_margin=0.))
        points=np.array([(x*.1,y*.1,0.) for x in range(-20,21) for y in range(-20,21)
                         if (x,y)!=(15,0)],np.float32)
        m.integrate(np.repeat(points,3,axis=0));m.build()
        return m

    def test_near_blind_region_and_facing(self):
        m=self.mapping();v=m.frontier_index.observation_model
        points=np.array([[.5,0.,0.],[1.5,0.,0.],[-1.5,0.,0.],[1.5,0.,1.5]])
        np.testing.assert_array_equal(v.visible([0,0,0],0,points,False),[False,True,False,False])
        np.testing.assert_array_equal(v.visible([0,0,0],np.pi,points,False),[False,False,True,False])

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

    def test_mature_underside_is_not_tread_observation_gain(self):
        m=self.mapping();v=m.frontier_index.observation_model
        v.task=dict(keys=[(1,15,0,(-1,0,1))],heights=[.14],serial=m.integrated_serial,armed=True)
        m.integrate(np.array([[1.5,0.,0.]]*3));m.build()
        result=v.progress()
        self.assertEqual(result['gain'],0)
        self.assertFalse(result['fresh'])

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

    def test_vertical_wall_band_blocks_between_sample_heights(self):
        m=self.mapping();v=m.frontier_index.observation_model
        self.assertTrue(v.visible([0,0,0],0,[[1.5,0,0]])[0])
        column=m.columns[(8,0)]
        for _ in range(3):column.add(.26,.05,3,orientation=-2)
        self.assertFalse(v.visible([0,0,0],0,[[1.5,0,0]])[0])

    def test_ray_checks_cells_touched_at_grid_corners(self):
        start=np.array([0.,0.,.5]);target=np.array([.2,.2,0.])
        columns={xy:(first,last) for xy,first,last in ray_columns(start,target,MapSettings())}
        self.assertEqual(columns[(1,0)],(.25,.25))
        self.assertEqual(columns[(0,1)],(.25,.25))

    def test_fast_floor_includes_small_clusters_and_defers_after_first_scan(self):
        m=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.))
        m.integrate(np.array([(x*.1,y*.1,0.) for x in range(-20,21) for y in range(-20,21)
                              if (x,y)!=(15,0) and (x!=11 or abs(y)>9)],np.float32))
        m.build()
        index=m.frontier_index;v=index.observation_model
        tiny=index.task_id(next(iter(index.by_column[(15,0)])))
        cid=index.task_id(next(iter(index.by_column[(11,0)])))
        base=np.array([0.,0.,0.])
        result=index.query(base,floor=(0.,.08),fast=True)
        self.assertIn(tiny,{c['id'] for c in result['clusters']})
        self.assertEqual(result['fast_skipped_clusters'],0)
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
