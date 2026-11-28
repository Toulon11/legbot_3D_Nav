"""Persistent clearance-gap intervals and shared sparse-cell consumers."""
import unittest
from unittest.mock import patch

import numpy as np

from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
from legbot_exploration.map_evidence import SurfaceColumn


def mapping(native=True,**options):
    return FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.,
        traversability_kernel_size=3,inflation_kernel_size=1,**options),native_updates=native)


def floor(m):
    m.integrate([(x*.1,y*.1,z) for x in range(-4,0) for y in range(-3,4) for z in (.1,2.8)],
                sensor_origin=(-.2,0.,.42));m.build()


class ColumnObstacleTest(unittest.TestCase):
    def test_chained_gaps_use_clearance_and_keep_equal_or_larger_gaps_open(self):
        c=SurfaceColumn()
        self.assertTrue(c.remember_obstacles([1.4,.3,1.05,.65],.5))
        self.assertEqual(c.obstacle_intervals,((.3,1.4),))
        self.assertEqual(c.obstacle_at(.8),(.3,1.4))
        self.assertIsNone(c.obstacle_at(1.41))
        c=SurfaceColumn();c.remember_obstacles(np.asarray([0.,.5,1.0001],dtype=np.float32),.5)
        self.assertEqual(len(c.obstacle_intervals),3)
        self.assertIsNone(c.obstacle_at(.25))
        self.assertTrue(c.remember_obstacles([.25],.5))
        self.assertEqual(len(c.obstacle_intervals),2)
        self.assertEqual(c.obstacle_at(.25),(0.,.5))

    def test_static_merge_and_clone_do_not_share_mutable_intervals(self):
        c=SurfaceColumn();c.remember_obstacles([.3,.6],.5)
        copy=c.clone();copy.remember_obstacles([.9,1.2],.5)
        self.assertEqual(c.obstacle_intervals,((.3,.6),))
        self.assertEqual(copy.obstacle_intervals,((.3,1.2),))
        self.assertFalse(copy.remember_obstacles([.6,.9],.5))

    def test_wall_without_g_is_known_on_all_covered_cuts_and_has_no_frontier(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=mapping(native);floor(m)
                self.assertIn((0,0),m.layers[2].frontiers.by_column)
                m.integrate([(0.,0.,z) for z in (.55,.9,1.25,1.6)],sensor_origin=(-.1,0.,.2));m.build()
                before=len(m.blocks)
                for k in (2,3):
                    row=m.observed_cell((k,0,0))
                    self.assertFalse(row['unknown']);self.assertTrue(row['solid'])
                    self.assertGreaterEqual(row['terrain_cost'],50.)
                    self.assertTrue(np.isfinite(row['G']));self.assertTrue(np.isfinite(row['C']))
                    self.assertNotIn((0,0),m.layers[k].frontiers.by_column)
                    self.assertEqual(m.planning_cost((k,0,0)),50.)
                    self.assertFalse(m.point_safe_check((0.,0.,(k-1)*.5+.42))['ready'])
                    self.assertEqual(list(m.unknown_blockers(k,[(0,0)],.1)),[])
                self.assertEqual(len(m.blocks),before)
                self.assertTrue(m.frontier_observed(4,(0,0)))
                # No G/C at this cut, but the lower wall occupies the source
                # ground's traversal band. It cannot become a high-cut task.
                self.assertNotIn((0,0),m.layers[4].frontiers.by_column)
                # Traversal spans are not fabricated solid optical volumes.
                self.assertTrue(all(low==high for low,high in m.surface_relations.physical_blockers((0,0))))

    def test_remembered_wall_applies_to_cuts_created_by_a_later_frame(self):
        m=mapping()
        m.integrate([(0.,0.,z) for z in (.6,.95,1.3,1.65,2.,2.35,2.6)],sensor_origin=(0.,0.,.2));m.build()
        self.assertTrue(m.layers)
        spans=m.columns[0,0].obstacle_intervals
        floor(m)
        for k in range(2,6):
            self.assertTrue(m.observed_cell((k,0,0))['solid'])
            self.assertNotIn((0,0),m.layers[k].frontiers.by_column)
        self.assertTrue(m.frontier_observed(6,(0,0)))
        self.assertIsNone(m.observed_cell((2,1,0)))
        m.integrate([(-.3,0.,.1)],sensor_origin=(-.3,0.,.42));m.build()
        self.assertEqual(m.columns[0,0].obstacle_intervals,spans)
        self.assertTrue(m.observed_cell((4,0,0))['solid'])
        m._reset_slice_products();m.build()
        self.assertTrue(m.observed_cell((4,0,0))['solid'])

    def test_obstacle_query_does_not_replace_frozen_gc(self):
        m=mapping()
        m.integrate([(0.,0.,1.46)]);m.build()
        original=m.cell((3,0,0)).copy();version=m.revision
        self.assertFalse(m.observed_cell((3,0,0))['solid'])
        m.integrate([(0.,0.,1.52)]);m.build()
        self.assertGreater(m.revision,version);self.assertIn((3,0,0),m.geometry_changed)
        raw=m.cell((3,0,0)).copy()
        self.assertTrue(m.observed_cell((3,0,0))['solid'])
        self.assertEqual(m.cell((3,0,0))['G'],original['G'])
        self.assertEqual(m.observed_cell((3,0,0))['C'],raw['C'])
        self.assertEqual(m.observed_cell((3,0,0))['G'],original['G'])
        self.assertEqual(m.cell((3,0,0)).tobytes(),raw.tobytes())
        block=m.blocks[3,0,0]
        self.assertFalse(m.traversable_block((3,0,0),block)[0,0])

    def test_new_interval_revises_map_even_when_surface_fusion_reports_no_change(self):
        m=mapping()
        m.integrate([(0.,0.,z) for z in (.901,1.41,1.45)]);m.build()
        original=m.cell((2,0,0)).copy();version=m.revision;used=m.accepted_points
        self.assertFalse(m.observed_cell((2,0,0))['solid'])
        m.integrate([(0.,0.,1.40)]);m.build()
        self.assertEqual(m.accepted_points,used)
        self.assertGreater(m.revision,version);self.assertIn((2,0,0),m.geometry_changed)
        self.assertTrue(m.observed_cell((2,0,0))['solid'])
        self.assertEqual(m.cell((2,0,0))['G'],original['G'])
        self.assertFalse(m._obstacle_changes)

    def test_ground_noise_recognition_removes_a_previous_isolated_level_return(self):
        m=mapping()
        m.integrate([(0.,0.,.5)],sensor_origin=(0.,0.,.5));m.build()
        self.assertFalse(m.obstacle_footprint((1,0,0)))
        m.integrate([(x*.05,y*.05,.52) for x in range(-2,3) for y in range(-2,3)],sensor_origin=(0.,0.,.9));m.build()
        self.assertIsNone(m.obstacle_at((1,0,0)))
        self.assertFalse(m.obstacle_footprint((1,0,0)))
        self.assertFalse(m.observed_cell((2,0,0))['solid'])

    def test_known_obstacle_accessor_does_not_report_missing_ground(self):
        from test_online_pct import native_wrapper
        native_wrapper()
        from legbot_exploration.block_pct import BlockPlanner
        m=mapping();floor(m)
        m.integrate([(0.,0.,z) for z in (.55,.9,1.25)],sensor_origin=(0.,0.,.2));m.build()
        reader=BlockPlanner.__new__(BlockPlanner);reader.map=m;reader._read_blocks=set();reader._intervals=False
        reader._dependency_tiles=set()
        result=reader._read(2-m.first_layer,-int(m.lower[0]),-int(m.lower[1]))
        self.assertEqual(result[0],50.)
        with patch.object(m,'iter_cells',side_effect=AssertionError('full map scan')), \
                patch.object(m,'layers_at',side_effect=AssertionError('linear layer scan')):
            self.assertTrue(m.observed_cell((2,0,0))['solid'])
            self.assertTrue(m.obstacle_footprint((2,0,0)))

    def test_native_route_cannot_switch_cuts_to_cross_a_remembered_wall(self):
        from test_online_pct import native_wrapper
        wrapper=native_wrapper()
        from legbot_exploration.block_pct import BlockPlanner
        m=mapping()
        m.integrate([(x*.1,y*.1,z) for x in range(-6,7) for y in range(-4,5) for z in (.1,2.8)],
                    sensor_origin=(0.,0.,.42));m.build()
        planner=BlockPlanner(m,wrapper)
        self.assertIsNotNone(planner.plan((-.4,0.,.1),(.4,0.,.1)))
        m.integrate([(0.,y*.1,z) for y in range(-4,5) for z in (.55,.9,1.25,1.6,1.95,2.3,2.6)],
                    sensor_origin=(-.1,0.,.2));m.build()
        # Bypass the necessary-connectivity shortcut to exercise native A*.
        with patch.object(m.connectivity,'possible',return_value=True):
            self.assertIsNone(planner.plan((-.4,0.,.1),(.4,0.,.1)))
        self.assertGreater(planner.search_nodes,0)
        self.assertTrue(m.observed_cell((5,0,0))['solid'])
        self.assertEqual(list(m.unknown_blockers(5,[(0,0)],.1)),[])

    def test_obstacle_footprint_cache_and_connectivity_mask_follow_new_evidence(self):
        m=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=.11,safety_margin=0.,
            traversability_kernel_size=3,inflation_kernel_size=1))
        floor(m)
        key=(2,-1,0)
        self.assertFalse(m.obstacle_footprint(key))
        m.integrate([(0.,0.,z) for z in (.55,.9,1.25)],sensor_origin=(0.,0.,.2));m.build()
        self.assertTrue(m.obstacle_footprint(key));self.assertEqual(m.planning_cost(key),50.)
        self.assertIn(key,m.geometry_changed)
        b=m.settings.block_size;block=m.blocks[2,-1,0]
        self.assertFalse(m.traversable_block((2,-1,0),block)[(-1)%b,0])

    def test_rejected_frame_does_not_commit_obstacle_intervals(self):
        m=mapping(max_surface_records=1)
        m.integrate([(0.,0.,.1)],sensor_origin=(0.,0.,.5));m.build()
        before=m.columns[0,0].obstacle_intervals
        m.integrate([(0.,0.,.3),(0.,0.,.6)],sensor_origin=(0.,0.,.8))
        self.assertEqual(m.error,'pct_surface_record_limit')
        self.assertEqual(m.columns[0,0].obstacle_intervals,before)
        self.assertFalse(m._obstacle_changes)


if __name__=='__main__':unittest.main()
