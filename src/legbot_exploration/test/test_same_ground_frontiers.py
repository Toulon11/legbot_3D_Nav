"""Known interval references, scoped auxiliary heights and physical tasks."""
import unittest
from unittest.mock import patch

import numpy as np

from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings


def mapping(native=True):
    return FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.,
        traversability_kernel_size=3,inflation_kernel_size=1),native_updates=native)


def imported(m,k,xy,g,c,sid=0,solid=False):
    row=m.cell((k,*xy),True)
    row['G'],row['C']=g,c
    row['g_surface_id']=sid;row['c_surface_id']=sid+100
    row['measured']=True;row['unknown']=False;row['solid']=solid
    row['hard']=row['solid_hard']=solid;row['unknown_hard']=False
    row['flat']=row['feature_valid']=True
    row['passage_cost']=50. if solid else 0.
    row['relief_cost']=0.;row['terrain_cost']=row['inflated_terrain_cost']=50. if solid else 0.
    m.column_layers.setdefault(xy,set()).add(k)
    m._index_column_intervals(xy)
    return row


class SameGroundFrontierTest(unittest.TestCase):
    def test_lower_cut_wall_outside_source_cut_spans_still_rejects_upper_boundary(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=mapping(native)
                m.integrate([(x*.1,y*.1,z) for x in range(1,6) for y in range(-3,4)
                             for z in (2.056,3.3)],sensor_origin=(.1,0.,2.4))
                m.build()
                self.assertIn((0,0),m.layers[5].frontiers.by_column)
                # Wall exists only below the queried 2.5 m cut; its top
                # still occupies the robot's space above this source floor.
                m.integrate([(0.,0.,z) for z in np.arange(1.59,2.31,.03)],
                            sensor_origin=(.1,0.,1.8));m.build()
                self.assertTrue(m.frontier_observed(5,(0,0)))
                self.assertTrue(m.ground_band_obstacle((0,0),2.056))
                self.assertFalse(m.frontier_height_compatible(5,(0,0),(1,0),2.056,1))
                self.assertNotIn((0,0),m.layers[5].frontiers.by_column)
                self.assertTrue(m.frontier_index.nodes,'genuinely missing boundaries remain')

    def test_wall_with_ground_close_to_floor_is_not_a_small_step(self):
        m=mapping()
        m.integrate([(0.,0.,z) for z in np.arange(1.41,1.95,.02)],
                    sensor_origin=(.1,0.,1.78));m.build()
        self.assertTrue(m.ground_band_obstacle((0,0),1.46))
        self.assertFalse(m.ground_band_obstacle((0,0),2.96),
                         'old lower-floor wall must not block a new upstairs surface')

    def test_tread_within_sixteen_centimeters_is_not_a_wall(self):
        m=mapping()
        m.integrate([(0.,0.,z) for z in np.arange(.36,.511,.02)],sensor_origin=(0.,0.,.8))
        m.build()
        self.assertFalse(m.ground_band_obstacle((0,0),.36))

    def test_missing_and_existing_rows_use_maximum_g_and_minimum_covering_c(self):
        m=mapping()
        imported(m,1,(0,0),.1,2.8,1)
        imported(m,2,(0,0),.2,2.6,2)
        imported(m,3,(0,0),.3,2.7,3)
        before=len(m.blocks)
        for k in (2,4):
            with self.subTest(cut=k),patch.object(m,'layers_at',side_effect=AssertionError('cut scan')):
                row=m.observed_cell((k,0,0))
                self.assertAlmostEqual(row['G'],.3);self.assertAlmostEqual(row['C'],2.6)
                self.assertEqual(row['g_surface_id'],3);self.assertEqual(row['c_surface_id'],102)
        self.assertEqual(len(m.blocks),before)
        self.assertAlmostEqual(m.cell((2,0,0))['G'],.2)

    def test_tighter_virtual_interval_rechecks_clearance(self):
        m=mapping()
        imported(m,1,(0,0),.1,1.1,1)
        imported(m,2,(0,0),.75,1.5,2)
        row=m.observed_cell((2,0,0))
        self.assertAlmostEqual(row['G'],.75);self.assertAlmostEqual(row['C'],1.1)
        self.assertTrue(row['solid']);self.assertGreaterEqual(row['terrain_cost'],50.)
        self.assertFalse(m.point_safe_check((0.,0.,1.07),relaxed=True)['ready'])

    def test_auxiliary_c_filters_wall_cuts_but_does_not_write_g(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=mapping(native)
                xy=[(x*.1,y*.1) for x in range(-5,2) for y in range(-4,5)]
                m.integrate([(x,y,z) for x,y in xy for z in (.36,2.98)],sensor_origin=(0.,0.,.62))
                m.integrate([(.1,y*.1,z) for y in range(-4,5) for z in np.arange(.40,2.99,.02)],
                            sensor_origin=(0.,0.,.62));m.build()
                self.assertTrue(m.observed_cell((1,1,0))['solid'])
                for k in (2,3,4,5):
                    self.assertTrue(m.observed_cell((k,1,0))['solid'])
                    self.assertFalse(m.observed_cell((k,1,0))['unknown'])
                    self.assertNotIn((1,0),m.layers[k].frontiers.by_column)
                self.assertIn((-6,0),m.layers[2].frontiers.by_column)

    def test_unrelated_lower_room_c_does_not_suppress_new_platform(self):
        m=mapping()
        xy=[(x*.1,y*.1) for x in range(-5,6) for y in range(-4,5)]
        m.integrate([(x,y,z) for x,y in xy for z in (1.46,)],sensor_origin=(0.,0.,1.78));m.build()
        m.integrate([(x*.1,y*.1,2.96) for x in range(3) for y in range(-2,3)],sensor_origin=(0.,0.,3.48))
        m.build()
        self.assertTrue(m.frontier_observed(6,(-1,0)))
        self.assertNotIn((-1,0),m.layers[6].frontiers.by_column)
        row=m.observed_cell((6,0,0))
        self.assertTrue(m.frontier_height_compatible(6,(-1,0),(0,0),float(row['G']),int(row['g_surface_id'])))

    def test_g_branch_enforces_step_limit_and_keeps_completely_missing_columns(self):
        m=mapping()
        for k in (2,3,4,5):imported(m,k,(0,0),.95,2.8,1)
        target=imported(m,3,(1,0),1.11,2.7,2)
        self.assertIsNone(m.observed_cell((2,1,0)))
        self.assertTrue(m.frontier_height_compatible(2,(1,0),(0,0),.95,1))
        target['G']=1.12;m._index_column_intervals((1,0))
        self.assertFalse(m.frontier_height_compatible(2,(1,0),(0,0),.95,1))
        self.assertTrue(m.frontier_height_compatible(2,(0,1),(0,0),.95,1))

    def test_auxiliary_query_uses_only_changed_column_indexes(self):
        m=mapping()
        for k in range(1,6):imported(m,k,(0,0),.36,2.98,1)
        imported(m,1,(1,0),.48,.54,2,solid=True)
        before=len(m.blocks)
        with patch.object(m,'layers_at',side_effect=AssertionError('linear column scan')), \
             patch.object(m,'iter_cells',side_effect=AssertionError('whole map scan')):
            self.assertFalse(m.frontier_height_compatible(2,(1,0),(0,0),.36,1))
        self.assertEqual(len(m.blocks),before)

    def test_filter_keeps_only_compatible_sources_and_recomputes_height(self):
        m=mapping()
        for k in range(1,6):
            imported(m,k,(0,0),.36,2.98,1)
            imported(m,k,(0,1),.40,2.98,1)
        imported(m,1,(1,0),.48,.54,2,solid=True)
        m.frontier_index.update({(0,0),(0,1),(1,0)})
        node=next(m.frontier_index.nodes[key] for key in m.layers[2].frontiers.by_column[(1,0)])
        self.assertAlmostEqual(node.height,.4)
        self.assertEqual(node.sources,((0,1,.4,1),))

    def test_auxiliary_only_change_invalidates_existing_upper_boundary(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=mapping(native)
                for k in range(1,6):imported(m,k,(0,0),.36,2.98,1)
                lower=imported(m,1,(1,0),.45,.51,2,solid=True)
                m.frontier_index.update({(0,0),(1,0)})
                self.assertIn((1,0),m.layers[2].frontiers.by_column)
                lower['C']=.55;m._index_column_intervals((1,0))
                index=m.layers[2].frontiers
                with patch.object(m,'iter_cells',side_effect=AssertionError('full map scan')):
                    result=index.update({(1,0)},m._boundary_kernel if native else None)
                    m.frontier_index.commit([(index,result)])
                self.assertNotIn((1,0),index.by_column)
                self.assertIsNone(m.observed_cell((2,1,0)))


if __name__=='__main__':
    unittest.main()
