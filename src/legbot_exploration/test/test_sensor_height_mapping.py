"""All measured returns select G/C independently of sensor height and normals."""
import unittest
from unittest.mock import patch
import numpy as np
from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings


class MeasuredHeightMappingTest(unittest.TestCase):
    def mapping(self,native=True):
        return FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.,
            traversability_kernel_size=3,inflation_kernel_size=1),native_updates=native)

    def test_sensor_above_below_and_same_height_all_build_ground(self):
        for native in (False,True):
            for z in (1.0,1.78,3.0):
                with self.subTest(native=native,origin=z):
                    m=self.mapping(native)
                    with patch('legbot_exploration.surface_orientation.classify',side_effect=AssertionError('obsolete gate')):
                        m.integrate([(0.,0.,1.78)],sensor_origin=(0.,0.,z));m.build()
                    self.assertAlmostEqual(m.observed_cell((4,0,0))['G'],1.78,places=5)
                    self.assertTrue(np.isposinf(m.observed_cell((4,0,0))['C']))

    def test_invalid_sensor_origin_is_rejected(self):
        for origin in ((0.,0.),(0.,0.,float('nan'))):
            with self.assertRaises(ValueError):self.mapping().integrate([(0.,0.,1.)],sensor_origin=origin)

    def test_new_upper_return_updates_its_own_column_without_erasing_lower_gc(self):
        for native in (False,True):
            m=self.mapping(native)
            m.integrate([(x*.1,y*.1,z) for x in range(-3,4) for y in range(-3,4)
                         for z in (1.46,2.82)],sensor_origin=(0.,0.,1.78));m.build()
            lower=tuple(float(m.cell((3,0,0))[f]) for f in ('G','C'))
            with patch.object(m,'iter_cells',side_effect=AssertionError('whole map scan')):
                m.integrate([(0.,0.,2.96)],sensor_origin=(0.,0.,3.48));m.build()
            self.assertAlmostEqual(m.observed_cell((6,0,0))['G'],2.96,places=5)
            self.assertEqual(tuple(float(m.cell((3,0,0))[f]) for f in ('G','C')),lower)
            self.assertAlmostEqual(m.observed_cell((6,-1,0))['G'],2.82,places=5)
            self.assertTrue(m.frontier_observed(6,(-1,0)))
            self.assertFalse(m.surface_observation.has_frontier_ground((-1,0),2.96,layer=6))
            m.integrate([(-.1,0.,2.96)]);m.build()
            self.assertTrue(m.frontier_observed(6,(-1,0)))

    def test_two_sensor_sides_reuse_one_height_record(self):
        m=self.mapping()
        for z in (1.78,3.48):m.integrate([(0.,0.,2.96)],sensor_origin=(0.,0.,z));m.build()
        self.assertEqual(len(m.columns[0,0]),1)
        self.assertAlmostEqual(m.observed_cell((7,0,0))['G'],2.96,places=5)
        self.assertIsNone(m.cell((7,0,0)))

    def test_new_upper_bound_replaces_open_gc_below_it(self):
        m=self.mapping();m.integrate([(0.,0.,2.96)]);m.build()
        self.assertEqual(m.column_layers[0,0],{6})
        m.integrate([(0.,0.,4.1)]);m.build()
        self.assertEqual(m.column_layers[0,0],{6,7,8,9})
        self.assertAlmostEqual(m.observed_cell((7,0,0))['G'],2.96,places=5)
        self.assertAlmostEqual(m.observed_cell((7,0,0))['C'],4.1,places=5)
        self.assertAlmostEqual(m.observed_cell((9,0,0))['G'],4.1,places=5)


if __name__=='__main__':unittest.main()
