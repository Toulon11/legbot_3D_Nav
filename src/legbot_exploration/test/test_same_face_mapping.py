"""Noise grouping and ordered G/C selection no longer depend on face roles."""
import unittest
import numpy as np
from legbot_exploration.map_evidence import SurfaceColumn,SurfaceRecord
from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings


class SameFaceMappingTest(unittest.TestCase):
    def test_mixed_role_hits_share_one_frozen_noise_group(self):
        c=SurfaceColumn()
        for i,(z,role) in enumerate(((.772,-1),(.769,1),(.77,0),(.771,-2))):
            c.add(z,.05,3,frame=i+1,orientation=role,footprint=1<<i)
        self.assertEqual(len(c),1)
        r=c.surfaces[0];self.assertTrue(r.mature)
        self.assertAlmostEqual(r.height,(.772+.769+.77)/3)
        self.assertEqual(r.footprint,15)
        self.assertEqual(r.last_seen,4)

    def test_legacy_record_roles_do_not_change_nearest_ordered_bounds(self):
        c=SurfaceColumn()
        c.surfaces=[SurfaceRecord(i+1,h,3,h,h,mature_at=1,orientation=role)
                    for i,(h,role) in enumerate(((.1,1),(.772,-1),(.9,0),(2.8,-2)))]
        c._sort();geometry=c.observed_geometry(.05)
        for cut,g,top in ((.5,.1,.772),(1.,.9,2.8),(3.,2.8,None)):
            a,b=c.at(cut,geometry)
            self.assertAlmostEqual(a.height,g)
            self.assertEqual(None if b is None else b.height,top)

    def test_repeated_other_side_does_not_remove_an_observed_work_cell(self):
        m=FixedLayerMap(MapSettings(maturity_min_points=3,robot_radius=0.,safety_margin=0.,
            traversability_kernel_size=3,inflation_kernel_size=1))
        for origin in ((0.,0.,.5),(0.,0.,1.1)):
            m.integrate([(0.,0.,z) for z in (.1,.772,2.8)]*3,sensor_origin=origin);m.build()
            self.assertAlmostEqual(m.observed_cell((1,0,0))['C'],.772,places=5)
            self.assertAlmostEqual(m.observed_cell((2,0,0))['G'],.772,places=5)
            self.assertAlmostEqual(m.observed_cell((2,0,0))['C'],2.8,places=5)
        self.assertEqual(len(m.columns[0,0]),3)

    def test_distinct_close_returns_keep_low_clearance_and_obstacle_span(self):
        m=FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.))
        m.integrate([(0.,0.,z) for z in (.1,.4,.8)]);m.build()
        row=m.observed_cell((1,0,0))
        self.assertTrue(row['solid']);self.assertGreaterEqual(row['terrain_cost'],50.)
        self.assertAlmostEqual(row['G'],.4,places=5);self.assertAlmostEqual(row['C'],.8,places=5)
        self.assertTrue(m.ground_band_obstacle((0,0),.1))

    def test_ordered_sweep_stays_equal_to_scalar_reference(self):
        rng=np.random.default_rng(17)
        for _ in range(20):
            c=SurfaceColumn()
            for h in sorted(rng.uniform(-1.,4.,15)):
                c.add(h,.01,1,orientation=int(rng.choice((-2,-1,0,1))))
            g=c.observed_geometry(.01)
            for k,a,b in c.at_cuts(range(-4,20),g,0.,.25):
                self.assertEqual((a,b),c.at(k*.25,g))


if __name__=='__main__':unittest.main()
