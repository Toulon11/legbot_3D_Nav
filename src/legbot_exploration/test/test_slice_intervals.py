"""Strict measured intervals and the single incremental frontier rule."""
import unittest
from unittest.mock import patch

import numpy as np

from legbot_exploration.fixed_layer_map import FixedLayerMap, MapSettings
from legbot_exploration.map_evidence import SurfaceColumn, SurfaceRecord
from legbot_exploration.pct_frontier_index import DIRECTIONS, rebuild_frontiers


def mapping(native=True, **settings):
    settings=dict(allow_open_ceiling=False,**settings) if 'allow_open_ceiling' not in settings else settings
    return FixedLayerMap(MapSettings(maturity_min_points=1, robot_radius=0., safety_margin=0.,
        traversability_kernel_size=3, inflation_kernel_size=1, **settings), native_updates=native)


def hits(m, points, orientation):
    with patch('legbot_exploration.surface_orientation.classify',
               return_value=np.full(len(points), orientation)):
        m.integrate(points, sensor_origin=(0., 0., .8))


def room(m, g=.1, c=2.1, holes=(), bound=3):
    xy=[(x*.1,y*.1) for x in range(-bound,bound+1) for y in range(-bound,bound+1)
        if (x,y) not in holes]
    hits(m, [(x,y,g) for x,y in xy], 1)
    hits(m, [(x,y,c) for x,y in xy], -1)
    m.build()


class SliceIntervalTest(unittest.TestCase):
    def test_ordered_sweep_matches_independent_scalar_selection(self):
        rng=np.random.default_rng(719)
        for _ in range(100):
            heights=sorted(rng.uniform(-1.,4.,20))
            records=[SurfaceRecord(i+1,h,1,h,h,orientation=int(rng.choice((-1,0,1))))
                     for i,h in enumerate(heights)]
            geometry=(heights,records)
            column=SurfaceColumn()
            for k,g,c in column.at_cuts(range(-5,25),geometry,.03,.25):
                reference=column.at(.03+k*.25,geometry)
                self.assertEqual((g,c),reference)

    def assert_rule(self, m):
        for k, layer in m.layers.items():
            candidates={(x+dx,y+dy) for x,y in m.column_layers
                        for dx,dy in ((0,0),)+DIRECTIONS}
            expected=set()
            for xy in candidates:
                if m.observed_cell((k,*xy)) is not None:
                    continue
                neighbors=[m.observed_cell((k,xy[0]+dx,xy[1]+dy)) for dx,dy in DIRECTIONS]
                if any(row is not None and (row['solid'] or row['terrain_cost']>=50.) for row in neighbors):
                    continue
                for dx,dy in DIRECTIONS:
                    row=m.observed_cell((k,xy[0]+dx,xy[1]+dy))
                    if (row is not None and not row['solid'] and 0<=row['terrain_cost']<=20 and
                            m.frontier_height_compatible(k,xy,(xy[0]+dx,xy[1]+dy),
                                float(row['G']),int(row['g_surface_id']))):
                        expected.add(xy)
                        break
            self.assertEqual(set(layer.frontiers.by_column), expected)
            self.assertTrue(all(len(keys)==1 for keys in layer.frontiers.by_column.values()))
        rebuilt=rebuild_frontiers(m)
        self.assertEqual(dict(m.frontier_index.nodes), dict(rebuilt.nodes))

    def test_closed_room_has_only_strictly_internal_cuts(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=mapping(native)
                room(m,g=0.,c=2.)
                self.assertEqual(m.column_layers[0,0], {1,2,3})
                self.assertIsNone(m.observed_cell((0,0,0)))
                self.assertIsNone(m.observed_cell((4,0,0)))
                self.assertTrue(all(0.<k*.5<2. for k,_ in m.layers_at(0,0)))
                self.assert_rule(m)

    def test_missing_ceiling_waits_without_fabricating_a_boundary_slice(self):
        m=mapping()
        hits(m, [(0.,0.,.1)], 1)
        m.build()
        self.assertFalse(m.layers)
        self.assertFalse(m.build())
        self.assertEqual(len(list(m.columns[0,0].values())),1)
        hits(m, [(0.,0.,1.1)], -1)
        m.build()
        self.assertEqual(m.column_layers[0,0],{1,2})

    def test_thin_interval_has_no_outside_fallback(self):
        m=mapping()
        hits(m, [(0.,0.,.1)], 1)
        hits(m, [(0.,0.,.3)], -1)
        m.build()
        self.assertFalse(m.layers)
        self.assertIsNone(m.observed_cell((1,0,0)))

    def test_explicit_open_sky_keeps_one_strict_upper_cut(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=mapping(native, allow_open_ceiling=True)
                hits(m, [(0.,0.,0.)], 1)
                m.build()
                self.assertEqual(m.column_layers[0,0],{1})
                self.assertTrue(np.isposinf(m.cell((1,0,0))['C']))
                self.assertIsNotNone(m.observed_cell((1,0,0)))
                self.assertIsNone(m.observed_cell((0,0,0)))
                self.assertIsNotNone(m.observed_cell((10,0,0)))
                self.assertIsNone(m.observed_cell((11,0,0)))
                hits(m, [(0.,0.,1.)], -1)
                m.build()
                self.assertEqual(m.column_layers[0,0],{1,3})
                self.assertEqual(m.cell((1,0,0))['C'],1.)
                self.assertIsNone(m.observed_cell((2,0,0)))
                self.assertAlmostEqual(m.observed_cell((3,0,0))['G'],1.)

    def test_known_interval_alias_requires_no_materialization_or_geometry_scan(self):
        m=mapping()
        room(m)
        row=m.cell((2,0,0))
        row['G']=row['C']=np.nan
        row['unknown']=True
        before=len(m.blocks)
        with patch.object(m,'layers_at',side_effect=AssertionError('linear cut scan')), \
             patch.object(SurfaceColumn,'observed_geometry',side_effect=AssertionError('raw rescan')):
            borrowed=m.observed_cell((2,0,0))
        self.assertAlmostEqual(borrowed['G'],.1)
        self.assertAlmostEqual(borrowed['C'],2.1,places=6)
        self.assertEqual(len(m.blocks),before)
        self.assertTrue(np.isnan(row['G']))
        m.frontier_index.update({(0,0)})
        self.assertNotIn((0,0),m.layers[2].frontiers.by_column)

    def test_independent_rooms_do_not_bridge_free_space_across_blocking_slab(self):
        m=mapping()
        for g,c in ((.1,.9),(1.1,1.9)):
            hits(m,[(0.,0.,g)],1)
            hits(m,[(0.,0.,c)],-1)
        m.build()
        self.assertEqual(m.column_layers[0,0],{1,2,3})
        self.assertIsNotNone(m.observed_cell((1,0,0)))
        self.assertTrue(m.observed_cell((2,0,0))['solid'])
        self.assertAlmostEqual(m.observed_cell((2,0,0))['G'],.9,places=5)
        self.assertIsNotNone(m.observed_cell((3,0,0)))

    def test_new_upper_room_keeps_lower_room_and_explores_unobserved_upper_xy(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=mapping(native)
                room(m,g=.1,c=.9)
                lower=m.cell((1,0,0)).tobytes()
                room(m,g=1.1,c=2.2,holes={(0,0)})
                self.assertIsNone(m.observed_cell((3,0,0)))
                self.assertIn((0,0),m.layers[3].frontiers.by_column)
                self.assertEqual(m.cell((1,0,0)).tobytes(),lower)
                hits(m,[(0.,0.,1.1)],1)
                hits(m,[(0.,0.,2.2)],-1)
                m.build()
                self.assertNotIn((0,0),m.layers[3].frontiers.by_column)
                self.assertEqual(m.cell((1,0,0)).tobytes(),lower)
                self.assert_rule(m)

    def test_incomplete_gc_cannot_become_known_from_a_numeric_g_alone(self):
        m=mapping()
        room(m)
        for _,row in m.layers_at(0,0):
            row['C']=np.nan
        self.assertIsNone(m.observed_cell((2,0,0)))
        self.assertFalse(m.traversable_block((2,0,0),m.blocks[2,0,0])[0,0])
        m.frontier_index.update({(0,0)})
        self.assertIn((0,0),m.layers[2].frontiers.by_column)

    def test_scan_task_freshness_does_not_expire_static_intervals(self):
        m=mapping()
        room(m)
        frame=m.integrated_serial
        evidence=m.surface_observation
        self.assertTrue(evidence.frontier_settled((0,0),.1,layer=1))
        self.assertFalse(evidence.has_frontier_ground((0,0),.1,after=frame,layer=1))
        hits(m,[(0.,0.,.1)],1)
        m.build()
        self.assertTrue(evidence.has_frontier_ground((0,0),.1,after=frame,layer=1))

    def test_unknown_neighbors_are_members_and_known_obstacles_are_not_sources(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=mapping(native)
                room(m,holes={(0,0)})
                self.assertIn((0,0),m.layers[1].frontiers.by_column)
                self.assert_rule(m)
                # Shrink one column's clearance while retaining the cut inside it.
                hits(m,[(.1,0.,.55)],-1)
                m.build()
                self.assertTrue(m.observed_cell((1,1,0))['solid'])
                self.assertNotIn((1,0),m.layers[1].frontiers.by_column)
                self.assert_rule(m)

    def test_ceiling_only_hits_do_not_resolve_unknown_ground(self):
        m=mapping()
        room(m,holes={(0,0)})
        hits(m,[(0.,0.,2.1)],-1)
        m.build()
        self.assertIn((0,0),m.layers[1].frontiers.by_column)
        hits(m,[(0.,0.,.1)],1)
        m.build()
        self.assertNotIn((0,0),m.layers[1].frontiers.by_column)
        self.assert_rule(m)

    def test_boundary_change_updates_only_dependencies(self):
        m=mapping()
        room(m,holes={(0,0)})
        with patch.object(m,'iter_cells',side_effect=AssertionError('whole map scan')):
            hits(m,[(0.,0.,.1)],1)
            hits(m,[(0.,0.,2.1)],-1)
            m.build()
        self.assert_rule(m)


if __name__=='__main__':
    unittest.main()
