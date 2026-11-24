"""Role-free measured geometry, bounded open aliases and atomic frontiers."""
import unittest
from unittest.mock import patch
import numpy as np
from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings
from legbot_exploration.pct_frontier_index import rebuild_frontiers


def mapping(native=True,**options):
    return FixedLayerMap(MapSettings(maturity_min_points=1,robot_radius=0.,safety_margin=0.,
        traversability_kernel_size=3,inflation_kernel_size=1,**options),native_updates=native)


class SharedCloudGeometryTest(unittest.TestCase):
    def test_measured_treads_are_not_unknown_when_neighbor_g_differs(self):
        for native in (False,True):
            with self.subTest(native=native):
                m=mapping(native)
                # Field example: (6.6,4.2), G=2.688, C=4.099, neighbor
                # G=2.809. A 4 m cut is known on both sides of this edge.
                m.integrate([(6.6+x*.1,4.2+y*.1,z) for x in range(-3,4)
                             for y in range(-3,4)
                             for z in (2.6884878 if y<=0 else 2.8093071,4.099195)])
                with patch.object(m.surface_relations,'classify',side_effect=AssertionError('known-cell edge recognition')):
                    m.build()
                    self.assertTrue(m.frontier_observed(8,(66,42)))
                    self.assertFalse(m.layers[8].frontiers._qualified_boundary(
                        (66,42),[(66,43,2.8093071,1)]))
                for k in (6,7,8):
                    self.assertNotIn((66,42),m.layers[k].frontiers.by_column)
                self.assertFalse(m.surface_observation.has_frontier_ground((66,42),2.8093071,layer=8))
                self.assertEqual(dict(m.frontier_index.nodes),dict(rebuild_frontiers(m).nodes))

    def test_virtual_known_tread_excludes_frontier_without_copying_a_work_cell(self):
        for native in (False,True):
            m=mapping(native)
            m.integrate([(x*.1,y*.1,z) for x in range(-3,4) for y in range(-3,4)
                         for z in (2.69 if y<=0 else 2.82,4.10)])
            m.build();row=m.cell((8,0,0));row['G']=row['C']=np.nan;row['unknown']=True
            m.frontier_index.update({(0,0)})
            self.assertAlmostEqual(m.observed_cell((8,0,0))['G'],2.69,places=5)
            self.assertTrue(np.isnan(row['G']))
            self.assertNotIn((0,0),m.layers[8].frontiers.by_column)
            self.assertEqual(dict(m.frontier_index.nodes),dict(rebuild_frontiers(m).nodes))

    def test_uncovered_xy_remains_frontier_until_real_gc_arrives(self):
        for native in (False,True):
            m=mapping(native)
            m.integrate([(x*.1,y*.1,z) for x in range(-3,4) for y in range(-3,4)
                         if (x,y)!=(0,0) for z in (2.69,4.10)]);m.build()
            for k in (6,7,8):self.assertIn((0,0),m.layers[k].frontiers.by_column)
            m.integrate([(0.,0.,2.69),(0.,0.,4.10)]);m.build()
            for k in (6,7,8):self.assertNotIn((0,0),m.layers[k].frontiers.by_column)
            self.assertTrue(m.frontier_index.nodes,'unobserved outer boundaries remain')
            self.assertEqual(dict(m.frontier_index.nodes),dict(rebuild_frontiers(m).nodes))

    def test_all_sensor_sides_and_sparse_hits_provide_ordered_gc_without_classifier(self):
        for native in (False,True):
            for origin in ((0.,0.,.1),(0.,0.,1.47),(0.,0.,3.)):
                with self.subTest(native=native,origin=origin):
                    m=mapping(native)
                    with patch('legbot_exploration.surface_orientation.classify',side_effect=AssertionError('classification gate')):
                        m.integrate([(0.,0.,1.47),(0.,0.,2.82)],sensor_origin=origin);m.build()
                    row=m.observed_cell((4,0,0))
                    self.assertAlmostEqual(row['G'],1.47,places=5)
                    self.assertAlmostEqual(row['C'],2.82,places=5)

    def test_open_alias_limit_is_inclusive_and_queries_never_allocate(self):
        for native in (False,True):
            m=mapping(native);m.integrate([(0.,0.,0.)]);m.build();before=len(m.blocks)
            with patch.object(m,'layers_at',side_effect=AssertionError('cut scan')):
                for k in range(1,11):
                    row=m.observed_cell((k,0,0));self.assertEqual(row['G'],0.)
                    self.assertTrue(np.isposinf(row['C']))
                self.assertIsNone(m.observed_cell((0,0,0)))
                self.assertIsNone(m.observed_cell((11,0,0)))
                self.assertIsNone(m.observed_cell((2,1,0)))
            self.assertEqual(len(m.blocks),before)
            self.assertEqual(m.column_layers[0,0],{1})

    def test_missing_or_inverted_c_is_not_an_open_ceiling(self):
        m=mapping();m.integrate([(0.,0.,0.)]);m.build()
        row=m.cell((1,0,0))
        for ceiling in (float('nan'),-float('inf'),-1.,0.):
            row['C']=ceiling
            self.assertIsNone(m.observed_cell((1,0,0)))
            self.assertIsNone(m.observed_cell((2,0,0)))

    def test_open_ground_primary_cut_has_frontiers_without_materialized_cells(self):
        for native in (False,True):
            m=mapping(native)
            m.integrate([(x*.1,y*.1,1.46) for x in range(-3,4) for y in range(-3,4)])
            m.build();k,info=m.frontier_selection_layer((0.,0.,1.46))
            self.assertEqual(k,4);self.assertTrue(m.layers[k].frontiers.nodes)
            self.assertIsNone(m.cell((k,0,0)))
            self.assertAlmostEqual(m.observed_cell((k,0,0))['G'],1.46,places=5)
            self.assertFalse(any(key[1:3] in m.columns for key in m.layers[k].frontiers.nodes))

    def test_open_alias_is_known_but_does_not_confirm_an_independent_target(self):
        for native in (False,True):
            m=mapping(native)
            m.integrate([(x*.1,y*.1,0.) for x in range(-6,7) for y in range(-4,5)]);m.build()
            m.integrate([(x*.1,y*.1,1.46) for x in range(3) for y in range(-2,3)]);m.build()
            self.assertEqual(m.observed_cell((4,-1,0))['G'],0.)
            self.assertTrue(m.frontier_observed(4,(-1,0)))
            self.assertNotIn((-1,0),m.layers[4].frontiers.by_column)
            self.assertFalse(m.surface_observation.has_frontier_ground((-1,0),1.46,layer=4))
            m.integrate([(-.1,0.,1.46)]);m.build()
            self.assertTrue(m.frontier_observed(4,(-1,0)))
            self.assertNotIn((-1,0),m.layers[4].frontiers.by_column)
            self.assertEqual(dict(m.frontier_index.nodes),dict(rebuild_frontiers(m).nodes))

    def test_new_ceiling_contracts_old_alias_and_invalidates_other_cut(self):
        for native in (False,True):
            m=mapping(native)
            m.integrate([(x*.1,y*.1,0.) for x in range(-3,4) for y in range(-3,4)])
            # Another observed column creates a higher cut/index.
            m.integrate([(1.,0.,1.8)]);m.build()
            self.assertEqual(m.observed_cell((4,0,0))['G'],0.)
            version=m.revision
            with patch.object(m,'iter_cells',side_effect=AssertionError('whole map scan')):
                m.integrate([(0.,0.,.8)]);m.build()
            self.assertGreater(m.revision,version)
            self.assertIn((4,0,0),m.geometry_changed)
            self.assertTrue(m.frontier_observed(4,(0,0)))
            self.assertFalse(m.surface_observation.has_frontier_ground((0,0),0.,layer=4))
            self.assertAlmostEqual(m.observed_cell((1,0,0))['C'],.8,places=5)
            self.assertEqual(dict(m.frontier_index.nodes),dict(rebuild_frontiers(m).nodes))

    def test_same_ground_aliases_settle_all_indexed_cuts_and_display(self):
        from pathlib import Path
        from legbot_exploration.online_pct import OnlinePct
        m=mapping()
        m.integrate([(x*.1,y*.1,z) for x in range(-3,4) for y in range(-3,4)
                     if (x,y)!=(0,0) for z in (1.46,2.8)]);m.build()
        groups=[]
        for k in (3,4,5):
            node=next(iter(m.layers[k].frontiers.by_column[0,0]))
            cid=m.frontier_index.membership[node];b=m.frontier_index.nodes[node]
            groups.append((cid,[[0.,0.,b.height]]))
        root=Path(__file__).resolve().parents[3]/'install/pct_planner/share/pct_planner'
        backend=OnlinePct.__new__(OnlinePct);backend.map=m;backend.prepare=lambda *args:None
        self.assertTrue(all(backend.refresh_frontier_displays(1,groups)))
        m.integrate([(0.,0.,1.46),(0.,0.,2.8)]);m.build()
        self.assertTrue(all((0,0) not in m.layers[k].frontiers.by_column for k in (3,4,5)))
        self.assertEqual(backend.refresh_frontier_displays(1,groups),[[],[],[]])


if __name__=='__main__':unittest.main()
