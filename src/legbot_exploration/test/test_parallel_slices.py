"""Compare the committed map, slice clusters and physical tasks across runners."""
import unittest
from unittest.mock import patch

import numpy as np

import _legbot_layer_kernels
from legbot_exploration.fixed_layer_map import FixedLayerMap, MapSettings
from legbot_exploration.online_pct import OnlinePct
from legbot_exploration.map_evidence import changed_slice_indices


class ParallelSliceTest(unittest.TestCase):
    def test_incremental_geometry_and_frontiers_match_serial_reference(self):
        settings = MapSettings(block_size=16, maturity_min_points=3)
        maps = [
            FixedLayerMap(settings, update_workers=1, native_updates=False),
            FixedLayerMap(settings, update_workers=1),
            FixedLayerMap(settings, update_workers=4),
        ]
        xys = [(x,y) for x in range(-18,19) for y in range(-12,13)
               if y != 0 or x < -2 or x > 2]
        initial = [(x*.1,y*.1,h) for x,y in xys for h in
                   (-.1+.12*(x>=1), 1.5+.14*(x>=5), 3.)]
        batches = [
            (initial, None),
            # One ambiguous return resolves lower floor aliases only.
            ([(0.,0.,-.1)], (0.,0.,.5)),
            ([(0.,0.,1.5),(0.,.1,.25),(.3,.1,-.03),(-.4,.3,1.53)], None),
            ([(x*.1,0.,1.5) for x in range(-2,3)], None),
            (initial[::3], None),
        ]

        def groups(index):
            return {frozenset(cluster['members']) for cluster in index.clusters.values()}

        parallel_workers=0
        for batch_id, (points, sensor) in enumerate(batches):
            with self.subTest(batch=batch_id):
                for mapping in maps:
                    mapping.integrate(points, sensor_origin=sensor)
                    if mapping is maps[0]:
                        with patch('legbot_exploration.fixed_layer_map.changed_slice_indices',
                                   side_effect=lambda old,new,layers,*args:list(layers)):
                            mapping.build()
                    else:
                        mapping.build()
                parallel_workers=max(parallel_workers,maps[2].last_work['slice_workers'])
                reference = maps[0]
                for mapping in maps[1:]:
                    self.assertEqual(mapping.revision,reference.revision)
                    self.assertEqual(mapping.counts,reference.counts)
                    self.assertEqual(mapping.geometry_changed,reference.geometry_changed)
                    self.assertEqual(mapping.blocks.keys(),reference.blocks.keys())
                    frontier = mapping.frontier_index
                    self.assertEqual(frontier.nodes,reference.frontier_index.nodes)
                    self.assertEqual(groups(frontier),groups(reference.frontier_index))
                    self.assertEqual(frontier.query(evaluate_observers=False)['clusters'],
                                     reference.frontier_index.query(evaluate_observers=False)['clusters'])
                    for key,block in mapping.blocks.items():
                        self.assertIs(mapping.layers[key[0]].blocks[key[1:]],block)
                        for name in block.dtype.names:
                            if block.dtype[name].kind=='f':
                                np.testing.assert_allclose(block[name],reference.blocks[key][name],
                                                           atol=1e-12,rtol=0)
                            else:
                                np.testing.assert_array_equal(block[name],reference.blocks[key][name])
                if batch_id == 1:
                    frontier=maps[2].frontier_index
                    heights={frontier.nodes[key].height for key in frontier.by_column[(0,0)]}
                    self.assertTrue(any(abs(h-1.5)<1e-5 for h in heights))
                    self.assertFalse(any(abs(h+.1)<1e-5 for h in heights))
        self.assertIs(maps[2]._cost_kernel,_legbot_layer_kernels.update_costs)
        self.assertIs(maps[2]._boundary_kernel,_legbot_layer_kernels.find_boundaries)
        self.assertGreater(parallel_workers,1)
        self.assertGreater(maps[2].last_work['geometry_skipped_cells'],0)

    def test_changed_intervals_cover_moving_removed_and_equal_height_surfaces(self):
        rng=np.random.default_rng(12)
        layers=tuple(range(-8,33))
        for _ in range(100):
            previous=tuple(sorted((round(float(z),3),i+1,bool(i%2))
                                  for i,z in enumerate(rng.uniform(-.5,2.8,12))))
            current=list(previous)
            del current[int(rng.integers(len(current)))]
            at=int(rng.integers(len(current)))
            height,identity,mature=current[at]
            current[at]=(height+.07,identity,not mature)
            current.extend(((.5,100,False),(.5,101,True)))
            current=tuple(sorted(current))
            retained=layers[:-2]
            selected=set(changed_slice_indices(previous,current,layers,retained,0.,.1))
            def selected_surfaces(records,k):
                below=[r for r in records if r[0]<=k*.1+1e-7]
                above=[r for r in records if r[0]>k*.1+1e-7]
                return (below[-1] if below else None,above[0] if above else None)
            expected={k for k in layers if k not in retained or
                      selected_surfaces(previous,k)!=selected_surfaces(current,k)}
            self.assertTrue(expected<=selected,(expected-selected,previous,current))

    def test_changed_intervals_include_support_hidden_below_an_underside(self):
        from legbot_exploration.map_evidence import SurfaceColumn,SurfaceRecord
        rng=np.random.default_rng(36);layers=tuple(range(-5,35))
        for _ in range(100):
            previous=tuple(sorted((round(float(z),3),i+1,True,int(rng.choice((-1,0,1))))
                                  for i,z in enumerate(rng.uniform(-.3,3.,14))))
            current=list(previous)
            del current[int(rng.integers(len(current)))]
            current.append((round(float(rng.uniform(-.3,3.)),3),100,True,1))
            current=tuple(sorted(current))
            selected=set(changed_slice_indices(previous,current,layers,layers,0.,.1))
            def value(records,k):
                surfaces=[SurfaceRecord(i,h,1,h,h,mature_at=1,orientation=o) for h,i,_,o in records]
                g,c=SurfaceColumn().at(k*.1,(tuple(r.height for r in surfaces),surfaces))
                return tuple(None if r is None else r.id for r in (g,c))
            expected={k for k in layers if value(previous,k)!=value(current,k)}
            self.assertTrue(expected<=selected,(expected-selected,previous,current))

    def test_new_neighbor_cuts_and_maturity_keep_full_recompute_geometry(self):
        settings=MapSettings(block_size=8,slice_dh=.1,maturity_min_points=3)
        reference=FixedLayerMap(settings,update_workers=1,native_updates=False)
        mapping=FixedLayerMap(settings,update_workers=4)
        initial=[(x*.1,y*.1,z) for x in range(-2,3) for y in range(-2,3)
                 for z in (0.,.8,1.6,2.4)]
        batches=[initial,[(0.,0.,1.05)],[(0.,0.,.8)],[(0.,0.,.8)],
                 [(.3,0.,3.4)],[(0.,0.,.198)],[(0.,0.,.21)]]
        for i,batch in enumerate(batches):
            for m in (reference,mapping):
                m.integrate(batch)
                if m is reference:
                    with patch('legbot_exploration.fixed_layer_map.changed_slice_indices',
                               side_effect=lambda old,new,layers,*args:list(layers)):
                        m.build()
                else:m.build()
            self.assertEqual(mapping.geometry_changed,reference.geometry_changed)
            self.assertEqual(mapping.counts,reference.counts)
            self.assertEqual(mapping.blocks.keys(),reference.blocks.keys())
            self.assertEqual(mapping.frontier_index.nodes,reference.frontier_index.nodes)
            groups=lambda m:{frozenset(c['members']) for c in m.frontier_index.clusters.values()}
            self.assertEqual(groups(mapping),groups(reference))
            for key,block in mapping.blocks.items():
                for field in block.dtype.names:
                    np.testing.assert_array_equal(block[field],reference.blocks[key][field],
                                                  err_msg=str((i,key,field)))
            if i==1:
                self.assertLess(mapping.last_work['geometry_update_cells'],
                                mapping.last_work['geometry_candidate_cells']//2)
            if i==3:
                self.assertTrue(mapping.cell((8,0,0))['g_mature'])
            if i==4:
                self.assertIn(34,mapping.column_layers[(0,0)])
                self.assertAlmostEqual(mapping.cell((34,0,0))['G'],2.4,places=5)
            if i==6:
                self.assertAlmostEqual(mapping.cell((2,0,0))['G'],0.,places=5)
                self.assertAlmostEqual(mapping.cell((3,0,0))['G'],.204,places=5)
        if mapping._pool:mapping._pool.shutdown()

    def test_build_retries_all_costs_after_a_worker_fails(self):
        m=FixedLayerMap(MapSettings(block_size=8),update_workers=2)
        m.integrate([(x*.1,y*.1,z) for x in range(-5,6) for y in range(-5,6) for z in (0.,1.5)])
        adapter=OnlinePct.__new__(OnlinePct)
        adapter.map=m
        adapter.version=None
        adapter.planner=object()
        run_costs=m._cost_kernel
        with patch.object(m,'_cost_kernel',side_effect=RuntimeError('cost worker failed')):
            with self.assertRaisesRegex(RuntimeError,'cost worker failed'):
                m.build()
        self.assertEqual(m.revision,0)
        self.assertTrue(m.dirty)
        self.assertFalse(m.frontier_index.nodes)
        self.assertIs(m._cost_kernel,run_costs)
        with self.assertRaisesRegex(RuntimeError,'pct_map_commit_failed'):
            adapter.prepare(b'',1)
        adapter.commit_pending(b'',1)
        self.assertEqual(adapter.version,1)
        self.assertEqual(m.revision,1)
        self.assertFalse(m.dirty)
        self.assertTrue(m.frontier_index.nodes)
        self.assertGreater(m.counts['measured_surface_cells'],0)


if __name__ == '__main__':
    unittest.main()
