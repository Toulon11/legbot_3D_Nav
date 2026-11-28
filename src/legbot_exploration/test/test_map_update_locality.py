"""Local map indexing/export must preserve safety and unrelated height evidence."""
import json
import unittest
from types import SimpleNamespace
import numpy as np

from legbot_exploration.fixed_layer_map import MapSettings
from legbot_exploration.interval_map import IntervalLayerMap
from legbot_exploration.pct_inspection import capture, SURFACE_DTYPE
from legbot_exploration.benchmark_metrics import SurfaceCoverage
from legbot_exploration.surface_update_stream import SurfaceUpdateStream


def room(floors=2):
    m=IntervalLayerMap(MapSettings(maturity_min_points=1,robot_radius=.1,safety_margin=0.,
        traversability_kernel_size=3,inflation_kernel_size=3),native_updates=True)
    m.integrate([(x*.1,y*.1,h) for x in range(-4,5) for y in range(-4,5)
                 for k in range(floors) for h in (3.*k,3.*k+1.5)])
    m.build()
    return m


class LocalMapTest(unittest.TestCase):
    def test_ordered_support_query_work_does_not_grow_with_number_of_floors(self):
        work=[];actual=[]
        for floors in (2,8):
            m=room(floors)
            masks=m.support_blocks((-3,-3),(3,3),-.16,.16)
            work.append(m.support_query_work)
            got={(k,bx*m.settings.block_size+int(i),by*m.settings.block_size+int(j))
                 for (k,bx,by),mask in masks.items() for i,j in zip(*np.nonzero(mask))}
            expected={key for key,row in m.iter_cells(True)
                      if -3<=key[1]<=3 and -3<=key[2]<=3 and -.16<=row['G']<=.16}
            self.assertEqual(got,expected)
            actual.append(got)
        self.assertEqual(work[0],work[1]);self.assertEqual(actual[0],actual[1])

    def test_upper_change_keeps_lower_node_cost_cache_and_export(self):
        m=room();backend=SimpleNamespace(map=m,metadata={})
        first=capture(backend);self.assertTrue(first['reset'])
        lower=m._interval_nodes[0,0][1]
        lowbytes=dict(first['surfaces'])[(1,0,0)]
        m._cache_auxiliary(m._seam_costs,(1,0,0),12.)
        m._cache_auxiliary(m._obstacle_costs,(1,0,0),False)
        m.integrate([(0.,0.,4.3)]);m.build()
        delta=capture(backend)
        self.assertIs(m._interval_nodes[0,0][1],lower)
        self.assertEqual(m._seam_costs[1,0,0],12.)
        self.assertNotIn((1,0,0),dict(delta['surfaces']))
        self.assertEqual(dict(first['surfaces'])[(1,0,0)],lowbytes)
        self.assertFalse(capture(backend)['surfaces'])
        self.assertFalse(capture(backend)['returns'])
        m.integrate([(0.,0.,.2)]);m.build()
        self.assertNotIn((1,0,0),m._seam_costs)
        self.assertNotIn((1,0,0),m._obstacle_costs)

    def test_export_chunks_match_committed_rows_and_do_not_alias_them(self):
        m=room();backend=SimpleNamespace(map=m,metadata={})
        snapshot=capture(backend)
        payload=dict(snapshot['surfaces'])[(1,0,0)]
        rows=np.frombuffer(payload,dtype=SURFACE_DTYPE)
        g=rows['z'].copy()
        m.blocks[1,0,0]['G'][:]=99.
        np.testing.assert_array_equal(rows['z'],g)

    def test_delta_gap_waits_for_complete_snapshot_and_keeps_coverage_unique(self):
        coverage=SurfaceCoverage();stream=SurfaceUpdateStream(coverage)
        def update(version,base,full=False,points=(),epoch=0):
            return dict(version=version,base_version=base,full=full,epoch=epoch,coverage=list(points))
        self.assertFalse(stream.apply(update(2,1,points=[[0,0,0,32]])))
        self.assertFalse(stream.covers(1))
        self.assertTrue(stream.apply(update(3,2,True,[[0,0,0,32]])))
        self.assertTrue(stream.apply(update(4,3,points=[[.1,0,0,32]])))
        self.assertEqual(coverage.count,2)
        self.assertFalse(stream.apply(update(6,5,points=[[.2,0,0,32]])))
        self.assertFalse(stream.covers(4))
        self.assertTrue(stream.apply(update(7,6,True,[[0,0,0,32],[.1,0,0,32],[.2,0,0,32]])))
        self.assertEqual(coverage.count,3)
        self.assertFalse(stream.apply(update(8,7,epoch=1)))
        self.assertFalse(stream.covers(8))


if __name__=='__main__':unittest.main()
