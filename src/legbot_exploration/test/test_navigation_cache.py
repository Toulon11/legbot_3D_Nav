"""Cached navigation evidence must remain local, safe and exportable."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from test_map_update_locality import room
from test_online_pct import native_wrapper
from legbot_exploration.pct_inspection import capture, surface_chunk, SURFACE_DTYPE, NAVIGATION_REASONS


class NavigationCacheTest(unittest.TestCase):
    def test_safety_state_reuses_work_and_invalidates_neighbor_but_not_other_floor(self):
        m=room();key=(1,0,0)
        m._invalidate_auxiliary({key})
        with patch.object(m,'_compute_planning_state',wraps=m._compute_planning_state) as compute:
            first=m.planning_state(key)
            for _ in range(10):self.assertEqual(m.planning_state(key),first)
            self.assertEqual(compute.call_count,1)
            m._invalidate_auxiliary({(13,0,0)})
            self.assertEqual(m.planning_state(key),first)
            self.assertEqual(compute.call_count,1)
            m._invalidate_auxiliary({(1,1,0)})
            self.assertEqual(m.planning_state(key),first)
            self.assertEqual(compute.call_count,2)

    def test_export_uses_navigation_footprint_cost_and_preserves_wire_evidence_bits(self):
        m=room();key=(1,0,0);backend=SimpleNamespace(map=m,metadata={})
        before=capture(backend)
        original=np.frombuffer(dict(before['surfaces'])[(1,0,0)],dtype=SURFACE_DTYPE)
        with patch.object(m,'obstacle_footprint',return_value=True):
            m._invalidate_auxiliary({(1,1,0)})
            self.assertIn((1,0,0),m.export_dirty_blocks)
            changed=capture(backend)
            rows=np.frombuffer(dict(changed['surfaces'])[(1,0,0)],dtype=SURFACE_DTYPE)
            at=np.flatnonzero((rows['x']==0)&(rows['y']==0))[0]
            self.assertEqual(rows['cost'][at],m.planning_cost(key))
            self.assertEqual(rows['cost'][at],50.)
            self.assertEqual(rows['rgb'].view('<u4')[at],0xff3030)
            self.assertEqual(NAVIGATION_REASONS[(int(rows['flags'][at])>>8)&255],
                             'known_obstacle_footprint')
            np.testing.assert_array_equal(rows['flags']&255,original['flags']&255)
            self.assertEqual(SURFACE_DTYPE.itemsize,64)
        m._invalidate_auxiliary({(1,1,0)})
        self.assertLessEqual(m.planning_cost(key),20.)

    def test_native_record_hits_keep_dependencies_and_absolute_owner_after_map_growth(self):
        native_wrapper()
        from legbot_exploration.block_pct import BlockPlanner
        m=room();planner=BlockPlanner(m,native_wrapper());owner=1
        x,y=-int(m.lower[0]),-int(m.lower[1])
        first=planner._read_interval(owner-m.first_layer,x,y)
        self.assertLessEqual(first[0],20.)
        with patch.object(m,'observed_cell',side_effect=AssertionError('cache was not reused')):
            planner._read_blocks.clear();planner._dependency_tiles.clear()
            m.first_layer-=1
            second=planner._read_interval(owner-m.first_layer,x,y)
        self.assertEqual(second[:-1],first[:-1])
        self.assertEqual(second[-1],first[-1]+1)
        self.assertIn((owner,0,0),planner._read_blocks)
        self.assertIn((owner,1,0),planner._read_blocks)
        m._invalidate_auxiliary({(owner,1,0)})
        self.assertNotIn((owner,0,0),m._planning_records)

    def test_timed_out_search_edges_are_diagnostics_not_actionable_scan_blockers(self):
        native_wrapper()
        from legbot_exploration.block_pct import BlockPlanner
        m=room();planner=BlockPlanner(m,native_wrapper())
        noop=lambda *args:None
        planner.planner=SimpleNamespace(set_blocked_edge_radius=noop,set_surface_step_limit=noop,
            init_interval_accessor=noop,set_passage_clearance=noop,set_time_limit=noop,
            search=lambda *args:False,get_search_node_count=lambda:10,timed_out=lambda:True,
            get_blocked_edges=lambda:{0:(1-m.first_layer,4,4,1-m.first_layer,4,4)})
        with patch.object(planner,'_unknown_blockers',return_value=[[.1,0.,0.,1]]):
            self.assertIsNone(planner.plan((-.1,0.,0.),(.1,0.,0.),collect_blockers=True))
        self.assertTrue(planner.search_diagnostics['timed_out'])
        self.assertTrue(planner.search_diagnostics['encountered_unknown'])
        self.assertEqual(planner.search_diagnostics['unknown_blockers'],[])
        self.assertFalse(planner._failures)

    def test_new_measured_neighbor_wall_invalidates_a_previously_free_native_record(self):
        native_wrapper()
        from legbot_exploration.block_pct import BlockPlanner
        m=room();planner=BlockPlanner(m,native_wrapper())
        def read():
            return planner._read_interval(1-m.first_layer,-int(m.lower[0]),-int(m.lower[1]))
        self.assertLessEqual(read()[0],20.)
        upper=m.planning_state((13,0,0))
        m.integrate([(.1,0.,z) for z in np.arange(.1,1.5,.1)]);m.build()
        self.assertEqual(read()[0],50.)
        self.assertIs(m._planning_states[(13,0,0)],upper)


if __name__=='__main__':unittest.main()
