"""Surface-frontier semantics independent of GPU, plus native A* integration."""
from types import SimpleNamespace
import unittest
import numpy as np
from legbot_exploration.pct_frontiers import extract_frontiers
from legbot_exploration.online_pct import OnlinePct
from test_online_pct import PCT


def fixture(heights=(0.,), size=50):
    g=np.full((len(heights),size,size),np.nan,dtype=np.float32)
    c=np.full_like(g,np.nan);cost=np.full_like(g,50)
    for layer,h in enumerate(heights):
        g[layer,5:-5,5:-5]=h;c[layer,5:-5,5:-5]=h+1.5
        cost[layer,12:-12,12:-12]=1
    zero=np.zeros_like(g)
    return SimpleNamespace(tomogram=np.stack([cost,zero,zero,g,c]),resolution=.1,
                           center=np.zeros(2),offset=np.array([size//2]*2))


class PctFrontierTest(unittest.TestCase):
    def test_duplicate_slices_are_one_physical_frontier(self):
        one=extract_frontiers(fixture(),.16,.5)
        duplicate=extract_frontiers(fixture((0.,0.,0.)),.16,.5)
        self.assertEqual(one['frontier_cells'],156)
        self.assertEqual(duplicate['frontier_cells'],one['frontier_cells'])
        self.assertEqual(len(duplicate['clusters']),1)
        self.assertEqual(duplicate['no_observation_clusters'],0)
        self.assertTrue(duplicate['clusters'][0]['observations'])

    def test_overlapping_floors_keep_surface_height(self):
        result=extract_frontiers(fixture((0.,1.5)),.16,.5)
        self.assertEqual(len(result['clusters']),2)
        self.assertEqual({c['frontier'][2] for c in result['clusters']},{0.,1.5})
        for c in result['clusters']:
            self.assertTrue(c['observations'])
            self.assertTrue(all(p[2]==c['frontier'][2] for p in c['observations']))

    def test_native_inflation_moves_observer_inland_not_frontier(self):
        p=fixture();result=extract_frontiers(p,.16,.5)
        for c in result['clusters']:
            for xyz in c['observations']:
                i,j=np.rint(np.asarray(xyz[:2])/.1+p.offset).astype(int)
                self.assertLessEqual(p.tomogram[0,0,i,j],20)
        p.tomogram[0].fill(50)
        blocked=extract_frontiers(p,.16,.5)
        self.assertEqual(blocked['frontier_cells'],result['frontier_cells'])
        self.assertEqual(blocked['no_observation_clusters'],1)
        self.assertEqual(blocked['clusters'][0]['observations'],[])

    def test_new_surface_removes_observation_gap(self):
        p=fixture();before=extract_frontiers(p,.16,.5)
        p.tomogram[3,0,24:26,24:26]=np.nan
        gap=extract_frontiers(p,.16,.5)
        self.assertGreater(gap['frontier_cells'],before['frontier_cells'])
        p.tomogram[3,0,24:26,24:26]=0
        filled=extract_frontiers(p,.16,.5)
        self.assertEqual(filled['frontier_cells'],before['frontier_cells'])

    def test_empty_map_not_a_healthy_complete_map(self):
        p=fixture();p.tomogram[3:5].fill(np.nan)
        result=extract_frontiers(p,.16,.5)
        self.assertEqual(result['frontier_cells'],0)
        self.assertEqual(result['known_surface_cells'],0)
        self.assertEqual(result['safe_cells'],0)

    def test_cache_is_sparse_and_follows_geometry_revision(self):
        a=OnlinePct.__new__(OnlinePct);a.version=1;a.geometry_revision=1;a.metadata={}
        a.map=SimpleNamespace(settings=SimpleNamespace(step_max=.16,minimum_clearance=.5),
                              dirty=set(),evidence_dirty=set())
        a.cfg=SimpleNamespace(trav=SimpleNamespace(step_max=.16,interval_min=.5))
        a.planner=fixture()
        first=a.frontiers(b'',1)
        a.version=2
        unchanged=a.frontiers(b'',2)
        self.assertIs(first['clusters'],unchanged['clusters'])
        # Frontend must not retain a previous dense native planner.
        self.assertFalse(any(value is a.planner for key,value in vars(a).items() if key!='planner'))
        a.planner=fixture();a.planner.tomogram[3,0,24:26,24:26]=np.nan
        a.geometry_revision=2;a.version=3
        changed=a.frontiers(b'',3)
        self.assertGreater(changed['frontier_cells'],first['frontier_cells'])
        positions=np.asarray(changed['clusters'][0]['observations'])
        self.assertGreater(np.linalg.norm(positions[0]-positions[1]),2.)

    def test_diagonal_unknown_is_also_frontier(self):
        p=fixture();before=extract_frontiers(p,.16,.5)
        p.tomogram[3,0,25,25]=np.nan
        after=extract_frontiers(p,.16,.5)
        self.assertEqual(after['frontier_cells']-before['frontier_cells'],8)

    def test_observation_runs_original_astar_and_rejects_other_floor(self):
        a=OnlinePct(str(PCT))
        a.configure({'maturity_min_points':1})
        points=np.asarray([[i*.1,j*.1,z] for i in range(-20,21) for j in range(-20,21) for z in (0.,1.5)],dtype='float32')
        a.integrate(memoryview(points));a.prepare(b'',7)
        result=a.frontiers(b'',7)
        observation=next(c for c in result['clusters'] if abs(c['frontier'][2])<.05)['observations'][0]
        target=np.asarray(observation)+[0,0,.32]
        reason,route,_=a.plan_surface(b'',7,(0,0,.32),target)
        self.assertEqual(reason,'ok');self.assertTrue(route)
        np.testing.assert_allclose(np.asarray(route)[-1,2],.32)
        self.assertEqual(a.plan_surface(b'',7,(0,0,.32),target+[0,0,.6])[0],'pct_unsafe_goal')


if __name__=='__main__':
    unittest.main()
