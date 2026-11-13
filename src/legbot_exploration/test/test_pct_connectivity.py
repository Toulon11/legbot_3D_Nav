"""Reachability prefilter must be incremental and never mix planner semantics."""
import unittest
from unittest.mock import patch
import numpy as np
from legbot_exploration.fixed_layer_map import FixedLayerMap,MapSettings


def tile_fixture():
    m=FixedLayerMap(MapSettings(block_size=4,robot_radius=0.,safety_margin=0.))
    m.first_layer=0;m.last_layer=1
    m.lower=np.array([-8,-8]);m.upper=np.array([12,12])
    return m


def set_cell(m,key,safe=True,height=0.):
    row=m.cell(key,True)
    row['G']=height
    row['g_mature']=row['measured']=row['feature_valid']=True
    row['unknown']=row['solid']=False
    row['hard']=row['unknown_hard']=not safe
    row['inflated_terrain_cost']=1.
    m.column_layers.setdefault(key[1:],set()).add(key[0])


class ConnectivityTest(unittest.TestCase):
    def test_diagonal_cross_block_connections_and_bridge_removal(self):
        m=tile_fixture();keys={(0,x,x) for x in range(-5,10)}
        for key in keys:set_cell(m,key)
        m.connectivity.update(keys)
        self.assertTrue(m.connectivity.possible((-5,-5),(9,9)))
        set_cell(m,(0,4,4),False);m.connectivity.update({(0,4,4)})
        self.assertFalse(m.connectivity.possible((-5,-5),(9,9)))
        self.assertEqual(m.connectivity.last_work['relabelled_tiles'],1)
        set_cell(m,(0,4,4));m.connectivity.update({(0,4,4)})
        self.assertTrue(m.connectivity.possible((-5,-5),(9,9)))

    def test_all_eight_hard_neighbours_leave_a_safe_but_unreachable_target(self):
        m=tile_fixture();keys={(0,x,y) for x in range(-5,6) for y in range(-5,6)}
        for key in keys:set_cell(m,key)
        for x in (-1,0,1):
            for y in (-1,0,1):
                if x or y:set_cell(m,(0,x,y),False)
        m.connectivity.update(keys)
        self.assertFalse(m.connectivity.possible((-5,-5),(0,0)))
        # A traversable equivalent cut at the same XY can bridge a native
        # gateway. The coarse check must not incorrectly reject that path.
        upper={(1,x,0) for x in range(-3,1)}
        for key in upper:set_cell(m,key)
        m.connectivity.update(upper)
        self.assertTrue(m.connectivity.possible((-5,-5),(0,0)))

    def test_cost_only_changes_relabel_only_the_changed_tile(self):
        m=tile_fixture();keys={(0,x,0) for x in range(-5,10)}
        for key in keys:set_cell(m,key)
        m.connectivity.update(keys);revision=m.connectivity.revision
        m.cell((0,0,0))['inflated_terrain_cost']=25.
        with patch.object(m,'iter_cells',side_effect=AssertionError('full map traversal')):
            m.connectivity.update({(0,0,0)})
        self.assertFalse(m.connectivity.possible((-5,0),(9,0)))
        self.assertEqual(m.connectivity.revision,revision+1)
        self.assertEqual(m.connectivity.last_work['checked_tiles'],1)
        m.cell((0,0,0))['inflated_terrain_cost']=26.
        m.connectivity.update({(0,0,0)})
        self.assertEqual(m.connectivity.revision,revision+1)

    def test_components_match_full_projection_after_random_local_changes(self):
        from scipy.ndimage import label
        m=tile_fixture();rng=np.random.default_rng(19)
        safe=rng.random((16,16))>.35
        keys={(0,x-8,y-8) for x in range(16) for y in range(16)}
        for key in keys:set_cell(m,key,bool(safe[key[1]+8,key[2]+8]))
        m.connectivity.update(keys)
        for _ in range(20):
            x,y=map(int,rng.integers(0,16,2));safe[x,y]=not safe[x,y]
            set_cell(m,(0,x-8,y-8),bool(safe[x,y]));m.connectivity.update({(0,x-8,y-8)})
            full,_=label(safe,structure=np.ones((3,3),bool))
            for start in np.argwhere(safe)[::7]:
                for goal in np.argwhere(safe)[::13]:
                    self.assertEqual(m.connectivity.possible(start-8,goal-8),full[tuple(start)]==full[tuple(goal)])


if __name__=='__main__':unittest.main()
