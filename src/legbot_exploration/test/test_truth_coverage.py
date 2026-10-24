"""以实际 Gazebo 平面场景验证离线真值掩码，不启动仿真。"""

import os
import unittest

from legbot_exploration.truth_coverage import scene_geometry, verify_grid


WORLD = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..',
                                     'legbot_bringup', 'worlds', 'ExplorationFlat.sdf'))


class TruthCoverageTest(unittest.TestCase):
    def test_scene_and_missing_ground_cell(self):
        ground, obstacles = scene_geometry(WORLD)
        self.assertGreater(len(ground), 0)
        self.assertGreater(len(obstacles), 20)
        data = [0] * 16
        full = verify_grid(WORLD, -27.5, 5.5, 4, 4, .25, data)
        self.assertEqual(full['unobserved_reachable_cells'], 0)
        self.assertGreater(full['reachable_ground_cells'], 0)
        data[0] = -1
        partial = verify_grid(WORLD, -27.5, 5.5, 4, 4, .25, data)
        self.assertEqual(partial['unobserved_reachable_cells'], 1)


if __name__ == '__main__':
    unittest.main()
