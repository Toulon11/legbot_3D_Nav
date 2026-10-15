import math
import unittest

from legbot_exploration.exploration_core import Settings, VoxelMap, select_frontier


class CoreTest(unittest.TestCase):
    def test_ground_rays_produce_unknown_frontier(self):
        cfg = Settings()
        voxel_map = VoxelMap(cfg)
        robot = (0.0, 0.0, 0.3)
        points = [(4.0 * math.cos(i * math.pi / 100 - math.pi / 2),
                   4.0 * math.sin(i * math.pi / 100 - math.pi / 2), -0.2)
                  for i in range(101)]
        self.assertEqual(voxel_map.integrate((0.0, 0.0, 0.4), points, robot[2]), 101)
        grid = voxel_map.project(robot[2], robot[:2])
        self.assertGreater(sum(state == 0 for state in grid.values()), 100)
        self.assertFalse(any(state == 1 for state in grid.values()))
        goal, _, clusters = select_frontier(voxel_map, robot, cfg)
        self.assertGreater(clusters, 0)
        self.assertIsNotNone(goal)
        self.assertNotIn(voxel_map.key((*goal, robot[2]))[:2], grid)
        self.assertGreaterEqual(math.dist(goal, robot[:2]), cfg.min_goal_distance)

    def test_obstacle_endpoint_and_empty_map(self):
        cfg = Settings()
        voxel_map = VoxelMap(cfg)
        self.assertEqual(select_frontier(voxel_map, (0.0, 0.0, 0.3), cfg)[2], 0)
        voxel_map.integrate((0.0, 0.0, 0.4), [(2.0, 0.0, 0.4)], 0.3)
        grid = voxel_map.project(0.3, (0.0, 0.0))
        self.assertEqual(grid[(8, 0)], 1)
        self.assertEqual(grid[(0, 0)], 0)


if __name__ == '__main__':
    unittest.main()
