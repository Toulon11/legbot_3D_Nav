import math
import unittest

from legbot_exploration.exploration_core import GoalProgressGate, RecoveryGate, Settings, VoxelMap, select_frontier


class CoreTest(unittest.TestCase):
    def test_ground_rays_produce_unknown_frontier(self):
        cfg = Settings()
        voxel_map = VoxelMap(cfg)
        robot = (0.0, 0.0, 0.3)
        points = [(4.0 * math.cos(i * math.pi / 100 - math.pi / 2),
                   4.0 * math.sin(i * math.pi / 100 - math.pi / 2), -0.2)
                  for i in range(101)]
        self.assertEqual(voxel_map.integrate((0.0, 0.0, 0.4), points, robot[2]), 101)
        grid = voxel_map.project(robot[:2])
        self.assertGreater(sum(state == 0 for state in grid.values()), 100)
        self.assertFalse(any(state == 1 for state in grid.values()))
        goal, _, clusters, _, _ = select_frontier(voxel_map, robot, cfg)
        self.assertGreater(clusters, 0)
        self.assertIsNotNone(goal)
        self.assertNotIn(voxel_map.key((*goal, robot[2]))[:2], grid)
        self.assertGreaterEqual(math.dist(goal, robot[:2]), cfg.min_goal_distance)
        staged_cfg = Settings(max_goal_distance=2.0)
        staged_map = VoxelMap(cfg)
        ring = [(4.0 * math.cos(i * 2 * math.pi / 360),
                 4.0 * math.sin(i * 2 * math.pi / 360), -0.2) for i in range(360)]
        staged_map.integrate((0.0, 0.0, 0.4), ring, robot[2])
        stage_grid = staged_map.project(robot[:2])
        staged_goal, _, _, _, _ = select_frontier(staged_map, robot, staged_cfg)
        self.assertIsNotNone(staged_goal)
        self.assertEqual(stage_grid[staged_map.key((*staged_goal, robot[2]))[:2]], 0)
        self.assertLessEqual(math.dist(staged_goal, robot[:2]), staged_cfg.max_goal_distance)
        blocked_goal, _, _, _, _ = select_frontier(voxel_map, robot, cfg,
                                              external_obstacles=[goal])
        self.assertNotEqual(blocked_goal, goal)
        previous_size = len(voxel_map.voxels)
        shifted_points = [(x, y, z - 3.0) for x, y, z in points]
        voxel_map.integrate((0.0, 0.0, -2.6), shifted_points, -2.7)
        self.assertEqual(len(voxel_map.voxels), previous_size)
        self.assertEqual(voxel_map.project(robot[:2]), grid)
        self.assertIsNotNone(select_frontier(voxel_map, (0.0, 0.0, -2.7), cfg)[0])
        high_threshold = Settings(min_frontier_cells=10000)
        self.assertIsNone(select_frontier(voxel_map, robot, high_threshold)[0])
        high_threshold.min_frontier_cells = 1
        self.assertIsNotNone(select_frontier(voxel_map, robot, high_threshold)[0])

    def test_obstacle_endpoint_and_empty_map(self):
        cfg = Settings()
        voxel_map = VoxelMap(cfg)
        self.assertEqual(select_frontier(voxel_map, (0.0, 0.0, 0.3), cfg)[2], 0)
        voxel_map.integrate((0.0, 0.0, 0.4), [(2.0, 0.0, 0.4)], 0.3)
        grid = voxel_map.project((0.0, 0.0))
        self.assertEqual(grid[(8, 0)], 1)
        self.assertEqual(grid[(0, 0)], 0)

    def test_global_search_finds_frontier_beyond_local_window(self):
        robot = (0.0, 0.0, 0.3)
        ring = [(4 * math.cos(i * 2 * math.pi / 360),
                 4 * math.sin(i * 2 * math.pi / 360), -0.2) for i in range(360)]
        local_cfg = Settings(local_search_radius=1.0, max_goal_distance=2.0)
        local_map = VoxelMap(local_cfg)
        local_map.integrate((0.0, 0.0, 0.4), ring, robot[2])
        self.assertEqual(select_frontier(local_map, robot, local_cfg)[3], 0)
        global_cfg = Settings(global_search=True, local_search_radius=1.0,
                              max_goal_distance=2.0, exploration_min_x=-5.0,
                              exploration_max_x=5.0, exploration_min_y=-5.0,
                              exploration_max_y=5.0)
        global_map = VoxelMap(global_cfg)
        global_map.integrate((0.0, 0.0, 0.4), ring, robot[2])
        goal, _, _, frontier_cells, _ = select_frontier(global_map, robot, global_cfg)
        self.assertGreater(frontier_cells, 0)
        self.assertIsNotNone(goal)
        self.assertLessEqual(math.dist(goal, robot[:2]), 2.0)

    def test_recovery_requires_cluster_and_new_observations(self):
        gate = RecoveryGate(min_new_cells=8)
        self.assertFalse(gate.eligible(0, 0, 0, 100))  # isolated residual cells
        self.assertTrue(gate.eligible(1, 0, 0, 100))
        gate.start(100)
        gate.finish(103)
        self.assertTrue(gate.waiting_for_new_cells(103))
        self.assertFalse(gate.eligible(1, 0, 0, 110))
        self.assertTrue(gate.eligible(1, 0, 0, 111))
        self.assertFalse(gate.eligible(1, 1, 0, 111))
        gate.start(111)
        gate.finish(120)
        self.assertFalse(gate.waiting_for_new_cells(120))

    def test_reached_goal_without_new_observations_is_suppressed(self):
        gate = GoalProgressGate(min_new_cells=8)
        gate.finish((2.0, 1.0), 100, 103)
        self.assertEqual(gate.blocked(110), [(2.0, 1.0)])
        self.assertEqual(gate.blocked(111), [])
        gate.finish((3.0, 1.0), 111, 120)
        self.assertEqual(gate.blocked(120), [])


if __name__ == '__main__':
    unittest.main()
