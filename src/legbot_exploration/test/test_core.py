# 导入 数学函数，如开方、向下取整和距离计算。
import math
# 导入 unittest 提供的类型或函数。
import unittest

# 导入 legbot_exploration.exploration_core 提供的类型或函数。
from legbot_exploration.exploration_core import (GoalProgressGate, RecoveryGate, Settings,
    VoxelMap, assess_coverage, assess_reachable_coverage, select_frontier,
    select_observation_goal)


# 这些测试只调用核心算法，不需要启动 Gazebo 或 ROS 节点。
class CoreTest(unittest.TestCase):
    def test_global_coverage_counts_unknown_even_without_frontier(self):
        cfg = Settings(global_search=True, resolution=1., exploration_min_x=0.,
                       exploration_max_x=3., exploration_min_y=0., exploration_max_y=3.)
        grid = {(x, y): 1 for x in range(3) for y in range(3) if (x, y) != (1, 1)}
        coverage = assess_coverage(grid, cfg)
        self.assertEqual(coverage['total_cells'], 9)
        self.assertEqual(coverage['unknown_cells'], 1)
        self.assertEqual(coverage['coverage_ratio'], 8 / 9)
        self.assertIsNone(select_observation_goal(VoxelMap(cfg), (.5, .5, .3), cfg, grid))
        grid[(1, 1)] = 1
        self.assertEqual(assess_coverage(grid, cfg)['unknown_cells'], 0)
        self.assertIsNone(assess_coverage(grid, Settings()))

    def test_supplemental_goal_is_known_free(self):
        cfg = Settings(global_search=True, resolution=1., robot_radius=0.,
                       min_goal_distance=1., max_goal_distance=4., max_ray_range=5.,
                       exploration_min_x=0., exploration_max_x=6.,
                       exploration_min_y=0., exploration_max_y=2.)
        grid = {(x, y): 0 for x in range(5) for y in range(2)}
        goal = select_observation_goal(VoxelMap(cfg), (.5, .5, .3), cfg, grid)
        self.assertIsNotNone(goal)
        self.assertEqual(grid[(math.floor(goal[0]), math.floor(goal[1]))], 0)
        self.assertGreater(assess_coverage(grid, cfg)['unknown_cells'], 0)

    def test_reachable_coverage_excludes_enclosed_unknown_but_keeps_open_passage(self):
        cfg = Settings(global_search=True, resolution=1., robot_radius=0.,
                       exploration_min_x=0., exploration_max_x=5.,
                       exploration_min_y=0., exploration_max_y=5.)
        grid = {(x, y): 0 for x in range(5) for y in range(5) if (x, y) != (2, 2)}
        for cell in ((1, 1), (1, 2), (1, 3), (2, 1),
                     (2, 3), (3, 1), (3, 2), (3, 3)):
            grid[cell] = 1
        result = assess_reachable_coverage(grid, cfg, (.5, .5))
        self.assertTrue(result['reachability_valid'])
        self.assertEqual(result['unknown_cells'], 1)
        self.assertEqual(result['reachable_unknown_cells'], 0)
        self.assertEqual(result['excluded_unknown_cells'], 1)
        del grid[(2, 1)]
        result = assess_reachable_coverage(grid, cfg, (.5, .5))
        self.assertGreaterEqual(result['reachable_unknown_cells'], 2)
        self.assertEqual(result['excluded_unknown_cells'], 0)
        self.assertFalse(assess_reachable_coverage(grid, cfg, (-1., .5))[
            'reachability_valid'])

    def test_diagonal_gap_between_two_obstacles_is_not_a_passage(self):
        cfg = Settings(global_search=True, resolution=1., robot_radius=0.,
                       exploration_min_x=0., exploration_max_x=2.,
                       exploration_min_y=0., exploration_max_y=2.)
        result = assess_reachable_coverage({(0, 0): 0, (1, 0): 1, (0, 1): 1},
                                           cfg, (.5, .5))
        self.assertTrue(result['reachability_valid'])
        self.assertEqual(result['reachable_unknown_cells'], 0)
        self.assertEqual(result['excluded_unknown_cells'], 1)

    # 地面射线应产生空闲格和未知边界；远处边界应生成分段目标。
    def test_ground_rays_produce_unknown_frontier(self):
        cfg = Settings()
        voxel_map = VoxelMap(cfg)
        robot = (0.0, 0.0, 0.3)
        points = [(4.0 * math.cos(i * math.pi / 100 - math.pi / 2),
                   4.0 * math.sin(i * math.pi / 100 - math.pi / 2), -0.2)
                  # 依次处理集合中的每一项。
                  for i in range(101)]
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertEqual(voxel_map.integrate((0.0, 0.0, 0.4), points, robot[2]), 101)
        # 将计算结果保存为 grid（高度切片后的二维占用表）。
        grid = voxel_map.project(robot[:2])
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertGreater(sum(state == 0 for state in grid.values()), 100)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertFalse(any(state == 1 for state in grid.values()))
        goal, _, clusters, _, _ = select_frontier(voxel_map, robot, cfg)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertGreater(clusters, 0)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertIsNotNone(goal)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertNotIn(voxel_map.key((*goal, robot[2]))[:2], grid)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertGreaterEqual(math.dist(goal, robot[:2]), cfg.min_goal_distance)
        staged_cfg = Settings(max_goal_distance=2.0)
        staged_map = VoxelMap(cfg)
        ring = [(4.0 * math.cos(i * 2 * math.pi / 360),
                 4.0 * math.sin(i * 2 * math.pi / 360), -0.2) for i in range(360)]
        staged_map.integrate((0.0, 0.0, 0.4), ring, robot[2])
        stage_grid = staged_map.project(robot[:2])
        staged_goal, _, _, _, _ = select_frontier(staged_map, robot, staged_cfg)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertIsNotNone(staged_goal)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertEqual(stage_grid[staged_map.key((*staged_goal, robot[2]))[:2]], 0)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertLessEqual(math.dist(staged_goal, robot[:2]), staged_cfg.max_goal_distance)
        blocked_goal, _, _, _, _ = select_frontier(voxel_map, robot, cfg,
                                              # 将计算结果保存为 external_obstacles（SCAN 提供的障碍点列表）。
                                              external_obstacles=[goal])
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertNotEqual(blocked_goal, goal)
        previous_size = len(voxel_map.voxels)
        shifted_points = [(x, y, z - 3.0) for x, y, z in points]
        voxel_map.integrate((0.0, 0.0, -2.6), shifted_points, -2.7)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertEqual(len(voxel_map.voxels), previous_size)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertEqual(voxel_map.project(robot[:2]), grid)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertIsNotNone(select_frontier(voxel_map, (0.0, 0.0, -2.7), cfg)[0])
        high_threshold = Settings(min_frontier_cells=10000)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertIsNone(select_frontier(voxel_map, robot, high_threshold)[0])
        high_threshold.min_frontier_cells = 1
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertIsNotNone(select_frontier(voxel_map, robot, high_threshold)[0])

    # 空地图没有前沿；障碍高度范围内的射线终点会标为障碍格。
    def test_obstacle_endpoint_and_empty_map(self):
        cfg = Settings()
        voxel_map = VoxelMap(cfg)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertEqual(select_frontier(voxel_map, (0.0, 0.0, 0.3), cfg)[2], 0)
        voxel_map.integrate((0.0, 0.0, 0.4), [(2.0, 0.0, 0.4)], 0.3)
        # 将计算结果保存为 grid（高度切片后的二维占用表）。
        grid = voxel_map.project((0.0, 0.0))
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertEqual(grid[(8, 0)], 1)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertEqual(grid[(0, 0)], 0)

    # 局部窗口看不到的前沿，全图搜索仍能找到并给出近处落点。
    def test_global_search_finds_frontier_beyond_local_window(self):
        robot = (0.0, 0.0, 0.3)
        ring = [(4 * math.cos(i * 2 * math.pi / 360),
                 4 * math.sin(i * 2 * math.pi / 360), -0.2) for i in range(360)]
        local_cfg = Settings(local_search_radius=1.0, max_goal_distance=2.0)
        local_map = VoxelMap(local_cfg)
        local_map.integrate((0.0, 0.0, 0.4), ring, robot[2])
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertEqual(select_frontier(local_map, robot, local_cfg)[3], 0)
        global_cfg = Settings(global_search=True, local_search_radius=1.0,
                              # 将计算结果保存为 max_goal_distance（目标最大距离）。
                              max_goal_distance=2.0, exploration_min_x=-5.0,
                              # 将计算结果保存为 exploration_max_x（全图边界的 X 最大值）。
                              exploration_max_x=5.0, exploration_min_y=-5.0,
                              # 将计算结果保存为 exploration_max_y（全图边界的 Y 最大值）。
                              exploration_max_y=5.0)
        global_map = VoxelMap(global_cfg)
        global_map.integrate((0.0, 0.0, 0.4), ring, robot[2])
        goal, _, _, frontier_cells, _ = select_frontier(global_map, robot, global_cfg)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertGreater(frontier_cells, 0)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertIsNotNone(goal)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertLessEqual(math.dist(goal, robot[:2]), 2.0)

    # 没有有效簇或回退后地图增长不足时，不应反复触发轨迹回退。
    def test_recovery_requires_cluster_and_new_observations(self):
        gate = RecoveryGate(min_new_cells=8)
        self.assertFalse(gate.eligible(0, 0, 0, 100))  # isolated residual cells
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertTrue(gate.eligible(1, 0, 0, 100))
        gate.start(100)
        gate.finish(103)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertTrue(gate.waiting_for_new_cells(103))
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertFalse(gate.eligible(1, 0, 0, 110))
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertTrue(gate.eligible(1, 0, 0, 111))
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertFalse(gate.eligible(1, 1, 0, 111))
        gate.start(111)
        gate.finish(120)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertFalse(gate.waiting_for_new_cells(120))

    # 到达目标却没增加新地图时先屏蔽；观察格增长到门槛后再解锁。
    def test_reached_goal_without_new_observations_is_suppressed(self):
        gate = GoalProgressGate(min_new_cells=8)
        gate.finish((2.0, 1.0), 100, 103)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertEqual(gate.blocked(110), [(2.0, 1.0)])
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertEqual(gate.blocked(111), [])
        gate.finish((3.0, 1.0), 111, 120)
        # 测试断言：实际值必须符合这里给出的预期。
        self.assertEqual(gate.blocked(120), [])


if __name__ == '__main__':
    unittest.main()
