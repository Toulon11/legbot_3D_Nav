// 包含 "legbot_exploration/single_floor_exploration.hpp" 提供的 C++ 声明。
#include "legbot_exploration/single_floor_exploration.hpp"

// 包含 <cassert> 提供的 C++ 声明。
#include <cassert>
// 包含 <cmath> 提供的 C++ 声明。
#include <cmath>
// 包含 <vector> 提供的 C++ 声明。
#include <vector>

// C++ 核心单元测试入口：下面使用 assert 验证地图与选点，不启动 ROS。
int main() {
  // 使用本包命名空间，以便直接写 Settings、VoxelMap 等类名。
  using namespace legbot_exploration;
  Settings audit_cfg;
  audit_cfg.global_search = true;
  audit_cfg.resolution = 1.0;
  audit_cfg.robot_radius = 0.0;
  audit_cfg.exploration_min_x = 0.0;
  audit_cfg.exploration_max_x = 1.0;
  audit_cfg.exploration_min_y = 0.0;
  audit_cfg.exploration_max_y = 1.0;
  auto open_audit = assess_global_frontiers({{{0, 0}, 0}, {{1, 0}, 0}},
                                            audit_cfg, {0.5, 0.5});
  assert(open_audit.valid && open_audit.cells.count({2, 0}) > 0);
  auto closed_audit = assess_global_frontiers(
      {{{0, 0}, 0}, {{1, 0}, 1}, {{-1, 0}, 1},
       {{0, 1}, 1}, {{0, -1}, 1}, {{10, 10}, 0}}, audit_cfg, {0.5, 0.5});
  assert(closed_audit.valid && closed_audit.frontier_cells == 0 &&
         closed_audit.reachable_free_cells == 1);
  assert(!assess_global_frontiers({{{0, 0}, 0}}, audit_cfg, {5.5, 5.5}).valid);
  // 场景一：空地图还没有空闲和未知边界，不会选出前沿簇。
  Settings cfg;
  VoxelMap map(cfg);
  Point robot{0.0, 0.0, 0.3};
  assert(select_frontier(map, robot, cfg, {}).clusters == 0);
  // 场景二：构造半圆形的地面点；每条射线走过的格子应成为已观察空闲格。
  std::vector<Point> points;
  // 依次处理集合中的每一项。
  for (int i = 0; i <= 100; ++i) {
    double angle = i * std::acos(-1.0) / 100.0 - std::acos(-1.0) / 2.0;
    // 把本次结果放入容器，供下一轮处理。
    points.push_back({4.0 * std::cos(angle), 4.0 * std::sin(angle), -0.2});
  }
  assert(map.integrate({0.0, 0.0, 0.4}, points, robot[2]) == points.size());
  // 将右侧计算值存入 grid（二维投影地图）。
  auto grid = map.project({robot[0], robot[1]});
  assert(grid.size() > 100);
  // 将右侧计算值存入 selection（前沿搜索结果）。
  auto selection = select_frontier(map, robot, cfg, {});
  assert(selection.clusters > 0);
  assert(selection.goal);
  auto key = map.key({(*selection.goal)[0], (*selection.goal)[1], robot[2]});
  assert(!grid.count({key.x, key.y}));
  assert(std::hypot((*selection.goal)[0], (*selection.goal)[1]) >= cfg.min_goal_distance);
  // 场景三：完整地面环的真正前沿在 4 米附近，2 米上限应选近处空闲分段落点。
  Settings staged_cfg = cfg;
  staged_cfg.max_goal_distance = 2.0;
  VoxelMap stage_map(cfg);
  std::vector<Point> ring;
  // 依次处理集合中的每一项。
  for (int i = 0; i < 360; ++i) {
    double angle = i * 2.0 * std::acos(-1.0) / 360.0;
    // 把本次结果放入容器，供下一轮处理。
    ring.push_back({4.0 * std::cos(angle), 4.0 * std::sin(angle), -0.2});
  }
  stage_map.integrate({0.0, 0.0, 0.4}, ring, robot[2]);
  auto stage_grid = stage_map.project({robot[0], robot[1]});
  auto staged = select_frontier(stage_map, robot, staged_cfg, {});
  assert(staged.goal);
  auto staged_key = stage_map.key({(*staged.goal)[0], (*staged.goal)[1], robot[2]});
  assert(stage_grid.at({staged_key.x, staged_key.y}) == 0);
  assert(std::hypot((*staged.goal)[0], (*staged.goal)[1]) <= staged_cfg.max_goal_distance);
  // 起点邻格只统计原始空闲证据，不因障碍膨胀而停止前沿搜索。
  Settings inflated_cfg = cfg;
  inflated_cfg.robot_radius = 0.50;
  inflated_cfg.planner_occupancy_radius = 0.35;
  auto adjusted = select_frontier(stage_map, robot, inflated_cfg, {}, {{0.625, 0.125}});
  inflated_cfg.planner_occupancy_radius = 0.0;
  auto doubled = select_frontier(stage_map, robot, inflated_cfg, {}, {{0.625, 0.125}});
  assert(adjusted.open_start_neighbors == doubled.open_start_neighbors);
  // 场景四：1 米局部窗口没有前沿；全局矩形搜索则应找到同一片远处前沿。
  Settings local_cfg = cfg;
  local_cfg.local_search_radius = 1.0;
  local_cfg.max_goal_distance = 2.0;
  VoxelMap global_map(local_cfg);
  global_map.integrate({0.0, 0.0, 0.4}, ring, robot[2]);
  assert(select_frontier(global_map, robot, local_cfg, {}).frontier_cells == 0);
  Settings global_cfg = local_cfg;
  global_cfg.global_search = true;
  global_cfg.exploration_min_x = -5.0;
  global_cfg.exploration_max_x = 5.0;
  global_cfg.exploration_min_y = -5.0;
  global_cfg.exploration_max_y = 5.0;
  VoxelMap bounded_map(global_cfg);
  bounded_map.integrate({0.0, 0.0, 0.4}, ring, robot[2]);
  auto global_selection = select_frontier(bounded_map, robot, global_cfg, {});
  assert(global_selection.frontier_cells > 0);
  assert(global_selection.goal);
  assert(std::hypot((*global_selection.goal)[0], (*global_selection.goal)[1]) <=
         global_cfg.max_goal_distance);
  Settings narrow_cfg = global_cfg;
  narrow_cfg.exploration_min_x = 0.0;
  narrow_cfg.exploration_max_x = 0.25;
  narrow_cfg.exploration_min_y = 0.0;
  narrow_cfg.exploration_max_y = 0.25;
  VoxelMap narrow_map(narrow_cfg);
  narrow_map.integrate({0.0, 0.0, 0.4}, ring, robot[2]);
  assert(select_frontier(narrow_map, robot, narrow_cfg, {}).frontier_cells == 0);
  auto narrow_audit = assess_global_frontiers(narrow_map.project_all(), narrow_cfg,
                                               {robot[0], robot[1]});
  assert(narrow_audit.valid && narrow_audit.frontier_cells > 0);
  auto outside_selection = select_frontier(narrow_map, robot, narrow_cfg, {}, {}, true,
                                          &narrow_audit.cells);
  assert(outside_selection.goal);
  // 场景五：额外规划器障碍盖住原目标时，不应继续选择该目标。
  // 将右侧计算值存入 blocked（避障膨胀后的禁行格）。
  auto blocked = select_frontier(map, robot, cfg, {}, {*selection.goal});
  assert(!blocked.goal || std::hypot((*blocked.goal)[0] - (*selection.goal)[0],
                                     (*blocked.goal)[1] - (*selection.goal)[1]) > 0.0);
  // 场景六：整体高度偏移 3 米，按机体相对高度保存的投影仍与原地图相同。
  auto before_drift = map.size();
  // 依次处理一帧点云点集。
  for (auto &point : points) point[2] -= 3.0;
  assert(map.integrate({0.0, 0.0, -2.6}, points, -2.7) == points.size());
  assert(map.size() == before_drift);
  assert(map.project({0.0, 0.0}).size() == grid.size());
  assert(select_frontier(map, {0.0, 0.0, -2.7}, cfg, {}).goal);
  // 场景七：提高最小簇规模会过滤目标，降到一格后目标重新出现。
  Settings residual_cfg = cfg;
  residual_cfg.min_frontier_cells = 10000;
  assert(!select_frontier(map, robot, residual_cfg, {}).goal);
  residual_cfg.min_frontier_cells = 1;
  assert(select_frontier(map, robot, residual_cfg, {}).goal);
  // 场景八：处在障碍高度范围内的端点是障碍；射线途经格仍为空闲。
  VoxelMap obstacle_map(cfg);
  obstacle_map.integrate({0.0, 0.0, 0.4}, {{2.0, 0.0, 0.4}}, robot[2]);
  auto obstacle_grid = obstacle_map.project({0.0, 0.0});
  assert(obstacle_grid.at({8, 0}) == 1);
  assert(obstacle_grid.at({0, 0}) == 0);

  // One dense scan is still one independent free observation per column.
  VoxelMap evidence_map(cfg);
  const std::vector<Point> repeated_hits(20, Point{2.0, 0.0, -0.2});
  evidence_map.integrate({0.0, 0.0, 0.4}, repeated_hits, robot[2]);
  assert(evidence_map.free_observation_frames().at({0, 0}) == 1);
  evidence_map.integrate({0.0, 0.0, 0.4}, repeated_hits, robot[2]);
  assert(evidence_map.free_observation_frames().at({0, 0}) == 2);
  evidence_map.integrate({0.0, 0.0, 0.4}, repeated_hits, robot[2]);
  evidence_map.integrate({0.0, 0.0, 0.4}, repeated_hits, robot[2]);
  assert(evidence_map.free_observation_frames().at({0, 0}) == 3);
  evidence_map.integrate({0.0, 0.0, 0.4}, {{1.0, 0.0, 0.4}}, robot[2]);
  assert(evidence_map.project_all().at({4, 0}) == 1);
  assert(evidence_map.free_observation_frames().count({4, 0}) == 0);

  // 动态前沿索引只重算变化格及其四邻格；结果须与独立全图搜索一致。
  std::unordered_map<Cell, int, CellHash> changing{{{0, 0}, 0}};
  FrontierIndex frontier_index;
  frontier_index.rebuild(changing);
  assert(frontier_index.cells().size() == 4);
  assert(frontier_index.cells().count({1, 0}) == 1);
  changing[{1, 0}] = 0;
  frontier_index.update(changing, {{1, 0}});
  assert(frontier_index.cells().count({1, 0}) == 0);
  assert(frontier_index.cells().count({2, 0}) == 1);
  auto indexed_audit = assess_global_frontiers(changing, audit_cfg, {0.5, 0.5},
                                               &frontier_index.cells());
  auto independent_audit = assess_global_frontiers(changing, audit_cfg, {0.5, 0.5});
  assert(indexed_audit.cells == independent_audit.cells);
  changing[{1, 0}] = 1;
  frontier_index.update(changing, {{1, 0}});
  assert(frontier_index.cells().count({2, 0}) == 0);
  assert(frontier_index.cells().count({1, 0}) == 0);
  assert(assess_global_frontiers(changing, audit_cfg, {0.5, 0.5},
                                &frontier_index.cells()).cells ==
         assess_global_frontiers(changing, audit_cfg, {0.5, 0.5}).cells);

  // 同一障碍体素被后续无命中射线穿过时，二维状态与变化格均需更新。
  Settings changing_cfg = audit_cfg;
  changing_cfg.max_ray_range = 10.0;
  VoxelMap changing_map(changing_cfg);
  changing_map.integrate({0.5, 0.5, 0.4}, {{2.5, 0.5, 0.4}}, 0.4);
  assert(changing_map.project_all().at({2, 0}) == 1);
  changing_map.take_dirty_cells();
  for (int repeat = 0; repeat < 6; ++repeat)
    changing_map.integrate({0.5, 0.5, 0.4}, {{4.5, 0.5, 0.4}}, 0.4);
  assert(changing_map.project_all().at({2, 0}) == 0);
  assert(changing_map.take_dirty_cells().count({2, 0}) == 1);

  GoalProgressGate goal_progress(8);
  goal_progress.finish({2.0, 1.0}, 100, 103);
  assert(goal_progress.blocked(110).size() == 1);
  assert(goal_progress.blocked(111).empty());
  goal_progress.finish({3.0, 1.0}, 111, 120);
  assert(goal_progress.blocked(120).empty());
  Settings coverage_cfg;
  coverage_cfg.global_search = true;
  coverage_cfg.resolution = 1.0;
  coverage_cfg.exploration_min_x = 0.0;
  coverage_cfg.exploration_max_x = 3.0;
  coverage_cfg.exploration_min_y = 0.0;
  coverage_cfg.exploration_max_y = 3.0;
  std::unordered_map<Cell, int, CellHash> enclosed;
  for (int x = 0; x < 3; ++x) for (int y = 0; y < 3; ++y)
    if (x != 1 || y != 1) enclosed[{x, y}] = 1;
  auto coverage = assess_coverage(enclosed, coverage_cfg);
  assert(coverage.valid && coverage.total_cells == 9 && coverage.unknown_cells == 1);
  assert(!select_observation_goal(VoxelMap(coverage_cfg), {0.5, 0.5, 0.3},
                                  coverage_cfg, enclosed, {}));
  enclosed[{1, 1}] = 1;
  assert(assess_coverage(enclosed, coverage_cfg).unknown_cells == 0);
  assert(!assess_coverage(enclosed, cfg).valid);

  // 障碍环内部仍是原始未知，但机器人无法到达，不阻止外部区域完成。
  Settings reach_cfg = coverage_cfg;
  reach_cfg.exploration_max_x = 5.0;
  reach_cfg.exploration_max_y = 5.0;
  reach_cfg.robot_radius = 0.0;
  std::unordered_map<Cell, int, CellHash> ring_grid;
  for (int x = 0; x < 5; ++x) for (int y = 0; y < 5; ++y)
    if (x != 2 || y != 2) ring_grid[{x, y}] = 0;
  for (const Cell &wall : std::vector<Cell>{{1,1},{1,2},{1,3},{2,1},{2,3},{3,1},{3,2},{3,3}})
    ring_grid[wall] = 1;
  auto outside = assess_reachable_coverage(ring_grid, reach_cfg, {0.5, 0.5});
  assert(outside.reachability_valid && outside.unknown_cells == 1);
  assert(outside.reachable_unknown_cells == 0 && outside.excluded_unknown_cells == 1);
  // 墙上保留一格未知通道时，内部未知仍可能连通，不能提前宣布完成。
  ring_grid.erase({2, 1});
  auto passage = assess_reachable_coverage(ring_grid, reach_cfg, {0.5, 0.5});
  assert(passage.reachable_unknown_cells >= 2 && passage.excluded_unknown_cells == 0);
  // 起点不在验收区时不具备完成判定依据。
  assert(!assess_reachable_coverage(ring_grid, reach_cfg, {-1.0, 0.5}).reachability_valid);
  reach_cfg.exploration_max_x = reach_cfg.exploration_max_y = 2.0;
  std::unordered_map<Cell, int, CellHash> blocked_corner{{{0,0},0},{{1,0},1},{{0,1},1}};
  auto corner = assess_reachable_coverage(blocked_corner, reach_cfg, {0.5,0.5});
  assert(corner.reachability_valid && corner.reachable_unknown_cells == 0 &&
         corner.excluded_unknown_cells == 1);

  coverage_cfg.robot_radius = 0.0;
  coverage_cfg.max_ray_range = 5.0;
  coverage_cfg.max_goal_distance = 4.0;
  coverage_cfg.exploration_max_x = 6.0;
  coverage_cfg.exploration_max_y = 2.0;
  std::unordered_map<Cell, int, CellHash> open;
  for (int x = 0; x < 5; ++x) for (int y = 0; y < 2; ++y) open[{x, y}] = 0;
  VoxelMap fallback_map(coverage_cfg);
  auto observation = select_observation_goal(fallback_map, {0.5, 0.5, 0.3},
                                              coverage_cfg, open, {});
  assert(observation);
  auto observation_voxel = fallback_map.key({(*observation)[0], (*observation)[1], 0.3});
  assert(open.at({observation_voxel.x, observation_voxel.y}) == 0);
  // 把此函数算出的值交还给调用者。
  return 0;
}
