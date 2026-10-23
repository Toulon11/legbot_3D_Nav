#include "legbot_exploration/exploration_core.hpp"

#include <cassert>
#include <cmath>
#include <vector>

int main() {
  using namespace legbot_exploration;
  Settings cfg;
  VoxelMap map(cfg);
  Point robot{0.0, 0.0, 0.3};
  assert(select_frontier(map, robot, cfg, {}).clusters == 0);
  std::vector<Point> points;
  for (int i = 0; i <= 100; ++i) {
    double angle = i * std::acos(-1.0) / 100.0 - std::acos(-1.0) / 2.0;
    points.push_back({4.0 * std::cos(angle), 4.0 * std::sin(angle), -0.2});
  }
  assert(map.integrate({0.0, 0.0, 0.4}, points, robot[2]) == points.size());
  auto grid = map.project({robot[0], robot[1]});
  assert(grid.size() > 100);
  auto selection = select_frontier(map, robot, cfg, {});
  assert(selection.clusters > 0);
  assert(selection.goal);
  auto key = map.key({(*selection.goal)[0], (*selection.goal)[1], robot[2]});
  assert(!grid.count({key.x, key.y}));
  assert(std::hypot((*selection.goal)[0], (*selection.goal)[1]) >= cfg.min_goal_distance);
  Settings staged_cfg = cfg;
  staged_cfg.max_goal_distance = 2.0;
  VoxelMap stage_map(cfg);
  std::vector<Point> ring;
  for (int i = 0; i < 360; ++i) {
    double angle = i * 2.0 * std::acos(-1.0) / 360.0;
    ring.push_back({4.0 * std::cos(angle), 4.0 * std::sin(angle), -0.2});
  }
  stage_map.integrate({0.0, 0.0, 0.4}, ring, robot[2]);
  auto stage_grid = stage_map.project({robot[0], robot[1]});
  auto staged = select_frontier(stage_map, robot, staged_cfg, {});
  assert(staged.goal);
  auto staged_key = stage_map.key({(*staged.goal)[0], (*staged.goal)[1], robot[2]});
  assert(stage_grid.at({staged_key.x, staged_key.y}) == 0);
  assert(std::hypot((*staged.goal)[0], (*staged.goal)[1]) <= staged_cfg.max_goal_distance);
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
  auto blocked = select_frontier(map, robot, cfg, {}, {*selection.goal});
  assert(!blocked.goal || std::hypot((*blocked.goal)[0] - (*selection.goal)[0],
                                     (*blocked.goal)[1] - (*selection.goal)[1]) > 0.0);
  auto before_drift = map.size();
  for (auto &point : points) point[2] -= 3.0;
  assert(map.integrate({0.0, 0.0, -2.6}, points, -2.7) == points.size());
  assert(map.size() == before_drift);
  assert(map.project({0.0, 0.0}).size() == grid.size());
  assert(select_frontier(map, {0.0, 0.0, -2.7}, cfg, {}).goal);
  Settings residual_cfg = cfg;
  residual_cfg.min_frontier_cells = 10000;
  assert(!select_frontier(map, robot, residual_cfg, {}).goal);
  residual_cfg.min_frontier_cells = 1;
  assert(select_frontier(map, robot, residual_cfg, {}).goal);
  VoxelMap obstacle_map(cfg);
  obstacle_map.integrate({0.0, 0.0, 0.4}, {{2.0, 0.0, 0.4}}, robot[2]);
  auto obstacle_grid = obstacle_map.project({0.0, 0.0});
  assert(obstacle_grid.at({8, 0}) == 1);
  assert(obstacle_grid.at({0, 0}) == 0);

  // Three isolated frontier cells are not a cluster: never backtrack just
  // because the raw frontier count is nonzero.
  RecoveryGate recovery(8);
  Selection residual;
  residual.frontier_cells = 3;
  assert(!recovery.eligible(residual, 100));
  Selection unreachable;
  unreachable.frontier_cells = 5;
  unreachable.clusters = 1;
  assert(recovery.eligible(unreachable, 100));
  recovery.start(100);
  recovery.finish(103);
  assert(recovery.waiting_for_new_cells(103));
  assert(!recovery.eligible(unreachable, 110));
  assert(recovery.eligible(unreachable, 111));
  unreachable.reachable_frontier_cells = 1;
  assert(!recovery.eligible(unreachable, 111));
  unreachable.reachable_frontier_cells = 0;
  recovery.start(111);
  recovery.finish(120);
  assert(!recovery.waiting_for_new_cells(120));
  GoalProgressGate goal_progress(8);
  goal_progress.finish({2.0, 1.0}, 100, 103);
  assert(goal_progress.blocked(110).size() == 1);
  assert(goal_progress.blocked(111).empty());
  goal_progress.finish({3.0, 1.0}, 111, 120);
  assert(goal_progress.blocked(120).empty());
  return 0;
}
