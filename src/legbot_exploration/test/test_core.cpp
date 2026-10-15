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
  auto grid = map.project(robot[2], {robot[0], robot[1]});
  assert(grid.size() > 100);
  auto selection = select_frontier(map, robot, cfg, {});
  assert(selection.clusters > 0);
  assert(selection.goal);
  auto key = map.key({(*selection.goal)[0], (*selection.goal)[1], robot[2]});
  assert(!grid.count({key.x, key.y}));
  assert(std::hypot((*selection.goal)[0], (*selection.goal)[1]) >= cfg.min_goal_distance);
  VoxelMap obstacle_map(cfg);
  obstacle_map.integrate({0.0, 0.0, 0.4}, {{2.0, 0.0, 0.4}}, robot[2]);
  auto obstacle_grid = obstacle_map.project(robot[2], {0.0, 0.0});
  assert(obstacle_grid.at({8, 0}) == 1);
  assert(obstacle_grid.at({0, 0}) == 0);
  return 0;
}
