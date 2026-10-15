#include "legbot_exploration/exploration_core.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_set>

namespace legbot_exploration {
namespace {
std::size_t mix(std::size_t seed, int value) {
  return seed ^ (std::hash<int>{}(value) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u));
}
constexpr std::array<Cell, 4> kFour{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
constexpr std::array<Cell, 8> kEight{{{1, 0}, {-1, 0}, {0, 1}, {0, -1},
                                      {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};
double distance(const XY &a, const XY &b) { return std::hypot(a[0] - b[0], a[1] - b[1]); }
}  // namespace

std::size_t CellHash::operator()(const Cell &c) const { return mix(mix(0, c.x), c.y); }
std::size_t VoxelHash::operator()(const Voxel &v) const {
  return mix(mix(mix(0, v.x), v.y), v.z);
}
Voxel VoxelMap::key(const Point &point) const {
  return {static_cast<int>(std::floor(point[0] / cfg_.resolution)),
          static_cast<int>(std::floor(point[1] / cfg_.resolution)),
          static_cast<int>(std::floor(point[2] / cfg_.resolution))};
}
Point VoxelMap::center(const Voxel &v) const {
  return {(v.x + 0.5) * cfg_.resolution, (v.y + 0.5) * cfg_.resolution,
          (v.z + 0.5) * cfg_.resolution};
}
XY VoxelMap::center(const Cell &c) const {
  return {(c.x + 0.5) * cfg_.resolution, (c.y + 0.5) * cfg_.resolution};
}
void VoxelMap::mark(const Voxel &v, bool hit) {
  auto it = voxels_.find(v);
  if (it == voxels_.end()) {
    if (voxels_.size() >= cfg_.max_voxels) return;
    it = voxels_.emplace(v, 0).first;
  }
  it->second = std::clamp(it->second + (hit ? 3 : -1), -3, 6);
}
std::size_t VoxelMap::integrate(const Point &origin, const std::vector<Point> &points,
                                double base_z) {
  std::size_t used = 0;
  for (const auto &point : points) {
    if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) continue;
    Point delta{point[0] - origin[0], point[1] - origin[1], point[2] - origin[2]};
    double length = std::hypot(delta[0], delta[1], delta[2]);
    if (length < cfg_.resolution || length > cfg_.max_ray_range) continue;
    int steps = std::max(1, static_cast<int>(std::ceil(length / (cfg_.resolution * 0.5))));
    std::optional<Voxel> last;
    for (int step = 0; step < steps; ++step) {
      double fraction = static_cast<double>(step) / steps;
      Voxel v = key({origin[0] + delta[0] * fraction,
                     origin[1] + delta[1] * fraction,
                     origin[2] + delta[2] * fraction});
      if (!last || !(v == *last)) {
        mark(v, false);
        last = v;
      }
    }
    bool obstacle = point[2] >= base_z + cfg_.obstacle_min_relative_z &&
                    point[2] <= base_z + cfg_.obstacle_max_relative_z;
    mark(key(point), obstacle);
    ++used;
  }
  if (used) ++version_;
  return used;
}
std::unordered_map<Cell, int, CellHash> VoxelMap::project(double base_z,
                                                            const XY &center_xy) const {
  std::unordered_map<Cell, int, CellHash> grid;
  double radius = cfg_.local_search_radius + cfg_.resolution * 2.0;
  double occupied_low = base_z + cfg_.obstacle_min_relative_z;
  double free_low = base_z + cfg_.free_min_relative_z;
  double high = base_z + cfg_.obstacle_max_relative_z;
  for (const auto &[voxel, evidence] : voxels_) {
    auto xyz = center(voxel);
    if (xyz[2] < free_low || xyz[2] > high || std::abs(xyz[0] - center_xy[0]) > radius ||
        std::abs(xyz[1] - center_xy[1]) > radius) continue;
    Cell cell{voxel.x, voxel.y};
    if (evidence > 0 && xyz[2] >= occupied_low) grid[cell] = 1;
    else if (evidence <= 0) grid.emplace(cell, 0);
  }
  return grid;
}

Selection select_frontier(const VoxelMap &map, const Point &robot, const Settings &cfg,
                          const std::vector<XY> &suppressed) {
  Selection result;
  auto robot_voxel = map.key(robot);
  Cell origin{robot_voxel.x, robot_voxel.y};
  auto grid = map.project(robot[2], {robot[0], robot[1]});
  int radius = static_cast<int>(std::ceil(cfg.local_search_radius / cfg.resolution));
  auto inside = [&](const Cell &c) {
    return std::abs(c.x - origin.x) <= radius && std::abs(c.y - origin.y) <= radius;
  };
  std::unordered_set<Cell, CellHash> blocked;
  int inflation = static_cast<int>(std::ceil(cfg.robot_radius / cfg.resolution));
  for (const auto &[cell, state] : grid) {
    if (state != 1) continue;
    for (int dx = -inflation; dx <= inflation; ++dx)
      for (int dy = -inflation; dy <= inflation; ++dy)
        if (std::hypot(dx, dy) * cfg.resolution <= cfg.robot_radius)
          blocked.insert({cell.x + dx, cell.y + dy});
  }
  std::unordered_set<Cell, CellHash> frontier;
  for (const auto &[cell, state] : grid) {
    if (state != 0) continue;
    for (const auto &d : kFour) {
      Cell next{cell.x + d.x, cell.y + d.y};
      if (inside(next) && !grid.count(next) && !blocked.count(next)) frontier.insert(next);
    }
  }
  std::vector<std::vector<Cell>> clusters;
  while (!frontier.empty()) {
    Cell seed = *frontier.begin();
    frontier.erase(seed);
    std::vector<Cell> cluster{seed};
    std::queue<Cell> queue;
    queue.push(seed);
    while (!queue.empty()) {
      Cell cell = queue.front();
      queue.pop();
      for (const auto &d : kEight) {
        Cell next{cell.x + d.x, cell.y + d.y};
        if (frontier.erase(next)) {
          cluster.push_back(next);
          queue.push(next);
        }
      }
    }
    if (static_cast<int>(cluster.size()) >= cfg.min_frontier_cells)
      clusters.push_back(std::move(cluster));
  }
  result.clusters = clusters.size();

  using QueueItem = std::pair<double, Cell>;
  auto greater = [](const QueueItem &a, const QueueItem &b) { return a.first > b.first; };
  std::priority_queue<QueueItem, std::vector<QueueItem>, decltype(greater)> queue(greater);
  std::unordered_map<Cell, double, CellHash> costs;
  costs[origin] = 0.0;
  queue.push({0.0, origin});
  while (!queue.empty()) {
    auto [cost, cell] = queue.top();
    queue.pop();
    if (cost > costs[cell]) continue;
    for (const auto &d : kEight) {
      Cell next{cell.x + d.x, cell.y + d.y};
      if (!inside(next) || blocked.count(next)) continue;
      if (d.x && d.y && (blocked.count({cell.x + d.x, cell.y}) ||
                         blocked.count({cell.x, cell.y + d.y}))) continue;
      double step = cfg.resolution * std::hypot(d.x, d.y);
      if (!grid.count(next)) step *= cfg.unknown_path_multiplier;
      double next_cost = cost + step;
      auto it = costs.find(next);
      if (it == costs.end() || next_cost < it->second) {
        costs[next] = next_cost;
        queue.push({next_cost, next});
      }
    }
  }

  double best_score = -std::numeric_limits<double>::infinity();
  for (auto &cluster : clusters) {
    double mx = 0.0, my = 0.0;
    for (const auto &c : cluster) { mx += c.x; my += c.y; }
    mx /= cluster.size(); my /= cluster.size();
    std::sort(cluster.begin(), cluster.end(), [&](const Cell &a, const Cell &b) {
      return std::hypot(a.x - mx, a.y - my) < std::hypot(b.x - mx, b.y - my);
    });
    for (const auto &cell : cluster) {
      auto found = costs.find(cell);
      if (found == costs.end()) continue;
      XY xy = map.center(cell);
      double range = distance(xy, {robot[0], robot[1]});
      if (range < cfg.min_goal_distance || range > cfg.max_goal_distance) continue;
      bool rejected = false;
      for (const auto &failed : suppressed)
        if (distance(xy, failed) < cfg.robot_radius * 2.0) { rejected = true; break; }
      if (rejected) continue;
      double score = cfg.frontier_gain_weight * std::log1p(cluster.size()) -
                     cfg.path_cost_weight * found->second;
      if (score > best_score) {
        result.goal = xy;
        result.score = score;
        best_score = score;
      }
      break;
    }
  }
  return result;
}

}  // namespace legbot_exploration
