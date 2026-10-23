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
  if (!std::isfinite(base_z)) return 0;
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
                     origin[2] - base_z + delta[2] * fraction});
      if (!last || !(v == *last)) {
        mark(v, false);
        last = v;
      }
    }
    bool obstacle = point[2] >= base_z + cfg_.obstacle_min_relative_z &&
                    point[2] <= base_z + cfg_.obstacle_max_relative_z;
    mark(key({point[0], point[1], point[2] - base_z}), obstacle);
    ++used;
  }
  if (used) ++version_;
  return used;
}
std::unordered_map<Cell, int, CellHash> VoxelMap::project(const XY &center_xy) const {
  std::unordered_map<Cell, int, CellHash> grid;
  double radius = cfg_.local_search_radius + cfg_.resolution * 2.0;
  double occupied_low = cfg_.obstacle_min_relative_z;
  double free_low = cfg_.free_min_relative_z;
  double high = cfg_.obstacle_max_relative_z;
  for (const auto &[voxel, evidence] : voxels_) {
    auto xyz = center(voxel);
    if (xyz[2] < free_low || xyz[2] > high) continue;
    if (cfg_.global_search) {
      if (xyz[0] < cfg_.exploration_min_x || xyz[0] > cfg_.exploration_max_x ||
          xyz[1] < cfg_.exploration_min_y || xyz[1] > cfg_.exploration_max_y) continue;
    } else if (std::abs(xyz[0] - center_xy[0]) > radius ||
               std::abs(xyz[1] - center_xy[1]) > radius) continue;
    Cell cell{voxel.x, voxel.y};
    if (evidence > 0 && xyz[2] >= occupied_low) grid[cell] = 1;
    else if (evidence <= 0) grid.emplace(cell, 0);
  }
  return grid;
}

void VoxelMap::set_exploration_bounds(double min_x, double max_x,
                                      double min_y, double max_y) {
  cfg_.exploration_min_x = min_x;
  cfg_.exploration_max_x = max_x;
  cfg_.exploration_min_y = min_y;
  cfg_.exploration_max_y = max_y;
}

bool RecoveryGate::waiting_for_new_cells(std::size_t observed_cells) const {
  return blocked_at_ && observed_cells - std::min(observed_cells, *blocked_at_) <
                            min_new_cells_;
}

bool RecoveryGate::eligible(const Selection &selection, std::size_t observed_cells) const {
  return selection.clusters > 0 && selection.reachable_frontier_cells == 0 &&
         selection.candidate_goals == 0 && !waiting_for_new_cells(observed_cells);
}

void RecoveryGate::start(std::size_t observed_cells) { started_at_ = observed_cells; }

void RecoveryGate::finish(std::size_t observed_cells) {
  if (!started_at_) return;
  if (observed_cells - std::min(observed_cells, *started_at_) < min_new_cells_)
    blocked_at_ = observed_cells;
  else
    blocked_at_.reset();
  started_at_.reset();
}

void GoalProgressGate::finish(const XY &goal, std::size_t before, std::size_t after) {
  if (after - std::min(after, before) >= min_new_cells_) return;
  entries_.push_back({goal, after + min_new_cells_});
}

std::vector<XY> GoalProgressGate::blocked(std::size_t observed_cells) {
  entries_.erase(std::remove_if(entries_.begin(), entries_.end(),
      [observed_cells](const Entry &entry) { return observed_cells >= entry.unlock_at; }),
      entries_.end());
  std::vector<XY> goals;
  goals.reserve(entries_.size());
  for (const auto &entry : entries_) goals.push_back(entry.goal);
  return goals;
}

Selection select_frontier(const VoxelMap &map, const Point &robot, const Settings &cfg,
                          const std::vector<XY> &suppressed,
                          const std::vector<XY> &external_obstacles) {
  Selection result;
  auto robot_voxel = map.key(robot);
  Cell origin{robot_voxel.x, robot_voxel.y};
  auto grid = map.project({robot[0], robot[1]});
  int radius = static_cast<int>(std::ceil(cfg.local_search_radius / cfg.resolution));
  auto inside = [&](const Cell &c) {
    if (cfg.global_search) {
      XY xy = map.center(c);
      return xy[0] >= cfg.exploration_min_x && xy[0] <= cfg.exploration_max_x &&
             xy[1] >= cfg.exploration_min_y && xy[1] <= cfg.exploration_max_y;
    }
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
  for (const auto &obstacle : external_obstacles) {
    if (!std::isfinite(obstacle[0]) || !std::isfinite(obstacle[1])) continue;
    Cell center{static_cast<int>(std::floor(obstacle[0] / cfg.resolution)),
                static_cast<int>(std::floor(obstacle[1] / cfg.resolution))};
    for (int dx = -inflation; dx <= inflation; ++dx)
      for (int dy = -inflation; dy <= inflation; ++dy)
        if (std::hypot(dx, dy) * cfg.resolution <= cfg.robot_radius)
          blocked.insert({center.x + dx, center.y + dy});
  }
  std::unordered_set<Cell, CellHash> frontier;
  for (const auto &[cell, state] : grid) {
    if (state != 0) continue;
    for (const auto &d : kFour) {
      Cell next{cell.x + d.x, cell.y + d.y};
      if (inside(next) && !grid.count(next) && !blocked.count(next)) frontier.insert(next);
    }
  }
  result.frontier_cells = frontier.size();
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
  if (clusters.empty() || !inside(origin)) return result;

  for (const auto &d : kEight) {
    Cell next{origin.x + d.x, origin.y + d.y};
    if (inside(next) && !blocked.count(next)) ++result.open_start_neighbors;
  }

  using QueueItem = std::pair<double, Cell>;
  auto greater = [](const QueueItem &a, const QueueItem &b) { return a.first > b.first; };
  std::priority_queue<QueueItem, std::vector<QueueItem>, decltype(greater)> queue(greater);
  std::unordered_map<Cell, double, CellHash> costs;
  std::unordered_map<Cell, Cell, CellHash> parents;
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
        parents[next] = cell;
        queue.push({next_cost, next});
      }
    }
  }

  double best_score = -std::numeric_limits<double>::infinity();
  for (auto &cluster : clusters) {
    for (const auto &cell : cluster)
      if (costs.count(cell)) ++result.reachable_frontier_cells;
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
      if (range < cfg.min_goal_distance) continue;
      if (range > cfg.max_goal_distance) {
        // The frontier is beyond the local planner's observed horizon. Follow
        // its lowest-cost path back to an observed free staging cell nearby.
        std::optional<XY> stage_goal;
        Cell stage = cell;
        while (!(stage == origin)) {
          auto parent = parents.find(stage);
          if (parent == parents.end()) break;
          auto state = grid.find(stage);
          if (state != grid.end() && state->second == 0 && !blocked.count(stage)) {
            XY candidate = map.center(stage);
            double stage_range = distance(candidate, {robot[0], robot[1]});
            if (stage_range >= cfg.min_goal_distance &&
                stage_range <= cfg.max_goal_distance) {
              stage_goal = candidate;
              break;
            }
          }
          stage = parent->second;
        }
        if (!stage_goal) continue;
        xy = *stage_goal;
      }
      ++result.candidate_goals;
      bool rejected = false;
      for (const auto &failed : suppressed)
        if (distance(xy, failed) < cfg.robot_radius * 2.0) { rejected = true; break; }
      if (rejected) { ++result.suppressed_goals; continue; }
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
