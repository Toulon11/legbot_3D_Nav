// 包含 "legbot_exploration/single_floor_exploration.hpp" 提供的 C++ 声明。
#include "legbot_exploration/single_floor_exploration.hpp"

// 包含 <algorithm> 提供的 C++ 声明。
#include <algorithm>
// 包含 <cmath> 提供的 C++ 声明。
#include <cmath>
// 包含 <functional> 提供的 C++ 声明。
#include <functional>
// 包含 <limits> 提供的 C++ 声明。
#include <limits>
// 包含 <queue> 提供的 C++ 声明。
#include <queue>
// 包含 <tuple> 提供的 C++ 声明。
#include <tuple>
// 包含 <unordered_set> 提供的 C++ 声明。
#include <unordered_set>

// 进入本包的 C++ 命名空间，避免与别的包重名。
namespace legbot_exploration {
// 进入本包的 C++ 命名空间，避免与别的包重名。
namespace {
// 把两个整数逐步混合为哈希值，减少不同格子碰撞。
std::size_t mix(std::size_t seed, int value) {
  // 把此函数算出的值交还给调用者。
  return seed ^ (std::hash<int>{}(value) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u));
}
// 定义编译时确定的常量，后续搜索会重复使用。
constexpr std::array<Cell, 4> kFour{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
// 定义编译时确定的常量，后续搜索会重复使用。
constexpr std::array<Cell, 8> kEight{{{1, 0}, {-1, 0}, {0, 1}, {0, -1},
                                      {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};
// 用勾股定理计算两个二维点的直线距离。
double distance(const XY &a, const XY &b) { return std::hypot(a[0] - b[0], a[1] - b[1]); }
}  // namespace

// 根据整数格号计算哈希值，供 unordered_map 快速查找。
std::size_t CellHash::operator()(const Cell &c) const { return mix(mix(0, c.x), c.y); }
// 根据整数格号计算哈希值，供 unordered_map 快速查找。
std::size_t VoxelHash::operator()(const Voxel &v) const {
  // 把此函数算出的值交还给调用者。
  return mix(mix(mix(0, v.x), v.y), v.z);
}
// 将点的米制坐标除以网格分辨率并向下取整，得到三维格号。
Voxel VoxelMap::key(const Point &point) const {
  // 返回每个坐标向下取整后的体素编号。
  return {static_cast<int>(std::floor(point[0] / cfg_.resolution)),
          static_cast<int>(std::floor(point[1] / cfg_.resolution)),
          static_cast<int>(std::floor(point[2] / cfg_.resolution))};
}
// 用整数格号加半格长度，求这个格子的实际中心坐标。
Point VoxelMap::center(const Voxel &v) const {
  // 返回按分辨率换算得到的网格中心坐标。
  return {(v.x + 0.5) * cfg_.resolution, (v.y + 0.5) * cfg_.resolution,
          (v.z + 0.5) * cfg_.resolution};
}
// 用整数格号加半格长度，求这个格子的实际中心坐标。
XY VoxelMap::center(const Cell &c) const {
  // 返回按分辨率换算得到的网格中心坐标。
  return {(c.x + 0.5) * cfg_.resolution, (c.y + 0.5) * cfg_.resolution};
}
// 更新射线扫到的体素：命中点加三，射线途中减一。
void VoxelMap::mark(const Voxel &v, bool hit) {
  // 将右侧计算值存入 it（哈希表当前位置）。
  auto it = voxels_.find(v);
  const bool existed = it != voxels_.end();
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (!existed) {
    // 地图已经达到体素数量上限，不能继续插入。
    if (voxels_.size() >= cfg_.max_voxels) return;
    // 将右侧计算值存入 it（哈希表当前位置）。
    it = voxels_.emplace(v, 0).first;
  }
  const double height = (v.z + 0.5) * cfg_.resolution;
  const auto contribution = [&](int evidence) {
    if (height < cfg_.free_min_relative_z ||
        height > cfg_.obstacle_max_relative_z) return 0;
    if (evidence > 0 && height >= cfg_.obstacle_min_relative_z) return 2;
    return evidence <= 0 ? 1 : 0;
  };
  const int previous = existed ? contribution(it->second) : 0;
  // 将证据值限制在 -3 到 6 之间，避免无限累计。
  it->second = std::clamp(it->second + (hit ? 3 : -1), -3, 6);
  const int current = contribution(it->second);
  if (previous == current) return;

  const Cell cell{v.x, v.y};
  const auto prior = projected_all_.find(cell);
  const int prior_state = prior == projected_all_.end() ? -1 : prior->second;
  auto &counts = columns_[cell];
  if (previous == 1) --counts.free;
  if (previous == 2) --counts.occupied;
  if (current == 1) ++counts.free;
  if (current == 2) ++counts.occupied;
  const int state = counts.occupied > 0 ? 1 : counts.free > 0 ? 0 : -1;
  if (state == -1) {
    projected_all_.erase(cell);
    columns_.erase(cell);
  } else {
    projected_all_[cell] = state;
  }
  if (state != 0) free_observation_frames_.erase(cell);
  if (state != prior_state) dirty_cells_.insert(cell);
}
// 遍历当前帧雷达点，沿每条射线累计空闲和障碍证据。
std::size_t VoxelMap::integrate(const Point &origin, const std::vector<Point> &points,
                                double base_z) {
  // 过滤包含无穷大或 NaN 的无效坐标。
  if (!std::isfinite(base_z)) return 0;
  // 将右侧计算值存入 used（有效雷达点数）。
  std::size_t used = 0;
  std::unordered_set<Cell, CellHash> frame_free_columns;
  const auto remember_free = [&](const Voxel &voxel) {
    const double height = (voxel.z + 0.5) * cfg_.resolution;
    if (height >= cfg_.free_min_relative_z &&
        height <= cfg_.obstacle_max_relative_z)
      frame_free_columns.insert({voxel.x, voxel.y});
  };
  // 依次处理一帧点云点集。
  for (const auto &point : points) {
    // 过滤包含无穷大或 NaN 的无效坐标。
    if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) continue;
    Point delta{point[0] - origin[0], point[1] - origin[1], point[2] - origin[2]};
    double length = std::hypot(delta[0], delta[1], delta[2]);
    // 只有表达式为真，才执行下面缩进或花括号内的代码。
    if (length < cfg_.resolution || length > cfg_.max_ray_range) continue;
    int steps = std::max(1, static_cast<int>(std::ceil(length / (cfg_.resolution * 0.5))));
    // 声明可为空的结果，尚未找到时保持空值。
    std::optional<Voxel> last;
    // 依次处理一条射线的离散采样次数。
    for (int step = 0; step < steps; ++step) {
      double fraction = static_cast<double>(step) / steps;
      Voxel v = key({origin[0] + delta[0] * fraction,
                     origin[1] + delta[1] * fraction,
                     origin[2] - base_z + delta[2] * fraction});
      // 只有表达式为真，才执行下面缩进或花括号内的代码。
      if (!last || !(v == *last)) {
        mark(v, false);
        remember_free(v);
        // 将右侧计算值存入 last（上一个射线体素）。
        last = v;
      }
    }
    bool obstacle = point[2] >= base_z + cfg_.obstacle_min_relative_z &&
                    point[2] <= base_z + cfg_.obstacle_max_relative_z;
    const Voxel endpoint = key({point[0], point[1], point[2] - base_z});
    mark(endpoint, obstacle);
    if (!obstacle) remember_free(endpoint);
    ++used;
  }
  const auto cap = static_cast<std::uint8_t>(
      std::clamp(cfg_.free_observation_saturation, 1, 255));
  for (const auto &cell : frame_free_columns) {
    const auto projected = projected_all_.find(cell);
    if (projected == projected_all_.end() || projected->second != 0) continue;
    auto &frames = free_observation_frames_[cell];
    if (frames < cap) ++frames;
  }
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (used) ++version_;
  // 把此函数算出的值交还给调用者。
  return used;
}
// 按高度筛选三维体素，并把相同 XY 的信息压到二维地图。
std::unordered_map<Cell, int, CellHash> VoxelMap::project(const XY &center_xy) const {
  return project_impl(center_xy);
}

std::unordered_map<Cell, int, CellHash> VoxelMap::project_all() const {
  return projected_all_;
}

std::unordered_set<Cell, CellHash> VoxelMap::take_dirty_cells() {
  auto result = std::move(dirty_cells_);
  dirty_cells_.clear();
  return result;
}

std::unordered_map<Cell, int, CellHash> VoxelMap::project_impl(
    const XY &center_xy) const {
  std::unordered_map<Cell, int, CellHash> grid;
  double radius = cfg_.local_search_radius + cfg_.resolution * 2.0;
  // 三维证据改变时已同步维护全局二维投影；这里只裁剪显示和局部选点范围。
  for (const auto &[cell, state] : projected_all_) {
    auto xy = center(cell);
    // 只有表达式为真，才执行下面缩进或花括号内的代码。
    if (cfg_.global_search) {
      // 只有表达式为真，才执行下面缩进或花括号内的代码。
      if (xy[0] < cfg_.exploration_min_x || xy[0] > cfg_.exploration_max_x ||
          xy[1] < cfg_.exploration_min_y || xy[1] > cfg_.exploration_max_y) continue;
    } else if (!cfg_.global_search &&
               (std::abs(xy[0] - center_xy[0]) > radius ||
                std::abs(xy[1] - center_xy[1]) > radius)) continue;
    grid.emplace(cell, state);
  }
  // 把此函数算出的值交还给调用者。
  return grid;
}

void VoxelMap::set_exploration_bounds(double min_x, double max_x,
                                      double min_y, double max_y) {
  cfg_.exploration_min_x = min_x;
  cfg_.exploration_max_x = max_x;
  cfg_.exploration_min_y = min_y;
  cfg_.exploration_max_y = max_y;
}

void GoalProgressGate::finish(const XY &goal, std::size_t before, std::size_t after) {
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (after - std::min(after, before) >= min_new_cells_) return;
  // 把本次结果放入容器，供下一轮处理。
  entries_.push_back({goal, after + min_new_cells_});
}

std::vector<XY> GoalProgressGate::blocked(std::size_t observed_cells) {
  // 从容器中取走已处理或失效的数据。
  entries_.erase(std::remove_if(entries_.begin(), entries_.end(),
      [observed_cells](const Entry &entry) { return observed_cells >= entry.unlock_at; }),
      entries_.end());
  std::vector<XY> goals;
  goals.reserve(entries_.size());
  // 依次处理候选导航目标。
  for (const auto &entry : entries_) goals.push_back(entry.goal);
  // 把此函数算出的值交还给调用者。
  return goals;
}

Coverage assess_coverage(const std::unordered_map<Cell, int, CellHash> &grid,
                         const Settings &cfg) {
  Coverage result;
  if (!cfg.global_search || !std::isfinite(cfg.resolution) || cfg.resolution <= 0 ||
      !std::isfinite(cfg.exploration_min_x) || !std::isfinite(cfg.exploration_max_x) ||
      !std::isfinite(cfg.exploration_min_y) || !std::isfinite(cfg.exploration_max_y) ||
      cfg.exploration_min_x >= cfg.exploration_max_x ||
      cfg.exploration_min_y >= cfg.exploration_max_y) return result;
  // 与 project() 使用同一格心边界；限制误配置造成的超大循环。
  const double width = (cfg.exploration_max_x - cfg.exploration_min_x) / cfg.resolution;
  const double height = (cfg.exploration_max_y - cfg.exploration_min_y) / cfg.resolution;
  if (width * height > 2000000.0) return result;
  const int x0 = static_cast<int>(std::ceil(cfg.exploration_min_x / cfg.resolution - 0.5));
  const int x1 = static_cast<int>(std::floor(cfg.exploration_max_x / cfg.resolution - 0.5));
  const int y0 = static_cast<int>(std::ceil(cfg.exploration_min_y / cfg.resolution - 0.5));
  const int y1 = static_cast<int>(std::floor(cfg.exploration_max_y / cfg.resolution - 0.5));
  if (x1 < x0 || y1 < y0) return result;
  result.valid = true;
  for (int x = x0; x <= x1; ++x) for (int y = y0; y <= y1; ++y) {
    ++result.total_cells;
    auto found = grid.find({x, y});
    if (found == grid.end()) ++result.unknown_cells;
    else if (found->second == 0) ++result.known_free_cells;
    else ++result.known_occupied_cells;
  }
  return result;
}

Coverage assess_reachable_coverage(const std::unordered_map<Cell, int, CellHash> &grid,
                                   const Settings &cfg, const XY &robot_xy) {
  Coverage result = assess_coverage(grid, cfg);
  if (!result.valid || !std::isfinite(robot_xy[0]) || !std::isfinite(robot_xy[1]) ||
      !std::isfinite(cfg.robot_radius) || cfg.robot_radius < 0.0) return result;
  const double r = cfg.resolution;
  const int x0 = static_cast<int>(std::ceil(cfg.exploration_min_x / r - 0.5));
  const int x1 = static_cast<int>(std::floor(cfg.exploration_max_x / r - 0.5));
  const int y0 = static_cast<int>(std::ceil(cfg.exploration_min_y / r - 0.5));
  const int y1 = static_cast<int>(std::floor(cfg.exploration_max_y / r - 0.5));
  auto inside = [&](Cell c) { return c.x >= x0 && c.x <= x1 && c.y >= y0 && c.y <= y1; };
  // 仅用累计地图中的静态障碍做膨胀；瞬时 SCAN 障碍不能缩小验收范围。
  std::unordered_set<Cell, CellHash> blocked;
  const int inflation = static_cast<int>(std::ceil(cfg.robot_radius / r));
  for (const auto &[cell, state] : grid) {
    if (state != 1) continue;
    for (int dx = -inflation; dx <= inflation; ++dx)
      for (int dy = -inflation; dy <= inflation; ++dy)
        if (std::hypot(dx, dy) * r <= cfg.robot_radius) {
          Cell neighbor{cell.x + dx, cell.y + dy};
          if (inside(neighbor)) blocked.insert(neighbor);
        }
  }
  Cell robot{static_cast<int>(std::floor(robot_xy[0] / r)),
             static_cast<int>(std::floor(robot_xy[1] / r))};
  // 起点必须有附近已观测的空闲格；定位偏出验收范围时不能宣布完成。
  if (!inside(robot)) return result;
  std::optional<Cell> seed;
  double nearest = std::numeric_limits<double>::infinity();
  const int search = static_cast<int>(std::ceil((cfg.robot_radius + r) / r));
  for (int dx = -search; dx <= search; ++dx) for (int dy = -search; dy <= search; ++dy) {
    Cell candidate{robot.x + dx, robot.y + dy};
    auto it = grid.find(candidate);
    if (!inside(candidate) || blocked.count(candidate) || it == grid.end() ||
        it->second != 0) continue;
    double d = std::hypot((candidate.x + 0.5) * r - robot_xy[0],
                          (candidate.y + 0.5) * r - robot_xy[1]);
    if (d <= cfg.robot_radius + r && d < nearest) { seed = candidate; nearest = d; }
  }
  if (!seed) return result;
  result.reachability_valid = true;
  std::queue<Cell> queue;
  std::unordered_set<Cell, CellHash> seen;
  queue.push(*seed);
  seen.insert(*seed);
  // 未知格保持潜在可通行，防止把尚未观察到的开放通道误判为障碍。
  while (!queue.empty()) {
    Cell cell = queue.front();
    queue.pop();
    ++result.reachable_cells;
    if (!grid.count(cell)) ++result.reachable_unknown_cells;
    for (const Cell &delta : kEight) {
      Cell next{cell.x + delta.x, cell.y + delta.y};
      if (delta.x && delta.y &&
          blocked.count({cell.x + delta.x, cell.y}) &&
          blocked.count({cell.x, cell.y + delta.y})) continue;
      if (inside(next) && !blocked.count(next) && seen.insert(next).second)
        queue.push(next);
    }
  }
  result.excluded_unknown_cells = result.unknown_cells - result.reachable_unknown_cells;
  return result;
}

void FrontierIndex::rebuild(const std::unordered_map<Cell, int, CellHash> &grid) {
  cells_.clear();
  for (const auto &[cell, state] : grid) {
    if (state != 0) continue;
    for (const auto &delta : kFour) {
      const Cell neighbor{cell.x + delta.x, cell.y + delta.y};
      if (!grid.count(neighbor)) cells_.insert(neighbor);
    }
  }
}

void FrontierIndex::update(
    const std::unordered_map<Cell, int, CellHash> &grid,
    const std::unordered_set<Cell, CellHash> &dirty_cells) {
  std::unordered_set<Cell, CellHash> candidates;
  for (const auto &cell : dirty_cells) {
    candidates.insert(cell);
    for (const auto &delta : kFour)
      candidates.insert({cell.x + delta.x, cell.y + delta.y});
  }
  for (const auto &cell : candidates) {
    if (grid.count(cell)) { cells_.erase(cell); continue; }
    bool borders_free = false;
    for (const auto &delta : kFour) {
      const auto it = grid.find({cell.x + delta.x, cell.y + delta.y});
      if (it != grid.end() && it->second == 0) {
        borders_free = true;
        break;
      }
    }
    if (borders_free) cells_.insert(cell);
    else cells_.erase(cell);
  }
}

GlobalFrontiers assess_global_frontiers(
    const std::unordered_map<Cell, int, CellHash> &grid,
    const Settings &cfg, const XY &robot_xy,
    const std::unordered_set<Cell, CellHash> *indexed_frontiers) {
  GlobalFrontiers result;
  const double r = cfg.resolution;
  if (grid.empty() || !std::isfinite(r) || r <= 0.0 ||
      !std::isfinite(cfg.robot_radius) || cfg.robot_radius < 0.0 ||
      !std::isfinite(robot_xy[0]) || !std::isfinite(robot_xy[1])) return result;

  // 验收只使用累计障碍；SCAN 瞬时障碍不能消除原始前沿。
  std::unordered_set<Cell, CellHash> blocked;
  const int inflation = static_cast<int>(std::ceil(cfg.robot_radius / r));
  for (const auto &[cell, state] : grid) {
    if (state != 1) continue;
    for (int dx = -inflation; dx <= inflation; ++dx)
      for (int dy = -inflation; dy <= inflation; ++dy)
        if (std::hypot(dx, dy) * r <= cfg.robot_radius)
          blocked.insert({cell.x + dx, cell.y + dy});
  }

  const Cell robot{static_cast<int>(std::floor(robot_xy[0] / r)),
                   static_cast<int>(std::floor(robot_xy[1] / r))};
  std::optional<Cell> seed;
  double nearest = std::numeric_limits<double>::infinity();
  const int search = static_cast<int>(std::ceil((cfg.robot_radius + r) / r));
  for (int dx = -search; dx <= search; ++dx)
    for (int dy = -search; dy <= search; ++dy) {
      Cell candidate{robot.x + dx, robot.y + dy};
      auto it = grid.find(candidate);
      if (blocked.count(candidate) || it == grid.end() || it->second != 0) continue;
      const double distance = std::hypot((candidate.x + 0.5) * r - robot_xy[0],
                                         (candidate.y + 0.5) * r - robot_xy[1]);
      if (distance <= cfg.robot_radius + r && distance < nearest) {
        seed = candidate;
        nearest = distance;
      }
    }
  if (!seed) return result;

  result.valid = true;
  std::queue<Cell> queue;
  std::unordered_set<Cell, CellHash> seen{*seed};
  std::unordered_set<Cell, CellHash> frontiers;
  queue.push(*seed);
  while (!queue.empty()) {
    const Cell cell = queue.front();
    queue.pop();
    ++result.reachable_free_cells;
    for (const Cell &delta : kFour) {
      const Cell next{cell.x + delta.x, cell.y + delta.y};
      if (blocked.count(next)) continue;
      auto it = grid.find(next);
      if (it == grid.end()) {
        if (!indexed_frontiers || indexed_frontiers->count(next))
          frontiers.insert(next);
      }
      else if (it->second == 0 && seen.insert(next).second) queue.push(next);
    }
  }
  result.frontier_cells = frontiers.size();
  result.cells = std::move(frontiers);
  return result;
}

std::optional<XY> select_observation_goal(const VoxelMap &map, const Point &robot,
                                          const Settings &cfg,
                                          const std::unordered_map<Cell, int, CellHash> &grid,
                                          const std::vector<XY> &suppressed,
                                          const std::vector<XY> &external_obstacles) {
  if (!assess_coverage(grid, cfg).valid) return std::nullopt;
  Cell origin{map.key(robot).x, map.key(robot).y};
  std::unordered_set<Cell, CellHash> blocked;
  auto inflate = [&](Cell center, double radius) {
    const int inflation = static_cast<int>(std::ceil(radius / cfg.resolution));
    for (int dx = -inflation; dx <= inflation; ++dx)
      for (int dy = -inflation; dy <= inflation; ++dy)
        if (std::hypot(dx, dy) * cfg.resolution <= radius)
          blocked.insert({center.x + dx, center.y + dy});
  };
  for (const auto &[cell, state] : grid) if (state == 1) inflate(cell, cfg.robot_radius);
  const double extra_radius = std::max(0.0, cfg.robot_radius - cfg.planner_occupancy_radius);
  for (const auto &xy : external_obstacles)
    if (std::isfinite(xy[0]) && std::isfinite(xy[1]))
      inflate({static_cast<int>(std::floor(xy[0] / cfg.resolution)),
               static_cast<int>(std::floor(xy[1] / cfg.resolution))}, extra_radius);
  auto free = [&](Cell cell) {
    auto it = grid.find(cell);
    return it != grid.end() && it->second == 0 && !blocked.count(cell);
  };
  // 只沿已观察的空闲格搜索；未知格绝不作为补充观察目标。
  std::queue<Cell> queue;
  std::unordered_set<Cell, CellHash> reached;
  queue.push(origin);
  reached.insert(origin);
  std::optional<XY> best;
  double best_score = -std::numeric_limits<double>::infinity();
  while (!queue.empty()) {
    Cell cell = queue.front(); queue.pop();
    XY xy = map.center(cell);
    double range = distance(xy, {robot[0], robot[1]});
    if (free(cell) && range >= cfg.min_goal_distance && range <= cfg.max_goal_distance) {
      bool ignored = false;
      for (const auto &old : suppressed)
        if (distance(xy, old) < cfg.robot_radius * 2.0) { ignored = true; break; }
      // 每米仅评估一个候选，避免无前沿时对整张地图重复大量射线检测。
      const int stride = std::max(1, static_cast<int>(std::round(1.0 / cfg.resolution)));
      if (!ignored && cell.x % stride == 0 && cell.y % stride == 0) {
        int visible = 0;
        int ray_cells = static_cast<int>(std::ceil(cfg.max_ray_range / cfg.resolution));
        for (int dx = -ray_cells; dx <= ray_cells && visible < 16; ++dx)
          for (int dy = -ray_cells; dy <= ray_cells && visible < 16; ++dy) {
            if (dx == 0 && dy == 0) continue;
            Cell unknown{cell.x + dx, cell.y + dy};
            XY target = map.center(unknown);
            if (target[0] < cfg.exploration_min_x || target[0] > cfg.exploration_max_x ||
                target[1] < cfg.exploration_min_y || target[1] > cfg.exploration_max_y ||
                grid.count(unknown) || distance(xy, target) > cfg.max_ray_range) continue;
            bool occluded = false;
            int steps = std::max(std::abs(dx), std::abs(dy)) * 2;
            for (int step = 1; step < steps; ++step) {
              Cell crossed{static_cast<int>(std::floor(cell.x + 0.5 + dx *
                            static_cast<double>(step) / steps)),
                           static_cast<int>(std::floor(cell.y + 0.5 + dy *
                            static_cast<double>(step) / steps))};
              auto it = grid.find(crossed);
              if (it != grid.end() && it->second == 1) { occluded = true; break; }
            }
            if (!occluded) ++visible;
          }
        double score = visible - 0.25 * range;
        if (visible && score > best_score) { best = xy; best_score = score; }
      }
    }
    for (const auto &d : kFour) {
      Cell next{cell.x + d.x, cell.y + d.y};
      if (!reached.count(next) && free(next)) { reached.insert(next); queue.push(next); }
    }
  }
  return best;
}

// 聚类未知边界、计算路径代价并选出目标。
Selection select_frontier(const VoxelMap &map, const Point &robot, const Settings &cfg,
                          const std::vector<XY> &suppressed,
                          const std::vector<XY> &external_obstacles,
                          bool all_observed,
                          const std::unordered_set<Cell, CellHash> *allowed_frontiers) {
  // 下面的流程与 Python select_frontier 对应：投影、障碍膨胀、前沿聚类、最短路、评分。
  Selection result;
  auto robot_voxel = map.key(robot);
  Cell origin{robot_voxel.x, robot_voxel.y};
  // 将右侧计算值存入 grid（二维投影地图）。
  auto grid = all_observed ? map.project_all() : map.project({robot[0], robot[1]});
  int min_x = origin.x, max_x = origin.x, min_y = origin.y, max_y = origin.y;
  if (all_observed) for (const auto &[cell, state] : grid) {
    min_x = std::min(min_x, cell.x);
    max_x = std::max(max_x, cell.x);
    min_y = std::min(min_y, cell.y);
    max_y = std::max(max_y, cell.y);
  }
  // 只加一圈未知邻格；不构造无限地图。
  const bool search_limited = all_observed &&
      (static_cast<double>(max_x) - min_x + 3.0) *
      (static_cast<double>(max_y) - min_y + 3.0) > 2000000.0;
  int radius = static_cast<int>(std::ceil(cfg.local_search_radius / cfg.resolution));
  auto inside = [&](const Cell &c) {
    if (all_observed)
      return c.x >= min_x - 1 && c.x <= max_x + 1 &&
             c.y >= min_y - 1 && c.y <= max_y + 1;
    // 只有表达式为真，才执行下面缩进或花括号内的代码。
    if (cfg.global_search) {
      XY xy = map.center(c);
      // 把此函数算出的值交还给调用者。
      return xy[0] >= cfg.exploration_min_x && xy[0] <= cfg.exploration_max_x &&
             xy[1] >= cfg.exploration_min_y && xy[1] <= cfg.exploration_max_y;
    }
    // 把此函数算出的值交还给调用者。
    return std::abs(c.x - origin.x) <= radius && std::abs(c.y - origin.y) <= radius;
  };
  // 第一步：把障碍按 robot_radius 扩张。机器人中心进入扩张区就可能擦碰障碍。
  std::unordered_set<Cell, CellHash> blocked;
  int inflation = static_cast<int>(std::ceil(cfg.robot_radius / cfg.resolution));
  // 依次处理当前状态。
  for (const auto &[cell, state] : grid) {
    // 只有表达式为真，才执行下面缩进或花括号内的代码。
    if (state != 1) continue;
    // 依次处理按机器人半径向外扩张的格数。
    for (int dx = -inflation; dx <= inflation; ++dx)
      // 依次处理按机器人半径向外扩张的格数。
      for (int dy = -inflation; dy <= inflation; ++dy)
        // 只有表达式为真，才执行下面缩进或花括号内的代码。
        if (std::hypot(dx, dy) * cfg.resolution <= cfg.robot_radius)
          blocked.insert({cell.x + dx, cell.y + dy});
  }
  // 依次处理SCAN 提供的额外障碍点。
  const double extra_radius = std::max(0.0, cfg.robot_radius - cfg.planner_occupancy_radius);
  const int extra_inflation = static_cast<int>(std::ceil(extra_radius / cfg.resolution));
  for (const auto &obstacle : external_obstacles) {
    // 过滤包含无穷大或 NaN 的无效坐标。
    if (!std::isfinite(obstacle[0]) || !std::isfinite(obstacle[1])) continue;
    Cell center{static_cast<int>(std::floor(obstacle[0] / cfg.resolution)),
                static_cast<int>(std::floor(obstacle[1] / cfg.resolution))};
    // SCAN 已膨胀的部分不重复计算，只补足探索安全半径的差额。
    for (int dx = -extra_inflation; dx <= extra_inflation; ++dx)
      for (int dy = -extra_inflation; dy <= extra_inflation; ++dy)
        // 只有表达式为真，才执行下面缩进或花括号内的代码。
        if (std::hypot(dx, dy) * cfg.resolution <= extra_radius)
          blocked.insert({center.x + dx, center.y + dy});
  }
  // 第二步：优先从动态索引直接读取前沿；独立调用仍可从投影图提取。
  std::unordered_set<Cell, CellHash> frontier;
  if (allowed_frontiers) {
    for (const auto &cell : *allowed_frontiers) {
      if (!inside(cell) || grid.count(cell) || blocked.count(cell)) continue;
      for (const auto &delta : kFour) {
        const auto neighbor = grid.find({cell.x + delta.x, cell.y + delta.y});
        if (neighbor != grid.end() && neighbor->second == 0) {
          frontier.insert(cell);
          break;
        }
      }
    }
  } else {
    for (const auto &[cell, state] : grid) {
      if (state != 0) continue;
      for (const auto &d : kFour) {
        Cell next{cell.x + d.x, cell.y + d.y};
        if (inside(next) && !grid.count(next) && !blocked.count(next))
          frontier.insert(next);
      }
    }
  }
  result.frontier_cells = frontier.size();
  // 未知格只允许作为前沿终点；执行路径不得从未知格继续向外穿行。
  const auto frontier_terminals = frontier;
  // 第三步：用八邻接的队列搜索，把相连前沿格聚成一个簇。
  std::vector<std::vector<Cell>> clusters;
  // 反复处理已知空闲格与未知格的交界，直到循环条件不再成立。
  while (!frontier.empty()) {
    Cell seed = *frontier.begin();
    // 从容器中取走已处理或失效的数据。
    frontier.erase(seed);
    std::vector<Cell> cluster{seed};
    std::queue<Cell> queue;
    // 把本次结果放入容器，供下一轮处理。
    queue.push(seed);
    // 反复处理待处理的搜索队列，直到循环条件不再成立。
    while (!queue.empty()) {
      Cell cell = queue.front();
      // 从容器中取走已处理或失效的数据。
      queue.pop();
      // 依次处理集合中的每一项。
      for (const auto &d : kEight) {
        Cell next{cell.x + d.x, cell.y + d.y};
        // 检查未知边界是否仍存在或是否满足簇规模。
        if (frontier.erase(next)) {
          // 把本次结果放入容器，供下一轮处理。
          cluster.push_back(next);
          // 把本次结果放入容器，供下一轮处理。
          queue.push(next);
        }
      }
    }
    // 检查未知边界是否仍存在或是否满足簇规模。
    if (static_cast<int>(cluster.size()) >= cfg.min_frontier_cells)
      // 把本次结果放入容器，供下一轮处理。
      clusters.push_back(std::move(cluster));
  }
  result.clusters = clusters.size();
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (clusters.empty() || !inside(origin) || search_limited) return result;

  // 依次处理集合中的每一项。
  for (const auto &d : kEight) {
    Cell next{origin.x + d.x, origin.y + d.y};
    const auto known = grid.find(next);
    if (inside(next) && known != grid.end() && known->second == 0)
      ++result.open_start_neighbors;
  }

  // 给较长的 C++ 类型起一个简短别名。
  // 第四步：从机器人所在格运行一次 Dijkstra，算出到各个格的最小累计路径代价。
  // pair 的 first 是代价、second 是格号；priority_queue 每次取出当前最低代价格。
  using QueueItem = std::pair<double, Cell>;
  auto greater = [](const QueueItem &a, const QueueItem &b) { return a.first > b.first; };
  std::priority_queue<QueueItem, std::vector<QueueItem>, decltype(greater)> queue(greater);
  std::unordered_map<Cell, double, CellHash> costs;
  std::unordered_map<Cell, Cell, CellHash> parents;
  // 将右侧计算值存入 costs（起点到各格的最短路径代价）。
  costs[origin] = 0.0;
  // 把本次结果放入容器，供下一轮处理。
  queue.push({0.0, origin});
  // 反复处理待处理的搜索队列，直到循环条件不再成立。
  while (!queue.empty()) {
    auto [cost, cell] = queue.top();
    // 从容器中取走已处理或失效的数据。
    queue.pop();
    // 比较新路径和旧路径的累计代价，只保留更短者。
    if (cost > costs[cell]) continue;
    if (!(cell == origin) && !grid.count(cell)) continue;
    // 依次处理集合中的每一项。
    for (const auto &d : kEight) {
      Cell next{cell.x + d.x, cell.y + d.y};
      // 检查目标格是否落在避障膨胀后的禁行区。
      if (!inside(next)) continue;
      const auto known = grid.find(next);
      if (known == grid.end() && !frontier_terminals.count(next)) continue;
      if (known != grid.end() && known->second != 0) continue;
      // 检查目标格是否落在避障膨胀后的禁行区。
      if (d.x && d.y) {
        const Cell side_x{cell.x + d.x, cell.y};
        const Cell side_y{cell.x, cell.y + d.y};
        const auto sx = grid.find(side_x), sy = grid.find(side_y);
        if (sx == grid.end() || sy == grid.end() ||
            sx->second != 0 || sy->second != 0) continue;
      }
      // 将右侧计算值存入 step（本次步长或射线采样序号）。
      double step = cfg.resolution * std::hypot(d.x, d.y);
      // 检查当前二维格的已知空闲、障碍或未知状态。
      if (!grid.count(next)) step *= cfg.unknown_path_multiplier;
      double next_cost = cost + step;
      // 将右侧计算值存入 it（哈希表当前位置）。
      auto it = costs.find(next);
      // 比较新路径和旧路径的累计代价，只保留更短者。
      if (it == costs.end() || next_cost < it->second) {
        // 将右侧计算值存入 costs（起点到各格的最短路径代价）。
        costs[next] = next_cost;
        // 将右侧计算值存入 parents（最短路径的前驱格）。
        parents[next] = cell;
        // 把本次结果放入容器，供下一轮处理。
        queue.push({next_cost, next});
      }
    }
  }

  // 第五步：每个簇挑一个真实的前沿格；远处前沿则先找近处空闲格作分段目标。
  // 评分 = 前沿簇规模收益 - 到该前沿的路径代价，分数最大的目标获选。
  double best_score = -std::numeric_limits<double>::infinity();
  // 依次处理有效前沿簇列表。
  for (auto &cluster : clusters) {
    // 依次处理一组相连的前沿格。
    for (const auto &cell : cluster)
      // 比较新路径和旧路径的累计代价，只保留更短者。
      if (costs.count(cell)) ++result.reachable_frontier_cells;
    double mx = 0.0, my = 0.0;
    // 依次处理一组相连的前沿格。
    for (const auto &c : cluster) { mx += c.x; my += c.y; }
    mx /= cluster.size(); my /= cluster.size();
    std::sort(cluster.begin(), cluster.end(), [&](const Cell &a, const Cell &b) {
      // 返回水平面两点的欧氏距离。
      return std::hypot(a.x - mx, a.y - my) < std::hypot(b.x - mx, b.y - my);
    });
    // 依次处理一组相连的前沿格。
    for (const auto &cell : cluster) {
      auto found = costs.find(cell);
      // 比较新路径和旧路径的累计代价，只保留更短者。
      if (found == costs.end()) continue;
      XY xy = map.center(cell);
      double range = distance(xy, {robot[0], robot[1]});
      // 根据当前目标是否存在或是否完成来决定下一步。
      if (range < cfg.min_goal_distance) continue;
      // 根据当前目标是否存在或是否完成来决定下一步。
      if (range > cfg.max_goal_distance) {
        // 前沿超出单次导航范围：沿最短路径往回找一个近处已观察空闲格。
        // 先到这个分段点，再重新观察和选点。
        // 声明可为空的结果，尚未找到时保持空值。
        std::optional<XY> stage_goal;
        Cell stage = cell;
        // 反复处理雷达射线起点，直到循环条件不再成立。
        while (!(stage == origin)) {
          auto parent = parents.find(stage);
          // 只有表达式为真，才执行下面缩进或花括号内的代码。
          if (parent == parents.end()) break;
          // 将右侧计算值存入 state（探索状态）。
          auto state = grid.find(stage);
          // 检查目标格是否落在避障膨胀后的禁行区。
          if (state != grid.end() && state->second == 0 && !blocked.count(stage)) {
            XY candidate = map.center(stage);
            double stage_range = distance(candidate, {robot[0], robot[1]});
            // 根据当前目标是否存在或是否完成来决定下一步。
            if (stage_range >= cfg.min_goal_distance &&
                stage_range <= cfg.max_goal_distance) {
              stage_goal = candidate;
              // 找到满足条件的结果，退出当前循环。
              break;
            }
          }
          stage = parent->second;
        }
        // 根据当前目标是否存在或是否完成来决定下一步。
        if (!stage_goal) continue;
        xy = *stage_goal;
      }
      ++result.candidate_goals;
      bool rejected = false;
      // 依次处理暂时禁止重选的目标。
      for (const auto &failed : suppressed)
        // 只有表达式为真，才执行下面缩进或花括号内的代码。
        if (distance(xy, failed) < cfg.robot_radius * 2.0) { rejected = true; break; }
      // 根据当前目标是否存在或是否完成来决定下一步。
      if (rejected) { ++result.suppressed_goals; continue; }
      double score = cfg.frontier_gain_weight * std::log1p(cluster.size()) -
                     cfg.path_cost_weight * found->second;
      // 只有表达式为真，才执行下面缩进或花括号内的代码。
      if (score > best_score) {
        result.goal = xy;
        result.score = score;
        best_score = score;
      }
      // 找到满足条件的结果，退出当前循环。
      break;
    }
  }
  // 返回本轮前沿搜索的结果和诊断计数。
  return result;
}

}  // namespace legbot_exploration
