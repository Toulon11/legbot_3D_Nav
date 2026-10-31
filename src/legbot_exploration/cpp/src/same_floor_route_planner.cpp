#include "legbot_exploration/same_floor_route_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_set>

namespace legbot_exploration {
namespace {
constexpr std::array<Cell, 8> kSteps{{{1, 0}, {-1, 0}, {0, 1}, {0, -1},
                                       {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};
Cell cell_of(const XY &p, double r) {
  return {static_cast<int>(std::floor(p[0] / r)),
          static_cast<int>(std::floor(p[1] / r))};
}
XY center_of(const Cell &p, double r) {
  return {(p.x + 0.5) * r, (p.y + 0.5) * r};
}
double distance(const XY &a, const XY &b) {
  return std::hypot(a[0] - b[0], a[1] - b[1]);
}
class ScanObstacleIndex {
 public:
  ScanObstacleIndex(const std::vector<XY> &points, double resolution)
      : resolution_(resolution) {
    if (resolution_ <= 0.0) return;
    for (const auto &p : points)
      if (std::isfinite(p[0]) && std::isfinite(p[1]))
        buckets_[cell_of(p, resolution_)].push_back(p);
  }

  std::vector<XY> near(const XY &a, const XY &b, double radius) const {
    std::vector<XY> points;
    if (resolution_ <= 0.0) return points;
    const int x0 = static_cast<int>(std::floor(
        (std::min(a[0], b[0]) - radius) / resolution_));
    const int x1 = static_cast<int>(std::floor(
        (std::max(a[0], b[0]) + radius) / resolution_));
    const int y0 = static_cast<int>(std::floor(
        (std::min(a[1], b[1]) - radius) / resolution_));
    const int y1 = static_cast<int>(std::floor(
        (std::max(a[1], b[1]) + radius) / resolution_));
    for (int x = x0; x <= x1; ++x)
      for (int y = y0; y <= y1; ++y) {
        const auto it = buckets_.find({x, y});
        if (it != buckets_.end())
          points.insert(points.end(), it->second.begin(), it->second.end());
      }
    return points;
  }

 private:
  double resolution_;
  std::unordered_map<Cell, std::vector<XY>, CellHash> buckets_;
};
double point_segment_distance(const XY &point, const XY &a, const XY &b) {
  const double dx = b[0] - a[0], dy = b[1] - a[1];
  const double length_squared = dx * dx + dy * dy;
  const double t = length_squared > 0.0 ?
      std::clamp(((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) /
                     length_squared, 0.0, 1.0) : 0.0;
  return distance(point, {a[0] + t * dx, a[1] + t * dy});
}
bool segment_intersects_square(const XY &a, const XY &b,
                               double min_x, double max_x,
                               double min_y, double max_y) {
  double first = 0.0, last = 1.0;
  const double dx = b[0] - a[0], dy = b[1] - a[1];
  const auto clip = [&](double p, double q) {
    if (std::abs(p) < 1e-12) return q >= 0.0;
    const double t = q / p;
    if (p < 0.0) first = std::max(first, t);
    else last = std::min(last, t);
    return first <= last;
  };
  return clip(-dx, a[0] - min_x) && clip(dx, max_x - a[0]) &&
         clip(-dy, a[1] - min_y) && clip(dy, max_y - a[1]);
}
double segment_square_distance(const XY &a, const XY &b,
                               double min_x, double max_x,
                               double min_y, double max_y) {
  if (segment_intersects_square(a, b, min_x, max_x, min_y, max_y)) return 0.0;
  const auto point_square_distance = [&](const XY &point) {
    return std::hypot(std::max({min_x - point[0], 0.0, point[0] - max_x}),
                      std::max({min_y - point[1], 0.0, point[1] - max_y}));
  };
  double nearest = std::min(point_square_distance(a), point_square_distance(b));
  for (const XY &corner : std::array<XY, 4>{{{min_x, min_y}, {min_x, max_y},
                                             {max_x, min_y}, {max_x, max_y}}})
    nearest = std::min(nearest, point_segment_distance(corner, a, b));
  return nearest;
}
double free_evidence_multiplier(const Cell &cell, const Settings &cfg,
                                const FreeObservationCounts *observations) {
  if (!observations) return 1.0;
  const auto it = observations->find(cell);
  if (it == observations->end() || it->second == 0)
    return std::numeric_limits<double>::infinity();
  const int saturation = std::max(1, cfg.free_observation_saturation);
  if (saturation == 1) return 1.0;
  const int frames = std::min<int>(it->second, saturation);
  return 1.0 + cfg.low_evidence_cost_weight *
      static_cast<double>(saturation - frames) / (saturation - 1);
}
double segment_evidence_cost(const XY &a, const XY &b, const Settings &cfg,
                             const FreeObservationCounts *observations) {
  const double length = distance(a, b);
  if (!observations) return length;
  if (cfg.resolution <= 0.0) return std::numeric_limits<double>::infinity();
  const int samples = std::max(1, static_cast<int>(
      std::ceil(length / (cfg.resolution * 0.5))));
  double cost = 0.0;
  for (int i = 0; i < samples; ++i) {
    const double t = (i + 0.5) / samples;
    const XY point{a[0] + t * (b[0] - a[0]),
                   a[1] + t * (b[1] - a[1])};
    const double multiplier = free_evidence_multiplier(
        cell_of(point, cfg.resolution), cfg, observations);
    if (!std::isfinite(multiplier)) return multiplier;
    cost += length * multiplier / samples;
  }
  return cost;
}
bool observed_free_corridor(
    const XY &a, const XY &b,
    const std::unordered_map<Cell, int, CellHash> &grid,
    const Settings &cfg, const std::vector<XY> &inflated_scan_obstacles,
    const FreeObservationCounts *free_observations = nullptr) {
  if (cfg.resolution <= 0.0 || cfg.robot_radius < 0.0 ||
      cfg.waypoint_safety_margin < 0.0 ||
      cfg.planner_occupancy_resolution <= 0.0) return false;
  const double radius = cfg.robot_radius + cfg.waypoint_safety_margin;
  const int min_x = static_cast<int>(std::floor((std::min(a[0], b[0]) - radius) /
                                            cfg.resolution));
  const int max_x = static_cast<int>(std::floor((std::max(a[0], b[0]) + radius) /
                                            cfg.resolution));
  const int min_y = static_cast<int>(std::floor((std::min(a[1], b[1]) - radius) /
                                            cfg.resolution));
  const int max_y = static_cast<int>(std::floor((std::max(a[1], b[1]) + radius) /
                                            cfg.resolution));
  for (int x = min_x; x <= max_x; ++x)
    for (int y = min_y; y <= max_y; ++y) {
      const double left = x * cfg.resolution, bottom = y * cfg.resolution;
      if (segment_square_distance(a, b, left, left + cfg.resolution,
                                  bottom, bottom + cfg.resolution) > radius + 1e-9)
        continue;
      const auto it = grid.find({x, y});
      if (it == grid.end() || it->second != 0) return false;
      if (!std::isfinite(free_evidence_multiplier(
              {x, y}, cfg, free_observations))) return false;
    }
  const double extra = std::max(0.0, radius - cfg.planner_occupancy_radius) +
      cfg.planner_occupancy_resolution / std::sqrt(2.0);
  for (const auto &obstacle : inflated_scan_obstacles)
    if (std::isfinite(obstacle[0]) && std::isfinite(obstacle[1]) &&
        point_segment_distance(obstacle, a, b) <= extra + 1e-9)
      return false;
  return true;
}
std::unordered_set<Cell, CellHash> blocked_cells(
    const std::unordered_map<Cell, int, CellHash> &grid,
    const Settings &cfg, const std::vector<XY> &obstacles) {
  std::unordered_set<Cell, CellHash> blocked;
  const auto inflate = [&](Cell cell, double radius) {
    const int reach = static_cast<int>(std::ceil(radius / cfg.resolution));
    for (int dx = -reach; dx <= reach; ++dx)
      for (int dy = -reach; dy <= reach; ++dy)
        if (std::hypot(dx, dy) * cfg.resolution <= radius + 1e-9)
          blocked.insert({cell.x + dx, cell.y + dy});
  };
  for (const auto &[cell, state] : grid)
    if (state == 1) inflate(cell, cfg.robot_radius);
  const double extra = std::max(0.0, cfg.robot_radius - cfg.planner_occupancy_radius);
  for (const auto &p : obstacles)
    if (std::isfinite(p[0]) && std::isfinite(p[1]))
      inflate(cell_of(p, cfg.resolution), extra);
  return blocked;
}
bool frontier_visible_without_known_obstacle(
    const XY &from, const XY &frontier,
    const std::unordered_map<Cell, int, CellHash> &grid, double resolution) {
  const int samples = std::max(1, static_cast<int>(
      std::ceil(distance(from, frontier) / (resolution * 0.4))));
  for (int i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / samples;
    const Cell cell = cell_of({from[0] + t * (frontier[0] - from[0]),
                               from[1] + t * (frontier[1] - from[1])}, resolution);
    const auto found = grid.find(cell);
    if (found != grid.end() && found->second == 1) return false;
  }
  return true;
}
std::unordered_set<Cell, CellHash> layer_hard_blocks(
    const std::vector<LayerSurface> &surfaces, double ground_height) {
  std::unordered_set<Cell, CellHash> blocks;
  for (const auto &s : surfaces) {
    if (std::abs(s.height - ground_height) > 0.20) continue;
    if (s.clearance == ClearanceKind::blocked ||
        s.kind == SurfaceKind::steep_or_disconnected)
      blocks.insert({s.x_cell, s.y_cell});
  }
  return blocks;
}
double layer_clearance(const XY &point,
                       const std::unordered_set<Cell, CellHash> &blocks,
                       double resolution, double search_radius) {
  const Cell center = cell_of(point, resolution);
  const int reach = static_cast<int>(std::ceil(search_radius / resolution)) + 1;
  double nearest = std::numeric_limits<double>::infinity();
  for (int dx = -reach; dx <= reach; ++dx)
    for (int dy = -reach; dy <= reach; ++dy)
      if (blocks.count({center.x + dx, center.y + dy}))
        nearest = std::min(nearest,
                           distance(point, center_of({center.x + dx, center.y + dy},
                                                     resolution)));
  return nearest;
}
}  // namespace

WaypointSafety waypoint_footprint_safety(
    const XY &point, const std::unordered_map<Cell, int, CellHash> &grid,
    const Settings &cfg, const std::vector<XY> &inflated_scan_obstacles) {
  if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
      cfg.resolution <= 0.0 || cfg.robot_radius < 0.0 ||
      cfg.waypoint_safety_margin < 0.0 ||
      cfg.planner_occupancy_resolution <= 0.0)
    return WaypointSafety::unknown_footprint;
  const double radius = cfg.robot_radius + cfg.waypoint_safety_margin;
  const int min_x = static_cast<int>(std::floor((point[0] - radius) / cfg.resolution));
  const int max_x = static_cast<int>(std::floor((point[0] + radius) / cfg.resolution));
  const int min_y = static_cast<int>(std::floor((point[1] - radius) / cfg.resolution));
  const int max_y = static_cast<int>(std::floor((point[1] + radius) / cfg.resolution));
  for (int x = min_x; x <= max_x; ++x)
    for (int y = min_y; y <= max_y; ++y) {
      const double dx = std::max({x * cfg.resolution - point[0], 0.0,
                                  point[0] - (x + 1) * cfg.resolution});
      const double dy = std::max({y * cfg.resolution - point[1], 0.0,
                                  point[1] - (y + 1) * cfg.resolution});
      if (std::hypot(dx, dy) > radius + 1e-9) continue;
      const auto it = grid.find({x, y});
      if (it == grid.end()) return WaypointSafety::unknown_footprint;
      if (it->second != 0) return WaypointSafety::occupied_footprint;
    }
  const double extra = std::max(0.0, radius - cfg.planner_occupancy_radius) +
      cfg.planner_occupancy_resolution * std::sqrt(0.5);
  for (const auto &obstacle : inflated_scan_obstacles) {
    if (!std::isfinite(obstacle[0]) || !std::isfinite(obstacle[1])) continue;
    if (std::abs(point[0] - obstacle[0]) <= extra &&
        std::abs(point[1] - obstacle[1]) <= extra &&
        distance(point, obstacle) <= extra + 1e-9)
      return WaypointSafety::scan_obstacle;
  }
  return WaypointSafety::safe;
}

std::vector<XY> nearby_safe_viewpoints(
    const std::unordered_map<Cell, int, CellHash> &grid, const XY &start,
    const Settings &cfg, const std::vector<XY> &inflated_scan_obstacles,
    double min_distance, double max_distance) {
  std::vector<std::pair<double, XY>> candidates;
  if (cfg.resolution <= 0.0 || min_distance <= 0.0 ||
      max_distance < min_distance || !std::isfinite(start[0]) ||
      !std::isfinite(start[1])) return {};
  for (const auto &[cell, state] : grid) {
    if (state != 0) continue;
    const XY point = center_of(cell, cfg.resolution);
    const double length = distance(start, point);
    if (length < min_distance || length > max_distance ||
        waypoint_footprint_safety(point, grid, cfg,
                                  inflated_scan_obstacles) != WaypointSafety::safe)
      continue;
    candidates.push_back({length, point});
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto &a, const auto &b) {
    if (a.first != b.first) return a.first < b.first;
    return a.second < b.second;
  });
  std::vector<XY> result;
  result.reserve(candidates.size());
  for (const auto &item : candidates) result.push_back(item.second);
  return result;
}

std::vector<XY> observation_viewpoints(
    const std::unordered_map<Cell, int, CellHash> &grid,
    const XY &frontier, const Settings &cfg,
    const std::vector<XY> &inflated_scan_obstacles, double viewpoint_radius) {
  std::vector<std::pair<double, XY>> ranked;
  if (cfg.resolution <= 0.0 || viewpoint_radius <= 0.0 ||
      !std::isfinite(frontier[0]) || !std::isfinite(frontier[1])) return {};
  Settings endpoint_cfg = cfg;
  endpoint_cfg.robot_radius += 0.15;
  for (const auto &[cell, state] : grid) {
    if (state != 0) continue;
    const XY point = center_of(cell, cfg.resolution);
    const double separation = distance(point, frontier);
    if (separation > std::max(viewpoint_radius, cfg.resolution * 1.1) ||
        waypoint_footprint_safety(point, grid, endpoint_cfg,
                                  inflated_scan_obstacles) != WaypointSafety::safe ||
        !frontier_visible_without_known_obstacle(
            point, frontier, grid, cfg.resolution)) continue;
    ranked.emplace_back(separation, point);
  }
  std::sort(ranked.begin(), ranked.end(), [](const auto &a, const auto &b) {
    return a.first != b.first ? a.first < b.first : a.second < b.second;
  });
  std::vector<XY> result;
  result.reserve(ranked.size());
  for (const auto &item : ranked) result.push_back(item.second);
  return result;
}

PlannedRoute plan_occupancy_route(
    const std::unordered_map<Cell, int, CellHash> &grid,
    const XY &start, const XY &target, const Settings &cfg,
    const std::vector<XY> &inflated_scan_obstacles,
    const FreeObservationCounts *free_observations) {
  PlannedRoute out;
  if (cfg.resolution <= 0.0 ||
      !std::isfinite(start[0]) || !std::isfinite(start[1]) ||
      !std::isfinite(target[0]) || !std::isfinite(target[1])) {
    out.reason = "invalid_route_input";
    return out;
  }
  const Cell origin = cell_of(start, cfg.resolution);
  const Cell finish = cell_of(target, cfg.resolution);
  const ScanObstacleIndex scan_index(inflated_scan_obstacles, cfg.resolution);
  const double scan_search_radius = cfg.robot_radius +
      cfg.waypoint_safety_margin + cfg.planner_occupancy_resolution;
  if (waypoint_footprint_safety(target, grid, cfg,
                                inflated_scan_obstacles) != WaypointSafety::safe) {
    out.reason = "unsafe_viewpoint";
    return out;
  }
  if (waypoint_footprint_safety(start, grid, cfg,
                                inflated_scan_obstacles) != WaypointSafety::safe ||
      !observed_free_corridor(start, center_of(origin, cfg.resolution), grid,
                              cfg, inflated_scan_obstacles,
                              free_observations)) {
    out.reason = "unsafe_start";
    return out;
  }
  std::unordered_map<Cell, bool, CellHash> traversable_cache;
  const auto traversable = [&](const Cell &cell) {
    const auto cached = traversable_cache.find(cell);
    if (cached != traversable_cache.end()) return cached->second;
    const XY point = center_of(cell, cfg.resolution);
    const auto nearby = scan_index.near(point, point, scan_search_radius);
    const bool safe = waypoint_footprint_safety(
        point, grid, cfg, nearby) == WaypointSafety::safe &&
        std::isfinite(free_evidence_multiplier(
            cell, cfg, free_observations));
    traversable_cache.emplace(cell, safe);
    return safe;
  };
  if (!traversable(origin) || !traversable(finish)) {
    out.reason = "unsafe_endpoint_cell";
    return out;
  }
  struct Item {
    double estimate, cost;
    Cell cell;
    bool operator<(const Item &rhs) const { return estimate > rhs.estimate; }
  };
  std::priority_queue<Item> open;
  std::unordered_map<Cell, double, CellHash> cost;
  std::unordered_map<Cell, Cell, CellHash> parent;
  const auto heuristic = [&](const Cell &cell) {
    return distance(center_of(cell, cfg.resolution), target);
  };
  open.push({heuristic(origin), 0.0, origin});
  cost[origin] = 0.0;
  Cell reached{};
  bool found = false;
  while (!open.empty()) {
    const Item current = open.top();
    open.pop();
    if (current.cost > cost[current.cell] + 1e-9) continue;
    if (current.cell == finish) { reached = current.cell; found = true; break; }
    for (const Cell step : kSteps) {
      const Cell next{current.cell.x + step.x, current.cell.y + step.y};
      if (!traversable(next)) continue;
      if (step.x && step.y &&
          (!traversable({current.cell.x + step.x, current.cell.y}) ||
           !traversable({current.cell.x, current.cell.y + step.y}))) continue;
      const XY a = center_of(current.cell, cfg.resolution);
      const XY b = center_of(next, cfg.resolution);
      const auto nearby = scan_index.near(a, b, scan_search_radius);
      if (!observed_free_corridor(a, b, grid, cfg, nearby,
                                  free_observations)) continue;
      const double edge_length = cfg.resolution * std::hypot(step.x, step.y);
      const double next_cost = current.cost + edge_length * 0.5 *
          (free_evidence_multiplier(current.cell, cfg, free_observations) +
           free_evidence_multiplier(next, cfg, free_observations));
      const auto it = cost.find(next);
      if (it == cost.end() || next_cost + 1e-9 < it->second) {
        cost[next] = next_cost;
        parent[next] = current.cell;
        open.push({next_cost + heuristic(next), next_cost, next});
      }
    }
  }
  if (!found) {
    out.reason = "occupancy_route_disconnected";
    return out;
  }
  std::vector<XY> reversed;
  for (Cell cursor = reached; !(cursor == origin); cursor = parent.at(cursor))
    reversed.push_back(center_of(cursor, cfg.resolution));
  out.points.push_back(start);
  for (auto it = reversed.rbegin(); it != reversed.rend(); ++it)
    out.points.push_back(*it);
  if (out.points.size() == 1 || distance(out.points.back(), target) > 1e-9)
    out.points.push_back(target);
  if (!observed_free_corridor(out.points[out.points.size() - 2], target, grid,
                              cfg, inflated_scan_obstacles,
                              free_observations)) {
    out.points.clear();
    out.reason = "unsafe_endpoint_approach";
    return out;
  }
  out.reason = "ok";
  return out;
}

PlannedRoute plan_surface_route(const std::vector<LayerSurface> &surfaces,
                                const Point &base, const XY &target,
                                double resolution, double nominal_body_height,
                                const std::unordered_map<Cell, int, CellHash> &grid,
                                const Settings &cfg,
                                const std::vector<XY> &inflated_scan_obstacles,
                                double search_radius) {
  PlannedRoute out;
  if (!(resolution > 0.0 && nominal_body_height > 0.0 && search_radius > 0.0)) {
    out.reason = "invalid_surface_config";
    return out;
  }
  const double ground = base[2] - nominal_body_height;
  const auto hard_blocks = layer_hard_blocks(surfaces, ground);
  const double clearance_radius = cfg.robot_radius + cfg.waypoint_safety_margin;
  std::unordered_map<Cell, int, CellHash> nodes;
  std::vector<const LayerSurface *> selected;
  selected.reserve(surfaces.size());
  for (const auto &s : surfaces) {
    const XY p = center_of({s.x_cell, s.y_cell}, resolution);
    if (s.kind != SurfaceKind::flat || s.clearance == ClearanceKind::blocked ||
        s.hit_observations < 1 || s.support_columns < 5 ||
        std::abs(s.height - ground) > 0.20 ||
        distance(p, {base[0], base[1]}) > search_radius ||
        waypoint_footprint_safety(p, grid, cfg, inflated_scan_obstacles) !=
            WaypointSafety::safe ||
        layer_clearance(p, hard_blocks, resolution,
                        clearance_radius + resolution) <=
            clearance_radius + resolution / std::sqrt(2.0)) continue;
    const Cell cell{s.x_cell, s.y_cell};
    const auto it = nodes.find(cell);
    if (it != nodes.end()) {
      if (std::abs(selected[it->second]->height - ground) <= std::abs(s.height - ground))
        continue;
      selected[it->second] = &s;
    } else {
      nodes[cell] = static_cast<int>(selected.size());
      selected.push_back(&s);
    }
  }
  if (selected.empty()) { out.reason = "no_supported_same_floor_surface"; return out; }
  auto nearest = [&](const XY &p, double tolerance) {
    int best = -1;
    double best_distance = tolerance;
    for (int i = 0; i < static_cast<int>(selected.size()); ++i) {
      const double d = distance(center_of({selected[i]->x_cell, selected[i]->y_cell}, resolution), p);
      if (d < best_distance) { best_distance = d; best = i; }
    }
    return best;
  };
  const int origin = nearest({base[0], base[1]}, 0.45);
  const int finish = nearest(target, 0.45);
  if (origin < 0 || finish < 0) {
    out.reason = "surface_endpoint_unobserved";
    return out;
  }
  if (!observed_free_corridor({base[0], base[1]},
                              center_of({selected[origin]->x_cell,
                                         selected[origin]->y_cell}, resolution),
                              grid, cfg, inflated_scan_obstacles) ||
      !observed_free_corridor(
          center_of({selected[finish]->x_cell, selected[finish]->y_cell}, resolution),
          target, grid, cfg, inflated_scan_obstacles)) {
    out.reason = "surface_endpoint_corridor_blocked";
    return out;
  }
  struct Item {
    double estimate, cost;
    int index;
    bool operator<(const Item &rhs) const { return estimate > rhs.estimate; }
  };
  std::priority_queue<Item> open;
  std::vector<double> cost(selected.size(), std::numeric_limits<double>::infinity());
  std::vector<int> parent(selected.size(), -1);
  const auto xy = [&](int index) {
    return center_of({selected[index]->x_cell, selected[index]->y_cell}, resolution);
  };
  cost[origin] = 0.0;
  open.push({distance(xy(origin), xy(finish)), 0.0, origin});
  constexpr std::array<Cell, 4> steps{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
  bool found = false;
  while (!open.empty()) {
    const Item current = open.top(); open.pop();
    if (current.cost > cost[current.index] + 1e-9) continue;
    if (current.index == finish) { found = true; break; }
    const LayerSurface &s = *selected[current.index];
    for (const Cell step : steps) {
      const auto it = nodes.find({s.x_cell + step.x, s.y_cell + step.y});
      if (it == nodes.end()) continue;
      const int next = it->second;
      if (!observed_free_corridor(xy(current.index), xy(next), grid, cfg,
                                  inflated_scan_obstacles)) continue;
      const double dz = selected[next]->height - s.height;
      if (std::abs(dz) > std::tan(0.40) * resolution + 1e-6) continue;
      const double edge = std::hypot(resolution, dz);
      if (current.cost + edge + 1e-9 < cost[next]) {
        cost[next] = current.cost + edge;
        parent[next] = current.index;
        open.push({cost[next] + distance(xy(next), xy(finish)), cost[next], next});
      }
    }
  }
  if (!found) { out.reason = "surface_route_disconnected"; return out; }
  std::vector<XY> reversed;
  for (int i = finish; i != origin; i = parent[i]) reversed.push_back(xy(i));
  out.points.push_back({base[0], base[1]});
  for (auto it = reversed.rbegin(); it != reversed.rend(); ++it) out.points.push_back(*it);
  if (distance(out.points.back(), target) > 1e-9) out.points.push_back(target);
  if (out.points.size() == 1) out.points.push_back(target);
  out.reason = "ok";
  return out;
}

bool route_conflicts_with_layer(const PlannedRoute &route,
                                const std::vector<LayerSurface> &surfaces,
                                double resolution, double ground_height,
                                double robot_radius) {
  if (route.points.size() < 2 || resolution <= 0.0 || robot_radius < 0.0)
    return false;
  const auto hard_blocks = layer_hard_blocks(surfaces, ground_height);
  std::unordered_set<Cell, CellHash> supported;
  for (const auto &s : surfaces)
    if (std::abs(s.height - ground_height) <= 0.20 &&
        s.kind == SurfaceKind::flat && s.clearance != ClearanceKind::blocked &&
        s.support_columns >= 5 && s.hit_observations >= 1)
      supported.insert({s.x_cell, s.y_cell});
  for (std::size_t i = 1; i < route.points.size(); ++i) {
    const XY &a = route.points[i - 1], &b = route.points[i];
    const int samples = std::max(1, static_cast<int>(
        std::ceil(distance(a, b) / (resolution * 0.5))));
    for (int sample = 1; sample <= samples; ++sample) {
      const double t = static_cast<double>(sample) / samples;
      const XY p{a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])};
      if (!supported.count(cell_of(p, resolution))) return true;
      if (layer_clearance(p, hard_blocks, resolution,
                          robot_radius + resolution) <=
          robot_radius + resolution / std::sqrt(2.0) + 1e-9)
        return true;
    }
  }
  return false;
}

std::vector<XY> stage_occupancy_route(
    const PlannedRoute &route,
    const std::unordered_map<Cell, int, CellHash> &grid,
    const Settings &cfg,
    const std::vector<XY> &inflated_scan_obstacles,
    double max_segment_length,
    const FreeObservationCounts *free_observations) {
  std::vector<XY> staged;
  if (route.reason != "ok" || route.points.size() < 2 ||
      !std::isfinite(max_segment_length) || max_segment_length <= 0.0 ||
      !std::isfinite(cfg.max_goal_distance) || cfg.max_goal_distance <= 0.0)
    return staged;
  const double limit = std::min(max_segment_length, cfg.max_goal_distance);
  std::vector<double> original_cost(route.points.size(), 0.0);
  if (free_observations) {
    for (std::size_t i = 1; i < route.points.size(); ++i)
      original_cost[i] = original_cost[i - 1] + segment_evidence_cost(
          route.points[i - 1], route.points[i], cfg, free_observations);
  }
  std::size_t current = 0;
  while (current + 1 < route.points.size()) {
    std::size_t farthest = current;
    for (std::size_t next = current + 1; next < route.points.size(); ++next) {
      if (distance(route.points[current], route.points[next]) > limit + 1e-9 ||
          !observed_free_corridor(route.points[current], route.points[next],
                                  grid, cfg, inflated_scan_obstacles,
                                  free_observations))
        continue;
      if (free_observations &&
          segment_evidence_cost(route.points[current], route.points[next],
                                cfg, free_observations) >
              (original_cost[next] - original_cost[current]) *
                  (1.0 + cfg.shortcut_cost_tolerance) + 1e-9)
        continue;
      farthest = next;
    }
    if (farthest == current) return {};
    staged.push_back(route.points[farthest]);
    current = farthest;
  }
  return staged;
}

std::vector<XY> stage_surface_route(
    const PlannedRoute &route,
    const std::unordered_map<Cell, int, CellHash> &grid,
    const Settings &cfg, const std::vector<XY> &inflated_scan_obstacles,
    const std::vector<LayerSurface> &surfaces, double layer_resolution,
    double ground_height, double max_segment_length) {
  std::vector<XY> staged;
  if (route.reason != "ok" || route.points.size() < 2 ||
      max_segment_length <= 0.0 || cfg.max_goal_distance <= 0.0) return staged;
  const double limit = std::min(max_segment_length, cfg.max_goal_distance);
  std::size_t current = 0;
  while (current + 1 < route.points.size()) {
    std::size_t farthest = current;
    for (std::size_t next = current + 1; next < route.points.size(); ++next) {
      if (distance(route.points[current], route.points[next]) > limit + 1e-9 ||
          !observed_free_corridor(route.points[current], route.points[next],
                                  grid, cfg, inflated_scan_obstacles) ||
          route_conflicts_with_layer(
              {{route.points[current], route.points[next]}, "ok"}, surfaces,
              layer_resolution, ground_height,
              cfg.robot_radius + cfg.waypoint_safety_margin)) continue;
      farthest = next;
    }
    if (farthest == current) return {};
    staged.push_back(route.points[farthest]);
    current = farthest;
  }
  return staged;
}

}  // namespace legbot_exploration
