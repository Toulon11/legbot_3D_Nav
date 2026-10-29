#include "legbot_exploration/route_planner.hpp"

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
bool free_cell(const Cell &cell,
               const std::unordered_map<Cell, int, CellHash> &grid,
               const std::unordered_set<Cell, CellHash> &blocked) {
  const auto it = grid.find(cell);
  return it != grid.end() && it->second == 0 && !blocked.count(cell);
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

std::vector<XY> startup_viewpoints(
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

PlannedRoute plan_occupancy_route(
    const std::unordered_map<Cell, int, CellHash> &grid,
    const XY &start, const XY &target, const Settings &cfg,
    const std::vector<XY> &inflated_scan_obstacles, double viewpoint_radius) {
  PlannedRoute out;
  if (!(cfg.resolution > 0.0 && viewpoint_radius > 0.0) ||
      !std::isfinite(start[0]) || !std::isfinite(start[1]) ||
      !std::isfinite(target[0]) || !std::isfinite(target[1])) {
    out.reason = "invalid_route_input";
    return out;
  }
  viewpoint_radius = std::max(viewpoint_radius, cfg.resolution * 1.1);
  const auto blocked = blocked_cells(grid, cfg, inflated_scan_obstacles);
  Settings endpoint_cfg = cfg;
  endpoint_cfg.robot_radius += 0.15;
  const auto endpoint_blocked = blocked_cells(grid, endpoint_cfg,
                                              inflated_scan_obstacles);
  const Cell origin = cell_of(start, cfg.resolution);
  std::unordered_set<Cell, CellHash> goals;
  for (const auto &[cell, state] : grid) {
    if (state != 0 || blocked.count(cell)) continue;
    const double separation = distance(center_of(cell, cfg.resolution), target);
    if (separation <= viewpoint_radius && !endpoint_blocked.count(cell) &&
        frontier_visible_without_known_obstacle(center_of(cell, cfg.resolution),
                                                target, grid, cfg.resolution) &&
        waypoint_footprint_safety(center_of(cell, cfg.resolution), grid, cfg,
                                  inflated_scan_obstacles) == WaypointSafety::safe)
      goals.insert(cell);
  }
  if (goals.empty()) {
    out.reason = "no_observed_free_viewpoint";
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
  const std::unordered_set<Cell, CellHash> no_blocks;
  const auto heuristic = [&](const Cell &cell) {
    return std::max(0.0, distance(center_of(cell, cfg.resolution), target) -
                              viewpoint_radius);
  };
  open.push({heuristic(origin), 0.0, origin});
  cost[origin] = 0.0;
  Cell reached{};
  bool found = false;
  while (!open.empty()) {
    const Item current = open.top();
    open.pop();
    if (current.cost > cost[current.cell] + 1e-9) continue;
    if (goals.count(current.cell)) { reached = current.cell; found = true; break; }
    for (const Cell step : kSteps) {
      const Cell next{current.cell.x + step.x, current.cell.y + step.y};
      if (!free_cell(next, grid, no_blocks)) continue;
      if (step.x && step.y &&
          (!free_cell({current.cell.x + step.x, current.cell.y}, grid, no_blocks) ||
           !free_cell({current.cell.x, current.cell.y + step.y}, grid, no_blocks))) continue;
      const double next_cost = current.cost + cfg.resolution * std::hypot(step.x, step.y);
      const auto it = cost.find(next);
      if (it == cost.end() || next_cost + 1e-9 < it->second) {
        cost[next] = next_cost;
        parent[next] = current.cell;
        open.push({next_cost + heuristic(next), next_cost, next});
      }
    }
  }
  if (!found) {
    // A disconnected reference grid is not a movement verdict. Hand SCAN one
    // verified free destination and let its live planner decide how to get there.
    auto best = goals.begin();
    for (auto it = goals.begin(); it != goals.end(); ++it)
      if (distance(center_of(*it, cfg.resolution), target) <
          distance(center_of(*best, cfg.resolution), target)) best = it;
    out.points = {start, center_of(*best, cfg.resolution)};
    out.reason = "direct_safe_viewpoint";
    return out;
  }
  std::vector<XY> reversed;
  for (Cell cursor = reached; !(cursor == origin); cursor = parent.at(cursor))
    reversed.push_back(center_of(cursor, cfg.resolution));
  out.points.push_back(start);
  for (auto it = reversed.rbegin(); it != reversed.rend(); ++it)
    out.points.push_back(*it);
  if (out.points.size() == 1)
    out.points.push_back(center_of(reached, cfg.resolution));
  out.reason = "ok";
  return out;
}

PlannedRoute plan_surface_route(const std::vector<LayerSurface> &surfaces,
                                const Point &base, const XY &target,
                                double resolution, double nominal_body_height,
                                double search_radius) {
  PlannedRoute out;
  if (!(resolution > 0.0 && nominal_body_height > 0.0 && search_radius > 0.0)) {
    out.reason = "invalid_surface_config";
    return out;
  }
  const double ground = base[2] - nominal_body_height;
  std::unordered_map<Cell, int, CellHash> nodes;
  std::vector<const LayerSurface *> selected;
  selected.reserve(surfaces.size());
  for (const auto &s : surfaces) {
    const XY p = center_of({s.x_cell, s.y_cell}, resolution);
    if (s.kind != SurfaceKind::flat || s.clearance == ClearanceKind::blocked ||
        s.hit_observations < 1 || s.support_columns < 5 ||
        std::abs(s.height - ground) > 0.20 ||
        distance(p, {base[0], base[1]}) > search_radius) continue;
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
  if (out.points.size() == 1) out.points.push_back(xy(finish));
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
  // These are the waypoints handed to SCAN. SCAN chooses and validates the
  // trajectory between them; the exploration layer checks each destination.
  for (std::size_t i = 1; i < route.points.size(); ++i) {
    if (!supported.count(cell_of(route.points[i], resolution))) return true;
    // 表面格有面积，障碍格的边角可能比格心更靠近机体圆盘。
    const double cell_half_diagonal = resolution / std::sqrt(2.0);
    if (layer_clearance(route.points[i], hard_blocks, resolution,
                        robot_radius + resolution) <=
        robot_radius + cell_half_diagonal + 1e-9)
      return true;
  }
  return false;
}

std::vector<XY> stage_route(const PlannedRoute &route,
                            const std::unordered_map<Cell, int, CellHash> &grid,
                            const Settings &cfg,
                            const std::vector<XY> &inflated_scan_obstacles,
                            double target_spacing) {
  std::vector<XY> staged;
  if (route.points.size() < 2 || target_spacing <= 0.0) return staged;
  std::vector<double> travelled(route.points.size(), 0.0);
  for (std::size_t i = 1; i < route.points.size(); ++i)
    travelled[i] = travelled[i - 1] + distance(route.points[i - 1], route.points[i]);
  const std::size_t finish = route.points.size() - 1;
  if (travelled.back() <= 1e-6 ||
      waypoint_footprint_safety(route.points.back(), grid, cfg,
                                inflated_scan_obstacles) != WaypointSafety::safe)
    return staged;
  const int count = std::clamp(static_cast<int>(
      std::ceil(travelled.back() / target_spacing)), 1, 3);
  std::size_t previous = 0;
  for (int part = 1; part < count; ++part) {
    const double desired = travelled.back() * part / count;
    std::size_t best = 0;
    double best_error = std::numeric_limits<double>::infinity();
    for (std::size_t i = previous + 1; i < finish; ++i) {
      if (waypoint_footprint_safety(route.points[i], grid, cfg,
                                    inflated_scan_obstacles) != WaypointSafety::safe)
        continue;
      const double error = std::abs(travelled[i] - desired);
      if (error < best_error) { best = i; best_error = error; }
    }
    if (best == 0) break;
    staged.push_back(route.points[best]);
    previous = best;
  }
  staged.push_back(route.points.back());
  return staged;
}

}  // namespace legbot_exploration
