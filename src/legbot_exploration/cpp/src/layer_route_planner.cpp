#include "legbot_exploration/layer_route_planner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace legbot_exploration {
namespace {
Cell cell_of(const XY &p, double resolution) {
  return {static_cast<int>(std::floor(p[0] / resolution)),
          static_cast<int>(std::floor(p[1] / resolution))};
}
XY center_of(Cell cell, double resolution) {
  return {(cell.x + 0.5) * resolution, (cell.y + 0.5) * resolution};
}
double distance(const XY &a, const XY &b) {
  return std::hypot(a[0] - b[0], a[1] - b[1]);
}

class LayerSafety {
 public:
  template <class EachSurface, class EachOccupied>
  LayerSafety(EachSurface each_surface, EachOccupied each_occupied,
              double ground, double body_height, double resolution,
              double footprint_radius)
      : resolution_(resolution), radius_(footprint_radius) {
    each_surface([&](const LayerSurface &surface) {
      if (std::abs(surface.height - ground) > 0.20 ||
          surface.kind != SurfaceKind::flat ||
          surface.clearance != ClearanceKind::observed_clear ||
          surface.support_columns < 5 || surface.hit_observations < 1)
        return;
      const Cell cell{surface.x_cell, surface.y_cell};
      const auto it = supported_.find(cell);
      if (it == supported_.end() ||
          std::abs(surface.height - ground) < std::abs(it->second->height - ground))
        supported_[cell] = &surface;
    });
    const double lower = ground + 0.12;
    const double upper = ground + std::max(0.55, body_height + 0.20);
    each_occupied([&](const LayerPoint &point) {
      if (std::isfinite(point.x) && std::isfinite(point.y) &&
          std::isfinite(point.z) && point.z > lower && point.z < upper)
        occupied_.insert(cell_of({point.x, point.y}, resolution_));
    });
  }

  const auto &supported() const { return supported_; }

  bool footprint(const XY &point) const {
    const int x0 = static_cast<int>(std::floor((point[0] - radius_) / resolution_));
    const int x1 = static_cast<int>(std::floor((point[0] + radius_) / resolution_));
    const int y0 = static_cast<int>(std::floor((point[1] - radius_) / resolution_));
    const int y1 = static_cast<int>(std::floor((point[1] + radius_) / resolution_));
    for (int x = x0; x <= x1; ++x)
      for (int y = y0; y <= y1; ++y) {
        const double left = x * resolution_, right = left + resolution_;
        const double bottom = y * resolution_, top = bottom + resolution_;
        const double dx = std::max({left - point[0], 0.0, point[0] - right});
        const double dy = std::max({bottom - point[1], 0.0, point[1] - top});
        if (std::hypot(dx, dy) > radius_ + 1e-9) continue;
        const Cell cell{x, y};
        if (!supported_.count(cell) || occupied_.count(cell)) return false;
      }
    return true;
  }

  bool corridor(const XY &a, const XY &b) const {
    const int steps = std::max(1, static_cast<int>(
        std::ceil(distance(a, b) / (resolution_ * 0.5))));
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      if (!footprint({a[0] + t * (b[0] - a[0]),
                      a[1] + t * (b[1] - a[1])})) return false;
    }
    return true;
  }

 private:
  double resolution_, radius_;
  std::unordered_map<Cell, const LayerSurface *, CellHash> supported_;
  std::unordered_set<Cell, CellHash> occupied_;
};
}  // namespace

template <class EachSurface, class EachOccupied>
LayerPlan plan_layer_route_impl(EachSurface each_surface, EachOccupied each_occupied,
                           const Point &base, const XY &target,
                           double resolution, double nominal_body_height,
                           const Settings &cfg, double max_segment_length,
                           double search_radius, double timeout_ms) {
  LayerPlan out;
  if (resolution <= 0.0 || nominal_body_height <= 0.0 ||
      max_segment_length <= 0.0 || search_radius <= 0.0 ||
      cfg.robot_radius < 0.0 || cfg.waypoint_safety_margin < 0.0 ||
      timeout_ms <= 0.0) {
    out.reference.reason = "invalid_layer_planning_config";
    return out;
  }
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double, std::milli>(timeout_ms);
  const double ground = base[2] - nominal_body_height;
  const LayerSafety safety(each_surface, each_occupied, ground,
      nominal_body_height, resolution,
      cfg.robot_radius + cfg.waypoint_safety_margin);
  std::unordered_map<Cell, const LayerSurface *, CellHash> nodes;
  for (const auto &[cell, surface] : safety.supported()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      out.reference.reason = "layer_planning_timeout";
      return out;
    }
    const XY p = center_of(cell, resolution);
    if (distance(p, {base[0], base[1]}) <= search_radius && safety.footprint(p))
      nodes.emplace(cell, surface);
  }
  if (nodes.empty()) {
    out.reference.reason = "no_inflated_safe_layer_surface";
    return out;
  }
  const auto nearest = [&](const XY &point) {
    Cell best{};
    double best_distance = 0.45;
    bool found = false;
    for (const auto &[cell, surface] : nodes) {
      const double d = distance(point, center_of(cell, resolution));
      if (d < best_distance) { best = cell; best_distance = d; found = true; }
    }
    return std::pair<bool, Cell>{found, best};
  };
  const auto [start_found, start] = nearest({base[0], base[1]});
  const auto [finish_found, finish] = nearest(target);
  if (!start_found || !finish_found) {
    out.reference.reason = "surface_endpoint_unobserved";
    return out;
  }
  if (!safety.corridor({base[0], base[1]}, center_of(start, resolution)) ||
      !safety.corridor(center_of(finish, resolution), target)) {
    out.reference.reason = "layer_endpoint_corridor_blocked";
    return out;
  }
  struct QueueItem {
    double estimate, cost;
    Cell cell;
    bool operator<(const QueueItem &rhs) const { return estimate > rhs.estimate; }
  };
  std::priority_queue<QueueItem> open;
  std::unordered_map<Cell, double, CellHash> cost;
  std::unordered_map<Cell, Cell, CellHash> parent;
  const auto heuristic = [&](Cell cell) {
    return distance(center_of(cell, resolution), center_of(finish, resolution));
  };
  cost[start] = 0.0;
  open.push({heuristic(start), 0.0, start});
  constexpr std::array<Cell, 4> steps{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
  bool reached = false;
  while (!open.empty()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      out.reference.reason = "layer_planning_timeout";
      return out;
    }
    const auto current = open.top(); open.pop();
    if (current.cost > cost[current.cell] + 1e-9) continue;
    if (current.cell == finish) { reached = true; break; }
    for (const Cell offset : steps) {
      const Cell next{current.cell.x + offset.x, current.cell.y + offset.y};
      const auto found = nodes.find(next);
      if (found == nodes.end()) continue;
      const double dz = found->second->height - nodes.at(current.cell)->height;
      if (std::abs(dz) > std::tan(0.40) * resolution + 1e-6) continue;
      if (!safety.corridor(center_of(current.cell, resolution),
                           center_of(next, resolution))) continue;
      const double edge = std::hypot(resolution, dz);
      const double next_cost = current.cost + edge;
      const auto previous = cost.find(next);
      if (previous == cost.end() || next_cost + 1e-9 < previous->second) {
        cost[next] = next_cost;
        parent[next] = current.cell;
        open.push({next_cost + heuristic(next), next_cost, next});
      }
    }
  }
  if (!reached) {
    out.reference.reason = "surface_route_disconnected";
    return out;
  }
  std::vector<XY> reverse;
  for (Cell cell = finish; !(cell == start); cell = parent.at(cell))
    reverse.push_back(center_of(cell, resolution));
  out.reference.points.push_back({base[0], base[1]});
  for (auto it = reverse.rbegin(); it != reverse.rend(); ++it)
    out.reference.points.push_back(*it);
  if (out.reference.points.size() == 1 ||
      distance(out.reference.points.back(), target) > 1e-9)
    out.reference.points.push_back(target);
  else
    out.reference.points.back() = target;
  const double limit = std::min(max_segment_length, cfg.max_goal_distance);
  std::size_t current = 0;
  while (current + 1 < out.reference.points.size()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      out.reference.reason = "layer_planning_timeout";
      out.reference.points.clear();
      out.waypoints.clear();
      return out;
    }
    std::size_t farthest = current;
    for (std::size_t next = current + 1; next < out.reference.points.size(); ++next)
      if (distance(out.reference.points[current], out.reference.points[next]) <= limit &&
          safety.corridor(out.reference.points[current], out.reference.points[next]))
        farthest = next;
    if (farthest == current) {
      out.reference.reason = "layer_execution_corridor_blocked";
      out.reference.points.clear();
      out.waypoints.clear();
      return out;
    }
    out.waypoints.push_back(out.reference.points[farthest]);
    current = farthest;
  }
  out.reference.reason = "ok";
  return out;
}

LayerPlan plan_layer_route(const std::vector<LayerSurface> &surfaces,
                           const std::vector<Point> &occupied_voxels,
                           const Point &base, const XY &target,
                           double resolution, double nominal_body_height,
                           const Settings &cfg, double max_segment_length,
                           double search_radius) {
  const auto each_surface = [&](auto visit) {
    for (const auto &surface : surfaces) visit(surface);
  };
  const auto each_occupied = [&](auto visit) {
    for (const auto &point : occupied_voxels)
      visit(LayerPoint{point[0], point[1], point[2]});
  };
  return plan_layer_route_impl(each_surface, each_occupied, base, target,
      resolution, nominal_body_height, cfg, max_segment_length,
      search_radius, 2000.0);
}

LayerPlan plan_layer_route(const OnlineLayerMap &map,
                           const Point &base, const XY &target,
                           double resolution, double nominal_body_height,
                           const Settings &cfg, double max_segment_length,
                           double search_radius, double timeout_ms) {
  const auto each_surface = [&](auto visit) { map.for_each_surface(visit); };
  const auto each_occupied = [&](auto visit) { map.for_each_occupied(visit); };
  return plan_layer_route_impl(each_surface, each_occupied, base, target,
      resolution, nominal_body_height, cfg, max_segment_length,
      search_radius, timeout_ms);
}

}  // namespace legbot_exploration
