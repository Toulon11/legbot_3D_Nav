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
  return segment_safe_check(a, b,
      OccupancyMapView{grid,cfg,inflated_scan_obstacles,free_observations}).safe;
}
}  // namespace

PointSafetyResult point_safe_check(const XY &point, const OccupancyMapView &map) {
  PointSafetyResult out;
  out.evaluated=true;
  out.map_version=map.version;
  out.position={point[0],point[1],0.0};
  const auto &cfg=map.settings;
  if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
      !std::isfinite(cfg.resolution) || cfg.resolution<=0 ||
      !std::isfinite(cfg.robot_radius) || cfg.robot_radius<0 ||
      !std::isfinite(cfg.waypoint_safety_margin) || cfg.waypoint_safety_margin<0 ||
      cfg.planner_occupancy_resolution<=0) {
    out.reject(SafetyIssue::invalid_input);return out;
  }
  out.cell=cell_of(point,cfg.resolution);
  const double radius=cfg.robot_radius+cfg.waypoint_safety_margin;
  const int x0=static_cast<int>(std::floor((point[0]-radius)/cfg.resolution));
  const int x1=static_cast<int>(std::floor((point[0]+radius)/cfg.resolution));
  const int y0=static_cast<int>(std::floor((point[1]-radius)/cfg.resolution));
  const int y1=static_cast<int>(std::floor((point[1]+radius)/cfg.resolution));
  for (int x=x0;x<=x1;++x) for (int y=y0;y<=y1;++y) {
    const double dx=std::max({x*cfg.resolution-point[0],0.0,point[0]-(x+1)*cfg.resolution});
    const double dy=std::max({y*cfg.resolution-point[1],0.0,point[1]-(y+1)*cfg.resolution});
    if (std::hypot(dx,dy)>radius+1e-9) continue;
    ++out.checked_cells;
    const auto found=map.grid.find({x,y});
    if (found==map.grid.end()) out.reject(SafetyIssue::unknown);
    else if (found->second!=0) out.reject(SafetyIssue::occupied);
    else if (!std::isfinite(free_evidence_multiplier({x,y},cfg,map.free_observations)))
      out.reject(SafetyIssue::zero_free_evidence);
  }
  const double extra=std::max(0.0,radius-cfg.planner_occupancy_radius)+
      cfg.planner_occupancy_resolution/std::sqrt(2.0);
  for (const auto &p:map.scan_obstacles)
    if (std::isfinite(p[0]) && std::isfinite(p[1]) && distance(point,p)<=extra+1e-9)
      out.reject(SafetyIssue::scan_obstacle);
  return out;
}

SegmentSafetyResult segment_safe_check(const XY &a,const XY &b,const OccupancyMapView &map) {
  SegmentSafetyResult out;
  out.failure.map_version=map.version;
  out.failure.evaluated=true;
  const auto &cfg=map.settings;
  const auto at_a=point_safe_check(a,map),at_b=point_safe_check(b,map);
  if (!at_a.safe || !at_b.safe) {
    out.safe=false;out.failure=!at_a.safe?at_a:at_b;return out;
  }
  const double radius=cfg.robot_radius+cfg.waypoint_safety_margin;
  const int x0=static_cast<int>(std::floor((std::min(a[0],b[0])-radius)/cfg.resolution));
  const int x1=static_cast<int>(std::floor((std::max(a[0],b[0])+radius)/cfg.resolution));
  const int y0=static_cast<int>(std::floor((std::min(a[1],b[1])-radius)/cfg.resolution));
  const int y1=static_cast<int>(std::floor((std::max(a[1],b[1])+radius)/cfg.resolution));
  for (int x=x0;x<=x1;++x) for (int y=y0;y<=y1;++y) {
    const double left=x*cfg.resolution,bottom=y*cfg.resolution;
    if (segment_square_distance(a,b,left,left+cfg.resolution,bottom,bottom+cfg.resolution)>radius+1e-9) continue;
    ++out.failure.checked_cells;
    const auto found=map.grid.find({x,y});
    if (found==map.grid.end()) out.failure.reject(SafetyIssue::unknown);
    else if (found->second!=0) out.failure.reject(SafetyIssue::occupied);
    else if (!std::isfinite(free_evidence_multiplier({x,y},cfg,map.free_observations)))
      out.failure.reject(SafetyIssue::zero_free_evidence);
  }
  const double extra=std::max(0.0,radius-cfg.planner_occupancy_radius)+
      cfg.planner_occupancy_resolution/std::sqrt(2.0);
  for (const auto &p:map.scan_obstacles)
    if (std::isfinite(p[0]) && std::isfinite(p[1]) && point_segment_distance(p,a,b)<=extra+1e-9)
      out.failure.reject(SafetyIssue::scan_obstacle);
  out.safe=out.failure.safe;
  return out;
}

WaypointSafety waypoint_footprint_safety(
    const XY &point,const std::unordered_map<Cell,int,CellHash> &grid,
    const Settings &cfg,const std::vector<XY> &obstacles) {
  const auto result=point_safe_check(point,OccupancyMapView{grid,cfg,obstacles});
  if (result.safe) return WaypointSafety::safe;
  if (result.rejected[static_cast<std::size_t>(SafetyIssue::occupied)]) return WaypointSafety::occupied_footprint;
  if (result.rejected[static_cast<std::size_t>(SafetyIssue::scan_obstacle)]) return WaypointSafety::scan_obstacle;
  return WaypointSafety::unknown_footprint;
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
  const OccupancyMapView map{grid,cfg,inflated_scan_obstacles,free_observations};
  out.start_check=check_start(start,map);
  if (!out.start_check.ready) {
    out.reason="unsafe_start";
    out.failure_stage=out.start_check.stage;
    return out;
  }
  out.goal_check=point_safe_check(target,map);
  if (!out.goal_check.safe) {
    out.reason="unsafe_viewpoint";out.failure_stage="goal_position";return out;
  }
  std::unordered_map<Cell, bool, CellHash> traversable_cache;
  const auto traversable = [&](const Cell &cell) {
    const auto cached = traversable_cache.find(cell);
    if (cached != traversable_cache.end()) return cached->second;
    const XY point = center_of(cell, cfg.resolution);
    const auto nearby = scan_index.near(point, point, scan_search_radius);
    const bool safe = point_safe_check(point,
        OccupancyMapView{grid,cfg,nearby,free_observations}).safe;
    traversable_cache.emplace(cell, safe);
    return safe;
  };
  if (!traversable(origin) || !traversable(finish)) {
    out.reason = !traversable(origin) ? "unsafe_start_cell" : "unsafe_endpoint_cell";
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
    ++out.expanded_nodes;
    if (current.cell == finish) { reached = current.cell; found = true; break; }
    for (const Cell step : kSteps) {
      const Cell next{current.cell.x + step.x, current.cell.y + step.y};
      if (!traversable(next)) {++out.rejected_missing_nodes;continue;}
      if (step.x && step.y &&
          (!traversable({current.cell.x + step.x, current.cell.y}) ||
           !traversable({current.cell.x, current.cell.y + step.y}))) continue;
      const XY a = center_of(current.cell, cfg.resolution);
      const XY b = center_of(next, cfg.resolution);
      const auto nearby = scan_index.near(a, b, scan_search_radius);
      if (!observed_free_corridor(a, b, grid, cfg, nearby,
                                  free_observations)) {++out.rejected_corridor_edges;continue;}
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
    out.failure_stage="astar";
    return out;
  }
  std::vector<XY> reversed;
  for (Cell cursor = reached; !(cursor == origin); cursor = parent.at(cursor))
    reversed.push_back(center_of(cursor, cfg.resolution));
  out.points.push_back(start);
  if (distance(start,out.start_check.anchor)>1e-9)
    out.points.push_back(out.start_check.anchor);
  for (auto it = reversed.rbegin(); it != reversed.rend(); ++it)
    out.points.push_back(*it);
  if (out.points.size() == 1 || distance(out.points.back(), target) > 1e-9)
    out.points.push_back(target);
  out.failed_connection=segment_safe_check(out.points[out.points.size()-2],target,map);
  if (!out.failed_connection.safe) {
    out.points.clear();
    out.reason = "unsafe_endpoint_approach";
    out.failure_stage="goal_connection";
    return out;
  }
  out.reason = "ok";
  return out;
}

std::vector<XY> stage_occupancy_route(
    const PlannedRoute &route,
    const std::unordered_map<Cell, int, CellHash> &grid,
    const Settings &cfg,
    const std::vector<XY> &inflated_scan_obstacles,
    const FreeObservationCounts *free_observations) {
  std::vector<XY> staged;
  if (route.reason != "ok" || route.points.size() < 2)
    return staged;
  std::vector<double> original_cost(route.points.size(), 0.0);
  if (free_observations) {
    for (std::size_t i = 1; i < route.points.size(); ++i)
      original_cost[i] = original_cost[i - 1] + segment_evidence_cost(
          route.points[i - 1], route.points[i], cfg, free_observations);
  }
  std::size_t current = 0;
  while (current + 1 < route.points.size()) {
    std::size_t farthest = current;
    for (std::size_t next = route.points.size() - 1; next > current; --next) {
      if (!observed_free_corridor(route.points[current], route.points[next],
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
      break;
    }
    if (farthest == current) return {};
    staged.push_back(route.points[farthest]);
    current = farthest;
  }
  return staged;
}

}  // namespace legbot_exploration
