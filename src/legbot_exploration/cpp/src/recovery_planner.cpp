#include "legbot_exploration/recovery_planner.hpp"

#include <algorithm>
#include <cmath>

namespace legbot_exploration {
namespace {
double distance(const XY &a, const XY &b) {
  return std::hypot(a[0] - b[0], a[1] - b[1]);
}

double cell_distance(const XY &p, int x, int y, double resolution) {
  const double left = x * resolution, bottom = y * resolution;
  return std::hypot(std::max({left - p[0], 0.0, p[0] - left - resolution}),
                    std::max({bottom - p[1], 0.0, p[1] - bottom - resolution}));
}

// An already inflated footprint may leave only by increasing its clearance.
// Unknown space outside the starting footprint is never treated as free.
std::string retreat_corridor_reason(
    const std::unordered_map<Cell, int, CellHash> &grid,
    const XY &start, const XY &target, const Settings &cfg,
    const std::vector<XY> &scan_obstacles, bool reverse) {
  const double radius = cfg.robot_radius + cfg.waypoint_safety_margin;
  const double scan_radius =
      std::max(0.0, radius - cfg.planner_occupancy_radius) +
      cfg.planner_occupancy_resolution / std::sqrt(2.0);
  const int samples = std::max(1, static_cast<int>(
      std::ceil(distance(start, target) / (cfg.resolution * 0.5))));
  bool initial_overlap = false, clearance_gain = false;
  const int sx0 = static_cast<int>(std::floor((start[0] - radius) / cfg.resolution));
  const int sx1 = static_cast<int>(std::floor((start[0] + radius) / cfg.resolution));
  const int sy0 = static_cast<int>(std::floor((start[1] - radius) / cfg.resolution));
  const int sy1 = static_cast<int>(std::floor((start[1] + radius) / cfg.resolution));
  for (int x = sx0; x <= sx1; ++x) for (int y = sy0; y <= sy1; ++y) {
    const auto it = grid.find({x, y});
    if (it == grid.end() || it->second != 1) continue;
    const double initial = cell_distance(start, x, y, cfg.resolution);
    if (initial > radius + 1e-9) continue;
    initial_overlap = true;
    if (cell_distance(target, x, y, cfg.resolution) >= initial + 0.02)
      clearance_gain = true;
  }
  for (const auto &obstacle : scan_obstacles) {
    const double initial = distance(obstacle, start);
    if (initial > scan_radius + 1e-9) continue;
    initial_overlap = true;
    if (distance(obstacle, target) >= initial + 0.02) clearance_gain = true;
  }
  for (int i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / samples;
    const XY p{start[0] + t * (target[0] - start[0]),
               start[1] + t * (target[1] - start[1])};
    const int x0 = static_cast<int>(std::floor((p[0] - radius) / cfg.resolution));
    const int x1 = static_cast<int>(std::floor((p[0] + radius) / cfg.resolution));
    const int y0 = static_cast<int>(std::floor((p[1] - radius) / cfg.resolution));
    const int y1 = static_cast<int>(std::floor((p[1] + radius) / cfg.resolution));
    for (int x = x0; x <= x1; ++x) for (int y = y0; y <= y1; ++y) {
      const double current = cell_distance(p, x, y, cfg.resolution);
      if (current > radius + 1e-9) continue;
      const auto it = grid.find({x, y});
      const double initial = cell_distance(start, x, y, cfg.resolution);
      if (it == grid.end()) {
        if (initial > radius + 1e-9)
          return reverse ? "reverse_unknown_rear_corridor" :
                           "recovery_unknown_corridor";
        continue;
      }
      if (it->second != 1) continue;
      if (initial > radius + 1e-9 || current + 1e-6 < initial)
        return reverse ? "reverse_known_obstacle" :
                         "known_obstacle_in_recovery_corridor";
    }
    for (const auto &obstacle : scan_obstacles) {
      const double current = distance(obstacle, p);
      if (current > scan_radius + 1e-9) continue;
      const double initial = distance(obstacle, start);
      if (initial > scan_radius + 1e-9 || current + 1e-6 < initial)
        return reverse ? "reverse_scan_obstacle" :
                         "scan_obstacle_in_recovery_corridor";
    }
  }
  if (initial_overlap && !clearance_gain)
    return reverse ? "reverse_no_clearance_gain" :
                     "recovery_no_clearance_gain";
  return "ok";
}
}  // namespace

RecoveryState::RecoveryState(int failure_limit, double area_radius)
    : failure_limit_(std::max(1, failure_limit)),
      area_radius_(std::max(0.1, area_radius)) {}

void RecoveryState::record_failure(const XY &position,
                                     const std::string &reason) {
  if (!area_ || distance(*area_, position) > area_radius_) {
    area_ = position;
    failures_ = 0;
  }
  ++failures_;
  reason_ = reason;
}

void RecoveryState::start(const XY &position) {
  if (!area_ || distance(*area_, position) > area_radius_) area_ = position;
  phase_ = RecoveryPhase::nearby;
  attempted_.clear();
}

void RecoveryState::finish() {
  phase_ = RecoveryPhase::idle;
  failures_ = 0;
  area_.reset();
  attempted_.clear();
  reason_.clear();
}

bool RecoveryState::tried(const XY &point) const {
  return attempted_.count(point) != 0;
}

bool RecoveryState::mark_attempted(const XY &point) {
  return attempted_.insert(point).second;
}

const char *recovery_phase_name(RecoveryPhase phase) {
  switch (phase) {
    case RecoveryPhase::idle: return "idle";
    case RecoveryPhase::nearby: return "nearby";
    case RecoveryPhase::turning: return "turning";
    case RecoveryPhase::settling: return "settling";
    case RecoveryPhase::history: return "history";
    case RecoveryPhase::blocked: return "blocked";
  }
  return "invalid";
}

PlannedRoute plan_recovery_direct_route(
    const std::unordered_map<Cell, int, CellHash> &grid,
    const XY &start, const XY &target, const Settings &cfg,
    const std::vector<XY> &scan_obstacles, double max_distance) {
  PlannedRoute out;
  const double length = distance(start, target);
  if (!std::isfinite(length) || !std::isfinite(max_distance) ||
      cfg.resolution <= 0.0 || max_distance <= 0.0) {
    out.reason = "invalid_recovery_route_input";
    return out;
  }
  if (length < 0.3) {
    out.reason = "recovery_target_too_close";
    return out;
  }
  if (length > max_distance) {
    out.reason = "recovery_target_beyond_direct_limit";
    return out;
  }
  switch (waypoint_footprint_safety(target, grid, cfg, scan_obstacles)) {
    case WaypointSafety::unknown_footprint:
      out.reason = "recovery_destination_unknown_footprint";
      return out;
    case WaypointSafety::occupied_footprint:
      out.reason = "recovery_destination_occupied_footprint";
      return out;
    case WaypointSafety::scan_obstacle:
      out.reason = "recovery_destination_scan_obstacle";
      return out;
    case WaypointSafety::safe: break;
  }
  out.reason = retreat_corridor_reason(
      grid, start, target, cfg, scan_obstacles, false);
  if (out.reason != "ok") return out;
  out.points = {start, target};
  out.reason = "unverified_recovery_connection";
  return out;
}

PlannedRoute plan_recovery_reverse_route(
    const std::unordered_map<Cell, int, CellHash> &grid,
    const XY &start, double yaw, const Settings &cfg,
    const std::vector<XY> &scan_obstacles, double reverse_distance) {
  PlannedRoute out;
  if (!std::isfinite(yaw) || !std::isfinite(reverse_distance) ||
      reverse_distance < 0.20 || reverse_distance > 0.60 ||
      cfg.resolution <= 0.0 || cfg.robot_radius <= 0.0) {
    out.reason = "reverse_invalid_input";
    return out;
  }
  const XY target{start[0] - reverse_distance * std::cos(yaw),
                  start[1] - reverse_distance * std::sin(yaw)};
  const Cell centre{static_cast<int>(std::floor(target[0] / cfg.resolution)),
                    static_cast<int>(std::floor(target[1] / cfg.resolution))};
  const auto centre_it = grid.find(centre);
  if (centre_it == grid.end()) {
    out.reason = "reverse_destination_unknown";
    return out;
  }
  if (centre_it->second != 0) {
    out.reason = "reverse_destination_occupied";
    return out;
  }
  out.reason = retreat_corridor_reason(
      grid, start, target, cfg, scan_obstacles, true);
  if (out.reason != "ok") return out;
  out.points = {start, target};
  out.reason = "ok";
  return out;
}
}  // namespace legbot_exploration
