#include "legbot_exploration/route_orchestrator.hpp"
#include "legbot_exploration/recovery_planner.hpp"
#include "legbot_exploration/layer_route_planner.hpp"

#include <cmath>
#include <chrono>
#include <mutex>
#include <shared_mutex>
#include <utility>

namespace legbot_exploration {

namespace {
PlannerFailureClass classify_failure(const std::string &reason) {
  if (reason == "ok" || reason == "unverified_recovery_connection")
    return PlannerFailureClass::none;
  if (reason == "unsafe_start" || reason == "unsafe_start_cell" ||
      reason == "surface_start_unobserved" ||
      reason == "layer_start_corridor_blocked")
    return PlannerFailureClass::unsafe_start;
  if (reason == "unsafe_viewpoint" || reason == "unsafe_endpoint_cell" ||
      reason == "unsafe_endpoint_approach" ||
      reason == "surface_goal_unobserved" ||
      reason == "layer_goal_corridor_blocked")
    return PlannerFailureClass::unsafe_goal;
  if (reason == "layer_planning_timeout") return PlannerFailureClass::timeout;
  if (reason == "no_inflated_safe_layer_surface")
    return PlannerFailureClass::unavailable;
  if (reason == "occupancy_route_disconnected" ||
      reason == "surface_route_disconnected")
    return PlannerFailureClass::disconnected;
  return PlannerFailureClass::other;
}
}

const char *mode_name(ExplorationMode mode) {
  switch (mode) {
    case ExplorationMode::waiting_initial_data: return "waiting_initial_data";
    case ExplorationMode::flat: return "flat";
    case ExplorationMode::cross_layer: return "cross_layer";
    case ExplorationMode::recovery: return "recovery";
  }
  return "invalid";
}

const char *planner_name(PlannerKind kind) {
  switch (kind) {
    case PlannerKind::occupancy: return "occupancy";
    case PlannerKind::layer: return "layer";
    case PlannerKind::recovery: return "recovery";
  }
  return "invalid";
}

const char *planner_failure_name(PlannerFailureClass failure) {
  switch (failure) {
    case PlannerFailureClass::none: return "none";
    case PlannerFailureClass::unsafe_start: return "unsafe_start";
    case PlannerFailureClass::unsafe_goal: return "unsafe_goal";
    case PlannerFailureClass::disconnected: return "disconnected";
    case PlannerFailureClass::timeout: return "timeout";
    case PlannerFailureClass::unavailable: return "unavailable";
    case PlannerFailureClass::other: return "other";
  }
  return "other";
}

std::vector<PlannerResult> plan_candidates(
    const PlanningGoal &goal, const PlanningSnapshot &snapshot) {
  std::vector<PlannerResult> results;
  // Both independent planners receive the same observation position.
  const XY start{snapshot.base[0], snapshot.base[1]};
  if (goal.request_occupancy || goal.request_recovery) {
    PlannerResult occupancy_result;
    occupancy_result.goal = goal;
    occupancy_result.kind = goal.mode == ExplorationMode::recovery ?
        PlannerKind::recovery : PlannerKind::occupancy;
    occupancy_result.source = goal.mode == ExplorationMode::recovery ?
        "recovery" : "occupancy";
    occupancy_result.route = plan_occupancy_route(
        snapshot.grid, start, goal.observation, snapshot.settings,
        snapshot.scan_obstacles,
        goal.mode == ExplorationMode::recovery ? nullptr :
            snapshot.free_observations);
    occupancy_result.reason = occupancy_result.route.reason;
    if (goal.mode == ExplorationMode::recovery)
      occupancy_result.strict_reason = occupancy_result.reason;
    if (occupancy_result.reason == "ok") {
      if (goal.mode == ExplorationMode::recovery) {
        occupancy_result.waypoints = stage_occupancy_route(
            occupancy_result.route, snapshot.grid, snapshot.settings,
            snapshot.scan_obstacles);
        if (occupancy_result.waypoints.empty())
          occupancy_result.reason = "recovery_staging_failed";
      } else {
        occupancy_result.waypoints.assign(
            occupancy_result.route.points.begin() + 1,
            occupancy_result.route.points.end());
      }
    } else if (goal.mode == ExplorationMode::recovery) {
      occupancy_result.route = plan_recovery_direct_route(
          snapshot.grid, start, goal.observation, snapshot.settings,
          snapshot.scan_obstacles, snapshot.recovery_direct_max_distance);
      occupancy_result.reason = occupancy_result.route.reason;
      occupancy_result.direct_reason = occupancy_result.reason;
      if (occupancy_result.route.points.size() == 2) {
        occupancy_result.waypoints = {goal.observation};
        occupancy_result.source = "recovery_unverified";
      }
    }
    occupancy_result.failure_class = classify_failure(occupancy_result.reason);
    results.push_back(std::move(occupancy_result));
  }

  if (goal.request_layer && snapshot.layer_fresh && snapshot.shared_layer) {
    PlannerResult layer_result;
    layer_result.goal = goal;
    layer_result.kind = PlannerKind::layer;
    layer_result.source = "layer";
    const auto started = std::chrono::steady_clock::now();
    LayerPlan layer_plan;
    auto &shared = *snapshot.shared_layer;
    // Pause map integration while the planner holds the read lock.
    shared.planning_active.store(true);
    try {
      {
        std::unique_lock<std::shared_mutex> write_lock(shared.mutex);
        shared.map->rebuild_dirty();
      }
      {
        std::shared_lock<std::shared_mutex> read_lock(shared.mutex);
        layer_plan = plan_layer_route(*shared.map, snapshot.base,
            goal.observation, snapshot.layer_resolution,
            snapshot.body_height, snapshot.settings,
            snapshot.layer_planning_timeout_ms);
      }
    } catch (...) {
      shared.planning_active.store(false);
      throw;
    }
    shared.planning_active.store(false);
    layer_result.planning_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    shared.last_planning_ms.store(layer_result.planning_ms);
    layer_result.route = std::move(layer_plan.reference);
    layer_result.waypoints = std::move(layer_plan.waypoints);
    layer_result.reason = layer_result.route.reason;
    layer_result.failure_class = classify_failure(layer_result.reason);
    results.push_back(std::move(layer_result));
  }
  return results;
}

const PlannerResult *select_plan(
    const PlanningGoal &goal, const std::vector<PlannerResult> &results) {
  const PlannerResult *best = nullptr;
  for (const auto &result : results) {
    if (result.goal.goal_id != goal.goal_id ||
        result.goal.attempt_id != goal.attempt_id ||
        result.goal.occupancy_version != goal.occupancy_version ||
        result.goal.layer_version != goal.layer_version || !result.usable())
      continue;
    if (goal.mode == ExplorationMode::cross_layer) continue;
    if (goal.mode == ExplorationMode::recovery) {
      if (result.kind == PlannerKind::recovery) best = &result;
      continue;
    }
    if (result.kind == PlannerKind::layer && goal.request_layer)
      best = &result;
    else if (result.kind == PlannerKind::occupancy &&
             goal.request_occupancy && best == nullptr)
      best = &result;
  }
  return best;
}

bool PathProcessor::load(const PlanningGoal &goal, const PlannerResult &result,
                         bool recovery, const PlanningSnapshot *snapshot) {
  if (!result.usable() || result.goal.goal_id != goal.goal_id ||
      result.goal.attempt_id != goal.attempt_id ||
      result.goal.occupancy_version != goal.occupancy_version ||
      result.goal.layer_version != goal.layer_version)
    return false;
  auto execution_waypoints = result.waypoints;
  if (result.kind == PlannerKind::occupancy && !recovery) {
    if (!snapshot) return false;
    execution_waypoints = stage_occupancy_route(
        result.route, snapshot->grid, snapshot->settings,
        snapshot->scan_obstacles,
        snapshot->free_observations);
    if (execution_waypoints.empty()) return false;
  }
  waypoints_ = std::move(execution_waypoints);
  index_ = 0;
  source_ = result.source;
  recovery_ = recovery;
  goal_id_ = goal.goal_id;
  attempt_id_ = goal.attempt_id;
  begin_execution();
  return true;
}

void PathProcessor::clear() {
  waypoints_.clear();
  index_ = 0;
  source_ = "none";
  recovery_ = false;
  goal_id_ = 0;
  attempt_id_ = 0;
  begin_execution();
}

bool PathProcessor::advance() {
  if (!active()) return false;
  ++index_;
  begin_execution();
  return active();
}

void PathProcessor::begin_execution() {
  scan_goal_stamp_.reset();
  execution_resolved_ = false;
}

void PathProcessor::bind_scan_goal(std::int64_t stamp) {
  if (active() && stamp > 0 && !execution_resolved_)
    scan_goal_stamp_ = stamp;
}

std::optional<ExecutionResult> PathProcessor::resolve_execution(
    std::int64_t stamp, bool success, const std::string &reason) {
  if (!active() || !scan_goal_stamp_ || *scan_goal_stamp_ != stamp ||
      execution_resolved_)
    return std::nullopt;
  execution_resolved_ = true;
  return ExecutionResult{goal_id_, attempt_id_, index_, stamp, source_, reason,
                         success};
}

std::size_t PathProcessor::skip_within(const XY &position, double radius) {
  if (!std::isfinite(radius) || radius < 0.0) return 0;
  std::size_t skipped = 0;
  while (active() &&
         std::hypot(current()[0] - position[0], current()[1] - position[1]) <= radius) {
    ++skipped;
    advance();
  }
  return skipped;
}

}  // namespace legbot_exploration
