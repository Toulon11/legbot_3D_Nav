#include "legbot_exploration/route_orchestrator.hpp"
#include "legbot_exploration/recovery_planner.hpp"

#include <cmath>
#include <utility>

namespace legbot_exploration {

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
      occupancy_result.waypoints = stage_occupancy_route(
          occupancy_result.route, snapshot.grid, snapshot.settings,
          snapshot.scan_obstacles, snapshot.segment_length,
          goal.mode == ExplorationMode::recovery ? nullptr :
              snapshot.free_observations);
      if (occupancy_result.waypoints.empty())
        occupancy_result.reason = "occupancy_staging_failed";
    } else if (goal.mode == ExplorationMode::recovery) {
      occupancy_result.route = plan_recovery_direct_route(
          snapshot.grid, start, goal.observation, snapshot.settings,
          snapshot.scan_obstacles, snapshot.segment_length);
      occupancy_result.reason = occupancy_result.route.reason;
      occupancy_result.direct_reason = occupancy_result.reason;
      if (occupancy_result.route.points.size() == 2) {
        occupancy_result.waypoints = {goal.observation};
        occupancy_result.source = "recovery_unverified";
      }
    }
    results.push_back(std::move(occupancy_result));
  }

  if (goal.request_layer && snapshot.layer_fresh) {
    PlannerResult layer_result;
    layer_result.goal = goal;
    layer_result.kind = PlannerKind::layer;
    layer_result.source = "layer";
    layer_result.route = plan_surface_route(
        snapshot.surfaces, snapshot.base, goal.observation,
        snapshot.layer_resolution, snapshot.body_height, snapshot.grid,
        snapshot.settings, snapshot.scan_obstacles);
    layer_result.reason = layer_result.route.reason;
    if (layer_result.reason == "ok") {
      layer_result.waypoints = stage_surface_route(
          layer_result.route, snapshot.grid, snapshot.settings,
          snapshot.scan_obstacles, snapshot.surfaces, snapshot.layer_resolution,
          snapshot.base[2] - snapshot.body_height, snapshot.segment_length);
      if (layer_result.waypoints.empty())
        layer_result.reason = "layer_staging_failed";
    }
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
                         bool recovery) {
  if (!result.usable() || result.goal.goal_id != goal.goal_id ||
      result.goal.attempt_id != goal.attempt_id ||
      result.goal.occupancy_version != goal.occupancy_version ||
      result.goal.layer_version != goal.layer_version)
    return false;
  waypoints_ = result.waypoints;
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
