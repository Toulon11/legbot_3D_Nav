#pragma once

#include "legbot_exploration/point_safety.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace legbot_exploration {

struct PlannedRoute {
  std::vector<XY> points;  // Occupancy includes start; native PCT points are kept as returned.
  std::string reason;
  StartAdmission start_check;
  PointSafetyResult goal_check;
  SegmentSafetyResult failed_connection;
  std::string failure_stage;
  std::size_t expanded_nodes = 0;
  std::size_t rejected_slope_edges = 0;
  std::size_t rejected_corridor_edges = 0;
  std::size_t rejected_missing_nodes = 0;
};

enum class WaypointSafety {
  safe,
  unknown_footprint,
  occupied_footprint,
  scan_obstacle,
};

// A waypoint is executable only when every map cell intersecting the robot's
// inflated footprint has observed-free evidence. The SCAN cloud is already
// inflated, so only the remaining radius and cell extent are checked there.
WaypointSafety waypoint_footprint_safety(
    const XY &point, const std::unordered_map<Cell, int, CellHash> &grid,
    const Settings &cfg, const std::vector<XY> &inflated_scan_obstacles);

// Search only connected, observed-free cells and edges. Failure is reported
// to the exploration controller; recovery is a separate planner's job.
PlannedRoute plan_occupancy_route(
    const std::unordered_map<Cell, int, CellHash> &grid,
    const XY &start, const XY &target, const Settings &cfg,
    const std::vector<XY> &inflated_scan_obstacles,
    const FreeObservationCounts *free_observations = nullptr);

// Only for a connected occupancy A* route: shortcut across fully observed,
// inflated-free corridors, retaining turns needed around obstacles.
std::vector<XY> stage_occupancy_route(
    const PlannedRoute &route,
    const std::unordered_map<Cell, int, CellHash> &grid,
    const Settings &cfg,
    const std::vector<XY> &inflated_scan_obstacles,
    const FreeObservationCounts *free_observations = nullptr);

}  // namespace legbot_exploration
