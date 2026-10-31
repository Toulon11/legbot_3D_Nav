#pragma once

#include "legbot_exploration/single_floor_exploration.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace legbot_exploration {

struct PlannedRoute {
  std::vector<XY> points;  // Includes the start and an observed-free endpoint.
  std::string reason;
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

// Candidate observation positions and recovery destinations. This function
// never claims that a route to one of these positions exists.
std::vector<XY> nearby_safe_viewpoints(
    const std::unordered_map<Cell, int, CellHash> &grid, const XY &start,
    const Settings &cfg, const std::vector<XY> &inflated_scan_obstacles,
    double min_distance, double max_distance);

// Select an observed, inflated-safe position from which the frontier is
// visible. The route planners independently try to reach this exact position.
std::vector<XY> observation_viewpoints(
    const std::unordered_map<Cell, int, CellHash> &grid,
    const XY &frontier, const Settings &cfg,
    const std::vector<XY> &inflated_scan_obstacles,
    double viewpoint_radius = 2.0);

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
