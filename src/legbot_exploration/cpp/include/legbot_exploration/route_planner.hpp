#pragma once

#include "legbot_exploration/exploration_core.hpp"
#include "legbot_exploration/multi_layer_map.hpp"

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

// Startup scanning may leave the robot's own cell unknown. Candidate goals
// are observed-free with a safe footprint; SCAN remains responsible for the
// short motion across the unobserved gap around the robot.
std::vector<XY> startup_viewpoints(
    const std::unordered_map<Cell, int, CellHash> &grid, const XY &start,
    const Settings &cfg, const std::vector<XY> &inflated_scan_obstacles,
    double min_distance, double max_distance);

// Use observed-free cells to estimate a reference route. Inflation is checked
// at published destinations; SCAN owns the motion between them.
PlannedRoute plan_occupancy_route(
    const std::unordered_map<Cell, int, CellHash> &grid,
    const XY &start, const XY &target, const Settings &cfg,
    const std::vector<XY> &inflated_scan_obstacles,
    double viewpoint_radius = 0.75);

// Same-floor surface A*: support and overhead classification are checked on
// every node. The route is only a candidate; SCAN still validates execution.
PlannedRoute plan_surface_route(const std::vector<LayerSurface> &surfaces,
                                const Point &base, const XY &target,
                                double resolution, double nominal_body_height,
                                double search_radius = 7.0);

bool route_conflicts_with_layer(const PlannedRoute &route,
                                const std::vector<LayerSurface> &surfaces,
                                double resolution, double ground_height,
                                double robot_radius);

// Choose one to three inflated-safe destinations along the reference route.
// SCAN plans the motion between successive PoseStamped goals.
std::vector<XY> stage_route(const PlannedRoute &route,
                            const std::unordered_map<Cell, int, CellHash> &grid,
                            const Settings &cfg,
                            const std::vector<XY> &inflated_scan_obstacles,
                            double target_spacing = 3.0);

}  // namespace legbot_exploration
