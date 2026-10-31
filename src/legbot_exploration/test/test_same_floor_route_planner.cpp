#include "legbot_exploration/same_floor_route_planner.hpp"
#include "legbot_exploration/recovery_planner.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

int main() {
  using namespace legbot_exploration;
  Settings cfg;
  cfg.resolution = 1.0;
  cfg.robot_radius = 0.0;
  std::unordered_map<Cell, int, CellHash> grid;
  for (int x = 0; x <= 7; ++x)
    for (int y = 0; y <= 4; ++y) grid[{x, y}] = 0;
  for (int y = 1; y <= 3; ++y) grid[{3, y}] = 1;
  const auto route = plan_occupancy_route(grid, {0.5, 2.5}, {6.5, 2.5}, cfg, {});
  assert(route.reason == "ok" && route.points.size() > 2);
  const auto stages = stage_occupancy_route(route, grid, cfg, {});
  assert(stages.size() >= 2 && stages.back() == XY({6.5, 2.5}));
  for (const auto &point : stages)
    assert(waypoint_footprint_safety(point, grid, cfg, {}) == WaypointSafety::safe);

  // Missing observations and blocked starts never create a direct unverified route.
  grid.erase({0, 2});
  auto blind = plan_occupancy_route(grid, {0.5, 2.5}, {6.5, 2.5}, cfg, {});
  assert(blind.points.empty() && blind.reason == "unsafe_start");
  grid[{0, 2}] = 0;
  for (int y = 0; y <= 4; ++y) grid[{3, y}] = 1;
  auto disconnected = plan_occupancy_route(grid, {0.5, 2.5}, {6.5, 2.5}, cfg, {});
  assert(disconnected.points.empty() &&
         disconnected.reason == "occupancy_route_disconnected");
  grid.erase({6, 2});
  auto unknown_goal = plan_occupancy_route(grid, {0.5, 2.5}, {6.5, 2.5}, cfg, {});
  assert(unknown_goal.points.empty() && unknown_goal.reason == "unsafe_viewpoint");

  // A single-frame free corridor is legal but more costly than a slightly
  // longer well-observed route. Staging must preserve that choice.
  Settings evidence_cfg;
  evidence_cfg.resolution = 1.0;
  evidence_cfg.robot_radius = 0.0;
  std::unordered_map<Cell, int, CellHash> evidence_grid;
  FreeObservationCounts evidence_frames;
  for (int x = 0; x <= 8; ++x)
    for (int y = 0; y <= 4; ++y) {
      evidence_grid[{x, y}] = 0;
      evidence_frames[{x, y}] = 3;
    }
  for (int x = 1; x <= 7; ++x) evidence_frames[{x, 2}] = 1;
  auto confident_route = plan_occupancy_route(
      evidence_grid, {0.5, 2.5}, {8.5, 2.5}, evidence_cfg, {},
      &evidence_frames);
  assert(confident_route.reason == "ok");
  assert(std::any_of(confident_route.points.begin(), confident_route.points.end(),
                     [](const XY &p) { return std::abs(p[1] - 2.5) > 0.1; }));
  const auto confident_stages = stage_occupancy_route(
      confident_route, evidence_grid, evidence_cfg, {},
      &evidence_frames);
  assert(confident_stages.size() > 1);
  assert(confident_stages.back() == XY({8.5, 2.5}));

  // Even an otherwise free grid cell is hard blocked when it has zero
  // recorded free-ray frames.
  std::unordered_map<Cell, int, CellHash> narrow_grid;
  FreeObservationCounts narrow_frames;
  for (int x = 0; x <= 3; ++x) {
    narrow_grid[{x, 0}] = 0;
    narrow_frames[{x, 0}] = 3;
  }
  narrow_frames.erase({1, 0});
  auto no_evidence_route = plan_occupancy_route(
      narrow_grid, {0.5, 0.5}, {3.5, 0.5}, evidence_cfg, {},
      &narrow_frames);
  assert(no_evidence_route.points.empty());

  // The controller can select a nearby safe observation position independently.
  grid[{3, 2}] = 0;
  grid[{6, 2}] = 0;
  grid.erase({7, 2});
  const auto viewpoints = observation_viewpoints(grid, {7.5, 2.5}, cfg, {}, 2.0);
  assert(!viewpoints.empty());
  for (const auto &point : viewpoints)
    assert(waypoint_footprint_safety(point, grid, cfg, {}) == WaypointSafety::safe);

  RecoveryState recovery(3, 2.0);
  recovery.record_failure({0.0, 0.0}, "planning_failed");
  recovery.record_failure({1.0, 0.0}, "planning_failed");
  assert(!recovery.limit_reached());
  recovery.record_failure({1.5, 0.0}, "planning_failed");
  assert(recovery.limit_reached());
  recovery.start({1.5, 0.0});
  assert(recovery.phase() == RecoveryPhase::nearby);
  recovery.mark_attempted({2.5, 0.0});
  recovery.finish();
  assert(!recovery.active() && recovery.failures() == 0);

  // Reverse may leave the current foot blind spot. A partial step can remain
  // inside the starting inflated zone if it increases clearance throughout.
  Settings reverse_cfg;
  reverse_cfg.resolution = 0.1;
  reverse_cfg.robot_radius = 0.2;
  reverse_cfg.waypoint_safety_margin = 0.05;
  std::unordered_map<Cell, int, CellHash> rear_grid;
  for (int x = -12; x <= 8; ++x)
    for (int y = -8; y <= 8; ++y) rear_grid[{x, y}] = 0;
  rear_grid.erase({0, 0});
  rear_grid[{2, 0}] = 1;  // Existing front-side overlap may be left behind.
  auto reverse = plan_recovery_reverse_route(
      rear_grid, {0.05, 0.05}, 0.0, reverse_cfg, {}, 0.35);
  assert(reverse.reason == "ok" && reverse.points.size() == 2);
  auto direct = plan_recovery_direct_route(
      rear_grid, {0.05, 0.05}, {-0.55, 0.05}, reverse_cfg, {}, 3.0);
  assert(direct.reason == "unverified_recovery_connection");
  rear_grid[{-1, 0}] = 1; // movement toward an existing rear obstacle is unsafe
  reverse = plan_recovery_reverse_route(
      rear_grid, {0.05, 0.05}, 0.0, reverse_cfg, {}, 0.35);
  assert(reverse.points.empty() && reverse.reason == "reverse_known_obstacle");
  direct = plan_recovery_direct_route(
      rear_grid, {0.05, 0.05}, {-0.55, 0.05}, reverse_cfg, {}, 3.0);
  assert(direct.points.empty() &&
         direct.reason == "known_obstacle_in_recovery_corridor");
  rear_grid[{-1, 0}] = 0;
  Settings partial_cfg = reverse_cfg;
  partial_cfg.robot_radius = 0.35;
  rear_grid[{0, 3}] = 1; // side obstacle still touches the endpoint footprint
  reverse = plan_recovery_reverse_route(
      rear_grid, {0.05, 0.05}, 0.0, partial_cfg, {}, 0.35);
  assert(reverse.reason == "ok" && reverse.points.size() == 2);
  rear_grid[{0, 3}] = 0;
  rear_grid.erase({-4, 0});
  reverse = plan_recovery_reverse_route(
      rear_grid, {0.05, 0.05}, 0.0, reverse_cfg, {}, 0.35);
  assert(reverse.points.empty() &&
         reverse.reason == "reverse_unknown_rear_corridor");
  rear_grid[{-4, 0}] = 1;
  reverse = plan_recovery_reverse_route(
      rear_grid, {0.05, 0.05}, 0.0, reverse_cfg, {}, 0.35);
  assert(reverse.points.empty() &&
         reverse.reason == "reverse_known_obstacle");
  rear_grid[{-4, 0}] = 0;
  reverse = plan_recovery_reverse_route(
      rear_grid, {0.05, 0.05}, 0.0, reverse_cfg, {{-0.30, 0.05}}, 0.35);
  assert(reverse.points.empty() &&
         reverse.reason == "reverse_scan_obstacle");
  return 0;
}
