#include "legbot_exploration/route_planner.hpp"

#include <cassert>
#include <cmath>

int main() {
  using namespace legbot_exploration;
  Settings cfg;
  cfg.resolution = 1.0;
  cfg.robot_radius = 0.0;
  std::unordered_map<Cell, int, CellHash> grid;
  for (int x = 0; x <= 5; ++x)
    for (int y = 0; y <= 4; ++y) grid[{x, y}] = 0;
  for (int y = 1; y <= 3; ++y) grid[{2, y}] = 1;
  auto detour = plan_occupancy_route(grid, {0.5, 2.5}, {5.5, 2.5}, cfg, {});
  assert(detour.reason == "ok" && detour.points.size() > 2);
  for (const auto &point : detour.points)
    assert(grid.at({static_cast<int>(std::floor(point[0])),
                    static_cast<int>(std::floor(point[1]))}) == 0);
  auto stages = stage_route(detour, grid, cfg, {}, 3.0);
  assert(stages.size() >= 1 && stages.size() <= 3);
  assert(std::abs(stages.back()[0] - 5.5) < 1e-9);
  assert(stage_route({{{0.5, 0.5}, {2.5, 0.5}}, "ok"}, grid,
                     cfg, {}, 3.0).size() == 1);
  std::unordered_map<Cell, int, CellHash> straight_grid;
  PlannedRoute straight;
  for (int x = 0; x <= 9; ++x) {
    for (int y = -1; y <= 1; ++y) straight_grid[{x, y}] = 0;
    straight.points.push_back({x + 0.5, 0.5});
  }
  PlannedRoute medium = straight;
  medium.points.resize(6);
  assert(stage_route(medium, straight_grid, cfg, {}, 3.0).size() == 2);
  assert(stage_route(straight, straight_grid, cfg, {}, 3.0).size() == 3);

  // Both route sources use the same actual-footprint check. Unknown cells
  // touching the footprint are blocked even if the waypoint centre is free.
  Settings footprint_cfg;
  footprint_cfg.resolution = 1.0;
  footprint_cfg.robot_radius = 0.55;
  footprint_cfg.waypoint_safety_margin = 0.05;
  footprint_cfg.planner_occupancy_radius = 0.35;
  std::unordered_map<Cell, int, CellHash> footprint_grid;
  for (int x = -3; x <= 3; ++x)
    for (int y = -3; y <= 3; ++y) footprint_grid[{x, y}] = 0;
  const XY centre{0.5, 0.5};
  assert(waypoint_footprint_safety(centre, footprint_grid, footprint_cfg, {}) ==
         WaypointSafety::safe);
  footprint_grid.erase({1, 0});
  assert(waypoint_footprint_safety(centre, footprint_grid, footprint_cfg, {}) ==
         WaypointSafety::unknown_footprint);
  footprint_grid[{1, 0}] = 1;
  assert(waypoint_footprint_safety(centre, footprint_grid, footprint_cfg, {}) ==
         WaypointSafety::occupied_footprint);
  footprint_grid[{1, 0}] = 0;
  assert(waypoint_footprint_safety(centre, footprint_grid, footprint_cfg,
                                    {{0.75, 0.5}}) ==
         WaypointSafety::scan_obstacle);
  footprint_grid.erase({2, 0});
  assert(stage_route({{centre, {1.5, 0.5}}, "ok"}, footprint_grid,
                     footprint_cfg, {}, 1.5).empty());
  footprint_grid[{2, 0}] = 0;
  auto sparse_stages = stage_route(
      {{centre, {1.5, 0.5}, {2.5, 0.5}}, "ok"}, footprint_grid,
      footprint_cfg, {}, 0.75);
  assert(!sparse_stages.empty() && sparse_stages.size() <= 3);
  for (const auto &point : sparse_stages)
    assert(waypoint_footprint_safety(point, footprint_grid,
                                      footprint_cfg, {}) == WaypointSafety::safe);

  // The startup position may be unobserved, but the selected destination
  // itself must have a fully observed and inflated-free footprint.
  auto startup_grid = footprint_grid;
  startup_grid.erase({0, 0});
  const auto startup = startup_viewpoints(startup_grid, centre, footprint_cfg,
                                          {}, 1.0, 2.0);
  assert(!startup.empty());
  for (const auto &point : startup)
    assert(waypoint_footprint_safety(point, startup_grid, footprint_cfg, {}) ==
           WaypointSafety::safe);
  startup_grid[{1, 0}] = 1;
  for (const auto &point : startup_viewpoints(startup_grid, centre,
                                               footprint_cfg, {}, 1.0, 2.0))
    assert(waypoint_footprint_safety(point, startup_grid,
                                      footprint_cfg, {}) == WaypointSafety::safe);

  // A frontier cell remains unknown; the executable path must stop on free ground.
  auto frontier = plan_occupancy_route(grid, {0.5, 2.5}, {6.5, 2.5}, cfg, {});
  assert(frontier.reason == "ok" && frontier.points.back()[0] == 5.5);
  grid[{0, 2}] = 1;
  auto blocked = plan_occupancy_route(grid, {0.5, 2.5}, {5.5, 2.5}, cfg, {});
  assert(blocked.reason == "ok" && blocked.points.back()[0] == 5.5);

  // A nearby pillar may inflate over the robot's current cell. It must not
  // prevent selection of an independently checked safe destination.
  Settings pillar_cfg;
  pillar_cfg.resolution = 0.1;
  pillar_cfg.robot_radius = 0.35;
  std::unordered_map<Cell, int, CellHash> pillar_grid;
  for (int x = -10; x <= 20; ++x)
    for (int y = -10; y <= 10; ++y) pillar_grid[{x, y}] = 0;
  pillar_grid[{10, 0}] = 1;
  auto away_from_pillar = plan_occupancy_route(
      pillar_grid, {0.75, 0.05}, {0.05, 0.05}, pillar_cfg, {});
  assert(away_from_pillar.points.size() >= 2);
  auto safe_targets = stage_route(away_from_pillar, pillar_grid,
                                  pillar_cfg, {}, 3.0);
  assert(safe_targets.size() == 1);
  assert(waypoint_footprint_safety(safe_targets.back(), pillar_grid,
                                    pillar_cfg, {}) == WaypointSafety::safe);

  // A disconnected reference grid may still contain a safe target. SCAN
  // decides whether it can actually move there.
  for (int y = -10; y <= 10; ++y) pillar_grid[{5, y}] = 1;
  auto disconnected = plan_occupancy_route(
      pillar_grid, {0.75, 0.05}, {0.05, 0.05}, pillar_cfg, {});
  assert(disconnected.reason == "direct_safe_viewpoint");
  assert(waypoint_footprint_safety(disconnected.points.back(), pillar_grid,
                                    pillar_cfg, {}) == WaypointSafety::safe);

  std::vector<LayerSurface> surfaces;
  for (int x = 0; x < 5; ++x) {
    LayerSurface s;
    s.x_cell = x; s.y_cell = 0; s.height = 0.0;
    s.kind = SurfaceKind::flat;
    s.clearance = ClearanceKind::observed_clear;
    s.support_columns = 6; s.hit_observations = 3;
    surfaces.push_back(s);
  }
  auto surface = plan_surface_route(surfaces, {0.05, 0.05, 0.32}, {0.45, 0.05},
                                    0.1, 0.32);
  assert(surface.reason == "ok" && surface.points.size() == 5);
  surfaces[2].clearance = ClearanceKind::blocked;
  assert(route_conflicts_with_layer(surface, surfaces, 0.1, 0.0, 0.05));
  surfaces[2].clearance = ClearanceKind::observed_clear;
  surfaces[2].support_columns = 0;
  assert(route_conflicts_with_layer(surface, surfaces, 0.1, 0.0, 0.05));
  surfaces[2].support_columns = 6;
  assert(!route_conflicts_with_layer(surface, surfaces, 0.1, 0.0, 0.05));
  surfaces[2].clearance = ClearanceKind::blocked;
  surface = plan_surface_route(surfaces, {0.05, 0.05, 0.32}, {0.45, 0.05},
                               0.1, 0.32);
  assert(surface.reason == "surface_route_disconnected");

  // An unknown frontier can still be observed from a free interior point;
  // its execution endpoint must retain the extra 0.15 m obstacle margin.
  std::unordered_map<Cell, int, CellHash> viewpoint_grid;
  for (int x = 0; x <= 16; ++x)
    for (int y = -16; y <= 16; ++y) viewpoint_grid[{x, y}] = 0;
  viewpoint_grid[{10, 0}] = 1;
  viewpoint_grid.erase({14, 0});
  Settings viewpoint_cfg;
  viewpoint_cfg.resolution = 0.1;
  viewpoint_cfg.robot_radius = 0.35;
  auto viewpoint = plan_occupancy_route(
      viewpoint_grid, {0.05, 0.05}, {1.45, 0.05}, viewpoint_cfg, {});
  assert(viewpoint.reason == "ok");
  assert(std::hypot(viewpoint.points.back()[0] - 1.05,
                    viewpoint.points.back()[1] - 0.05) >= 0.50 - 1e-9);
}
