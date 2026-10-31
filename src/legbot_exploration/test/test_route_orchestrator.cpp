#include "legbot_exploration/route_orchestrator.hpp"
#include "legbot_exploration/layer_route_planner.hpp"

#include <cassert>
#include <algorithm>
#include <cmath>
#include <vector>

namespace le = legbot_exploration;

int main() {
  le::PlanningGoal goal;
  goal.goal_id = 7;
  goal.attempt_id = 3;
  goal.occupancy_version = 91;
  goal.layer_version = 12;
  goal.observation = {6.5, 0.5};
  goal.request_layer = true;
  le::PlannerResult occupancy;
  occupancy.goal = goal;
  occupancy.kind = le::PlannerKind::occupancy;
  occupancy.route = {{{0.0, 0.0}, {1.0, 0.0}}, "ok"};
  occupancy.waypoints = {{1.0, 0.0}};
  le::PlannerResult layer = occupancy;
  layer.kind = le::PlannerKind::layer;
  std::vector<le::PlannerResult> results{occupancy, layer};
  assert(le::select_plan(goal, results) == &results[1]);
  le::PathProcessor processor;
  assert(processor.load(goal, results[1], false));
  assert(processor.active() && processor.goal_id() == 7);
  processor.begin_execution();
  processor.bind_scan_goal(123456);
  assert(!processor.resolve_execution(123455, false, "global_reference_failed"));
  const auto failed = processor.resolve_execution(123456, false, "global_reference_failed");
  assert(failed && !failed->success && failed->goal_id == 7 &&
         failed->attempt_id == 3 && failed->stage_index == 0);
  assert(!processor.resolve_execution(123456, false, "global_reference_failed"));
  assert(!processor.advance() && !processor.active());
  processor.clear();
  results[1].goal.occupancy_version++;
  assert(le::select_plan(goal, results) == &results[0]);

  le::Settings cfg;
  cfg.resolution = 1.0;
  cfg.robot_radius = 0.0;
  std::unordered_map<le::Cell, int, le::CellHash> grid;
  for (int x = 0; x <= 7; ++x)
    for (int y = -2; y <= 2; ++y) grid[{x, y}] = 0;
  const std::vector<le::XY> no_obstacles;
  const std::vector<le::LayerSurface> no_surfaces;
  const le::PlanningSnapshot snapshot{grid, no_obstacles, no_surfaces, cfg,
      {1.5, 0.5, 0.32}, 0.1, 0.32, 3.0, false};
  goal.request_layer = false;
  auto planned = le::plan_candidates(goal, snapshot);
  assert(planned.size() == 1 && planned[0].usable());
  assert(planned[0].route.points.back() == goal.observation);
  le::PathProcessor occupancy_processor;
  assert(occupancy_processor.load(goal, planned[0], false, &snapshot));
  assert(occupancy_processor.waypoints().size() <= planned[0].waypoints.size());
  assert(occupancy_processor.waypoints().back() == goal.observation);
  grid.erase({1, 0});
  planned = le::plan_candidates(goal, snapshot);
  assert(planned.size() == 1 && !planned[0].usable());
  assert(planned[0].reason == "unsafe_start");
  assert(planned[0].source == "occupancy");
  goal.mode = le::ExplorationMode::recovery;
  goal.request_recovery = true;
  goal.observation = {3.5, 0.5};
  planned = le::plan_candidates(goal, snapshot);
  assert(planned.size() == 1 && planned[0].usable());
  assert(planned[0].source == "recovery_unverified");
  assert(planned[0].reason == "unverified_recovery_connection");
  assert(planned[0].strict_reason == "unsafe_start");
  assert(planned[0].direct_reason == "unverified_recovery_connection");
  for (int y = -2; y <= 2; ++y) grid[{2, y}] = 1;
  planned = le::plan_candidates(goal, snapshot);
  assert(planned.size() == 1 && !planned[0].usable());
  assert(planned[0].reason == "known_obstacle_in_recovery_corridor");
  assert(planned[0].strict_reason == "unsafe_start");
  assert(planned[0].direct_reason == "known_obstacle_in_recovery_corridor");
  goal.observation = {6.5, 0.5};
  planned = le::plan_candidates(goal, snapshot);
  assert(planned.size() == 1 && !planned[0].usable());
  assert(planned[0].strict_reason == "unsafe_start");
  assert(planned[0].direct_reason == "recovery_target_beyond_direct_limit");
  goal.observation = {1.6, 0.5};
  planned = le::plan_candidates(goal, snapshot);
  assert(planned[0].direct_reason == "recovery_target_too_close");
  goal.observation = {3.5, 0.5};
  grid.erase({3, 0});
  planned = le::plan_candidates(goal, snapshot);
  assert(planned[0].direct_reason == "recovery_destination_unknown_footprint");

  // The layer planner uses its own 3-D evidence, even if the 2-D grid is
  // entirely absent. A wholly unobserved strip blocks every route across it.
  le::Settings layer_cfg;
  layer_cfg.robot_radius = 0.20;
  layer_cfg.waypoint_safety_margin = 0.05;
  std::vector<le::LayerSurface> surfaces;
  for (int x = -5; x <= 35; ++x)
    for (int y = -5; y <= 25; ++y) {
      le::LayerSurface s;
      s.x_cell = x;
      s.y_cell = y;
      s.height = 0.0;
      s.kind = le::SurfaceKind::flat;
      s.clearance = le::ClearanceKind::observed_clear;
      s.support_columns = 8;
      s.hit_observations = 3;
      surfaces.push_back(s);
    }
  const std::vector<le::Point> empty_voxels;
  const std::unordered_map<le::Cell, int, le::CellHash> empty_grid;
  const le::PlanningSnapshot layer_snapshot{empty_grid, no_obstacles, surfaces,
      layer_cfg, {0.55, 1.05, 0.32}, 0.10, 0.32, 1.0, true, nullptr,
      &empty_voxels};
  goal.mode = le::ExplorationMode::flat;
  goal.request_occupancy = false;
  goal.request_recovery = false;
  goal.request_layer = true;
  goal.observation = {2.55, 1.05};
  planned = le::plan_candidates(goal, layer_snapshot);
  assert(planned.size() == 1 && planned[0].usable());
  assert(planned[0].source == "layer");
  assert(!planned[0].waypoints.empty());
  assert(planned[0].waypoints.back() == goal.observation);
  le::PathProcessor layer_processor;
  assert(layer_processor.load(goal, planned[0], false, &layer_snapshot));
  assert(layer_processor.waypoints() == planned[0].waypoints);
  surfaces.erase(std::remove_if(surfaces.begin(), surfaces.end(),
      [](const le::LayerSurface &s) { return s.x_cell == 15; }), surfaces.end());
  planned = le::plan_candidates(goal, layer_snapshot);
  assert(!planned[0].usable());
  surfaces.push_back([] {
    le::LayerSurface s;
    s.x_cell = 15; s.y_cell = 10; s.height = 0.0;
    s.kind = le::SurfaceKind::flat;
    s.clearance = le::ClearanceKind::observed_clear;
    s.support_columns = 8; s.hit_observations = 3;
    return s;
  }());
  planned = le::plan_candidates(goal, layer_snapshot);
  assert(!planned[0].usable());  // A one-cell bridge cannot fit the body.
  for (int y = -5; y <= 25; ++y) {
    if (y == 10) continue;
    le::LayerSurface s = surfaces.back();
    s.y_cell = y;
    surfaces.push_back(s);
  }
  const std::vector<le::Point> pillar{{1.55, 1.05, 0.30}};
  const le::PlanningSnapshot pillar_snapshot{empty_grid, no_obstacles, surfaces,
      layer_cfg, {0.55, 1.05, 0.32}, 0.10, 0.32, 1.0, true, nullptr, &pillar};
  planned = le::plan_candidates(goal, pillar_snapshot);
  assert(planned[0].usable());
  for (const auto &point : planned[0].waypoints)
    assert(std::hypot(point[0] - 1.55, point[1] - 1.05) > 0.25);

  // Shared map planning sees the same evidence as the full planning topics,
  // while keeping the map and its surface objects in one place.
  le::LayerMapConfig map_config;
  le::OnlineLayerMap live_map(map_config);
  std::vector<le::LayerPoint> ground;
  for (int x = -6; x <= 16; ++x)
    for (int y = -6; y <= 6; ++y)
      ground.push_back({(x + 0.5) * 0.1, (y + 0.5) * 0.1, 0.02});
  for (int frame = 0; frame < 3; ++frame)
    live_map.integrate({0.05, 0.05, 0.8}, ground, frame + 1.0);
  live_map.rebuild_dirty();
  assert(!live_map.surfaces().empty());
  le::SharedLayerMap shared;
  shared.map = &live_map;
  const auto full_plan = le::plan_layer_route(live_map.surfaces(),
      [&] {
        std::vector<le::Point> points;
        for (const auto &p : live_map.occupied_voxels())
          points.push_back({p.x, p.y, p.z});
        return points;
      }(), {0.05, 0.05, 0.32}, {0.85, 0.05}, 0.10, 0.32,
      layer_cfg, 1.0);
  const le::PlanningSnapshot direct_snapshot{empty_grid, no_obstacles,
      no_surfaces, layer_cfg, {0.05, 0.05, 0.32}, 0.10, 0.32, 1.0,
      true, nullptr, nullptr, &shared, 2000.0};
  goal.observation = {0.85, 0.05};
  planned = le::plan_candidates(goal, direct_snapshot);
  assert(planned.size() == 1);
  assert(planned[0].reason == full_plan.reference.reason);
  assert(planned[0].waypoints == full_plan.waypoints);
  assert(!shared.planning_active.load());
  return 0;
}
