#include "legbot_exploration/route_orchestrator.hpp"

#include <cassert>
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
  return 0;
}
