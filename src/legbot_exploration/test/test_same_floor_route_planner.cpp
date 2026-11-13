#include "legbot_exploration/same_floor_route_planner.hpp"
#include "legbot_exploration/recovery_planner.hpp"
#include "legbot_exploration/exploration_candidate.hpp"

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

  const std::vector<XY> no_scan_points;
  const OccupancyMapView corridor_view{grid,cfg,no_scan_points};
  assert(!segment_safe_check({0.5,2.5},{6.5,2.5},corridor_view).safe);

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

  // Ordinary admission checks the actual footprint AND its grid connection.
  Settings admission_cfg;
  admission_cfg.resolution=.1;admission_cfg.robot_radius=.01;admission_cfg.waypoint_safety_margin=0;
  std::unordered_map<Cell,int,CellHash> admission_grid;
  for(int x=-12;x<=8;++x) for(int y=-8;y<=8;++y) admission_grid[{x,y}]=0;
  FreeObservationCounts frames;
  for (const auto &[cell,state]:admission_grid) if(state==0) frames[cell]=3;
  frames.erase({0,0});
  const std::vector<XY> empty_obstacles;
  const OccupancyMapView evidence_view{admission_grid,admission_cfg,empty_obstacles,&frames,71};
  const auto evidence_check=point_safe_check({0.05,0.05},evidence_view);
  assert(!evidence_check.safe && evidence_check.map_version==71);
  assert(evidence_check.rejected[static_cast<std::size_t>(SafetyIssue::zero_free_evidence)]>0);
  frames[{0,0}]=3;
  assert(check_start({0.05,0.05},evidence_view).ready);
  // The physical pose can pass while its grid-centre anchor fails. Recovery
  // completion must use admission, not just the physical pose's disk result.
  Settings anchor_cfg;anchor_cfg.resolution=1.0;anchor_cfg.robot_radius=0.55;
  anchor_cfg.waypoint_safety_margin=0;
  std::unordered_map<Cell,int,CellHash> anchor_grid;
  for(int x=0;x<=3;++x) for(int y=-2;y<=2;++y) anchor_grid[{x,y}]=0;
  const OccupancyMapView anchor_view{anchor_grid,anchor_cfg,no_scan_points};
  assert(point_safe_check({0.8,0.5},anchor_view).safe);
  const auto incomplete_admission=check_start({0.8,0.5},anchor_view);
  assert(!incomplete_admission.ready && incomplete_admission.stage=="start_node");
  assert(check_start({1.5,0.5},anchor_view).ready);
  return 0;
}
