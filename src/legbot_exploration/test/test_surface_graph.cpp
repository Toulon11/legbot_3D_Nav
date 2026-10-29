#include "legbot_exploration/surface_graph.hpp"

#include <cmath>
#include <iostream>
#include <vector>

namespace le = legbot_exploration;

namespace {
le::LayerSurface surface(int x, double height, le::SurfaceKind kind,
                         le::ClearanceKind clearance = le::ClearanceKind::observed_clear) {
  le::LayerSurface s;
  s.x_cell = x;
  s.y_cell = 0;
  s.height = height;
  s.kind = kind;
  s.clearance = clearance;
  s.hit_observations = 3;
  s.support_columns = 8;
  return s;
}
bool check(bool condition, const char *message) {
  if (!condition) std::cerr << message << '\n';
  return condition;
}
}  // namespace

int main() {
  le::SurfaceGraphConfig config;
  config.search_radius = 4.0;
  const le::LayerPoint base{0.05, 0.05, 0.32};
  std::vector<le::LayerSurface> steps{
      surface(0, 0.0, le::SurfaceKind::flat),
      surface(1, 0.0, le::SurfaceKind::flat),
      surface(2, 0.1, le::SurfaceKind::step_candidate),
      surface(3, 0.2, le::SurfaceKind::step_candidate),
      // A second surface in the same XY column must not connect vertically.
      surface(0, 2.0, le::SurfaceKind::step_candidate)};
  const auto route = le::search_rising_surface(steps, base, config);
  if (!check(route.state == "preview_path_observed", "expected observed A* path") ||
      !check(route.path.size() == 3, "expected ground-ground-step route") ||
      !check(std::abs(route.path.front().z) < 0.001, "start selected the wrong floor") ||
      !check(std::abs(route.path.back().z - 0.1) < 0.001,
             "route should end at a reachable rising step")) return 1;

  // Removing the only approach cell must not turn a far candidate into a path.
  steps[1].clearance = le::ClearanceKind::blocked;
  const auto blocked = le::search_rising_surface(steps, base, config);
  if (!check(blocked.state == "rising_candidates_unreachable_in_window",
             "path crossed a blocked cell") ||
      !check(blocked.path.empty(), "blocked route should have no path")) return 1;

  // An unknown ceiling is represented honestly: preview exists, but its
  // clearance is not reported as confirmed.
  steps[1].clearance = le::ClearanceKind::unobserved;
  const auto provisional = le::search_rising_surface(steps, base, config);
  if (!check(provisional.state == "preview_path_clearance_unobserved",
             "unknown clearance was treated as confirmed")) return 1;

  // A large height jump between adjacent columns is not a staircase edge.
  steps[2].height = 0.4;
  steps[3].height = 0.5;
  const auto jump = le::search_rising_surface(steps, base, config);
  if (!check(jump.state == "rising_candidates_unreachable_in_window",
             "graph accepted a step higher than the configured limit")) return 1;

  // RViz may show only one candidate. The search must still consider a more
  // distant reachable stair when the nearest yellow cell is disconnected.
  config.max_preview_candidates = 1;
  std::vector<le::LayerSurface> many;
  for (int x = 0; x <= 20; ++x)
    many.push_back(surface(x, x == 20 ? 0.10 : 0.0,
                           x == 20 ? le::SurfaceKind::step_candidate :
                                     le::SurfaceKind::flat));
  many.push_back(surface(1, 0.30, le::SurfaceKind::step_candidate));
  const auto farther = le::search_rising_surface(many, base, config);
  if (!check(farther.path.size() == 21, "preview limit hid a reachable stair") ||
      !check(farther.rising_candidates == 2, "candidate count was truncated")) return 1;
  return 0;
}
