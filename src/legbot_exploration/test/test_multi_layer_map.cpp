#include "legbot_exploration/multi_layer_map.hpp"

#include <cassert>
#include <cmath>
#include <vector>

namespace le = legbot_exploration;

int main() {
  le::LayerMapConfig cfg;
  cfg.min_hit_observations = 2;
  le::OnlineLayerMap map(cfg);
  std::vector<le::LayerPoint> lower, upper;
  for (int x = -2; x <= 2; ++x) {
    for (int y = -2; y <= 2; ++y) {
      lower.push_back({(x + 0.5) * 0.1, (y + 0.5) * 0.1, 0.02});
      upper.push_back({(x + 0.5) * 0.1, (y + 0.5) * 0.1, 3.02});
    }
  }
  for (int i = 0; i < 2; ++i) {
    assert(map.integrate({0.05, 0.05, 0.8}, lower) == lower.size());
    assert(map.integrate({0.05, 0.05, 3.8}, upper) == upper.size());
  }
  map.rebuild_dirty();
  auto surfaces = map.surfaces();
  bool low_found = false, high_found = false;
  for (const auto &surface : surfaces) {
    if (surface.x_cell != 0 || surface.y_cell != 0) continue;
    if (std::abs(surface.height - 0.02) < 0.01 &&
        surface.kind == le::SurfaceKind::flat) low_found = true;
    if (std::abs(surface.height - 3.02) < 0.01 &&
        surface.kind == le::SurfaceKind::flat) high_found = true;
  }
  assert(low_found && high_found);

  // A ceiling seen only from below must not turn into a walkable upper layer.
  for (int i = 0; i < 2; ++i)
    assert(map.integrate({0.05, 0.05, 0.15}, {{0.05, 0.05, 0.42}}) == 1);
  map.rebuild_dirty();
  for (const auto &surface : map.surfaces())
    assert(std::abs(surface.height - 0.42) >= 0.01);
  bool vertical_return_preserved = false;
  for (const auto &point : map.occupied_voxels())
    if (std::abs(point.x - 0.05) < 0.01 &&
        std::abs(point.y - 0.05) < 0.01 &&
        std::abs(point.z - 0.45) < 0.01)
      vertical_return_preserved = true;
  assert(vertical_return_preserved);
  std::size_t indexed_occupied = 0;
  map.for_each_occupied([&](const le::LayerPoint &) { ++indexed_occupied; });
  assert(indexed_occupied == map.occupied_voxels().size());
  std::size_t indexed_surfaces = 0;
  map.for_each_surface([&](const le::LayerSurface &) { ++indexed_surfaces; });
  assert(indexed_surfaces == map.surfaces().size());
  bool ceiling_blocks_lower = false;
  for (const auto &surface : map.surfaces())
    if (surface.x_cell == 0 && surface.y_cell == 0 && surface.height < 0.1 &&
        surface.clearance == le::ClearanceKind::blocked)
      ceiling_blocks_lower = true;
  assert(ceiling_blocks_lower);

  // A low obstacle at the robot's column blocks the lower surface.
  for (int i = 0; i < 2; ++i)
    assert(map.integrate({0.05, 0.05, 0.8}, {{0.05, 0.05, 0.32}}) == 1);
  map.rebuild_dirty();
  bool blocked = false;
  for (const auto &surface : map.surfaces())
    if (surface.x_cell == 0 && surface.y_cell == 0 && surface.height < 0.1 &&
        surface.clearance == le::ClearanceKind::blocked) blocked = true;
  assert(blocked);

  // Repeated rays to the ground clear an obstacle that has been removed.
  for (int i = 0; i < 12; ++i)
    assert(map.integrate({0.05, 0.05, 0.8}, {{0.05, 0.05, 0.02}}) == 1);
  map.rebuild_dirty();
  bool cleared = false;
  for (const auto &surface : map.surfaces())
    if (surface.x_cell == 0 && surface.y_cell == 0 && surface.height < 0.1 &&
        surface.clearance != le::ClearanceKind::blocked) cleared = true;
  assert(cleared);
  for (const auto &point : map.occupied_voxels())
    assert(!(std::abs(point.x - 0.05) < 0.01 &&
             std::abs(point.y - 0.05) < 0.01 &&
             std::abs(point.z - 0.35) < 0.01));

  // A lone high return is visible as a candidate, but not a supported floor.
  for (int i = 0; i < 2; ++i)
    assert(map.integrate({0.05, 0.05, 2.8}, {{0.05, 0.05, 2.02}}) == 1);
  map.rebuild_dirty();
  bool lone_rejected = false;
  for (const auto &surface : map.surfaces())
    if (surface.x_cell == 0 && surface.y_cell == 0 &&
        std::abs(surface.height - 2.02) < 0.01 &&
        surface.kind == le::SurfaceKind::insufficient_support) lone_rejected = true;
  assert(lone_rejected);
}
