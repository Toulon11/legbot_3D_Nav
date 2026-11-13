#pragma once

#include "legbot_exploration/single_floor_exploration.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace legbot_exploration {

enum class SafetyIssue : std::size_t {
  invalid_input, unknown, occupied, scan_obstacle, zero_free_evidence,
  no_surface, height_mismatch, insufficient_support, insufficient_hits,
  non_flat_surface, clearance_unobserved, clearance_blocked, disconnected_support, count
};
const char *safety_issue_name(SafetyIssue issue);

// Counts are independent: an occupied column may also lack a walking surface.
struct PointSafetyResult {
  bool safe = true;
  bool evaluated = false;
  std::uint64_t map_version = 0;
  Point position{};
  Cell cell{};
  double surface_height = 0.0;
  std::size_t checked_cells = 0;
  std::array<std::size_t, static_cast<std::size_t>(SafetyIssue::count)> rejected{};
  void reject(SafetyIssue issue) {
    safe = false;
    evaluated = true;
    ++rejected[static_cast<std::size_t>(issue)];
  }
  std::string reason() const;
};
struct SegmentSafetyResult {
  bool safe = true;
  PointSafetyResult failure;
};
struct StartAdmission {
  bool ready = false;
  XY anchor{};
  PointSafetyResult actual, node;
  SegmentSafetyResult connection;
  std::string stage;  // actual_position / start_node / start_connection / ready
  bool physical_conflict=false;
  std::string reason; // Backend-specific admission cause, preserved for control policy.
};
std::string safety_json(const PointSafetyResult &result);
std::string admission_json(const StartAdmission &result);

struct OccupancyMapView {
  const std::unordered_map<Cell, int, CellHash> &grid;
  const Settings &settings;
  const std::vector<XY> &scan_obstacles;
  const FreeObservationCounts *free_observations = nullptr;
  std::uint64_t version = 0;
};
PointSafetyResult point_safe_check(const XY &point, const OccupancyMapView &map);
SegmentSafetyResult segment_safe_check(const XY &a, const XY &b,
                                       const OccupancyMapView &map);
StartAdmission check_start(const XY &position, const OccupancyMapView &map);


}  // namespace legbot_exploration
