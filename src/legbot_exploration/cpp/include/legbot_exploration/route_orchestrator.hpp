#pragma once

#include "legbot_exploration/same_floor_route_planner.hpp"
#include "legbot_exploration/multi_layer_map.hpp"
#include "legbot_exploration/shared_layer_map.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace legbot_exploration {

// One frontier decision owns one planning attempt. Map revisions identify the
// evidence used by the planners; a later revision alone does not cancel a route.
enum class ExplorationMode { waiting_initial_data, flat, cross_layer, recovery };
enum class PlannerKind { occupancy, layer, recovery };

struct PlanningGoal {
  std::uint64_t goal_id = 0;
  std::uint64_t attempt_id = 0;
  std::size_t occupancy_version = 0;
  std::size_t layer_version = 0;
  ExplorationMode mode = ExplorationMode::flat;
  XY target{};
  XY observation{};
  bool request_occupancy = true;
  bool request_layer = true;
  bool request_recovery = false;
};

struct PlannerResult {
  PlanningGoal goal;
  PlannerKind kind = PlannerKind::occupancy;
  PlannedRoute route;
  std::vector<XY> waypoints;
  std::string source;
  std::string reason;
  // Recovery first tries connected A*, then a short unverified connection.
  // Keep both outcomes so the fallback cannot hide an unsafe start.
  std::string strict_reason;
  std::string direct_reason;
  double planning_ms = 0.0;

  bool usable() const { return route.points.size() >= 2 && !waypoints.empty(); }
};

struct PlanningSnapshot {
  const std::unordered_map<Cell, int, CellHash> &grid;
  const std::vector<XY> &scan_obstacles;
  const std::vector<LayerSurface> &surfaces;
  const Settings &settings;
  Point base;
  double layer_resolution = 0.10;
  double body_height = 0.32;
  double segment_length = 3.0;
  bool layer_fresh = false;
  const FreeObservationCounts *free_observations = nullptr;
  const std::vector<Point> *layer_occupied = nullptr;
  SharedLayerMap *shared_layer = nullptr;
  double layer_planning_timeout_ms = 2000.0;
};

struct ExecutionResult {
  std::uint64_t goal_id = 0;
  std::uint64_t attempt_id = 0;
  std::size_t stage_index = 0;
  std::int64_t scan_goal_stamp = 0;
  std::string source;
  std::string reason;
  bool success = false;
};

// Planners return candidate routes only. They never publish SCAN goals.
std::vector<PlannerResult> plan_candidates(
    const PlanningGoal &goal, const PlanningSnapshot &snapshot);

// A single route processor chooses at most one candidate for an attempt.
// Revisions and IDs prevent accepting a result from another request.
const PlannerResult *select_plan(
    const PlanningGoal &goal, const std::vector<PlannerResult> &results);

// Owns the one route selected for execution. The controller may advance it
// only after feedback for the current waypoint has been accepted.
class PathProcessor {
 public:
  bool load(const PlanningGoal &goal, const PlannerResult &result, bool recovery,
            const PlanningSnapshot *snapshot = nullptr);
  void clear();
  bool active() const { return index_ < waypoints_.size(); }
  bool advance();
  void begin_execution();
  void bind_scan_goal(std::int64_t stamp);
  std::optional<ExecutionResult> resolve_execution(
      std::int64_t stamp, bool success, const std::string &reason);
  std::size_t skip_within(const XY &position, double radius);
  bool is_last() const { return active() && index_ + 1 == waypoints_.size(); }
  const XY &current() const { return waypoints_.at(index_); }
  const std::vector<XY> &waypoints() const { return waypoints_; }
  std::size_t index() const { return index_; }
  const std::string &source() const { return source_; }
  bool recovery() const { return recovery_; }
  std::uint64_t goal_id() const { return goal_id_; }
  std::uint64_t attempt_id() const { return attempt_id_; }

 private:
  std::vector<XY> waypoints_;
  std::size_t index_ = 0;
  std::string source_ = "none";
  bool recovery_ = false;
  std::uint64_t goal_id_ = 0;
  std::uint64_t attempt_id_ = 0;
  std::optional<std::int64_t> scan_goal_stamp_;
  bool execution_resolved_ = false;
};

const char *mode_name(ExplorationMode mode);
const char *planner_name(PlannerKind kind);

}  // namespace legbot_exploration
