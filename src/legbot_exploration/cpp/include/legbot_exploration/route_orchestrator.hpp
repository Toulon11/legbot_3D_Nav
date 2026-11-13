#pragma once

#include "legbot_exploration/same_floor_route_planner.hpp"
#include "legbot_exploration/exploration_candidate.hpp"
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
enum class PlannerKind { occupancy, layer, recovery_layer, scan_direct, observation_turn };
enum class PlannerFailureClass { none, unsafe_start, unsafe_goal, disconnected, timeout, unavailable, other };
enum class RecoveryRequest { position, reverse, shift, turn, delegate };
const char *recovery_request_name(RecoveryRequest request);

struct PlanningGoal {
  std::uint64_t goal_id = 0;
  std::uint64_t attempt_id = 0;
  std::size_t occupancy_version = 0;
  std::size_t layer_version = 0;
  ExplorationMode mode = ExplorationMode::flat;
  XY target{};
  XY observation{};
  std::optional<Point> observation_xyz;  // Explicit body-reference XYZ for PCT targets.
  std::optional<double> observation_yaw;
  std::optional<XY> pct_floor; // Latched reference G and the allowed height deviation.
  bool request_occupancy = true;
  bool request_layer = true;
  RecoveryRequest recovery_request = RecoveryRequest::position;
  bool collect_unknown_blockers = false;
  bool stair_segment = false;
  double recovery_yaw = 0.0, recovery_turn_angle = 0.0;
  std::optional<double> xy_tolerance; // Requested only on the final position waypoint.
  ExplorationCandidate candidate;
};

struct PlannerResult {
  PlanningGoal goal;
  PlannerKind kind = PlannerKind::occupancy;
  PlannedRoute route;
  std::vector<XY> waypoints;
  std::vector<Point> xyz;  // PCT geometry, preserved through execution.
  std::optional<double> start_ground_z;
  std::vector<PctUnknownBlocker> unknown_blockers;
  std::string backend_metadata = "{}";
  std::string source;
  std::string reason;
  PlannerFailureClass failure_class = PlannerFailureClass::none;
  double planning_ms = 0.0;

  bool usable() const {
    return !waypoints.empty() && ((kind==PlannerKind::observation_turn && goal.recovery_request==RecoveryRequest::turn && waypoints.size()==1) || route.points.size()>=2 ||
        ((kind==PlannerKind::layer || kind==PlannerKind::recovery_layer) && route.points.size()==1 && xyz.size()==1) ||
        (kind==PlannerKind::scan_direct && goal.recovery_request==RecoveryRequest::delegate && goal.mode==ExplorationMode::recovery && xyz.size()==1));
  }
};

struct PlanningSnapshot {
  const std::unordered_map<Cell, int, CellHash> &grid;
  const std::vector<XY> &scan_obstacles;
  const Settings &settings;
  Point base;
  double body_height = 0.32;
  bool layer_fresh = false;
  const FreeObservationCounts *free_observations = nullptr;
  SharedLayerMap *shared_layer = nullptr;
  std::uint64_t occupancy_version = 0;
  const std::vector<XY> *scan_raw_obstacles = nullptr;
};

struct ExecutionResult {
  std::uint64_t goal_id = 0;
  std::uint64_t attempt_id = 0;
  std::size_t stage_index = 0;
  std::int64_t scan_goal_stamp = 0;
  std::string source;
  std::string reason;
  bool success = false;
  std::optional<double> requested_xy_tolerance;
  std::optional<Point> actual_position;
  std::optional<double> goal_distance;
};

// Planners return candidate routes only. They never publish SCAN goals.
std::vector<PlannerResult> plan_candidates(
    const PlanningGoal &goal, const PlanningSnapshot &snapshot);

struct StartChecks {
  std::optional<StartAdmission> occupancy, layer;
  bool ready() const {
    return (occupancy && occupancy->ready) || (layer && layer->ready);
  }
  bool blind() const;
  bool layer_physical_conflict() const {return layer && layer->physical_conflict;}
};
StartChecks check_normal_starts(const PlanningSnapshot &snapshot,
                               bool occupancy, bool layer,std::optional<XY> floor=std::nullopt);
// Shared geometry backends; recovery requests enter through the recovery module.
std::vector<PlannerResult> plan_map_candidates(
    const PlanningGoal &goal, const PlanningSnapshot &snapshot);
std::vector<PlannerResult> plan_recovery_request(
    const PlanningGoal &goal, const PlanningSnapshot &snapshot);

struct ExecutionDispatch {
  RecoveryRequest request = RecoveryRequest::position;
  XY target{};
  std::optional<double> z;
  std::string action_payload;
  std::optional<double> xy_tolerance;
  std::vector<Point> reference_points;
};

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
      std::int64_t stamp, bool success, const std::string &reason,
      std::optional<Point> actual_position = std::nullopt);
  std::optional<ExecutionDispatch> dispatch(std::int64_t stamp, const Point &base) const;
  std::size_t skip_within(const XY &position, double radius);
  bool is_last() const { return active() && index_ + 1 == waypoints_.size(); }
  const XY &current() const { return waypoints_.at(index_); }
  const std::vector<XY> &waypoints() const { return waypoints_; }
  const std::vector<Point> &xyz() const { return xyz_; }
  std::optional<double> current_z() const {
    return active() && xyz_.size()==waypoints_.size()?std::optional<double>(xyz_[index_][2]):std::nullopt;
  }
  std::size_t index() const { return index_; }
  const std::string &source() const { return source_; }
  bool recovery() const { return recovery_; }
  std::uint64_t goal_id() const { return goal_id_; }
  std::uint64_t attempt_id() const { return attempt_id_; }

 private:
  std::vector<XY> waypoints_;
  std::vector<Point> xyz_;
  std::vector<std::vector<Point>> references_;
  std::optional<XY> shift_start_;
  std::size_t index_ = 0;
  std::string source_ = "none";
  bool recovery_ = false;
  std::uint64_t goal_id_ = 0;
  std::uint64_t attempt_id_ = 0;
  std::optional<std::int64_t> scan_goal_stamp_;
  bool execution_resolved_ = false;
  bool preserve_geometry_ = false;
  RecoveryRequest request_ = RecoveryRequest::position;
  double yaw_ = 0.0, turn_angle_ = 0.0;
  std::optional<double> final_xy_tolerance_;
};

const char *mode_name(ExplorationMode mode);
const char *planner_name(PlannerKind kind);
const char *planner_failure_name(PlannerFailureClass failure);

}  // namespace legbot_exploration
