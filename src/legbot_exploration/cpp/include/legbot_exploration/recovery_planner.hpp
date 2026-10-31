#pragma once

#include "legbot_exploration/same_floor_route_planner.hpp"

#include <optional>
#include <string>
#include <vector>

namespace legbot_exploration {

enum class RecoveryPhase { idle, nearby, turning, settling, history, blocked };

// Tracks a physical failure episode. A different frontier in the same area
// does not erase previous no-progress evidence.
class RecoveryPlanner {
 public:
  explicit RecoveryPlanner(int failure_limit = 3, double area_radius = 2.0);
  void record_failure(const XY &position, const std::string &reason);
  bool limit_reached() const { return failures_ >= failure_limit_; }
  void start(const XY &position);
  void finish();
  bool active() const { return phase_ != RecoveryPhase::idle; }
  int failures() const { return failures_; }
  RecoveryPhase phase() const { return phase_; }
  void set_phase(RecoveryPhase phase) { phase_ = phase; }
  const std::string &reason() const { return reason_; }
  bool mark_attempted(const XY &point);
  void reset_attempts() { attempted_.clear(); }
  std::vector<XY> nearby_candidates(
      const std::unordered_map<Cell, int, CellHash> &grid, const XY &start,
      const Settings &cfg, const std::vector<XY> &scan_obstacles,
      double min_distance, double max_distance) const;
  std::vector<XY> historical_candidates(
      const std::vector<XY> &history, const XY &start,
      const std::unordered_map<Cell, int, CellHash> &grid,
      const Settings &cfg, const std::vector<XY> &scan_obstacles) const;

 private:
  bool tried(const XY &point) const;
  int failure_limit_;
  double area_radius_;
  int failures_ = 0;
  std::optional<XY> area_;
  RecoveryPhase phase_ = RecoveryPhase::idle;
  std::string reason_;
  std::vector<XY> attempted_;
};

const char *recovery_phase_name(RecoveryPhase phase);

// Only recovery may propose a short, connectivity-unverified goal from a
// blind start. Initial inflated overlap is allowed only while clearance
// increases; newly swept obstacles and unknown cells still reject the route.
// SCAN remains responsible for accepting and executing the actual motion.
PlannedRoute plan_recovery_direct_route(
    const std::unordered_map<Cell, int, CellHash> &grid,
    const XY &start, const XY &target, const Settings &cfg,
    const std::vector<XY> &scan_obstacles, double max_distance);

// A fixed-heading reverse manoeuvre. It may end inside the starting inflated
// zone when clearance improves; every newly swept cell must be observed free.
PlannedRoute plan_recovery_reverse_route(
    const std::unordered_map<Cell, int, CellHash> &grid,
    const XY &start, double yaw, const Settings &cfg,
    const std::vector<XY> &scan_obstacles, double distance);

}  // namespace legbot_exploration
