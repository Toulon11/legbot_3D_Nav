#pragma once

#include "legbot_exploration/same_floor_route_planner.hpp"
#include "legbot_exploration/pct_backend.hpp"

#include <functional>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace legbot_exploration {

struct PlanningSnapshot;

struct PctRecoveryConstraints {
  std::optional<XY> floor;
  std::optional<double> ascent;
  std::vector<Point> contacts;
  std::function<bool(const Point &)> permits_goal;
  std::function<bool(const Point &)> permits_scan;
};

struct PctRecoveryAction {
  enum class Kind { scan, shift, reverse, delegate };
  Kind kind;
  Point position;
  Point gap{};
  double yaw=0.;
};

std::optional<double> verified_short_recovery_stop(
    const Point &start,const XY &target,const Point &stop,double yaw_error,bool partial_reverse=false);
std::vector<XY> stair_recovery_repositions(
    PctBackend &map,std::uint64_t version,const Point &base,const Point &forward,double yaw,double ascent,
    double tread_height,double max_step,bool reverse,
    const std::function<bool(const Point &,double)> &scan_inflated);
std::optional<PctRecoveryAction> plan_pct_landing_shift(
    const PlanningSnapshot &snapshot,std::uint64_t version,const XY &departure_floor,
    double tread_height,double ascent);

// Select one action per call. Execution ownership and frontier accounting
// stay with the caller, which refreshes blockers after each completed scan.
class PctBlockerRecovery {
 public:
  std::optional<PctRecoveryAction> next(
      const PlanningSnapshot &snapshot, std::uint64_t version, const Point &attitude,
      const std::vector<PctUnknownBlocker> &blockers, const PctRecoveryConstraints &constraints,
      bool can_move, const std::function<bool(const Point &)> &stance_clear,
      const std::vector<Point> &review={});
  std::optional<PctRecoveryAction> delegate(
      const PlanningSnapshot &snapshot, std::uint64_t version,
      const PctRecoveryConstraints &constraints, const std::vector<Point> &goals,
      double arrival_tolerance);
  void retry_delegates() { delegated_.clear(); }
  std::optional<double> scan_heading(
      PctBackend &map, std::uint64_t version, const Point &position, const Point &attitude,
      const Point &gap, const std::vector<PctUnknownBlocker> &blockers, double body_height,
      const PctRecoveryConstraints &constraints,bool review=false);

 private:
  std::set<std::array<long, 6>> scans_;
  std::set<std::array<long, 5>> moves_;
  std::vector<Point> delegated_;
};

}  // namespace legbot_exploration
