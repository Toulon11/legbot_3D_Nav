#pragma once

#include "legbot_exploration/cross_layer_task.hpp"
#include "legbot_exploration/recovery_planner.hpp"
#include "legbot_exploration/route_orchestrator.hpp"

#include <algorithm>
#include <map>
#include <set>

namespace legbot_exploration {

class ExplorationPolicy {
 public:
  explicit ExplorationPolicy(CrossLayerTask task) : cross_task_(std::move(task)) {}
  void schedule_global();
  void enter_floor(double ground);
  void finish_floor_selection_wait(double now);
  FloorViewLimits frontier_view_limits(FloorViewLimits limits, bool ascending,
                                       double max_step) const;

  struct RankedFrontiers {
    std::vector<std::pair<double, PctFrontier>> candidates;
    std::size_t considered = 0;
    std::size_t suppressed = 0;
  };
  RankedFrontiers rank_frontiers(double current, const Point &ground,
                                std::optional<double> yaw, const Settings &settings,
                                double direction_weight);
  const char *check_ascent(const Point &start, const std::vector<Point> &route,
                           std::optional<double> departure_height, bool starts_climb) const;
  bool adopt_ascent(const Point &start, const std::vector<Point> &route,
                    std::optional<double> departure_height, std::optional<double> yaw);

 private:
  friend class ExplorationController;
  CrossLayerTask cross_task_;
  bool pct_global_fallback_=false;
  std::size_t pct_local_fallback_count_=0;
  std::size_t pct_failed_local_rounds_=0,cross_filtered_observers_=0;
  double pct_local_failure_since_=0.,cross_deferred_evidence_time_=0.;
  std::optional<std::uint64_t> cross_deferred_signature_;
  std::optional<Point> cross_deferred_pose_;
  bool cross_deferred_checked_=false,cross_deferred_scan_blocked_=false,cross_deferred_feet_stable_=false;
  bool floor_complete_=false;
  std::uint64_t floor_work_checked_version_=0;
  std::optional<double> floor_selection_wait_started_;
  double floor_longest_selection_wait_ms_=0.;
  std::optional<double> floor_review_started_;
  double floor_review_elapsed_=0.;
  std::size_t floor_review_resolved_=0,floor_known_barriers_=0;
  std::size_t floor_number_=0;
  std::optional<double> floor_height_;
  std::string stair_climb_result_,stair_step_result_;
  std::string floor_confirmation_reason_="no_contact_data";
  std::map<std::uint64_t,double> pct_timeout_pending_;
  std::map<Point,double> pct_timed_out_observers_;
  std::uint64_t pct_next_candidate_=0;
  std::set<Point> failed_stair_observers_;
  std::uint64_t failed_stair_version_=0;
  PctFrontiers pct_frontiers_;
  std::string stair_probe_last_diagnostic_;
  bool local_stair_probe_active_=false;
  struct FloorAscent { Point start; double yaw; };
  std::optional<FloorAscent> floor_ascent_;
  StairScanBudget stair_scan_budget_;
  double pct_floor_wait_for_frame_after_=-1.;
  struct PctCooldown { Point xyz; double expiry; };
  std::vector<PctCooldown> pct_cooldowns_;
};

class RecoveryController {
 public:
  struct StairReposition {
    enum class Stage { reverse, forward, shift, review, blocked };
    Point start;
    PctFrontier forward_goal;
    double heading;
    double queued_at;
    std::optional<double> stopped_at;
    Point action_start{};
    XY action_target{};
    double action_yaw=0.;
    std::optional<double> tread_height;
    Stage stage=Stage::reverse;
    bool partial_reverse=false, shifted=false;
    std::string failure;
    Point blocked_pose{};
    std::array<bool,5> blocked_checks{};
  };
  struct BlockerRecovery {
    struct RetryWait {
      Point pose;
      std::optional<std::uint64_t> geometry, nearby;
      double support_height=0.;
      bool stable_contacts=false;
    };
    PctBlockerRecovery planner;
    PctFrontier goal;
    std::vector<PctUnknownBlocker> blockers;
    std::optional<PctRecoveryAction> action;
    std::optional<Point> stopped_pose;
    std::optional<RetryWait> retry_wait;
    std::optional<XY> platform, motion_height_band;
    Point motion_start{};
    double motion_yaw=0., result_at=0.;
    bool cross_layer=false, replan=false,platform_shift=false;
    bool unsafe_start=false, delegation_started=false;
    int floor_attempt=0;
    const char *floor_failure="floor_disconnected";
    std::string selection_policy;
  };

  void reset();
  void postpone(BlockerRecovery task) {
    // Selection already charged the failed attempt. Recovery must not charge
    // it again; refreshed records retain the same physical pending task.
    task.floor_attempt=0;
    const auto same=std::find_if(pending_blocker_recoveries_.begin(),pending_blocker_recoveries_.end(),
        [&](const auto &old) {
          const bool same_blockers=!task.blockers.empty() && old.blockers.size()==task.blockers.size() &&
              std::all_of(task.blockers.begin(),task.blockers.end(),[&](const auto &blocker) {
                return std::any_of(old.blockers.begin(),old.blockers.end(),[&](const auto &previous) {
                  return std::hypot(blocker.point[0]-previous.point[0],blocker.point[1]-previous.point[1])<.01 &&
                      std::abs(blocker.point[2]-previous.point[2])<.05;
                });
              });
          return old.platform==task.platform && ((task.goal.id && old.goal.id==task.goal.id) ||
              (same_blockers &&
               std::hypot(task.goal.frontier[0]-old.goal.frontier[0],task.goal.frontier[1]-old.goal.frontier[1])<.20 &&
               std::abs(task.goal.frontier[2]-old.goal.frontier[2])<.05));
        });
    if(same==pending_blocker_recoveries_.end())pending_blocker_recoveries_.push_back(std::move(task));
    else {same->goal=std::move(task.goal);same->blockers=std::move(task.blockers);}
  }
  std::optional<BlockerRecovery> take_pending() {
    if(pending_blocker_recoveries_.empty())return std::nullopt;
    auto task=std::move(pending_blocker_recoveries_.front());
    pending_blocker_recoveries_.erase(pending_blocker_recoveries_.begin());
    return task;
  }
  void clear_pending() {pending_blocker_recoveries_.clear();}
  void restore_attempts(const std::optional<Point> &position, bool may_reuse, BlockerRecovery &task);
  void save_attempts(const Point &position, PctBlockerRecovery planner);
  bool complete_motion(std::optional<Point> stop, double time, bool delegated);
  void complete_scan(bool success);
  std::optional<PctRecoveryAction> next_action(
      const PlanningSnapshot &snapshot, std::uint64_t version, const Point &attitude,
      const PctRecoveryConstraints &constraints, bool can_move,
      const std::function<bool(const Point &)> &stance_clear,
      const std::vector<Point> &review, double stop_tolerance);

 private:
  friend class ExplorationController;
  std::string recovery_last_failure_;
  double recovery_last_target_distance_=0.;
  std::optional<StairReposition> stair_reposition_;
  std::optional<BlockerRecovery> blocker_recovery_;
  std::optional<std::pair<Point,PctBlockerRecovery>> deferred_blocker_recovery_;
  std::vector<BlockerRecovery> pending_blocker_recoveries_;
};

class RoutePlanner {
 public:
  struct Viewpoints {
    std::vector<XY> xy;
    std::vector<PctObservationPose> layer;
    std::string layer_error;
  };

  Viewpoints find_viewpoints(const PlanningSnapshot &snapshot, const XY &target,
                            const PctFrontier *frontier, bool recovery,
                            bool use_occupancy, bool use_layer, const Point &attitude) const;
  struct Plans {
    std::vector<PlannerResult> candidates;
    std::optional<std::size_t> selected_index;
    const PlannerResult *selected() const {
      return selected_index ? &candidates[*selected_index] : nullptr;
    }
  };
  Plans plan(const PlanningGoal &goal, const PlanningSnapshot &snapshot) const;
};

}  // namespace legbot_exploration
