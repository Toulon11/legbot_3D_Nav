#include "exploration_controller.hpp"
#include "exploration_controller_util.hpp"

namespace legbot_exploration {

void RecoveryController::reset() {
  stair_reposition_.reset();
  blocker_recovery_.reset();
  deferred_blocker_recovery_.reset();
}

void RecoveryController::restore_attempts(const std::optional<Point> &position, bool may_reuse, BlockerRecovery &task) {
  if (!deferred_blocker_recovery_) return;
  auto &[pose, planner] = *deferred_blocker_recovery_;
  if (may_reuse && position && std::abs((*position)[2] - pose[2]) <= .10 &&
      std::hypot((*position)[0] - pose[0], (*position)[1] - pose[1]) <= .60)
    task.planner = std::move(planner);
  deferred_blocker_recovery_.reset();
}

void RecoveryController::save_attempts(const Point &position, PctBlockerRecovery planner) {
  deferred_blocker_recovery_.emplace(position, std::move(planner));
}

bool RecoveryController::complete_motion(std::optional<Point> stop, double time, bool delegated) {
  if (!blocker_recovery_ || !blocker_recovery_->action) return false;
  if (delegated && blocker_recovery_->action->kind != PctRecoveryAction::Kind::delegate) return false;
  blocker_recovery_->stopped_pose = stop;
  blocker_recovery_->result_at = time;
  return true;
}

void RecoveryController::complete_scan(bool success) {
  if (!blocker_recovery_ || !blocker_recovery_->action ||
      blocker_recovery_->action->kind != PctRecoveryAction::Kind::scan) return;
  blocker_recovery_->action.reset();
  blocker_recovery_->replan = success;
}

std::optional<PctRecoveryAction> RecoveryController::next_action(
    const PlanningSnapshot &snapshot, std::uint64_t version, const Point &attitude,
    const PctRecoveryConstraints &constraints, bool can_move,
    const std::function<bool(const Point &)> &stance_clear,
    const std::vector<Point> &review, double height_slack, double stop_tolerance) {
  auto &task = *blocker_recovery_;
  auto &pct = *snapshot.shared_layer->pct;
  auto action=task.delegation_started?std::nullopt:
      task.planner.next(snapshot,version,attitude,task.blockers,constraints,can_move,stance_clear,review);
  if(!action) {
    auto goals=task.cross_layer?std::vector<Point>{}:task.goal.observations;
    for(auto &point:goals)point[2]+=snapshot.body_height;
    if(task.unsafe_start || task.cross_layer) {
      auto alternatives=pct.recovery_targets(version, snapshot.base);
      for(auto &point:alternatives)point[2]+=snapshot.body_height;
      if(task.cross_layer) {
        const auto support=pct.point_check(version, snapshot.base,false,std::nullopt,0.,false,std::nullopt,true);
        const double height=support.ground_z.value_or(snapshot.base[2]-snapshot.body_height);
        alternatives.erase(std::remove_if(alternatives.begin(),alternatives.end(),[&](const auto &p) {
          return std::abs(p[2]-snapshot.body_height-height)>height_slack;
        }),alternatives.end());
      }
      goals.insert(goals.end(),alternatives.begin(),alternatives.end());
    }
    action=task.planner.delegate(snapshot,version,constraints,goals,stop_tolerance);
  }
  return action;
}

void ExplorationController::cancel_recovery_action() {
  if (!active_ || !active_->stamp || (!active_->turn && !active_->reverse)) return;
  std_msgs::msg::String request;
  request.data = std::to_string(*active_->stamp);
  output_.recovery_cancel(request);
}

std::optional<std::uint64_t> ExplorationController::pct_recovery_evidence() {
  LayerMapReadPhase lock(*shared_layer_);
  const auto yaw=policy_.cross_task_.ascent_yaw();
  const Point forward{(*base_)[0]+1.5*std::cos(yaw),(*base_)[1]+1.5*std::sin(yaw),(*base_)[2]};
  return shared_layer_->pct->recovery_evidence(shared_layer_->current_version.load(),*base_,forward);
}

void ExplorationController::defer_cross_layer(const std::string &reason) {
  policy_.cross_task_.defer(reason);
  if(base_ && layer_map_fresh()) {
    LayerMapReadPhase lock(*shared_layer_);
    policy_.cross_deferred_signature_=pct_recovery_evidence();
    policy_.cross_deferred_pose_=*base_;
    policy_.cross_deferred_scan_blocked_=scan_start_inflated(*base_,policy_.cross_task_.ascent_yaw());
  }
  policy_.cross_deferred_feet_stable_=stable_four_foot_support(wall_now());
  policy_.cross_deferred_evidence_time_=shared_layer_->last_integration_wall_seconds.load();
  control_mode_=ExplorationMode::flat;
}

void ExplorationController::block_stair_reposition(const std::string &reason) {
  const auto *frontier=recovery_.stair_reposition_?&recovery_.stair_reposition_->forward_goal:
      selected_pct_?&*selected_pct_:nullptr;
  if(frontier && frontier->attempt && base_) {
    LayerMapReadPhase lock(*shared_layer_);
    const auto version=shared_layer_->current_version.load();
    shared_layer_->pct->observation_task(version,"floor_execution_failed",frontier->id,*base_);
    shared_layer_->pct->observation_task(version,"cancel",frontier->id,*base_);
  }
  recovery_.stair_reposition_.reset();
  active_.reset();
  reset_planned_route();
  defer_cross_layer(reason);
  report("cross_layer_deferred",reason);
}

void ExplorationController::await_stair_stop() {
  recovery_.stair_reposition_->stopped_at=wall_now();
  active_.reset();
  reset_planned_route();
  report("cross_layer_reposition_waiting_for_map",recovery_.stair_reposition_->failure);
}

bool ExplorationController::try_queue_stair_reposition(const std::string &reason) {
  if(!policy_.cross_task_.engaged() || policy_.cross_task_.rise()<policy_.cross_task_.minimum_rise*.5 ||
     active_->source_at_send!="layer" || !selected_pct_ || !base_ || !base_yaw_ ||
     paused_ || wall_now()-base_wall_>options_.odom_timeout_ ||
     xy_distance({(*base_)[0],(*base_)[1]},
                 {active_->pose_at_send[0],active_->pose_at_send[1]})>.05 ||
     std::abs((*base_)[2]-active_->pose_at_send[2])>policy_.cross_task_.height_slack)return false;
  const auto target_z=path_processor_.current_z();
  if(reason!="local_replan_exhausted" || !target_z ||
     *target_z-(*base_)[2]<=policy_.cross_task_.height_slack ||
     !planner_occupancy_fresh(wall_now()))return false;
  const double heading=std::atan2(active_->xy[1]-(*base_)[1],active_->xy[0]-(*base_)[0]);
  if(!scan_start_inflated(*base_,heading))return false;
  if(!policy_.cross_task_.begin_reposition(cross_ground_pose())) {
    block_stair_reposition("stair_reposition_already_attempted_or_off_route");
    return true;
  }
  auto candidate=*selected_pct_;
  const Point ground{active_->xy[0],active_->xy[1],*target_z-options_.layer_body_height_};
  candidate.observations={ground};
  candidate.observation_yaws={heading};
  recovery_.stair_reposition_=StairReposition{};
  recovery_.stair_reposition_->start=*base_;
  recovery_.stair_reposition_->forward_goal=std::move(candidate);
  recovery_.stair_reposition_->heading=heading;
  recovery_.stair_reposition_->queued_at=wall_now();
  active_.reset();
  reset_planned_route();
  report("cross_layer_reposition_pending","scan_start_inflated");
  return true;
}

void ExplorationController::handle_stair_reposition(double current) {
  auto &task=*recovery_.stair_reposition_;
  if(paused_ || !ready_ || options_.observation_only_)return;
  const bool blocked=task.stage==StairReposition::Stage::blocked;
  const double wait_since=task.stopped_at.value_or(task.queued_at);
  const double step_limit=std::min(.16,shared_layer_->max_step_height);
  const bool feet_settled=!options_.floor_exploration_ ||
      (stable_foot_height_ && current-foot_sample_wall_<=.25 &&
       (task.stopped_at?(foot_sample_wall_>wait_since && four_feet_stable_since_ &&
           foot_sample_wall_-*four_feet_stable_since_>=.40):
           std::abs(*stable_foot_height_-(task.start[2]-options_.layer_body_height_))<=step_limit));
  if(!base_ || !base_yaw_ || current-base_wall_>options_.odom_timeout_ ||
     current-yaw_wall_>options_.odom_timeout_ || !layer_map_fresh() ||
     !planner_occupancy_fresh(current) ||
     (!task.stopped_at && (raw_occupancy_wall_<=0. || current-raw_occupancy_wall_>options_.occupancy_timeout_)) ||
     (task.stopped_at && (occupancy_wall_<=wait_since || base_wall_<=wait_since ||
        shared_layer_->last_integration_wall_seconds.load()<=wait_since))) {
    if(!blocked && current-wait_since>5.)block_stair_reposition("stair_reposition_map_or_pose_timeout");
    else report("cross_layer_reposition_waiting_for_map");
    return;
  }
  if(!feet_settled || (task.stopped_at && (!base_quiet_since_ ||
     current-std::max(*base_quiet_since_,wait_since)<.30))) {
    if(!blocked && current-wait_since>5.)block_stair_reposition("stair_reposition_feet_not_stable_on_tread");
    else report("cross_layer_reposition_waiting_for_stable_feet");
    return;
  }
  static const std::unordered_map<Cell,int,CellHash> no_grid;
  LayerMapReadPhase decision_lock(*shared_layer_);
  const auto version=shared_layer_->current_version.load();
  const auto &forward=task.forward_goal.observations.front();
  const auto inflated_at=[&](const Point &p,double heading){return scan_start_inflated(p,heading);};
  const auto repositions=[&](bool reverse) {
    return stair_recovery_repositions(*shared_layer_->pct,version,*base_,forward,
        *base_yaw_,policy_.cross_task_.ascent_yaw(),task.tread_height.value_or(cross_ground_pose()[2]),
        step_limit,reverse,inflated_at);
  };
  if(!task.stopped_at && task.stage==StairReposition::Stage::reverse) {
    if(output_.recovery_reverse_subscribers()==0 ||
       std::abs(std::remainder(*base_yaw_-task.heading,2*M_PI))>.5 ||
       xy_distance({(*base_)[0],(*base_)[1]},{task.start[0],task.start[1]})>.05) {
      block_stair_reposition("stair_reverse_executor_or_pose_unavailable");return;
    }
    std::string rejection="stair_reverse_stop_inflated";
    for(const auto &target:repositions(true)) {
      if(prepare_route(target,no_grid,external_obstacles_,"cross_layer_reverse",0.,nullptr,
                       std::nullopt,RecoveryRequest::reverse,true)) {
        task.action_start=*base_;
        task.action_target=target;
        task.action_yaw=*base_yaw_;
        task.tread_height=stable_foot_height_;
        return;
      }
      rejection=recovery_.recovery_last_failure_;
    }
    block_stair_reposition(rejection);return;
  }
  const double heading=std::atan2(forward[1]-(*base_)[1],forward[0]-(*base_)[0]);
  if(std::abs((*base_)[2]-task.start[2])>step_limit ||
     (task.tread_height && (!stable_foot_height_ ||
      std::abs(*stable_foot_height_-*task.tread_height)>step_limit))) {
    block_stair_reposition("stair_reposition_changed_tread");return;
  }
  if(task.stage==StairReposition::Stage::reverse || task.stage==StairReposition::Stage::shift) {
    const auto along=verified_short_recovery_stop(task.action_start,task.action_target,*base_,
        *base_yaw_-task.action_yaw,task.partial_reverse);
    if(!along) {
      block_stair_reposition("stair_reposition_stop_outside_verified_motion");return;
    }
    if(task.stage==StairReposition::Stage::reverse && *along<.16) {
      task.failure="stair_reposition_insufficient_progress";
      task.stage=StairReposition::Stage::review;
    }
  }
  const auto check=shared_layer_->pct->point_check(version,*base_);
  const bool safe=check.ready && (!stable_foot_height_ ||
      (check.ground_z && std::abs(*check.ground_z-*stable_foot_height_)<=.04));
  const bool inflated=scan_start_inflated(*base_,heading);
  const auto shifted_pose=[&](double distance) {
    return Point{(*base_)[0]-distance*std::sin(policy_.cross_task_.ascent_yaw()),
                     (*base_)[1]+distance*std::cos(policy_.cross_task_.ascent_yaw()),(*base_)[2]};
  };
  const auto left=shifted_pose(.22),right=shifted_pose(-.22);
  const auto goal=Point{forward[0],forward[1],forward[2]+options_.layer_body_height_};
  const std::array<bool,5> clear{safe,!inflated,!scan_start_inflated(left,heading),
      !scan_start_inflated(right,heading),shared_layer_->pct->point_check(version,goal).ready};
  if(blocked) {
    bool improved=false;
    for(std::size_t i=0;i<clear.size();++i)improved|=clear[i] && !task.blocked_checks[i];
    const bool moved=xy_distance({(*base_)[0],(*base_)[1]},
                                 {task.blocked_pose[0],task.blocked_pose[1]})>=.10;
    if(!improved && !moved) {
      report("cross_layer_deferred",policy_.cross_task_.reason());return;
    }
  }
  std::vector<PctUnknownBlocker> replan_blockers;
  if(safe && !inflated && task.stage!=StairReposition::Stage::review) {
    policy_.cross_task_.finish_reposition(cross_ground_pose());
    auto candidate=task.forward_goal;
    for(const auto &view:policy_.pct_frontiers_.stair_targets) {
      if(view.ground[2]<=cross_ground_pose()[2]+policy_.cross_task_.height_slack ||
         !policy_.cross_task_.allows(view.ground) ||
         std::find(candidate.observations.begin(),candidate.observations.end(),view.ground)!=
             candidate.observations.end())continue;
      candidate.observations.push_back(view.ground);
      candidate.observation_yaws.push_back(view.yaw);
    }
    pct_observation_progress_.erase(candidate.id);
    pct_round_deadline_=wall_now()+options_.pct_round_budget_;
    task.stage=StairReposition::Stage::forward;
    task.action_start=*base_;
    const char *detail=task.shifted?"pct_stair_replan_after_shift":"pct_stair_replan_after_reverse";
    if(prepare_route({candidate.frontier[0],candidate.frontier[1]},no_grid,external_obstacles_,
                     detail,0.,&candidate,std::nullopt,RecoveryRequest::position,false,
                     std::nullopt,nullptr,&replan_blockers)) {
      if(task.partial_reverse) {
        task.partial_reverse=false;
        report("cross_layer_start_recovered","reverse_no_progress");
      }
      return;
    }
    task.failure="stair_replan_unavailable";
    task.stage=StairReposition::Stage::review;
  } else if(task.stage!=StairReposition::Stage::review && !blocked) {
    task.failure=inflated?"stair_reposition_stop_inflated":"stair_reposition_stop_unsafe:"+check.reason;
  }
  if(!task.shifted && output_.recovery_reverse_subscribers()>0 &&
     raw_occupancy_wall_>0. && current-raw_occupancy_wall_<=options_.occupancy_timeout_) {
    for(const auto &target:repositions(false)) {
      if(prepare_route(target,no_grid,external_obstacles_,"cross_layer_shift",0.,nullptr,
                       std::nullopt,RecoveryRequest::shift,true)) {
        task.action_start=*base_;
        task.action_target=target;
        task.action_yaw=*base_yaw_;
        task.stage=StairReposition::Stage::shift;
        task.shifted=true;
        task.partial_reverse=false;
        return;
      }
    }
  }
  if(safe && !inflated && options_.floor_exploration_) {
    const Point continuation{policy_.cross_task_.ascent_yaw(),policy_.cross_task_.highest_height()-policy_.cross_task_.height_slack,
                                 policy_.cross_task_.progress_slack};
    auto stairs=shared_layer_->pct->frontiers(version,*base_,cross_layer_minimum_travel(),
        Point{cfg_.local_search_radius,options_.pct_local_above_,options_.pct_local_below_},options_.pct_query_budget_,
        continuation,current_pct_floor(),true);
    stairs.unknown_blockers.insert(stairs.unknown_blockers.end(),replan_blockers.begin(),replan_blockers.end());
    if(!stairs.unknown_blockers.empty() || task.failure=="stair_replan_unavailable") {
      const auto forward_goal=task.forward_goal;
      policy_.cross_task_.finish_reposition(cross_ground_pose());
      recovery_.stair_reposition_.reset();
      policy_.pct_frontiers_.unknown_blockers=std::move(stairs.unknown_blockers);
      recover_blocked_ascent(forward_goal);
      return;
    }
  }
  task.stage=StairReposition::Stage::blocked;
  task.blocked_pose=*base_;
  task.blocked_checks=clear;
  defer_cross_layer(task.failure);
  report("cross_layer_deferred",task.failure);
}

bool ExplorationController::begin_blocker_recovery(const PctFrontier &goal,
    const std::vector<PctUnknownBlocker> &blockers,
    bool cross_layer,int floor_attempt,const char *policy,
    std::optional<std::string> start_failure,
    std::optional<XY> platform) {
  if(blockers.empty() && goal.observations.empty() && !start_failure && !cross_layer)return false;
  BlockerRecovery task;
  task.goal=goal;
  if(goal.id && !goal.observations.empty()) {
    const auto cluster=std::find_if(policy_.pct_frontiers_.clusters.begin(),policy_.pct_frontiers_.clusters.end(),
                                   [&](const auto &c){return c.id==goal.id;});
    if(cluster!=policy_.pct_frontiers_.clusters.end()) {
      for(std::size_t i=0;i<cluster->observations.size();++i) {
        const auto &point=cluster->observations[i];
        if(std::find(task.goal.observations.begin(),task.goal.observations.end(),point)!=
           task.goal.observations.end())continue;
        task.goal.observation_yaws.resize(task.goal.observations.size(),base_yaw_.value_or(0.));
        task.goal.observations.push_back(point);
        task.goal.observation_yaws.push_back(i<cluster->observation_yaws.size()?
            cluster->observation_yaws[i]:base_yaw_.value_or(0.));
      }
    }
  }
  task.blockers=blockers;
  // Platform tasks use flat-floor recovery while the owning climb keeps
  // its platform state. Do not constrain a platform scan to the stair yaw.
  task.cross_layer=cross_layer && !platform;
  task.floor=platform?platform:cross_layer?std::nullopt:current_pct_floor();
  task.floor_attempt=floor_attempt;
  task.platform=platform;
  task.unsafe_start=start_failure.has_value();
  task.selection_policy=policy;
  task.floor_failure=blockers.empty()?"floor_disconnected":"floor_no_route";
  recovery_.restore_attempts(base_, cross_layer || goal.observations.empty(), task);
  if(floor_attempt)
    shared_layer_->pct->observation_task(shared_layer_->current_version.load(),
                                        "floor_recovery_attempt",goal.id,*base_);
  recovery_.blocker_recovery_=std::move(task);
  report(cross_layer?"cross_layer_recovery":"pct_blocker_recovery",
      start_failure.value_or(blockers.empty()?
          (cross_layer?"no_forward_progress":"pct_route_disconnected"):"recorded_unknown_blocker"));
  return true;
}

void ExplorationController::try_pct_landing_shift(double current) {
  if(!policy_.floor_height_ || !stable_four_foot_support(current) ||
     !base_quiet_since_ || current-*base_quiet_since_<.30 || !base_yaw_ ||
     current-yaw_wall_>options_.odom_timeout_ || output_.recovery_reverse_subscribers()==0) {
    report("pct_blocker_recovery_waiting","stable_platform_pose");return;
  }
  static const std::unordered_map<Cell,int,CellHash> no_grid;
  const auto action=plan_pct_landing_shift(planning_snapshot(no_grid,external_obstacles_),
      shared_layer_->current_version.load(),{*policy_.floor_height_,options_.floor_height_tolerance_},
      *stable_foot_height_,policy_.cross_task_.ascent_yaw());
  if(action) {
    BlockerRecovery task;
    task.cross_layer=task.platform_shift=true;
    task.action=action;
    task.motion_start=*base_;
    task.motion_yaw=*base_yaw_;
    recovery_.blocker_recovery_=std::move(task);
    if(prepare_route({action->position[0],action->position[1]},no_grid,external_obstacles_,
                    "pct_landing_recovery_shift",0.,nullptr,std::nullopt,RecoveryRequest::shift,true))return;
    recovery_.blocker_recovery_.reset();
  }
  recover_blocked_ascent();
}

void ExplorationController::finish_blocker_recovery(const char *reason) {
  auto task=std::move(*recovery_.blocker_recovery_);
  recovery_.blocker_recovery_.reset();
  if(task.floor_attempt) {
    const auto &goal=task.goal;
    settle_unavailable_floor_cluster(goal.id,task.floor_failure,goal.observations.empty()?
        std::nullopt:std::optional<Point>{goal.observations.front()});
    report("pct_floor_frontier_settled",task.floor_failure);
  } else if(task.cross_layer) {
    if(base_)recovery_.save_attempts(*base_, std::move(task.planner));
    defer_cross_layer(reason);
    policy_.cross_deferred_checked_=true;
    report("cross_layer_deferred",reason);
  } else if(task.goal.observations.empty()) {
    if(base_)recovery_.save_attempts(*base_, std::move(task.planner));
    report("pct_unsafe_start",reason);
  } else {
    policy_.pct_cooldowns_.push_back({task.goal.frontier,wall_now()+options_.cooldown_});
    report("pct_blocker_recovery_exhausted",reason);
  }
}

void ExplorationController::run_blocker_recovery(double current) {
  using Kind=PctRecoveryAction::Kind;
  auto &task=*recovery_.blocker_recovery_;
  if(paused_ || options_.observation_only_ || !ready_ || !base_ || !base_yaw_ ||
     output_.waypoint_subscribers()==0 ||
     current-base_wall_>options_.odom_timeout_ || current-yaw_wall_>options_.odom_timeout_ || !layer_map_fresh()) {
    report("pct_blocker_recovery_waiting","map_pose_or_control");return;
  }
  if(!base_quiet_since_ || current-*base_quiet_since_<.30) {
    report("pct_blocker_recovery_waiting","stable_pose");return;
  }
  LayerMapReadPhase lock(*shared_layer_);
  auto &pct=*shared_layer_->pct;
  const auto version=shared_layer_->current_version.load();
  static const std::unordered_map<Cell,int,CellHash> no_grid;
  const auto snapshot=planning_snapshot(no_grid,external_obstacles_);
  const bool feet_stable=stable_four_foot_support(current);
  const auto nearby_evidence=[&](Point pose) {
    const Point upper{pose[0]+.60,pose[1]+.60,pose[2]};
    pose[0]-=.60;pose[1]-=.60;
    return pct.recovery_evidence(version,pose,upper);
  };
  if(task.retry_wait) {
    const auto &wait=*task.retry_wait;
    const bool moved=std::hypot((*base_)[0]-wait.pose[0],(*base_)[1]-wait.pose[1])>=.10 ||
        std::abs((*base_)[2]-wait.pose[2])>=.05;
    if(!moved && pct.recovery_evidence(version,*base_)==wait.geometry) {
      report("pct_blocker_recovery_waiting","no_other_safe_delegate_goal");return;
    }
    if(!task.cross_layer && task.goal.observations.empty() &&
       pct.start_ready(version,*base_,task.floor)) {
      recovery_.blocker_recovery_.reset();
      report("pct_blocker_start_recovered");return;
    }
    if(moved || nearby_evidence(wait.pose)!=wait.nearby)task.planner.retry_delegates();
    task.retry_wait.reset();
    if(task.cross_layer && !task.goal.observations.empty())task.replan=true;
  }
  if(!task.action && base_wall_<=task.result_at) {
    report("pct_blocker_recovery_waiting","fresh_pose_after_delegation_failure");return;
  }
  PctRecoveryConstraints constraints;
  constraints.floor=task.floor;
  if(task.cross_layer) {
    constraints.ascent=policy_.cross_task_.ascent_yaw();
    if(feet_stable)constraints.contacts.assign(foot_contact_points_.begin(),foot_contact_points_.end());
    constraints.permits_goal=[&](Point point) {
      point[2]-=options_.layer_body_height_;
      return policy_.cross_task_.allows(point);
    };
  }
  const bool stair_scan=task.cross_layer;
  if(stair_scan)constraints.permits_scan=[&](const Point &point) {
    return policy_.stair_scan_budget_.allows(point);
  };
  const auto start_scan=[&](const PctRecoveryAction &action,bool repositioned=false) {
    if(stair_scan && !policy_.stair_scan_budget_.reserve(action.gap,cross_ground_pose()))return;
    const std::vector<PctUnknownBlocker> targets{{action.gap,0,version}};
    const auto review=pct.observation_task(version,"begin_blocker_review",0,*base_,action.yaw,0.,&targets);
    if(!review.ready)return;
    ObservationTask observation;
    observation.began=current;
    observation.initialized=observation.support_scan=observation.targeted_blocker_scan=true;
    observation.cross_layer=task.cross_layer;
    observation.center=action.yaw;
    observation.headings={action.yaw};
    task.action=action;
    observation_task_=std::move(observation);
    if(task.cross_layer)policy_.cross_task_.begin_support_scan();
    const auto state=!task.cross_layer?"pct_blocker_scan":
        repositioned?"cross_layer_stair_gap_scan":"cross_layer_support_scan";
    const bool recorded=std::any_of(task.blockers.begin(),task.blockers.end(),
        [&](const auto &blocker){return blocker.point==action.gap;});
    report(state,recorded?"recorded_unknown_blocker":"review_next_tread");
  };
  if(task.action && task.stopped_pose) {
    if(shared_layer_->last_integration_wall_seconds.load()<=task.result_at || base_wall_<=task.result_at ||
       (task.cross_layer && task.action->kind!=Kind::delegate &&
        (!feet_stable || foot_sample_wall_<=task.result_at))) {
      if(task.action->kind!=Kind::delegate && current-task.result_at>5.) {
        task.action.reset();task.stopped_pose.reset();
      } else report("pct_blocker_recovery_waiting","fresh_map_or_stable_feet");
      return;
    }
    const auto action=*task.action;
    const auto stop=action.kind==Kind::delegate?*task.stopped_pose:*base_;
    task.action.reset();task.stopped_pose.reset();
    if(action.kind==Kind::delegate) {
      const auto checked=pct.point_check(version,stop,false,std::nullopt,0.,false,task.floor);
      if(std::abs(stop[2]-action.position[2])>.08 || !checked.ready || !checked.ground_z ||
         std::abs(*checked.ground_z+options_.layer_body_height_-action.position[2])>.08 ||
         std::hypot((*base_)[0]-stop[0],(*base_)[1]-stop[1])>options_.recovery_goal_xy_tolerance_ ||
         std::abs((*base_)[2]-stop[2])>.08 ||
         !pct.start_ready(version,*base_,task.floor) ||
         (constraints.permits_goal && !constraints.permits_goal(stop))) {
        report("pct_delegate_stop_unverified","pct_unsafe_stop");return;
      }
      if(task.cross_layer) {
        policy_.cross_task_.finish_reposition(cross_ground_pose());
        start_stair_segment_scan(policy_.cross_task_.ascent_yaw());
      } else if(!task.goal.observations.empty()) {
        const auto &views=task.goal.observations;
        const auto selected=std::find_if(views.begin(),views.end(),[&](const auto &view) {
          return std::hypot(view[0]-action.position[0],view[1]-action.position[1])<1e-6 &&
              std::abs(view[2]+options_.layer_body_height_-action.position[2])<1e-6;
        });
        if(selected==views.end()) {
          finish_blocker_recovery("pct_blocker_delegation_reached");return;
        }
        ObservationTask observation;
        observation.frontier=task.goal;
        observation.began=current;
        observation.floor_attempt=task.floor_attempt;
        observation.fast_floor=task.selection_policy=="pct_frontier_fast";
        const auto index=static_cast<std::size_t>(selected-views.begin());
        const double yaw=index<task.goal.observation_yaws.size()?task.goal.observation_yaws[index]:*base_yaw_;
        if(task.floor_attempt)pct.observation_task(version,"floor_attempt",task.goal.id,action.position,yaw);
        observation_task_=std::move(observation);
      }
      recovery_.blocker_recovery_.reset();
      report("pct_blocker_delegation_reached");return;
    }
    const double permitted_height=action.kind==Kind::reverse?
        std::min(.16,shared_layer_->max_step_height):.05;
    const bool stop_verified=std::abs(stop[2]-task.motion_start[2])<=permitted_height &&
        verified_short_recovery_stop(task.motion_start,{action.position[0],action.position[1]},
            stop,*base_yaw_-task.motion_yaw).has_value();
    if(task.platform_shift) {
      if(!stop_verified) {finish_blocker_recovery("pct_landing_shift_stop_unverified");return;}
      policy_.cross_task_.finish_reposition(cross_ground_pose());
      recovery_.blocker_recovery_.reset();
      if(cross_layer_platform_ready())report("cross_layer_completed");
      return;
    }
    if(stop_verified) {
      const auto yaw=task.planner.scan_heading(pct,version,*base_,base_attitude_,action.gap,
          task.blockers,options_.layer_body_height_,constraints,task.cross_layer &&
          std::none_of(task.blockers.begin(),task.blockers.end(),
              [&](const auto &blocker){return blocker.point==action.gap;}));
      if(yaw) {
        if(task.cross_layer)policy_.cross_task_.finish_reposition(cross_ground_pose());
        start_scan({Kind::scan,*base_,action.gap,*yaw},true);return;
      }
    }
    task.replan=true;
  }
  if(task.action)return;
  const auto &goal=task.goal;
  std::optional<Point> target;
  if(!goal.observations.empty()) {
    target=goal.observations.front();
    (*target)[2]+=options_.layer_body_height_;
  }
  if(task.replan) {
    task.replan=false;
    if(task.cross_layer && options_.floor_exploration_ && !task.floor_attempt) {
      recovery_.save_attempts(*base_, std::move(task.planner));
      recovery_.blocker_recovery_.reset();
      return;
    }
    if(target) {
      pct_round_deadline_=wall_now()+options_.pct_round_budget_;
      pct_observation_progress_.erase(goal.id);
      std::vector<PctUnknownBlocker> refreshed;
      std::string failure;
      if(prepare_route({goal.frontier[0],goal.frontier[1]},no_grid,external_obstacles_,
                      task.cross_layer && !task.floor_attempt?"pct_stair_replan_after_gap_scan":task.selection_policy,
                      0.,&goal,std::nullopt,RecoveryRequest::position,false,task.platform,
                      &failure,&refreshed)) {
        recovery_.blocker_recovery_.reset();return;
      }
      if(refreshed.empty() &&
         (failure=="pct_planning_timeout" || failure=="planning_budget_exhausted")) {
        task.floor_failure="floor_route_timeout";
        finish_blocker_recovery("pct_blocker_replan_timeout");return;
      }
      task.blockers=std::move(refreshed);
    } else if(!task.unsafe_start) {
      // With no original endpoint the controller must find a new measured
      // continuation from the refreshed map.
      if(task.floor_attempt)pct.observation_task(version,"cancel",0,*base_);
      recovery_.save_attempts(*base_, std::move(task.planner));
      recovery_.blocker_recovery_.reset();return;
    }
  }
  const bool can_move=(!task.cross_layer || feet_stable) &&
      planner_occupancy_fresh(current) && raw_occupancy_wall_>0. &&
      current-raw_occupancy_wall_<=options_.occupancy_timeout_ &&
      output_.recovery_reverse_subscribers()>0;
  const auto stance_clear=[&](const Point &p) {
    return !scan_start_inflated(p,constraints.ascent.value_or(*base_yaw_));
  };
  const auto review=task.cross_layer && !task.delegation_started?
      pct.stair_review_points(version,*base_,*constraints.ascent):std::vector<Point>{};
  const auto action = recovery_.next_action(snapshot, version, base_attitude_, constraints,
      can_move, stance_clear, review, policy_.cross_task_.height_slack, options_.recovery_goal_xy_tolerance_);
  if(!action) {
    if(!task.cross_layer && pct.start_ready(version,*base_,task.floor)) {
      if(task.unsafe_start && task.goal.observations.empty()) {
        recovery_.blocker_recovery_.reset();
        report("pct_blocker_start_recovered");return;
      }
    } else if(!task.cross_layer && !task.unsafe_start) {
      task.unsafe_start=true;return;
    } else if(!task.cross_layer && (task.unsafe_start || task.delegation_started)) {
      task.retry_wait=BlockerRecovery::RetryWait{*base_,pct.recovery_evidence(version,*base_),
                                                 nearby_evidence(*base_)};
      report("pct_blocker_recovery_waiting","no_other_safe_delegate_goal");return;
    }
    finish_blocker_recovery("pct_blocker_no_verified_action");return;
  }
  if(action->kind==Kind::scan) {start_scan(*action);return;}
  task.action=action;
  task.motion_start=*base_;
  task.motion_yaw=*base_yaw_;
  const bool delegate=action->kind==Kind::delegate;
  if(delegate)task.delegation_started=true;
  const auto request=delegate?RecoveryRequest::delegate:
      action->kind==Kind::reverse?RecoveryRequest::reverse:RecoveryRequest::shift;
  if(prepare_route({action->position[0],action->position[1]},no_grid,external_obstacles_,
                   delegate?"pct_blocker_scan_delegate":"pct_blocker_reposition",0.,nullptr,
                   action->position,request,true,delegate?task.floor:std::nullopt))return;
  task.action.reset();
  if(delegate)report("pct_blocker_delegate_rejected",recovery_.recovery_last_failure_);
}

void ExplorationController::recover_blocked_ascent(const PctFrontier &forward) {
  if(begin_blocker_recovery(forward,policy_.pct_frontiers_.unknown_blockers,true))return;
  defer_cross_layer("no_forward_route");
  report("cross_layer_deferred",policy_.cross_task_.reason());
}

}  // namespace legbot_exploration
