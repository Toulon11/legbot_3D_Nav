#include "exploration_controller.hpp"
#include "exploration_controller_util.hpp"

namespace legbot_exploration {

std::optional<double> ExplorationController::confirmed_floor_height(std::uint64_t version) {
  if(!stable_foot_height_ || wall_now()-foot_sample_wall_>.25) {
    policy_.floor_confirmation_reason_=foot_sample_state_;
    return std::nullopt;
  }
  auto point=*base_;
  point[2]=*stable_foot_height_+options_.layer_body_height_;
  const auto check=shared_layer_->pct->point_check(version,point,false,std::nullopt,
      0.,false,XY{*stable_foot_height_,.04});
  if(!check.ready) {
    policy_.floor_confirmation_reason_=check.reason;
    return std::nullopt;
  }
  if(!check.ground_z || std::abs(*check.ground_z-*stable_foot_height_)>.04) {
    policy_.floor_confirmation_reason_="foot_height_disagrees_with_pct";
    return std::nullopt;
  }
  policy_.floor_confirmation_reason_="ready";
  return stable_foot_height_;
}

bool ExplorationController::cross_layer_platform_ready(std::optional<double> *platform_ground) {
  if(!base_ || (!policy_.cross_task_.engaged() && !(options_.floor_exploration_ && policy_.cross_task_.deferred())))
    return false;
  LayerMapReadPhase lock(*shared_layer_);
  const auto version=shared_layer_->current_version.load();
  if(options_.floor_exploration_) {
    if(!policy_.floor_height_ || policy_.cross_task_.rise()<=0. || policy_.cross_task_.progress()<=0.)
      return false;
    const auto check=shared_layer_->pct->point_check(version,*base_,false,
        XY{*policy_.floor_height_,options_.floor_height_tolerance_},policy_.cross_task_.ascent_yaw(),false,
        std::nullopt,false,policy_.cross_task_.platform_height());
    policy_.floor_confirmation_reason_=check.reason;
    if(platform_ground && check.ground_z && *check.ground_z>*policy_.floor_height_ &&
       (check.ready || check.reason=="platform_not_confirmed"))
      *platform_ground=check.ground_z;
    if(!check.ready || !check.platform_height)return false;
    policy_.enter_floor(*check.platform_height);
    empty_since_.reset();
    pct_observation_progress_.clear();
    stair_route_progress_.clear();
  } else if(policy_.cross_task_.rise()<policy_.cross_task_.minimum_rise ||
            !shared_layer_->pct->point_check(version,*base_,true).ready) {
    return false;
  }
  policy_.cross_task_.complete();
  policy_.cross_deferred_checked_=false;
  control_mode_=ExplorationMode::flat;
  return true;
}

void ExplorationPolicy::schedule_global() {
  pct_global_fallback_=true;
  pct_next_candidate_=0;
  pct_failed_local_rounds_=0;
  pct_local_failure_since_=0.;
  ++pct_local_fallback_count_;
}

void ExplorationPolicy::enter_floor(double ground) {
  floor_height_=ground;
  ++floor_number_;
  floor_complete_=false;
  pct_global_fallback_=false;
  pct_next_candidate_=0;
  pct_cooldowns_.clear();
  failed_stair_observers_.clear();
  pct_timeout_pending_.clear();
  pct_timed_out_observers_.clear();
  stair_scan_budget_.reset();
}

ExplorationPolicy::RankedFrontiers ExplorationPolicy::rank_frontiers(
    double current, const Point &ground, std::optional<double> yaw,
    const Settings &settings, double direction_weight) {
  pct_cooldowns_.erase(std::remove_if(pct_cooldowns_.begin(),pct_cooldowns_.end(),
      [current](const auto &c){return c.expiry<=current;}),pct_cooldowns_.end());
  for(auto it=pct_timeout_pending_.begin();it!=pct_timeout_pending_.end();)
    if(it->second<=current)it=pct_timeout_pending_.erase(it);else ++it;
  for(auto it=pct_timed_out_observers_.begin();it!=pct_timed_out_observers_.end();)
    if(it->second<=current)it=pct_timed_out_observers_.erase(it);else ++it;
  cross_filtered_observers_=0;
  const bool protect_deferred_progress=cross_task_.deferred() && cross_task_.rise()>=cross_task_.minimum_rise*.5 &&
      cross_task_.at_progress(ground);
  const auto cooling=[&](const auto &c) {
    return std::any_of(pct_cooldowns_.begin(),pct_cooldowns_.end(),[&](const auto &old) {
      return xy_distance({c.frontier[0],c.frontier[1]},{old.xyz[0],old.xyz[1]})<0.75 &&
          std::abs(c.frontier[2]-old.xyz[2])<0.16;
    });
  };
  RankedFrontiers ranked;
  auto &candidates = ranked.candidates;
  for (const auto &c:pct_frontiers_.clusters) {
    if (c.observations.empty()) continue;
    ++ranked.considered;
    if (cooling(c)) {++ranked.suppressed;continue;}
    std::vector<std::pair<double,std::size_t>> ordered;
    for(std::size_t i=0;i<c.observations.size();++i) {
      const auto &p=c.observations[i];
      if(pct_timed_out_observers_.count(p))continue;
      if((cross_task_.engaged() && (failed_stair_observers_.count(p) || !cross_task_.allows(p))) ||
         (protect_deferred_progress && p[2]<cross_task_.highest_height()-cross_task_.height_slack)) {
        ++cross_filtered_observers_;continue;
      }
      ordered.emplace_back(std::hypot(std::hypot(p[0]-ground[0],p[1]-ground[1]),p[2]-ground[2]),i);
    }
    if(ordered.empty())continue;
    if(!pct_frontiers_.fast_floor)std::sort(ordered.begin(),ordered.end());
    auto filtered=c;
    filtered.observations.clear();filtered.observation_yaws.clear();
    for(const auto &entry:ordered) {
      filtered.observations.push_back(c.observations[entry.second]);
      if(c.observation_yaws.size()==c.observations.size())
        filtered.observation_yaws.push_back(c.observation_yaws[entry.second]);
    }
    const auto &nearest=filtered.observations.front();
    const double heading=yaw?1.-std::cos(std::atan2(nearest[1]-ground[1],nearest[0]-ground[0])-*yaw):0.;
    const double score=settings.frontier_gain_weight*std::log1p(c.gain)-
        settings.path_cost_weight*ordered.front().first-direction_weight*heading;
    candidates.emplace_back(score,std::move(filtered));
  }
  std::sort(candidates.begin(),candidates.end(),[](const auto &a,const auto &b) {
    return a.first==b.first?a.second.id<b.second.id:a.first>b.first;
  });
  return ranked;
}

const char *ExplorationPolicy::check_ascent(
    const Point &start, const std::vector<Point> &route,
    std::optional<double> departure_height, bool starts_climb) const {
  if (starts_climb && !cross_task_.engaged() &&
      !cross_task_.starts_above_departure(start, route, departure_height))
    return "stair_route_does_not_climb";
  if (!cross_task_.deferred())
    return cross_task_.route_allowed(start, route, departure_height) ? nullptr : "cross_layer_route_regresses";
  const bool at_progress = cross_task_.rise() >= cross_task_.minimum_rise * .5 && cross_task_.at_progress(start);
  if (at_progress && std::any_of(route.begin(), route.end(), [&](const auto &point) {
        return point[2] < cross_task_.highest_height() - cross_task_.height_slack;
      }))
    return "cross_layer_route_regresses";
  return nullptr;
}

bool ExplorationPolicy::adopt_ascent(
    const Point &start, const std::vector<Point> &route,
    std::optional<double> departure_height, std::optional<double> yaw) {
  if (route.empty()) return false;
  cross_deferred_checked_ = false;
  if (cross_task_.deferred()) {
    if (cross_task_.at_progress(start) || route.back()[2] - start[2] < cross_task_.minimum_rise)
      return false;
    auto replacement = cross_task_;
    replacement.clear();
    if (!replacement.accept(start, route, departure_height)) return false;
    cross_task_ = std::move(replacement);
  } else if (!cross_task_.accept(start, route, departure_height)) {
    return false;
  }
  if (yaw) cross_task_.set_ascent_yaw(*yaw);
  return true;
}

std::optional<XY> ExplorationController::current_pct_floor() const {
  if(!options_.floor_exploration_ || !policy_.floor_height_)return std::nullopt;
  return XY{*policy_.floor_height_,options_.floor_height_tolerance_};
}

bool ExplorationController::settle_unavailable_floor_cluster(std::uint64_t id,const char *reason,
    std::optional<Point> viewpoint,
    double planning_ms) {
  LayerMapReadPhase lock(*shared_layer_);
  if(viewpoint)(*viewpoint)[2]+=options_.layer_body_height_;
  const auto result=shared_layer_->pct->observation_task(shared_layer_->current_version.load(),reason,id,
                                                        viewpoint.value_or(*base_),0.,viewpoint?1.:0.);
  if(!result.ready)return false;
  pct_observation_progress_.erase(id);
  const auto cluster=std::find_if(policy_.pct_frontiers_.clusters.begin(),policy_.pct_frontiers_.clusters.end(),
                                  [id](const auto &c){return c.id==id;});
  RCLCPP_INFO(logger_,
      "Floor %zu h=%.3f frontier %llu attempt=%d scope=%s map=%llu observer_ms=%.1f planning_ms=%.1f: %s (%s)",
      policy_.floor_number_,policy_.floor_height_.value_or(0.),static_cast<unsigned long long>(id),
      cluster==policy_.pct_frontiers_.clusters.end()?0:cluster->attempt,policy_.pct_frontiers_.scope.c_str(),
      static_cast<unsigned long long>(policy_.pct_frontiers_.version),
      cluster==policy_.pct_frontiers_.clusters.end()?0.:cluster->observer_ms,planning_ms,
      result.floor_attempt.c_str(),reason);
  return true;
}

double ExplorationController::cross_layer_minimum_travel() const {
  return options_.observation_goal_xy_tolerance_+shared_layer_->xy_resolution;
}

bool ExplorationController::try_floor_stair_probe(std::uint64_t version,double current,bool depart) {
  if(!options_.cross_layer_enabled_ || !base_yaw_ || current-yaw_wall_>options_.odom_timeout_ ||
     !base_quiet_since_ || current-*base_quiet_since_<.30)return false;
  const auto probe=selection_phase(selection_timings_,"stair_probe_ms",[&] {
    return shared_layer_->pct->floor_stair_probe(version,*base_,*current_pct_floor(),
        *base_yaw_,depart,policy_.floor_complete_,options_.observation_goal_xy_tolerance_);
  });
  if((probe.active || probe.finished) && probe.metadata!=policy_.stair_probe_last_diagnostic_) {
    RCLCPP_INFO(logger_,"Stair recognition: %s",probe.metadata.c_str());
    policy_.stair_probe_last_diagnostic_=probe.metadata;
  }
  observation_metadata_=probe.metadata;
  policy_.local_stair_probe_active_=probe.active;
  if(probe.finished) {report("pct_local_stair_probe_finished",probe.reason);return false;}
  if(probe.frontier.observations.empty()) {
    if(probe.active)report("pct_local_stair_probe_waiting",probe.reason);
    return probe.active;
  }
  const auto &point=probe.frontier.observations.front();
  if(std::hypot(point[0]-(*base_)[0],point[1]-(*base_)[1])<=options_.observation_goal_xy_tolerance_ &&
     std::abs(point[2]+options_.layer_body_height_-(*base_)[2])<=.04) {
    if(probe.returning) {
      const auto checked=shared_layer_->pct->observation_task(version,
          "finish_local_stair_return",0,*base_,0.,options_.observation_goal_xy_tolerance_);
      policy_.local_stair_probe_active_=!checked.ready;
      report("pct_local_stair_probe_finished");return checked.ready?false:true;
    }
    start_floor_stair_scan(probe.frontier.observation_yaws.front(),current);return true;
  }
  static const std::unordered_map<Cell,int,CellHash> no_grid;
  static const std::vector<XY> no_obstacles;
  pct_round_deadline_=wall_now()+options_.pct_round_budget_;
  pct_round_exhausted_=false;
  std::string failure;
  if(prepare_route({point[0],point[1]},no_grid,no_obstacles,
      probe.climbing && depart?"pct_stair_climb":
      probe.approaching?"pct_stair_approach":
      probe.returning?"pct_local_stair_return":"pct_local_stair_probe",0.,&probe.frontier,
      std::nullopt,RecoveryRequest::position,false,std::nullopt,&failure)) {
    if(probe.climbing && depart) {
      shared_layer_->pct->observation_task(version,"finish_stair_recognition",0,*base_);
      policy_.local_stair_probe_active_=false;
    }
    return true;
  }
  const Point rejected{point[0],point[1],point[2]+options_.layer_body_height_};
  const auto outcome=shared_layer_->pct->observation_task(version,"reject_stair_stance",0,rejected,
      probe.frontier.observation_yaws.front());
  RCLCPP_INFO(logger_,"Stair recognition route rejected (%s): %s",
              failure.c_str(),outcome.metadata.c_str());
  report("pct_local_stair_probe_waiting",failure.empty()?"no_verified_route":failure);return true;
}

bool ExplorationController::try_stair_route(StairRoute kind,std::optional<XY> platform_floor,
    PctFrontier *blocked) {
  const auto &views=kind==StairRoute::next_step?policy_.pct_frontiers_.stair_step_targets:
      kind==StairRoute::landing?policy_.pct_frontiers_.stair_landing_targets:policy_.pct_frontiers_.stair_targets;
  auto *outcome=kind==StairRoute::climb?&policy_.stair_climb_result_:
                kind==StairRoute::next_step?&policy_.stair_step_result_:nullptr;
  PctFrontier candidate;
  candidate.revision=policy_.pct_frontiers_.version;
  for(const auto &view:views) {
    if(platform_floor && std::abs(view.ground[2]-(*platform_floor)[0])>(*platform_floor)[1])continue;
    const auto cooldown=policy_.pct_timed_out_observers_.find(view.ground);
    if((cooldown!=policy_.pct_timed_out_observers_.end() && cooldown->second>wall_now()) ||
       (policy_.cross_task_.engaged() && (!policy_.cross_task_.allows(view.ground) ||
                                  policy_.failed_stair_observers_.count(view.ground))))continue;
    candidate.observations.push_back(view.ground);
    candidate.observation_yaws.push_back(view.yaw);
  }
  if(candidate.observations.empty()) {
    if(outcome)*outcome=views.empty()?"no_candidates":"candidates_filtered";
    return false;
  }
  candidate.frontier=candidate.observations.front();
  static const std::unordered_map<Cell,int,CellHash> no_grid;
  static const std::vector<XY> no_obstacles;
  pct_round_deadline_=wall_now()+options_.pct_round_budget_;
  pct_round_exhausted_=false;
  std::string failure;
  const bool ready=prepare_route({candidate.frontier[0],candidate.frontier[1]},no_grid,no_obstacles,
      kind==StairRoute::landing?"pct_stair_landing":
      kind==StairRoute::next_step?"pct_stair_step":"pct_stair_climb",0.,&candidate,
      std::nullopt,RecoveryRequest::position,false,platform_floor,&failure,
      blocked?&policy_.pct_frontiers_.unknown_blockers:nullptr,blocked);
  if(outcome)*outcome=ready?"route_ready":(failure.empty()?"route_unavailable":failure);
  return ready;
}

bool ExplorationController::skipping_start_floor() const {
  return options_.skip_start_floor_exploration_ && options_.cross_layer_enabled_ && options_.floor_exploration_ && policy_.floor_number_==1;
}

void ExplorationController::dispatch_floor_stairs(bool can_send,
    const char *waiting_state) {
  if(can_send && try_floor_stair_probe(shared_layer_->current_version.load(),wall_now(),true))return;
  report(waiting_state,"no_verified_stair_task");
}

FloorViewLimits ExplorationPolicy::frontier_view_limits(
    FloorViewLimits limits, bool ascending, double max_step) const {
  if (ascending) {
    limits.max_attempts = 4;
    limits.max_step = std::min(.16, max_step);
    limits.min_rise = cross_task_.minimum_rise;
  }
  return limits;
}

ExplorationController::FrontierSelection ExplorationController::select_exploration_target(std::optional<XY> platform, bool cross_layer,
    std::function<bool()> measured_ascent) {
  static const std::unordered_map<Cell,int,CellHash> no_grid;
  static const std::vector<XY> no_obstacles;
  const char *policy=cross_layer?(platform?"pct_platform_frontier":"pct_stair_frontier"):
      policy_.pct_frontiers_.fast_floor?"pct_frontier_fast":
      policy_.pct_frontiers_.reviewing?"pct_frontier_review":"pct_frontier";
  const auto continue_measured_ascent=[&] {
    auto fallback=std::exchange(measured_ascent,{});
    return fallback && fallback();
  };
  if(measured_ascent && continue_measured_ascent())return FrontierSelection::dispatched;
  std::string settled;
  for(const auto &cluster:policy_.pct_frontiers_.clusters) {
    if(!cluster.attempt)continue;
    const bool no_view=std::find(policy_.pct_frontiers_.no_view_clusters.begin(),
        policy_.pct_frontiers_.no_view_clusters.end(),cluster.id)!=policy_.pct_frontiers_.no_view_clusters.end();
    const bool timeout=std::find(policy_.pct_frontiers_.observer_timeout_clusters.begin(),
        policy_.pct_frontiers_.observer_timeout_clusters.end(),cluster.id)!=policy_.pct_frontiers_.observer_timeout_clusters.end();
    if(!no_view && !timeout)continue;
    // Ordinary selection has already admitted the actual start. An unsafe
    // candidate is a failed frontier attempt, not a reason to move that start.
    if(cross_layer && !cluster.unknown_blockers.empty() &&
       begin_blocker_recovery(cluster,cluster.unknown_blockers,cross_layer,cluster.attempt,policy,
                              std::nullopt,platform))
      return FrontierSelection::recovery;
    const char *reason=timeout?"floor_observer_timeout":
        cluster.floor_failure.empty()?"floor_no_observation":cluster.floor_failure.c_str();
    if(settle_unavailable_floor_cluster(cluster.id,reason,cluster.first_rejected))settled=reason;
  }
  for(auto cluster:policy_.pct_frontiers_.clusters) {
    if(!cluster.attempt || cluster.observations.empty())continue;
    ++last_candidate_goals_;
    pct_round_deadline_=wall_now()+options_.pct_round_budget_;
    pct_round_exhausted_=false;
    std::vector<PctUnknownBlocker> blockers;
    PctFrontier blocked;
    std::string failure;
    const double started=wall_now();
    if(prepare_route({cluster.frontier[0],cluster.frontier[1]},no_grid,no_obstacles,policy,0.,&cluster,
                      std::nullopt,RecoveryRequest::position,false,platform,&failure,&blockers,&blocked)) {
      policy_.pct_next_candidate_=0;
      policy_.pct_global_fallback_=false;
      policy_.pct_failed_local_rounds_=0;
      return FrontierSelection::dispatched;
    }
    if(continue_measured_ascent())return FrontierSelection::dispatched;
    if(cross_layer && !blockers.empty() && begin_blocker_recovery(blocked,blockers,cross_layer,cluster.attempt,policy,std::nullopt,platform))
      return FrontierSelection::recovery;
    if(failure.empty() || failure=="pct_unsafe_start" || failure=="pct_backend_unavailable" ||
       failure=="layer_map_version_changed" || failure=="layer_map_unavailable_or_changed" ||
       failure=="no_inflated_safe_layer_surface")
      return FrontierSelection::pending;
    const char *reason=failure=="pct_planning_timeout" || pct_round_exhausted_?
        "floor_route_timeout":"floor_no_route";
    if(settle_unavailable_floor_cluster(cluster.id,reason,cluster.observations.front(),
                                       1000.*(wall_now()-started)))settled=reason;
    break;
  }
  if(!settled.empty()) {
    report("pct_floor_frontier_settled",settled);
    return FrontierSelection::pending;
  }
  if(policy_.pct_frontiers_.query_pending) {
    if(cross_layer && !platform && !policy_.pct_frontiers_.unknown_blockers.empty() &&
       begin_blocker_recovery({},policy_.pct_frontiers_.unknown_blockers,true))
      return FrontierSelection::recovery;
    report(cross_layer?"cross_layer_waiting_for_forward_route":"pct_local_search_pending",
           "observer_query_budget");
    return FrontierSelection::pending;
  }
  return FrontierSelection::exhausted;
}

void ExplorationController::run_pct_cycle(double current) {
  if(base_ && current-base_wall_<=options_.odom_timeout_)policy_.cross_task_.update(cross_ground_pose());
  if(handle_observation(current))return;
  // Route handoff does not select a target or read/revalidate a map. Keep
  // it independent of a concurrent map commit or visualization refresh.
  if(active_) {
    empty_since_.reset();
    report(active_->stamp?"waiting_for_scan_completion":"waiting_for_scan_forward");return;
  }
  if(path_processor_.active()) {
    if(!try_dispatch_route())report("pct_waiting_for_control");
    return;
  }
  if(recovery_.stair_reposition_) {handle_stair_reposition(current);return;}
  if(recovery_.blocker_recovery_) {run_blocker_recovery(current);return;}
  if (!base_ || current-base_wall_>options_.odom_timeout_ || !layer_map_fresh() ||
      pct_cloud_wall_==0.0 || current-pct_cloud_wall_>options_.cloud_timeout_) {
    empty_since_.reset();report("waiting_for_pct_data");return;
  }
  selection_cycle_started_=wall_now();
  selection_timings_.clear();
  LayerMapReadPhase decision_lock(*shared_layer_);
  selection_timings_["read_prepare_ms"]=1000.*(wall_now()-selection_cycle_started_);
  selection_timings_["read_wait_ms"]=shared_layer_->last_read_wait_ms.load();
  selection_timings_["read_commit_ms"]=shared_layer_->last_read_commit_ms.load();
  if (control_mode_==ExplorationMode::waiting_initial_data)
    control_mode_=ExplorationMode::flat;
  static const std::unordered_map<Cell,int,CellHash> no_grid;
  static const std::vector<XY> no_obstacles;
  const bool can_send=!options_.observation_only_ && !paused_ && ready_ &&
      output_.waypoint_subscribers()>0;
  const auto version=shared_layer_->current_version.load();
  std::optional<double> platform_ground;
  if(selection_phase(selection_timings_,"platform_check_ms",[&] {
      return cross_layer_platform_ready(&platform_ground);
  })) {
    report("cross_layer_completed");return;
  }
  if(can_send && policy_.cross_task_.deferred() &&
     shared_layer_->last_integration_wall_seconds.load()>policy_.cross_deferred_evidence_time_) {
    const auto signature=pct_recovery_evidence();
    const bool feet_stable=stable_four_foot_support(current);
    const bool new_view=!policy_.cross_deferred_checked_ || signature!=policy_.cross_deferred_signature_ ||
        (feet_stable && !policy_.cross_deferred_feet_stable_) ||
        !policy_.cross_deferred_pose_ || std::hypot((*base_)[0]-(*policy_.cross_deferred_pose_)[0],
                                          (*base_)[1]-(*policy_.cross_deferred_pose_)[1])>.10 ||
        std::abs((*base_)[2]-(*policy_.cross_deferred_pose_)[2])>.05 ||
        (policy_.cross_deferred_scan_blocked_ && !scan_start_inflated(*base_,policy_.cross_task_.ascent_yaw()));
    if(new_view) {
      policy_.cross_deferred_checked_=true;
      policy_.cross_deferred_feet_stable_=feet_stable;
      policy_.cross_deferred_signature_=signature;
      policy_.cross_deferred_pose_=*base_;
      policy_.cross_deferred_scan_blocked_=scan_start_inflated(*base_,policy_.cross_task_.ascent_yaw());
      if(policy_.cross_task_.can_resume(cross_ground_pose()) &&
         shared_layer_->pct->start_ready(version,*base_)) {
        policy_.cross_task_.resume();
        control_mode_=ExplorationMode::cross_layer;
        policy_.pct_global_fallback_=false;
      } else if(policy_.floor_height_ && stable_foot_height_ &&
                std::abs(*stable_foot_height_-*policy_.floor_height_)<=.04 &&
                confirmed_floor_height(version)) {
        policy_.cross_task_.clear();
        control_mode_=ExplorationMode::flat;
        policy_.schedule_global();
      }
    }
  }
  if(options_.floor_exploration_ && policy_.cross_task_.deferred()) {
    report("cross_layer_deferred",policy_.cross_task_.reason());return;
  }
  if(options_.floor_exploration_ && !policy_.floor_height_ && ready_) {
    policy_.floor_height_=confirmed_floor_height(version);
    if(policy_.floor_height_) {
      policy_.floor_number_=1;
      RCLCPP_INFO(logger_,"PCT floor %zu reference height %.3f m",policy_.floor_number_,*policy_.floor_height_);
      if(skipping_start_floor())
        RCLCPP_INFO(logger_,"Initial floor uses shared exploration until a usable staircase is recognized");
    }
  }
  // Ordinary floor selection only records ascending evidence. Confirmation
  // and its movement/scan actions belong to floor handoff (or explicit skip).
  if(policy_.floor_height_ && !policy_.cross_task_.engaged() && can_send &&
     (skipping_start_floor() || policy_.floor_complete_) &&
     try_floor_stair_probe(version,current,true))return;
  std::optional<Point> continuation;
  if(policy_.cross_task_.engaged())
    continuation=Point{policy_.cross_task_.ascent_yaw(),policy_.cross_task_.highest_height()-policy_.cross_task_.height_slack,
                           policy_.cross_task_.progress_slack};
  if(options_.floor_exploration_ && policy_.floor_height_ && !continuation &&
     shared_layer_->last_integration_wall_seconds.load()<=policy_.pct_floor_wait_for_frame_after_) {
    report("pct_floor_waiting_for_observation_frame");return;
  }
  const bool markers_already_dirty=pct_markers_dirty_;
  {
    // Same read-lock contract as native planning; no dense map copy.
    {
      LayerMapReadPhase lock(*shared_layer_);
      policy_.pct_frontiers_=selection_phase(selection_timings_,"frontiers_ms",[&] {
        return shared_layer_->pct->frontiers(shared_layer_->current_version.load(),*base_,
          continuation?cross_layer_minimum_travel():options_.observation_min_distance_,
          (!continuation && policy_.pct_global_fallback_)?std::nullopt:
              std::optional<Point>({cfg_.local_search_radius,options_.pct_local_above_,options_.pct_local_below_}),
          options_.pct_query_budget_,continuation,
          policy_.floor_height_?std::optional<XY>({*policy_.floor_height_,options_.floor_height_tolerance_}):std::nullopt,
          false,std::nullopt,
          policy_.floor_height_?std::optional{policy_.frontier_view_limits(options_.pct_floor_view_limits_, continuation.has_value(),
                                          shared_layer_->max_step_height)}:std::nullopt,
          base_yaw_);
      });
      for(const auto &[phase,elapsed]:policy_.pct_frontiers_.observer_phase_ms)
        selection_timings_[phase]+=elapsed;
    }
    pct_markers_dirty_=true;
  }
  layer_version_=policy_.pct_frontiers_.version;
  if(policy_.failed_stair_version_!=policy_.pct_frontiers_.geometry_version)policy_.failed_stair_observers_.clear();
  last_frontier_cells_=policy_.pct_frontiers_.frontier_cells;
  last_frontier_clusters_=policy_.pct_frontiers_.clusters.size();
  last_candidate_goals_=0;last_suppressed_goals_=0;
  if (!policy_.pct_frontiers_.valid) {
    empty_since_.reset();report("pct_frontier_unavailable","pct_backend_error");return;
  }
  if (!policy_.pct_frontiers_.known_surface_cells || !policy_.pct_frontiers_.safe_cells) {
    empty_since_.reset();
    report("pct_map_without_safe_surface");
    return;
  }
  // Unsafe/disconnected starts cannot prove that exploration is complete.
  bool start_ready=false;
  std::string start_error;
  {
    LayerMapReadPhase lock(*shared_layer_);
    start_ready=selection_phase(selection_timings_,"start_check_ms",[&] {
      return shared_layer_->pct->start_ready(shared_layer_->current_version.load(),*base_,
          !continuation?current_pct_floor():std::nullopt);
    });
    start_error=shared_layer_->pct->last_error();
  }
  if (!start_error.empty()) {
    policy_.pct_frontiers_.valid=false;policy_.pct_frontiers_.error=start_error;
    empty_since_.reset();report("pct_frontier_unavailable","pct_backend_error");return;
  }
  if (!start_ready) {
    empty_since_.reset();
    if(can_send && continuation) {
      report("cross_layer_recovery","pct_unsafe_start");
      const auto current_check=shared_layer_->pct->point_check(version,*base_);
      if(current_check.physical_conflict || current_check.reason=="solid_or_steep_surface") {
        try_pct_landing_shift(current);
        return;
      }
      if(begin_blocker_recovery({},policy_.pct_frontiers_.unknown_blockers,true))return;
      if(!policy_.pct_frontiers_.unknown_blockers.empty()) {
        defer_cross_layer("stair_gap_no_verified_view");
        report("cross_layer_deferred",policy_.cross_task_.reason());
      } else defer_cross_layer("pct_unsafe_start");
      return;
    }
    if(can_send) {
      const auto check=shared_layer_->pct->point_check(version,*base_,false,std::nullopt,0.,false,current_pct_floor());
      if(begin_blocker_recovery({},policy_.pct_frontiers_.unknown_blockers,false,0,"pct_frontier",check.reason))return;
    }
    report("pct_unsafe_start");return;
  }
  if(options_.floor_exploration_ && !policy_.floor_height_) {
    empty_since_.reset();
    report("pct_waiting_for_floor_height",ready_?policy_.floor_confirmation_reason_:"execution_not_ready");return;
  }
  if(options_.floor_exploration_ && !policy_.cross_task_.engaged()) {
    if(!can_send) {
      empty_since_.reset();
      report(options_.observation_only_?"pct_observing":paused_?"paused":"pct_waiting_for_control");return;
    }
    if(policy_.pct_frontiers_.scoped_frontier_cells) {
      policy_.floor_complete_=false;
      empty_since_.reset();
    } else if(!policy_.pct_global_fallback_) {
      policy_.schedule_global();
      report("pct_floor_local_empty","verify_current_floor_globally");return;
    } else if(!policy_.pct_frontiers_.query_pending && !policy_.pct_frontiers_.pending_review_clusters) {
      const char *waiting_state=policy_.floor_complete_?"pct_waiting_for_stairs":
          (policy_.pct_frontiers_.deferred_floor_frontier_cells?
           "pct_floor_handoff_with_deferred_frontiers":"pct_floor_complete");
      policy_.floor_complete_=true;
      dispatch_floor_stairs(can_send,waiting_state);return;
    }
  }
  std::optional<XY> platform_floor;
  PctFrontier blocked_goal;
  const auto try_stair_continuation=[&]() {
    return try_stair_route(StairRoute::next_step,std::nullopt,&blocked_goal) ||
           try_stair_route(StairRoute::climb,std::nullopt,&blocked_goal);
  };
  const auto finish_platform_search=[&]() {
    if(can_send && !policy_.pct_frontiers_.query_pending) {
      if(!policy_.pct_global_fallback_) {
        policy_.schedule_global();
        report("cross_layer_platform_local_empty","verify_platform_globally");return;
      }
      policy_.pct_next_candidate_=0;
      if(try_stair_route(StairRoute::landing,platform_floor,&blocked_goal) ||
         try_stair_continuation())return;
      recover_blocked_ascent(blocked_goal);
    } else report("cross_layer_platform_waiting",policy_.pct_frontiers_.query_pending?
        "observer_query_budget":"no_same_height_observation");
  };
  if(options_.floor_exploration_ && policy_.cross_task_.engaged()) {
    if(can_send)policy_.stair_step_result_.clear();
    const bool platform_visible=policy_.pct_frontiers_.stair_platform_visible || policy_.cross_task_.platform_height().has_value();
    std::optional<XY> landing_floor;
    if(platform_ground)landing_floor=XY{*platform_ground,.05};
    if(platform_visible && landing_floor) {
      policy_.cross_task_.exploring_platform((*landing_floor)[0]);
      platform_floor=XY{*policy_.cross_task_.platform_height(),.05};
    }
    else if(policy_.cross_task_.platform_height())platform_floor=XY{*policy_.cross_task_.platform_height(),.05};
    if(!platform_floor) {
      if(!can_send)report("cross_layer_waiting_for_forward_route","execution_not_ready");
      else {
        const auto selected=select_exploration_target(std::nullopt,true,try_stair_continuation);
        if(selected==FrontierSelection::exhausted &&
           (!landing_floor || !try_stair_route(StairRoute::landing,landing_floor,&blocked_goal)))
          recover_blocked_ascent(blocked_goal);
      }
      return;
    }
    auto stairs=std::move(policy_.pct_frontiers_);
    auto platform_base=*base_;
    platform_base[2]=(*platform_floor)[0]+options_.layer_body_height_;
    policy_.pct_frontiers_=shared_layer_->pct->frontiers(version,platform_base,cross_layer_minimum_travel(),
        policy_.pct_global_fallback_?std::nullopt:
            std::optional<Point>({cfg_.local_search_radius,options_.pct_local_above_,options_.pct_local_below_}),
        options_.pct_query_budget_,Point{policy_.cross_task_.ascent_yaw(),(*platform_floor)[0]-(*platform_floor)[1],
                                    policy_.cross_task_.progress_slack},current_pct_floor(),false,platform_floor,
        options_.pct_floor_view_limits_);
    policy_.pct_frontiers_.stair_flights=stairs.stair_flights;
    policy_.pct_frontiers_.stair_platform_visible=stairs.stair_platform_visible;
    policy_.pct_frontiers_.unknown_blockers=std::move(stairs.unknown_blockers);
    policy_.pct_frontiers_.stair_step_unsafe=stairs.stair_step_unsafe;
    policy_.pct_frontiers_.stair_targets=std::move(stairs.stair_targets);
    policy_.pct_frontiers_.stair_step_targets=std::move(stairs.stair_step_targets);
    policy_.pct_frontiers_.stair_landing_targets=std::move(stairs.stair_landing_targets);
    last_frontier_cells_=policy_.pct_frontiers_.frontier_cells;
    last_frontier_clusters_=policy_.pct_frontiers_.clusters.size();
    if(!policy_.pct_frontiers_.valid) {report("pct_frontier_unavailable","pct_platform_query");return;}
    if(can_send && select_exploration_target(platform_floor,true)==FrontierSelection::exhausted)
      finish_platform_search();
    return;
  }
  if(options_.floor_exploration_ && policy_.floor_height_ && !policy_.cross_task_.engaged()) {
    if(select_exploration_target(std::nullopt,false)==FrontierSelection::exhausted &&
       !policy_.pct_global_fallback_) {
      policy_.schedule_global();
      report("pct_local_exhausted","try_global_next_tick");
    }
    return;
  }
  if (!policy_.pct_frontiers_.frontier_cells) {
    if(policy_.cross_task_.engaged() || policy_.cross_task_.deferred()) {
      if(can_send && policy_.cross_task_.engaged() && !policy_.pct_frontiers_.query_pending)
        recover_blocked_ascent();
      else report("cross_layer_waiting_for_frontier");
      return;
    }
    if (!empty_since_) empty_since_=current;
    report(current-*empty_since_>=options_.no_frontier_hold_?"pct_region_complete":"pct_frontiers_empty_hold");
    return;
  }
  empty_since_.reset();
  auto ranked = policy_.rank_frontiers(current, cross_ground_pose(), base_yaw_,
                                        cfg_, options_.pct_direction_weight_);
  last_candidate_goals_ += ranked.considered;
  last_suppressed_goals_ += ranked.suppressed;
  const auto &candidates = ranked.candidates;
  if (candidates.empty()) {
    if(policy_.cross_task_.engaged()) {
      if(can_send && !policy_.pct_frontiers_.query_pending)recover_blocked_ascent();
      else report("cross_layer_waiting_for_forward_route");
      return;
    }
    if(policy_.pct_frontiers_.query_pending) {report("pct_local_search_pending","observer_query_budget");return;}
    if(!policy_.pct_global_fallback_) {
      const bool pending=std::any_of(policy_.pct_frontiers_.clusters.begin(),policy_.pct_frontiers_.clusters.end(),[&](const auto &c) {
        auto at=policy_.pct_timeout_pending_.find(c.id);return at!=policy_.pct_timeout_pending_.end() && at->second>current;
      });
      if(pending) {report("pct_local_search_pending","timeout_is_not_local_completion");return;}
      policy_.schedule_global();
      report("pct_local_exhausted","try_global_next_tick");return;
    }
    policy_.pct_global_fallback_=false;
    report(last_candidate_goals_?"pct_candidates_cooling_down":"pct_frontiers_without_new_observation");
    return;
  }
  if (!can_send) {
    report(options_.observation_only_?"pct_observing":paused_?"paused":"pct_waiting_for_control");return;
  }
  // Resume after the last attempted cluster across ticks. A slow set of
  // high-gain failures must not starve the rest of the candidate pool.
  std::size_t first=0;
  if(policy_.pct_next_candidate_) {
    auto at=std::find_if(candidates.begin(),candidates.end(),[&](const auto &c){return c.second.id==policy_.pct_next_candidate_;});
    if(at!=candidates.end())first=static_cast<std::size_t>(at-candidates.begin());
  }
  pct_round_deadline_=wall_now()+options_.pct_round_budget_;pct_round_exhausted_=false;
  if(pct_observation_progress_.size()>4096)pct_observation_progress_.clear();
  const auto batch_count=std::min<std::size_t>(candidates.size(),4);
  for (std::size_t n=0;n<batch_count;++n) {
    const auto i=(first+n)%candidates.size();
    if(wall_now()>=pct_round_deadline_) {pct_round_exhausted_=true;break;}
    const auto &c=candidates[i].second;
    policy_.pct_next_candidate_=candidates[(i+1)%candidates.size()].second.id;
    std::vector<PctUnknownBlocker> blockers;
    const char *policy=policy_.pct_frontiers_.fast_floor?"pct_frontier_fast":
        policy_.pct_frontiers_.reviewing?"pct_frontier_review":"pct_frontier";
    if (prepare_route({c.frontier[0],c.frontier[1]},no_grid,no_obstacles,policy,candidates[i].first,&c,
                      std::nullopt,RecoveryRequest::position,false,std::nullopt,nullptr,
                      policy_.cross_task_.engaged()?&policy_.pct_frontiers_.unknown_blockers:&blockers,&blocked_goal)) {
      policy_.pct_next_candidate_=0;policy_.pct_global_fallback_=false;
      policy_.pct_failed_local_rounds_=0;policy_.pct_local_failure_since_=0.;return;
    }
    if(!policy_.cross_task_.engaged() && (!blockers.empty() || !blocked_goal.observations.empty()) &&
       begin_blocker_recovery(blocked_goal,blockers,false,
           0,policy))return;
    if(!policy_.cross_task_.engaged())blocked_goal={};
    if(pct_round_exhausted_)break;
  }
  if(!policy_.cross_task_.engaged() && !policy_.pct_global_fallback_) {
    if(policy_.pct_failed_local_rounds_++==0)policy_.pct_local_failure_since_=current;
    if(policy_.pct_failed_local_rounds_>=4 && current-policy_.pct_local_failure_since_>=8.) {
      policy_.schedule_global();
      report("pct_local_deferred","local_planning_retry_limit");return;
    }
  }
  if(pct_round_exhausted_) {report("pct_planning_yield","candidate_round_budget");return;}
  if(policy_.pct_frontiers_.query_pending) {report("pct_local_search_pending","observer_query_budget");return;}
  if(policy_.cross_task_.engaged() &&
     (candidates.size()<=4 || !policy_.pct_frontiers_.unknown_blockers.empty())) {
    recover_blocked_ascent(blocked_goal);return;
  }
  report("pct_frontiers_unresolved","no_usable_pct_route");
}

}  // namespace legbot_exploration
