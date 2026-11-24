#include "exploration_controller.hpp"
#include "exploration_controller_util.hpp"

namespace legbot_exploration {

bool ExplorationController::refresh_exploration_height() {
  if(!options_.floor_exploration_ || !base_ || !layer_map_fresh())return false;
  LayerMapReadPhase lock(*shared_layer_);
  const auto version=shared_layer_->current_version.load();
  const auto support=shared_layer_->pct->point_check(version,*base_,false,std::nullopt,
      0.,false,std::nullopt,true);
  policy_.floor_confirmation_reason_=support.reason;
  if(!support.ready || !support.ground_z)return false;
  const double height=*support.ground_z;
  if(policy_.floor_height_ &&
     std::abs(height-*policy_.floor_height_)<=options_.floor_height_tolerance_+1e-7)return false;
  policy_.finish_floor_selection_wait(wall_now());
  policy_.enter_floor(height);
  const auto entered=shared_layer_->pct->observation_task(version,"enter_frontier_floor",0,
      {(*base_)[0],(*base_)[1],height},0.,options_.floor_height_tolerance_);
  height_review_status_=entered.metadata;
  // A height band is a scheduling reference, not the end of the accepted
  // ascent. Preserve its progress and offer safe upward work before rail views.
  policy_.cross_priority_pending_=policy_.cross_task_.engaged() &&
      policy_.cross_task_.phase()!="exploring_platform";
  policy_.cross_deferred_checked_=false;
  control_mode_=policy_.cross_task_.engaged()?ExplorationMode::cross_layer:ExplorationMode::flat;
  empty_since_.reset();
  pct_observation_progress_.clear();
  stair_route_progress_.clear();
  RCLCPP_INFO(logger_,"Exploration height band %zu reference %.3f m",policy_.floor_number_,height);
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
  floor_exploration_started_sim_.reset();floor_exploration_started_wall_.reset();
  floor_exploration_ended_sim_.reset();floor_exploration_ended_wall_.reset();
  floor_selection_wait_started_.reset();
  floor_longest_selection_wait_ms_=0.;
  floor_review_started_.reset();
  floor_review_elapsed_=0.;
  floor_review_resolved_=floor_known_barriers_=0;
  pct_global_fallback_=false;
  pct_next_candidate_=0;
  pct_cooldowns_.clear();
  failed_stair_observers_.clear();
  pct_timeout_pending_.clear();
  pct_timed_out_observers_.clear();
}

void ExplorationPolicy::finish_floor_selection_wait(double now) {
  if(floor_review_started_) {
    floor_review_elapsed_+=now-*floor_review_started_;
    floor_review_started_.reset();
  }
  if(!floor_selection_wait_started_)return;
  floor_longest_selection_wait_ms_=std::max(floor_longest_selection_wait_ms_,
      1000.*(now-*floor_selection_wait_started_));
  floor_selection_wait_started_.reset();
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
      if(((cross_task_.engaged() || cross_task_.deferred()) && (observer_failed(p,current) || !cross_task_.allows(p))) ||
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

bool ExplorationPolicy::observer_failed(const Point &point,double current) const {
  return std::any_of(failed_stair_observers_.begin(),failed_stair_observers_.end(),[&](const auto &entry) {
    const auto &[old,expiry]=entry;
    return expiry>current && std::hypot(point[0]-old[0],point[1]-old[1])<=.20 &&
        std::abs(point[2]-old[2])<=.05;
  });
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
    std::optional<double> departure_height, std::optional<double> yaw,bool scheduled_ascent) {
  if (route.empty()) return false;
  cross_deferred_checked_ = false;
  if (cross_task_.deferred()) {
    auto replacement = cross_task_;
    if (cross_task_.at_progress(start)) replacement.resume();
    else {
      if (route.back()[2] - start[2] < cross_task_.minimum_rise) return false;
      replacement.clear();
    }
    if (!replacement.accept(start, route, departure_height,scheduled_ascent)) return false;
    cross_task_ = std::move(replacement);
  } else if (!cross_task_.accept(start, route, departure_height,scheduled_ascent)) {
    return false;
  }
  if (yaw) cross_task_.set_ascent_yaw(*yaw);
  cross_priority_pending_=true;
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
  if(std::string_view(reason)=="floor_known_barrier" && result.floor_attempt=="blacklisted")
    ++policy_.floor_known_barriers_;
  pct_observation_progress_.erase(id);
  const auto cluster=std::find_if(policy_.pct_frontiers_.clusters.begin(),policy_.pct_frontiers_.clusters.end(),
                                  [id](const auto &c){return c.id==id;});
  RCLCPP_INFO(logger_,
      "Height band %zu h=%.3f frontier %llu attempt=%d scope=%s map=%llu observer_ms=%.1f planning_ms=%.1f: %s (%s)",
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

const char *ExplorationController::floor_stair_motion_wait(double current) const {
  if(!base_yaw_ || current-yaw_wall_>options_.odom_timeout_)return "fresh_heading";
  // Executed routes release ownership only after SCAN's controlled-stop ACK.
  // Ordinary exploration has no second inertial-settling gate; ascending
  // selection must use the same contract. Roll/pitch jitter on a tread must
  // not indefinitely prevent planning the next publicly checked route.
  return nullptr;
}

bool ExplorationController::try_upward_support(PctFrontier *blocked,
                                               std::optional<double> minimum_height) {
  const auto &views=policy_.pct_frontiers_.upward_support_targets;
  auto *outcome=&policy_.upward_result_;
  PctFrontier candidate;
  candidate.revision=policy_.pct_frontiers_.version;
  for(const auto &view:views) {
    if(minimum_height && view.ground[2]<*minimum_height)continue;
    const auto cooldown=policy_.pct_timed_out_observers_.find(view.ground);
    if((cooldown!=policy_.pct_timed_out_observers_.end() && cooldown->second>wall_now()) ||
       ((policy_.cross_task_.engaged() || policy_.cross_task_.deferred()) && (!policy_.cross_task_.allows(view.ground) ||
                                  policy_.observer_failed(view.ground,wall_now()))))continue;
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
      "pct_upward_support",0.,&candidate,std::nullopt,RecoveryRequest::position,false,std::nullopt,&failure,
      blocked?&policy_.pct_frontiers_.unknown_blockers:nullptr,blocked);
  const char *key="pct_upward_support";
  const auto progress=stair_route_progress_.find(key);
  if(!ready && progress!=stair_route_progress_.end() && progress->second.next<progress->second.observations.size())
    pct_stair_search_pending_=true;
  if(outcome)*outcome=ready?"route_ready":(failure.empty()?"route_unavailable":failure);
  return ready;
}

bool ExplorationController::skipping_start_floor() const {
  return options_.skip_start_floor_exploration_ && options_.cross_layer_enabled_ && options_.floor_exploration_ && policy_.floor_number_==1;
}

void ExplorationController::dispatch_ascent_targets(bool can_send,
    const char *waiting_state) {
  if(!can_send || !options_.cross_layer_enabled_) {
    report(waiting_state);return;
  }
  const double current=wall_now();
  if(const auto *wait=floor_stair_motion_wait(current)) {
    report(waiting_state,wait);return;
  }
  const auto version=shared_layer_->current_version.load();
  if(!shared_layer_->pct->start_ready(version,*base_)) {
    const auto check=shared_layer_->pct->point_check(version,*base_);
    if(begin_blocker_recovery({}, {}, true, 0, "pct_stair_start", check.reason))
      report("pct_stair_start_recovery",check.reason);
    else report("pct_waiting_for_ascent",check.reason);
    return;
  }
  const auto ascent_floor=current_pct_floor();
  auto ascent_frontiers=selection_phase(selection_timings_,"ascent_frontiers_ms",[&] {
    return shared_layer_->pct->frontiers(version,*base_,cross_layer_minimum_travel(),std::nullopt,
        options_.pct_query_budget_,std::nullopt,ascent_floor,false,std::nullopt,
        policy_.frontier_view_limits(options_.pct_floor_view_limits_,true,shared_layer_->max_step_height),
        base_yaw_,true);
  });
  if(!ascent_frontiers.valid) {
    report("pct_frontier_unavailable","pct_ascent_query");return;
  }
  policy_.pct_frontiers_=std::move(ascent_frontiers);
  last_frontier_cells_=policy_.pct_frontiers_.frontier_cells;
  last_frontier_clusters_=policy_.pct_frontiers_.clusters.size();
  last_candidate_goals_=last_suppressed_goals_=0;
  pct_markers_dirty_=true;
  PctFrontier blocked;
  if(try_upward_support(&blocked))return;
  const auto &support_failure=policy_.upward_result_;
  if(support_failure=="pct_planning_timeout" || support_failure=="planning_budget_exhausted" ||
     support_failure=="pct_route_disconnected") {
    // Do not let remaining high-cut viewpoints monopolize every round after
    // a native search failure. Give the nearer cut its turn immediately.
    const auto alternative=shared_layer_->pct->observation_task(version,"advance_ascent_slice",0,*base_);
    stair_route_progress_.erase("pct_upward_support");
    pct_stair_search_pending_=alternative.ready;
    if(alternative.ready) {
      report(waiting_state,"try_next_ascent_slice_after_route_failure");return;
    }
  }
  const auto selected=select_exploration_target(std::nullopt,FrontierMode::floor_ascent);
  if(selected==FrontierSelection::dispatched || selected==FrontierSelection::recovery ||
     recovery_.blocker_recovery_ || observation_task_ || active_ || path_processor_.active())return;
  // A failed route to a measured high target is enough to try a nearer cut;
  // an unfinished frontier query on that far cut must not force a scan first.
  const bool support_route_failed=!policy_.pct_frontiers_.upward_support_targets.empty() &&
      policy_.upward_result_!="candidates_filtered" && !pct_stair_search_pending_;
  const bool supports_exhausted=policy_.pct_frontiers_.upward_support_targets.empty() &&
      !policy_.pct_frontiers_.support_query_pending;
  if((!policy_.pct_frontiers_.query_pending || support_route_failed || supports_exhausted) &&
     !pct_stair_search_pending_) {
    const auto alternative=shared_layer_->pct->observation_task(version,"advance_ascent_slice",0,*base_);
    if(alternative.ready) {
      pct_stair_search_pending_=true;
      report(waiting_state,"try_next_ascent_slice");return;
    }
  }
  // Finish the finite support pass before choosing which rejected footprint
  // to review; each batch resumes its cursor, rather than restarting forever.
  if(policy_.pct_frontiers_.support_query_pending && !pct_stair_search_pending_) {
    report(waiting_state,"support_search_pending");return;
  }
  // A pending frontier query must not starve an independent safe action or
  // the targeted evidence review required by the measured next support.
  if(!policy_.pct_frontiers_.unknown_blockers.empty() &&
     begin_blocker_recovery(blocked,policy_.pct_frontiers_.unknown_blockers,true,0,"pct_ascent_blocker"))return;
  report(waiting_state,policy_.pct_frontiers_.query_pending || pct_stair_search_pending_?
      "candidate_search_pending":"no_safe_ascending_target");
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

ExplorationController::FrontierSelection ExplorationController::select_exploration_target(
    std::optional<XY> platform, FrontierMode mode, std::optional<double> minimum_frontier_height) {
  static const std::unordered_map<Cell,int,CellHash> no_grid;
  static const std::vector<XY> no_obstacles;
  const bool cross_layer=mode==FrontierMode::cross_layer;
  const char *policy=cross_layer?"pct_platform_frontier":
      mode==FrontierMode::floor_ascent?"pct_floor_ascent_frontier":
      policy_.pct_frontiers_.reviewing?"pct_frontier_review":
      policy_.pct_frontiers_.fast_floor?"pct_frontier_fast":"pct_frontier";
  std::string settled;
  std::set<std::uint64_t> unavailable;
  const auto in_height_range=[&](const PctFrontier &cluster) {
    return !minimum_frontier_height || cluster.frontier[2]>=*minimum_frontier_height;
  };
  for(const auto &cluster:policy_.pct_frontiers_.clusters) {
    if(!cluster.attempt || !in_height_range(cluster))continue;
    const bool no_view=std::find(policy_.pct_frontiers_.no_view_clusters.begin(),
        policy_.pct_frontiers_.no_view_clusters.end(),cluster.id)!=policy_.pct_frontiers_.no_view_clusters.end();
    const bool timeout=std::find(policy_.pct_frontiers_.observer_timeout_clusters.begin(),
        policy_.pct_frontiers_.observer_timeout_clusters.end(),cluster.id)!=policy_.pct_frontiers_.observer_timeout_clusters.end();
    if(!no_view && !timeout)continue;
    unavailable.insert(cluster.id);
    // Ordinary selection has already admitted the actual start. An unsafe
    // candidate is a failed frontier attempt, not a reason to move that start.
    if(cross_layer && !cluster.unknown_blockers.empty())
      begin_blocker_recovery(cluster,cluster.unknown_blockers,cross_layer,cluster.attempt,policy,
                             std::nullopt,platform,true);
    const char *reason=timeout?"floor_observer_timeout":
        cluster.floor_failure.empty()?"floor_no_observation":cluster.floor_failure.c_str();
    if(settle_unavailable_floor_cluster(cluster.id,reason,cluster.first_rejected))settled=reason;
  }
  for(auto cluster:policy_.pct_frontiers_.clusters) {
    if(!cluster.attempt || !in_height_range(cluster) || cluster.observations.empty() ||
       unavailable.count(cluster.id))continue;
    if(cross_layer || mode==FrontierMode::floor_ascent) {
      const auto poses=cluster.observations;
      const auto yaws=cluster.observation_yaws;
      cluster.observations.clear();
      cluster.observation_yaws.clear();
      for(std::size_t view=0;view<poses.size();++view) {
        if(policy_.observer_failed(poses[view],wall_now()))continue;
        cluster.observations.push_back(poses[view]);
        if(view<yaws.size())cluster.observation_yaws.push_back(yaws[view]);
      }
      if(cluster.observations.empty())continue;
    }
    ++last_candidate_goals_;
    const double started=wall_now();
    pct_round_deadline_=started+options_.pct_round_budget_;
    if(mode==FrontierMode::floor && cluster.attempt>1 && policy_.floor_review_started_) {
      const double budget=std::min(1.,options_.pct_floor_view_limits_.review_budget);
      pct_round_deadline_=std::min({pct_round_deadline_,started+budget-cluster.observer_ms/1000.,
          *policy_.floor_review_started_+options_.pct_floor_review_wait_budget_-policy_.floor_review_elapsed_});
      if(pct_round_deadline_<=started) {
        if(settle_unavailable_floor_cluster(cluster.id,"floor_route_timeout"))settled="floor_route_timeout";
        continue;
      }
    }
    pct_round_exhausted_=false;
    std::vector<PctUnknownBlocker> blockers;
    PctFrontier blocked;
    std::string failure;
    if(prepare_route({cluster.frontier[0],cluster.frontier[1]},no_grid,no_obstacles,policy,0.,&cluster,
                      std::nullopt,RecoveryRequest::position,false,platform,&failure,&blockers,&blocked)) {
      if(mode==FrontierMode::floor)policy_.finish_floor_selection_wait(wall_now());
      policy_.pct_next_candidate_=0;
      policy_.pct_global_fallback_=false;
      policy_.pct_failed_local_rounds_=0;
      return FrontierSelection::dispatched;
    }
    if(failure.empty() || failure=="pct_unsafe_start" || failure=="pct_backend_unavailable" ||
       failure=="layer_map_version_changed" || failure=="layer_map_unavailable_or_changed" ||
       failure=="no_inflated_safe_layer_surface")
      return FrontierSelection::pending;
    if(cluster.direction_limited) {
      const bool timeout=failure=="pct_planning_timeout" || pct_round_exhausted_;
      const auto rejected=shared_layer_->pct->observation_task(shared_layer_->current_version.load(),
          timeout?"floor_reject_view_timeout":"floor_reject_view",cluster.id,
          cluster.observations.front(),0.,0.,&blockers);
      if(rejected.ready) {
        report("pct_local_search_pending","try_remaining_observation_directions");
        return FrontierSelection::pending;
      }
    }
    if(cross_layer && !blockers.empty())
      begin_blocker_recovery(blocked,blockers,cross_layer,cluster.attempt,policy,std::nullopt,platform,true);
    const char *reason=failure=="pct_planning_timeout" || pct_round_exhausted_?
        "floor_route_timeout":"floor_no_route";
    if(settle_unavailable_floor_cluster(cluster.id,reason,cluster.observations.front(),
                                       1000.*(wall_now()-started)))settled=reason;
    // Other independent candidates in this batch still get their turn.
  }
  if(!settled.empty()) {
    report(mode==FrontierMode::floor?"pct_floor_frontier_settled":"pct_ascent_frontier_settled",settled);
    return FrontierSelection::pending;
  }
  if(policy_.pct_frontiers_.query_pending) {
    report(cross_layer?"cross_layer_waiting_for_forward_route":"pct_local_search_pending",
           "observer_query_budget");
    return FrontierSelection::pending;
  }
  if(cross_layer && pct_stair_search_pending_) {
    report("cross_layer_waiting_for_forward_route","candidate_search_pending");
    return FrontierSelection::pending;
  }
  if(cross_layer && !minimum_frontier_height && resume_pending_blocker_recovery())
    return FrontierSelection::recovery;
  return FrontierSelection::exhausted;
}

bool ExplorationController::run_home_mission(double current) {
  if(!mission_returning_)return false;
  if(mission_finished_) {report("mission_complete","explored_target_and_returned_home");return true;}
  if(!base_ || !base_yaw_ || !layer_map_fresh() || current-base_wall_>options_.odom_timeout_) {
    report("return_home_waiting","fresh_map_and_pose");return true;
  }
  if(recovery_.blocker_recovery_) {run_blocker_recovery(current);return true;}
  const auto ground=cross_ground_pose();
  const auto &home=options_.mission_home_;
  if(std::hypot(ground[0]-home[0],ground[1]-home[1])<=.20 && std::abs(ground[2]-home[2])<=.08) {
    mission_finished_=true;
    RCLCPP_INFO(logger_,"Mission finished: target exploration complete and home reached (%.3f,%.3f,%.3f)",
                ground[0],ground[1],ground[2]);
    report("mission_complete","explored_target_and_returned_home");return true;
  }
  if(paused_ || !ready_ || options_.observation_only_ || output_.waypoint_subscribers()==0 ||
      current<mission_route_retry_at_) {report("return_home_waiting","route_retry_or_control");return true;}
  {
    LayerMapReadPhase lock(*shared_layer_);
    const auto version=shared_layer_->current_version.load();
    if(!shared_layer_->pct->start_ready(version,*base_)) {
      const auto check=shared_layer_->pct->point_check(version,*base_);
      begin_blocker_recovery({}, {}, false,0,"pct_home_start",check.reason);
      report("return_home_start_recovery",check.reason);return true;
    }
  }
  static const std::unordered_map<Cell,int,CellHash> no_grid;
  static const std::vector<XY> no_obstacles;
  PctFrontier target;target.frontier=home;target.observations={home};
  policy_.cross_task_.clear();
  pct_round_deadline_=wall_now()+options_.pct_round_budget_;
  std::string failure;
  if(prepare_route({home[0],home[1]},no_grid,no_obstacles,"pct_return_home",0.,&target,
      std::nullopt,RecoveryRequest::position,false,std::nullopt,&failure)) {
    report("return_home_dispatched","verified_map_route");return true;
  }
  mission_route_retry_at_=current+2.;
  report("return_home_route_unavailable",failure);return true;
}

void ExplorationController::run_pct_cycle(double current) {
  pct_stair_search_pending_=false;
  // Observation execution/pose handling must not wait behind an unrelated
  // height-review map read on this single-threaded control executor.
  if(handle_observation(current))return;
  LayerMapReadPhase ready_map(*shared_layer_,true);
  if(!ready_map.ready()) {
    map_read_retry_=true;
    selection_yielded_at_=wall_now();selection_resume_at_=selection_yielded_at_+.05;
    report("waiting_for_map_update");return;
  }
  map_read_retry_=false;
  if(pending_scan_failure_ && active_ && active_->stamp &&
     pending_scan_failure_->first==*active_->stamp) {
    const auto pending=*pending_scan_failure_;pending_scan_failure_.reset();
    handle_scan_failure(pending.first,pending.second);return;
  }
  if(base_ && current-base_wall_<=options_.odom_timeout_ && layer_map_fresh()) {
    LayerMapReadPhase lock(*shared_layer_);
    const auto review=shared_layer_->pct->observation_task(shared_layer_->current_version.load(),
        "observe_height",0,*base_);
    height_review_status_=review.metadata;
    if(review.ready)RCLCPP_INFO(logger_,"New height evidence review: %s",review.metadata.c_str());
    policy_.cross_task_.update(cross_ground_pose());
  }
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
  if(run_home_mission(current))return;
  if(base_ && current-base_wall_<=options_.odom_timeout_ && layer_map_fresh() &&
     refresh_exploration_height()) {
    report("pct_height_band_changed","prefer_same_height_frontiers");
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
  PctFrontier blocked_goal;
  if(can_send && policy_.cross_task_.engaged() &&
     policy_.cross_task_.phase()!="exploring_platform" && policy_.cross_priority_pending_) {
    policy_.cross_priority_pending_=false;
    dispatch_ascent_targets(can_send,"cross_layer_waiting_for_forward_route");
    // Once a bounded ascent search is exhausted, ordinary same-height work
    // can proceed while the accepted cross-layer context remains available.
    if(!active_ && !path_processor_.active() && !observation_task_ && !recovery_.blocker_recovery_)
      policy_.cross_priority_pending_=policy_.pct_frontiers_.query_pending || pct_stair_search_pending_;
    return;
  }
  // Exhaust the current height band before selecting an upward target.
  if(policy_.floor_height_) {
    if(policy_.floor_complete_) {
      bool available=false;
      if(can_send && version!=policy_.floor_work_checked_version_) {
        LayerMapReadPhase lock(*shared_layer_);
        policy_.floor_work_checked_version_=shared_layer_->current_version.load();
        available=shared_layer_->pct->observation_task(policy_.floor_work_checked_version_,
            "floor_work_available",0,{(*base_)[0],(*base_)[1],*policy_.floor_height_},0.,
            options_.floor_height_tolerance_).ready;
      }
      if(!available) {
        if(options_.mission_return_home_ &&
           std::abs(*policy_.floor_height_-options_.mission_target_height_)<=options_.floor_height_tolerance_ &&
           std::abs(cross_ground_pose()[2]-options_.mission_target_height_)<=.05) {
          mission_exploration_finished_=true;mission_returning_=true;
          recovery_.reset();policy_.cross_task_.clear();
          RCLCPP_INFO(logger_,"Target height exploration settled; beginning verified return-home route");
          run_home_mission(current);return;
        }
        dispatch_ascent_targets(can_send);return;
      }
      policy_.floor_complete_=false;
      policy_.pct_global_fallback_=false;
      RCLCPP_INFO(logger_,"Height band %zu resumes ordinary exploration after local map changes",policy_.floor_number_);
    }
    if(skipping_start_floor()) {
      dispatch_ascent_targets(can_send);
      return;
    }
  }
  if(options_.floor_exploration_ && policy_.floor_height_ &&
     shared_layer_->last_integration_wall_seconds.load()<=policy_.pct_floor_wait_for_frame_after_) {
    report("pct_floor_waiting_for_observation_frame");return;
  }
  if(can_send && options_.floor_exploration_ && policy_.floor_height_ &&
     !policy_.floor_selection_wait_started_)
    policy_.floor_selection_wait_started_=selection_cycle_started_;
  if(can_send && options_.floor_exploration_ && policy_.floor_height_ &&
     !policy_.floor_exploration_started_sim_) {
    policy_.floor_exploration_started_sim_=clock_->now().seconds();
    policy_.floor_exploration_started_wall_=wall_now();
    report("pct_floor_exploration_started","ordinary_local_global_search");
  }
  auto view_limits=policy_.frontier_view_limits(options_.pct_floor_view_limits_,false,
                                              shared_layer_->max_step_height);
  if(options_.floor_exploration_ && policy_.floor_height_ &&
     options_.pct_floor_review_wait_budget_>0.) {
    const double elapsed=policy_.floor_review_elapsed_+(policy_.floor_review_started_?
        wall_now()-*policy_.floor_review_started_:0.);
    view_limits.review_remaining=can_send?std::max(0.,options_.pct_floor_review_wait_budget_-elapsed):0.;
  }
  const bool markers_already_dirty=pct_markers_dirty_;
  {
    // Same read-lock contract as native planning; no dense map copy.
    {
      LayerMapReadPhase lock(*shared_layer_);
      policy_.pct_frontiers_=selection_phase(selection_timings_,"frontiers_ms",[&] {
        return shared_layer_->pct->frontiers(shared_layer_->current_version.load(),*base_,
          options_.observation_min_distance_,
          policy_.pct_global_fallback_?std::nullopt:
              std::optional<Point>({cfg_.local_search_radius,options_.pct_local_above_,options_.pct_local_below_}),
          options_.pct_query_budget_,std::nullopt,
          policy_.floor_height_?std::optional<XY>({*policy_.floor_height_,options_.floor_height_tolerance_}):std::nullopt,
          false,std::nullopt,
          policy_.floor_height_?std::optional{view_limits}:std::nullopt,
          base_yaw_);
      });
      for(const auto &[phase,elapsed]:policy_.pct_frontiers_.observer_phase_ms)
        selection_timings_[phase]+=elapsed;
    }
    pct_markers_dirty_=true;
  }
  if(view_limits.review_remaining && policy_.pct_frontiers_.reviewing && !policy_.floor_review_started_)
    policy_.floor_review_started_=selection_cycle_started_;
  layer_version_=policy_.pct_frontiers_.version;
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
      return shared_layer_->pct->start_ready(shared_layer_->current_version.load(),*base_);
    });
    start_error=shared_layer_->pct->last_error();
  }
  if (!start_error.empty()) {
    policy_.pct_frontiers_.valid=false;policy_.pct_frontiers_.error=start_error;
    empty_since_.reset();report("pct_frontier_unavailable","pct_backend_error");return;
  }
  if (!start_ready) {
    empty_since_.reset();
    if(can_send) {
      const auto check=shared_layer_->pct->point_check(version,*base_);
      if(begin_blocker_recovery({},policy_.pct_frontiers_.unknown_blockers,false,0,"pct_frontier",check.reason))return;
    }
    report("pct_unsafe_start");return;
  }
  if(options_.floor_exploration_ && !policy_.floor_height_) {
    empty_since_.reset();
    report("pct_waiting_for_floor_height",ready_?policy_.floor_confirmation_reason_:"execution_not_ready");return;
  }
  if(options_.floor_exploration_) {
    if(!can_send) {
      policy_.finish_floor_selection_wait(wall_now());
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
      const char *waiting_state=policy_.floor_complete_?"pct_waiting_for_ascent":
          (policy_.pct_frontiers_.deferred_floor_frontier_cells?
           "pct_floor_handoff_with_deferred_frontiers":"pct_floor_complete");
      policy_.finish_floor_selection_wait(wall_now());
      RCLCPP_INFO(logger_,"Height band %zu selection summary: longest_wait_ms=%.1f failed_small_clusters=%zu "
          "failed_small_cells=%zu remaining_frontier_cells=%zu review_wait_ms=%.1f "
          "review_resolved_cells=%zu review_deferred_clusters=%zu missing_connection_clusters=%zu known_barriers=%zu",
          policy_.floor_number_,policy_.floor_longest_selection_wait_ms_,
          policy_.pct_frontiers_.small_skipped_clusters,policy_.pct_frontiers_.small_skipped_cells,
          policy_.pct_frontiers_.deferred_floor_frontier_cells,1000.*policy_.floor_review_elapsed_,
          policy_.floor_review_resolved_,policy_.pct_frontiers_.deferred_review_clusters,
          policy_.pct_frontiers_.missing_connection_clusters,policy_.floor_known_barriers_);
      policy_.floor_complete_=true;
      if(!policy_.floor_exploration_ended_sim_) {
        policy_.floor_exploration_ended_sim_=clock_->now().seconds();
        policy_.floor_exploration_ended_wall_=wall_now();
      }
      report(waiting_state,"current_floor_tasks_settled");return;
    }
  }
  if(options_.floor_exploration_ && policy_.floor_height_) {
    if(select_exploration_target(std::nullopt,FrontierMode::floor)==FrontierSelection::exhausted &&
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
        auto at=policy_.pct_timeout_pending_.find(c.id);
        if(at!=policy_.pct_timeout_pending_.end() && at->second>current)return true;
        // A cluster-level retry delay may end before its timed-out views'
        // cooldown. Those views still await retry, rather than proving that
        // the local frontier container has been exhausted.
        return std::any_of(c.observations.begin(),c.observations.end(),[&](const auto &point) {
          const auto view=policy_.pct_timed_out_observers_.find(point);
          return view!=policy_.pct_timed_out_observers_.end() && view->second>current;
        });
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
