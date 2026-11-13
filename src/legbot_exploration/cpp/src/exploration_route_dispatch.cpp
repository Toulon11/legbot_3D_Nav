#include "exploration_controller.hpp"
#include "exploration_controller_util.hpp"

namespace legbot_exploration {

bool ExplorationController::try_dispatch_route() {
  if(active_ || !path_processor_.active())return false;
  if(!base_ || wall_now()-base_wall_>options_.odom_timeout_ || options_.observation_only_ || paused_ ||
     !ready_ || output_.waypoint_subscribers()==0)return false;
  publish_target(path_processor_.current(),path_processor_.recovery()?"recovery_route":
                 path_processor_.source()+"_route");
  return active_.has_value();
}

void ExplorationController::publish_target(const XY &xy, const std::string &detail) {
  if(active_ || !base_)return;
  // A latched plan_ready from the preceding waypoint must not arm this one.
  pending_scan_failure_.reset();
  pending_scan_ack_.reset();
  path_processor_.begin_execution();
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = clock_->now();
  const auto dispatch=path_processor_.dispatch(stamp_ns(msg.header.stamp),*base_);
  if(!dispatch) {report("execution_dispatch_unavailable");return;}
  msg.header.frame_id = options_.frame_id_;
  msg.pose.position.x = xy[0];
  msg.pose.position.y = xy[1];
  // The exploration launch enables SCAN message-Z. Preserve PCT XYZ;
  // occupancy/recovery XY routes use the measured current base height.
  msg.pose.position.z = path_processor_.current_z().value_or((*base_)[2]);
  msg.pose.orientation.w = 1.0;
  active_ = Active{};
  active_->xy = xy;
  active_->input_stamp = stamp_ns(msg.header.stamp);
  active_->pose_at_send = *base_;
  active_->trajectory_start_index = trajectory_.poses.size();
  active_->map_version_at_send = map_.version();
  active_->layer_version_at_send = layer_version_;
  active_->goal_id = path_processor_.goal_id();
  active_->attempt_id = path_processor_.attempt_id();
  active_->stage_index = path_processor_.index();
  active_->source_at_send = path_processor_.source();
  active_->reference_route = path_processor_.waypoints();
  active_->reference_point_count = dispatch->reference_points.size();
  const XY here{(*base_)[0], (*base_)[1]};
  active_->raw_distance_at_send = nearest_obstacle_distance(
      here, raw_obstacles_, raw_occupancy_wall_);
  active_->inflated_distance_at_send = nearest_obstacle_distance(
      here, external_obstacles_, occupancy_wall_);
  active_->recovery = control_mode_ == ExplorationMode::recovery;
  active_->observed_cells_at_send = selected_pct_?0:map_.project({(*base_)[0], (*base_)[1]}).size();
  active_scan_bspline_.reset();
  if(dispatch->request==RecoveryRequest::turn || dispatch->request==RecoveryRequest::reverse ||
     dispatch->request==RecoveryRequest::shift) {
    active_->stamp=active_->input_stamp;
    active_->turn=dispatch->request==RecoveryRequest::turn;
    active_->reverse=!active_->turn;
    path_processor_.bind_scan_goal(*active_->stamp);
    std_msgs::msg::String action;action.data=dispatch->action_payload;
    if(active_->turn) output_.recovery_turn(action);
    else {
      output_.recovery_reverse(action);
    }
  } else {
    scan_planner::msg::GoalRequest request;
    request.target=msg;request.goal_id=active_->goal_id;request.attempt_id=active_->attempt_id;
    request.waypoint_index=static_cast<std::uint32_t>(active_->stage_index);
    request.has_xy_tolerance=dispatch->xy_tolerance.has_value();
    request.xy_tolerance=dispatch->xy_tolerance.value_or(0.0);
    for(const auto &p:dispatch->reference_points) {
      geometry_msgs::msg::Point point;point.x=p[0];point.y=p[1];point.z=p[2];
      request.reference_points.push_back(point);
    }
    output_.waypoint(request);
  }
  publish_goal_diagnostic(*active_, "sent", "");
  report("goal_sent", detail);
}

void ExplorationController::publish_planned_path(const PlannedRoute &route,const std::vector<Point> *reference_xyz) {
  nav_msgs::msg::Path path;
  path.header.frame_id = options_.frame_id_;
  path.header.stamp = clock_->now();
  std::size_t point_index=0;
  for (const auto &xy : route.points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = xy[0];
    pose.pose.position.y = xy[1];
    const auto &xyz=reference_xyz?*reference_xyz:path_processor_.xyz();
    pose.pose.position.z = xyz.size()==route.points.size() ?
        xyz[point_index][2] : (base_ ? (*base_)[2] : 0.0);
    ++point_index;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  output_.planned_path(path);
  // Candidate paths have separate diagnostic topics. These three topics
  // show only the route that the path processor actually selected.
  nav_msgs::msg::Path empty;
  empty.header = path.header;
  const auto &source = path_processor_.source();
  const bool has_route = !path.poses.empty();
  output_.selected_occupancy_path(
      has_route && source == "occupancy" ? path : empty);
  output_.selected_layer_path(
      has_route && source == "layer" ? path : empty);
  output_.selected_recovery_path(
      has_route && source.rfind("recovery", 0) == 0 ? path : empty);
}

void ExplorationController::publish_candidate_path(const PlannerResult &result) {
  nav_msgs::msg::Path path;
  path.header.frame_id = options_.frame_id_;
  path.header.stamp = clock_->now();
  if (result.usable()) {
    std::size_t point_index=0;
    for (const auto &xy : result.route.points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = xy[0];
      pose.pose.position.y = xy[1];
      pose.pose.position.z = result.xyz.size()==result.route.points.size() ?
          result.xyz[point_index][2] : (base_ ? (*base_)[2] : 0.0);
      ++point_index;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
  }
  switch (result.kind) {
    case PlannerKind::occupancy: output_.occupancy_plan(path); break;
    case PlannerKind::layer: output_.layer_plan(path); break;
    case PlannerKind::recovery_layer: output_.recovery_layer_plan(path); break;
    case PlannerKind::observation_turn:
    case PlannerKind::scan_direct: break;  // No validated path to display.
  }
}

void ExplorationController::reset_planned_route() {
  path_processor_.clear();
  route_selection_target_.reset();
  selected_pct_.reset();
  selected_pct_observation_.reset();
  selected_local_stair_probe_=false;
  selected_local_stair_return_=false;
  selected_stair_approach_=false;
  selected_stair_segment_=false;
  selected_platform_frontier_=false;
  selected_fast_floor_=false;
  selected_floor_attempt_=0;
  if (options_.frontier_source_=="pct") pct_markers_dirty_=true;
  route_skipped_nearby_ = 0;
  publish_planned_path({});
  nav_msgs::msg::Path empty;
  empty.header.frame_id = options_.frame_id_;
  empty.header.stamp = clock_->now();
  output_.occupancy_plan(empty);
  output_.layer_plan(empty);
  output_.recovery_layer_plan(empty);
}

bool ExplorationController::skip_nearby_route_waypoints() {
  if (!base_) return false;
  // Layer routes have already been staged with their geometry and tolerances.
  if (path_processor_.source() == "layer" || path_processor_.source() == "recovery_layer")
    return path_processor_.active();
  const XY position{(*base_)[0], (*base_)[1]};
  route_skipped_nearby_ += path_processor_.skip_within(position, options_.goal_reach_xy_);
  return path_processor_.active();
}

bool ExplorationController::layer_map_fresh() const {
  return shared_layer_ && shared_layer_->pct &&
      shared_layer_->published_wall_seconds.load()>0 &&
      wall_now()-shared_layer_->last_sensor_wall_seconds.load()<=options_.layer_timeout_;
}

PlanningSnapshot ExplorationController::planning_snapshot(
    const std::unordered_map<Cell,int,CellHash> &grid,
    const std::vector<XY> &obstacles) const {
  return {grid,obstacles,cfg_,*base_,options_.layer_body_height_,
          layer_map_fresh(),&map_.free_observation_frames(),shared_layer_.get(),
          map_.version(),
          raw_occupancy_wall_>0. && wall_now()-raw_occupancy_wall_<=options_.occupancy_timeout_?
              &raw_obstacles_:nullptr};
}

RoutePlanner::Viewpoints RoutePlanner::find_viewpoints(
    const PlanningSnapshot &snapshot, const XY &target, const PctFrontier *frontier,
    bool recovery, bool use_occupancy, bool use_layer, const Point &attitude,
    std::optional<XY> floor) const {
  Viewpoints views;
  if (frontier) {
    for (const auto &point : frontier->observations)
      views.xy.push_back({point[0], point[1]});
    return views;  // Keep the PCT cluster's order and yaw indices.
  }
  if (recovery) {
    views.xy.push_back(target);
    return views;
  }
  if (use_occupancy)
    views.xy = observation_viewpoints(snapshot.grid, target, snapshot.settings, snapshot.scan_obstacles);
  if (use_layer) {
    auto &shared = *snapshot.shared_layer;
    LayerMapReadPhase lock(shared);
    const Point ground{target[0], target[1], snapshot.base[2] - snapshot.body_height};
    views.layer = shared.pct->observation_poses(shared.current_version.load(), snapshot.base,
        attitude, {ground}, std::nullopt, std::nullopt, floor);
    views.layer_error = shared.pct->last_error();
    for (const auto &view : views.layer)
      views.xy.push_back({view.stance[0], view.stance[1]});
  }
  std::sort(views.xy.begin(), views.xy.end(), [&](const auto &a, const auto &b) {
    const double da = xy_distance(a, target), db = xy_distance(b, target);
    return da == db ? a < b : da < db;
  });
  views.xy.erase(std::unique(views.xy.begin(), views.xy.end()), views.xy.end());
  return views;
}

RoutePlanner::Plans RoutePlanner::plan(const PlanningGoal &goal, const PlanningSnapshot &snapshot) const {
  auto candidates = plan_candidates(goal, snapshot);
  const auto *selected = select_plan(goal, candidates);
  const auto index = selected ? std::optional<std::size_t>(selected - candidates.data()) : std::nullopt;
  return {std::move(candidates), index};
}

bool ExplorationController::prepare_route(const XY &target,
    const std::unordered_map<Cell, int, CellHash> &grid,
    const std::vector<XY> &obstacles,
    const std::string &detail, double selection_score,
    const PctFrontier *pct_candidate,
    std::optional<Point> recovery_xyz,
    RecoveryRequest request,
    bool stair_short_action,
    std::optional<XY> platform_floor,
    std::string *selection_failure,
    std::vector<PctUnknownBlocker> *unknown_blockers,
    PctFrontier *blocked_goal) {
  SelectionPhaseTimer route_timer(selection_timings_,"route_ms");
  const bool recovery = stair_short_action;
  if(recovery && base_ &&
     (request==RecoveryRequest::position || request==RecoveryRequest::delegate) &&
     xy_distance({(*base_)[0],(*base_)[1]},target)<=options_.recovery_goal_xy_tolerance_) {
    // SCAN would acknowledge a delegated target already inside the tolerance.
    recovery_.recovery_last_failure_="recovery_goal_already_within_tolerance";
    return false;
  }
  if (recovery && base_)
    recovery_.recovery_last_target_distance_ = xy_distance(
        {(*base_)[0], (*base_)[1]}, target);
  if (shared_layer_)
    layer_version_ = shared_layer_->current_version.load();
  const bool layer_fresh = layer_map_fresh();
  if (options_.planner_mode_=="layer" && !layer_fresh) {
    report("waiting_for_layer_map");return false;
  }
  const auto viewpoint_snapshot = planning_snapshot(grid, obstacles);
  const auto views = route_planner_.find_viewpoints(
      viewpoint_snapshot, target, pct_candidate, recovery, options_.planner_mode_ != "layer",
      options_.planner_mode_ != "occupancy" && layer_fresh, base_attitude_, current_pct_floor());
  if (!views.layer_error.empty())
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "PCT observation search: %s", views.layer_error.c_str());
  const auto &viewpoints = views.xy;
  const auto &layer_views = views.layer;
  if (viewpoints.empty()) {
    if (recovery) recovery_.recovery_last_failure_ = "no_safe_recovery_viewpoint";
    if (!recovery) suppressed_.push_back({target, wall_now() + options_.cooldown_});
    report("route_unavailable", "no_safe_observation_position");
    return false;
  }
  nav_msgs::msg::Path empty;
  empty.header.frame_id = options_.frame_id_;
  empty.header.stamp = clock_->now();
  output_.occupancy_plan(empty);
  output_.layer_plan(empty);
  output_.recovery_layer_plan(empty);
  auto snapshot = planning_snapshot(grid, obstacles);
  if(platform_floor && !recovery)snapshot.base[2]=(*platform_floor)[0]+options_.layer_body_height_;
  std::string failure = "no_planner_result";
  bool unsafe_start_failure = false;
  bool incompatible_failure = false;
  const auto selected_goal_id=++goal_id_;
  std::size_t attempted_viewpoints=0;
  std::size_t first_viewpoint=0;
  const bool fast_floor=pct_candidate && pct_candidate->id && detail=="pct_frontier_fast";
  const int floor_attempt=pct_candidate && !recovery?pct_candidate->attempt:0;
  const bool floor_search=floor_attempt && !recovery_.blocker_recovery_;
  const auto reject_view=[&](std::size_t i,bool timeout=false,
                             const std::vector<PctUnknownBlocker> *blockers=nullptr) {
    if(floor_search)shared_layer_->pct->observation_task(layer_version_,
        timeout?"floor_reject_view_timeout":"floor_reject_view",pct_candidate->id,
        pct_candidate->observations[i],0.,0.,blockers);
  };
  const double planning_started=wall_now();
  PctObservationProgress *progress=nullptr;
  if(pct_candidate && !floor_attempt) {
    const auto &key=detail;
    progress=pct_candidate->id?&pct_observation_progress_[pct_candidate->id]:&stair_route_progress_[key];
    const auto revision=pct_candidate->id?pct_candidate->revision:
        policy_.pct_frontiers_.geometry_version;
    if(progress->revision!=revision || progress->observations!=pct_candidate->observations ||
       progress->global!=policy_.pct_global_fallback_)
      *progress={revision,pct_candidate->observations,0,policy_.pct_global_fallback_};
    first_viewpoint=progress->next;
  }
  bool search_timed_out=false;
  const bool stair_approach=detail=="pct_stair_approach";
  const bool local_probe=stair_approach || detail=="pct_local_stair_probe" || detail=="pct_local_stair_return";
  const bool stair_climb=detail=="pct_stair_climb" || detail=="pct_stair_step";
  const bool cross_layer_target=pct_candidate &&
      (policy_.cross_task_.engaged() || stair_climb || detail=="pct_stair_landing");
  const bool stair_segment=options_.floor_exploration_ && cross_layer_target && !platform_floor;
  const auto departure_height=stair_climb?policy_.floor_height_:std::nullopt;
  const double minimum_travel=stair_approach?options_.observation_goal_xy_tolerance_:
      (cross_layer_target || local_probe)?cross_layer_minimum_travel():options_.observation_min_distance_;
  const XY current_xy{(*base_)[0],(*base_)[1]};
  for (std::size_t i = first_viewpoint; i < viewpoints.size() && attempted_viewpoints<(floor_attempt?3u:12u); ++i) {
    if(pct_candidate && pct_round_deadline_>0. && wall_now()>=pct_round_deadline_) {
      if(selection_failure)*selection_failure="planning_budget_exhausted";
      pct_round_exhausted_=true;
      if(floor_attempt) {
        search_timed_out=true;
        break;
      }
      return false;
    }
    if(progress)progress->next=i+1;
    PlanningGoal goal;
    if(!recovery && xy_distance(current_xy,viewpoints[i])<minimum_travel) {
      reject_view(i);
      failure="observation_position_too_close";continue;
    }
    ++attempted_viewpoints;
    goal.goal_id = selected_goal_id;
    goal.attempt_id = ++planning_attempt_id_;
    goal.occupancy_version = map_.version();
    goal.layer_version = shared_layer_?shared_layer_->current_version.load():layer_version_;
    goal.target = target;
    goal.observation = viewpoints[i];
    goal.mode=stair_short_action?ExplorationMode::recovery:
        stair_climb?ExplorationMode::cross_layer:control_mode_;
    if(platform_floor) goal.pct_floor=platform_floor;
    else if(pct_candidate && !policy_.cross_task_.engaged() && !stair_climb && (!local_probe || stair_approach))
      goal.pct_floor=current_pct_floor();
    goal.request_occupancy=!stair_short_action && !pct_candidate && options_.planner_mode_!="layer";
    goal.request_layer=options_.planner_mode_!="occupancy";
    goal.recovery_request=recovery?request:RecoveryRequest::position;
    goal.collect_unknown_blockers=unknown_blockers!=nullptr;
    goal.stair_segment=stair_segment;
    goal.recovery_yaw=base_yaw_.value_or(0.0);
    if(recovery && (request==RecoveryRequest::position || request==RecoveryRequest::delegate))
      goal.xy_tolerance=options_.recovery_goal_xy_tolerance_;
    else if(!recovery) goal.xy_tolerance=options_.observation_goal_xy_tolerance_;
    const auto layer_view=std::find_if(layer_views.begin(),layer_views.end(),[&](const auto &view) {
      return view.stance[0]==goal.observation[0] && view.stance[1]==goal.observation[1];
    });
    if(layer_view!=layer_views.end()) {
      goal.observation_xyz=layer_view->stance;
      goal.observation_yaw=layer_view->yaw;
    }
    if(recovery_xyz) goal.observation_xyz=*recovery_xyz;
    if(goal.recovery_request==RecoveryRequest::delegate && !goal.observation_xyz)
      goal.observation_xyz=Point{goal.observation[0],goal.observation[1],(*base_)[2]};
    goal.candidate.id=goal.goal_id;
    goal.candidate.source=recovery?CandidateSource::recovery:CandidateSource::occupancy_frontier;
    goal.candidate.frontier={target[0],target[1],(*base_)[2]-options_.layer_body_height_};
    if (pct_candidate) {
      goal.candidate.source=CandidateSource::layer_frontier;
      goal.candidate.frontier=pct_candidate->frontier;
      goal.candidate.surface=SurfaceLocation{pct_candidate->cell,pct_candidate->frontier[2]};
      goal.candidate.expected_gain=pct_candidate->gain;
      goal.observation_xyz=pct_candidate->observations[i];
      if(i<pct_candidate->observation_yaws.size())goal.observation_yaw=pct_candidate->observation_yaws[i];
      (*goal.observation_xyz)[2]+=options_.layer_body_height_;
    }
    if(fast_floor && !floor_search) {
      const auto check=shared_layer_->pct->observation_task(goal.layer_version,"validate_fast_view",
          pct_candidate->id,*goal.observation_xyz,goal.observation_yaw.value_or(0.));
      if(!check.ready) {
        failure=check.reason;
        continue;
      }
    }
    goal.candidate.selection_score=selection_score;
    goal.candidate.map_version=pct_candidate?policy_.pct_frontiers_.version:map_.version();
    goal.candidate.allow_occupancy=goal.request_occupancy;
    goal.candidate.allow_layer=goal.request_layer;
    {
      std_msgs::msg::String message;
      std::ostringstream out;
      out << "{\"goal_id\":" << goal.goal_id << ",\"attempt_id\":"
          << goal.attempt_id << ",\"occupancy_version\":"
          << goal.occupancy_version << ",\"layer_version\":"
          << goal.layer_version << ",\"mode\":\""
          << mode_name(goal.mode) << "\",\"planner_mask\":\""
          << (goal.recovery_request==RecoveryRequest::delegate?"scan_direct":goal.request_occupancy?(goal.request_layer?"occupancy,layer":"occupancy"):"layer")
          << "\",\"recovery_request\":\"" << recovery_request_name(goal.recovery_request)
          << "\",\"xy_tolerance\":" << (goal.xy_tolerance?std::to_string(*goal.xy_tolerance):"null") << ",\"x\":" << target[0] << ",\"y\":" << target[1]
          << ",\"observation_x\":" << goal.observation[0]
          << ",\"observation_y\":" << goal.observation[1]
          << ",\"candidate_source\":\"" << candidate_source_name(goal.candidate.source)
          << "\",\"target_z\":" << goal.candidate.frontier[2]
          << ",\"observation_yaw\":" << (goal.observation_yaw?std::to_string(*goal.observation_yaw):"null")
          << ",\"observation_z\":" << (goal.observation_xyz?(*goal.observation_xyz)[2]:(*base_)[2])
          << ",\"candidate_map_version\":" << goal.candidate.map_version
          << ",\"selection_policy\":" << json_string(detail)
          << ",\"floor_frontier_attempt\":" << floor_attempt
          << ",\"frontier_cluster_id\":" << (pct_candidate?pct_candidate->id:0)
          << ",\"selection_scope\":\"" << (pct_candidate?policy_.pct_frontiers_.scope:"recovery_or_occupancy") << "\""
          << ",\"selection_score\":" << goal.candidate.selection_score << '}';
      message.data = out.str();
      output_.planning_goal(message);
    }
    const auto plans = route_planner_.plan(goal, snapshot);
    const auto &results = plans.candidates;
    std::vector<PctUnknownBlocker> rejected_blockers;
    for (const auto &result : results) {
      const bool disconnection=result.failure_class==PlannerFailureClass::disconnected;
      const bool has_blockers=!result.unknown_blockers.empty();
      const bool first_flat_blocker=has_blockers && !cross_layer_target &&
          unknown_blockers && unknown_blockers->empty();
      if(floor_search && !result.usable())
        rejected_blockers.insert(rejected_blockers.end(),result.unknown_blockers.begin(),result.unknown_blockers.end());
      if((!floor_search || has_blockers) && pct_candidate && blocked_goal && (first_flat_blocker ||
         (blocked_goal->observations.empty() && (has_blockers || disconnection)))) {
        *blocked_goal=*pct_candidate;
        blocked_goal->observations={pct_candidate->observations[i]};
        blocked_goal->observation_yaws.clear();
        if(goal.observation_yaw)blocked_goal->observation_yaws.push_back(*goal.observation_yaw);
      }
      if((!floor_search || has_blockers) && unknown_blockers && !result.usable() &&
         (cross_layer_target || !blocked_goal || first_flat_blocker))
        unknown_blockers->insert(unknown_blockers->end(),result.unknown_blockers.begin(),result.unknown_blockers.end());
      search_timed_out=search_timed_out || result.failure_class==PlannerFailureClass::timeout;
      if(pct_candidate && !floor_attempt && !cross_layer_target &&
         result.failure_class==PlannerFailureClass::timeout) {
        policy_.pct_timeout_pending_[pct_candidate->id]=wall_now()+2.;
        policy_.pct_timed_out_observers_[pct_candidate->observations[i]]=wall_now()+12.;
      }
      if (!recovery && !result.usable()) {
        if (result.failure_class == PlannerFailureClass::unsafe_start)
          unsafe_start_failure = true;
        else if (result.failure_class != PlannerFailureClass::unavailable &&
                 result.failure_class != PlannerFailureClass::timeout)
          incompatible_failure = true;
      }
      publish_candidate_path(result);
      std_msgs::msg::String message;
      std::ostringstream out;
      out << "{\"goal_id\":" << result.goal.goal_id
          << ",\"attempt_id\":" << result.goal.attempt_id
          << ",\"occupancy_version\":" << result.goal.occupancy_version
          << ",\"layer_version\":" << result.goal.layer_version
          << ",\"recovery_request\":\"" << recovery_request_name(result.goal.recovery_request)
          << "\",\"planner\":\"" << planner_name(result.kind)
          << "\",\"usable\":" << (result.usable() ? "true" : "false")
          << ",\"waypoints\":" << result.waypoints.size()
          << ",\"planning_ms\":" << result.planning_ms
          << ",\"candidate_source\":\"" << candidate_source_name(result.goal.candidate.source) << "\""
          << ",\"backend\":" << result.backend_metadata
          << ",\"reason\":\"" << result.reason << "\""
          << ",\"failure_class\":\""
          << planner_failure_name(result.failure_class) << "\"";
      if (recovery) {
        out << ",\"target_distance\":" << recovery_.recovery_last_target_distance_;
      }
      out << ",\"failure_stage\":\"" << result.route.failure_stage << "\""
          << ",\"start_check\":" << admission_json(result.route.start_check)
          << ",\"goal_check\":" << safety_json(result.route.goal_check)
          << ",\"failed_connection\":" << safety_json(result.route.failed_connection.failure)
          << ",\"expanded_nodes\":" << result.route.expanded_nodes
          << ",\"rejected_missing_nodes\":" << result.route.rejected_missing_nodes
          << ",\"rejected_slope_edges\":" << result.route.rejected_slope_edges
          << ",\"rejected_corridor_edges\":" << result.route.rejected_corridor_edges
          << ",\"robot_radius\":" << cfg_.robot_radius
          << ",\"safety_margin\":" << cfg_.waypoint_safety_margin << '}';
      message.data = out.str();
      output_.planning_result(message);
      if (result.kind==PlannerKind::layer || result.kind==PlannerKind::recovery_layer)
        output_.layer_diagnostic(message);
      if (!result.usable()) failure = result.reason;
    }
    const auto *chosen = plans.selected();
    if (!chosen) {
      if(floor_search) {
        const auto endpoint=std::find_if(results.begin(),results.end(),[](const auto &result) {
          return result.kind==PlannerKind::layer;
        });
        if(endpoint==results.end() || endpoint->failure_class==PlannerFailureClass::unsafe_start ||
           endpoint->failure_class==PlannerFailureClass::unavailable)break;
        reject_view(i,endpoint->failure_class==PlannerFailureClass::timeout,&rejected_blockers);
      }
      if (!recovery && unsafe_start_failure && !incompatible_failure) break;
      continue;
    }
    std::vector<Point> ground_route;
    auto ascent_start=cross_ground_pose();
    if((stair_climb || stair_segment) && chosen->start_ground_z)
      ascent_start[2]=*chosen->start_ground_z;
    if(options_.cross_layer_enabled_ && pct_candidate && chosen->kind==PlannerKind::layer &&
       !platform_floor && (!options_.floor_exploration_ || stair_climb || policy_.cross_task_.engaged())) {
      ground_route=chosen->xyz;
      for(auto &p:ground_route)p[2]-=options_.layer_body_height_;
      if (const auto *reason = policy_.check_ascent(ascent_start, ground_route, departure_height, stair_climb)) {
        failure = reason;
        incompatible_failure = true;
        continue;
      }
    }
    if(!recovery && !chosen->waypoints.empty() &&
       xy_distance(current_xy,chosen->waypoints.back())<minimum_travel) {
      reject_view(i);
      failure="observation_position_too_close";continue;
    }
    if (!path_processor_.load(goal, *chosen, recovery, &snapshot)) {
      reject_view(i);
      incompatible_failure = true;
      failure = "route_staging_failed";
      continue;
    }
    if(recovery_.stair_reposition_ && recovery_.stair_reposition_->stage==StairReposition::Stage::forward &&
       chosen->kind==PlannerKind::layer) {
      const auto &failed=recovery_.stair_reposition_->start;
      const auto &points=path_processor_.xyz();
      Point from=*base_;
      bool blocked_departure=false;
      for(const auto &to:points) {
        const double dx=to[0]-from[0],dy=to[1]-from[1];
        if(std::hypot(dx,dy)>1e-6 && std::hypot(from[0]-failed[0],from[1]-failed[1])<=.20 &&
           scan_start_inflated(from,std::atan2(dy,dx))) {
          blocked_departure=true;
          break;
        }
        from=to;
      }
      if(blocked_departure) {
        path_processor_.clear();
        failure="stair_retry_departure_inflated";
        incompatible_failure=true;
        continue;
      }
    }
    const RouteSelection selection{pct_candidate, detail, progress, planning_started, recovery,
        local_probe, stair_approach, stair_segment, stair_climb, fast_floor, floor_attempt,
        platform_floor, departure_height};
    if (!accept_route(*chosen, selection, ascent_start, ground_route)) {
      reject_view(i);
      reset_planned_route();
      failure = "observation_position_already_reached";
      incompatible_failure = true;
      continue;
    }
    return true;
  }
  if (!recovery && !floor_attempt && !recovery_.blocker_recovery_ &&
      (!unknown_blockers || unknown_blockers->empty()) && (!blocked_goal || blocked_goal->observations.empty())) {
    if(pct_candidate && pct_candidate->id && !cross_layer_target) {
      pct_observation_progress_.erase(pct_candidate->id);
      policy_.pct_cooldowns_.push_back({pct_candidate->frontier,wall_now()+(search_timed_out?12.:options_.cooldown_)});
    }
    else if(!pct_candidate)suppressed_.push_back({target, wall_now() + options_.cooldown_});
  }
  if (recovery) recovery_.recovery_last_failure_ = failure;
  if(selection_failure)*selection_failure=failure;
  report("route_unavailable", failure);
  return false;
}

bool ExplorationController::accept_route(
    const PlannerResult &chosen, const RouteSelection &selection,
    const Point &ascent_start, const std::vector<Point> &ground_route) {
  const auto &goal = chosen.goal;
  const auto &target = goal.target;
  route_selection_target_ = target;
  selected_pct_=selection.frontier?std::optional<PctFrontier>(*selection.frontier):std::nullopt;
  if(selected_pct_ && (selected_pct_->id==0 || selection.stair_segment) && goal.observation_yaw)
    selected_pct_->observation_yaws={*goal.observation_yaw};
  selected_pct_observation_=selection.frontier?goal.observation_xyz:std::nullopt;
  selected_local_stair_probe_=selection.local_probe;
  selected_local_stair_return_=selection.detail=="pct_local_stair_return";
  selected_stair_approach_=selection.stair_approach;
  selected_stair_segment_=selection.stair_segment;
  selected_platform_frontier_=selection.platform_floor && selection.frontier && selection.frontier->id;
  selected_fast_floor_=selection.fast_floor;
  selected_floor_attempt_=selection.floor_attempt;
  if (selection.frontier) pct_markers_dirty_=true;
  route_skipped_nearby_ = 0;
  if (!skip_nearby_route_waypoints()) return false;
  if (selection.frontier) {
    if (selection.progress) selection.progress->next = 0;
    policy_.pct_timeout_pending_.erase(selection.frontier->id);
  }
  if (selection.floor_attempt) {
    shared_layer_->pct->observation_task(goal.layer_version,"floor_attempt",
        selection.frontier->id,*goal.observation_xyz,goal.observation_yaw.value_or(0.));
    RCLCPP_INFO(logger_,"Floor %zu h=%.3f frontier %llu attempt=%d scope=%s map=%llu observer_ms=%.1f planning_ms=%.1f: route_ready",
        policy_.floor_number_,*policy_.floor_height_,static_cast<unsigned long long>(selection.frontier->id),
        selection.floor_attempt,policy_.pct_frontiers_.scope.c_str(),
        static_cast<unsigned long long>(policy_.pct_frontiers_.version),selection.frontier->observer_ms,
        1000.*(wall_now()-selection.started));
  }
  if(!selection.recovery)control_mode_ = goal.mode;
  if(selection.platform_floor && !selection.recovery)policy_.cross_task_.exploring_platform((*selection.platform_floor)[0]);
  if (policy_.adopt_ascent(ascent_start, ground_route, selection.departure_height,
                           selection.stair_climb ? goal.observation_yaw : std::nullopt))
    control_mode_ = ExplorationMode::cross_layer;
  PlannedRoute sent_route;
  sent_route.points = path_processor_.waypoints();
  if(chosen.kind==PlannerKind::layer)publish_planned_path(chosen.route,&chosen.xyz);
  else if(chosen.kind==PlannerKind::scan_direct)publish_planned_path({});
  else publish_planned_path(sent_route);
  RCLCPP_INFO(logger_,
      "Route goal=%llu frontier=(%.2f, %.2f) observation=(%.2f, %.2f) planner=%s stages=%zu",
      static_cast<unsigned long long>(goal.goal_id), target[0], target[1],
      goal.observation[0], goal.observation[1], path_processor_.source().c_str(),
      path_processor_.waypoints().size());
  publish_target(path_processor_.current(),
                 selection.recovery ? "cross_layer_repositioning" : path_processor_.source() + "_route" +
                 (selection.detail == "low_yield_frontier" ? "_low_yield" : ""));
  if (selection.recovery) recovery_.recovery_last_failure_.clear();
  return true;
}

void ExplorationController::advance_route() {
  if (!path_processor_.active()) return;
  if (!path_processor_.advance()) reset_planned_route();
}

}  // namespace legbot_exploration
