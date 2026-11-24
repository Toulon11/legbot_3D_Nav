#include "exploration_controller.hpp"
#include "exploration_controller_util.hpp"

namespace legbot_exploration {

// 计算二维欧氏距离，返回 double 精度
double xy_distance(const XY &a, const XY &b) {
  // 返回水平面两点的欧氏距离。
  return std::hypot(a[0] - b[0], a[1] - b[1]);
}
std::string json_string(const std::string &value) {
  std::ostringstream out;out << '"';
  for (unsigned char ch:value) {
    if (ch=='"' || ch=='\\') out << '\\' << ch;
    else if (ch<0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(ch) << std::dec;
    else out << ch;
  }
  out << '"';return out.str();
}
// 获取单调时钟的当前时间（秒），用于所有超时检测
double wall_now() {
  // 把此函数算出的值交还给调用者。
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
// ROS Time（sec + nanosec）转纳秒整数，便于比较大小
int64_t stamp_ns(const builtin_interfaces::msg::Time &stamp) {
  // 把此函数算出的值交还给调用者。
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

ExplorationController::ExplorationController(
    ExplorationSettings options, ExplorationOutput output,
    std::shared_ptr<SharedLayerMap> shared, rclcpp::Clock::SharedPtr clock,
    rclcpp::Logger logger, tf2_ros::Buffer *foot_tf)
    : options_(std::move(options)), output_(std::move(output)), clock_(std::move(clock)),
      logger_(std::move(logger)), foot_tf_(foot_tf), policy_(options_.cross_layer),
      cfg_(options_.map), map_(cfg_), shared_layer_(std::move(shared)),
      goal_progress_gate_(options_.min_new_cells_) {}

void ExplorationController::start() {
  trajectory_.header.frame_id = options_.frame_id_;
  report(options_.observation_only_ ? "observation_only" : "waiting_for_data");
}

void ExplorationController::set_paused(bool paused) {
  paused_ = paused;
  if(paused_ && observation_task_)end_observation(false,"paused");
  else if(paused_)cancel_recovery_action();
  report(paused_ ? "paused" : "waiting_for_data");
}

void ExplorationController::on_ready(bool ready) {
  ready_ = ready;
}

void ExplorationController::on_scan_reference(const scan_planner::msg::Bspline &msg) {
  if (active_ && !active_->reverse &&
      stamp_ns(msg.start_time) + 500000000LL >= active_->input_stamp)
    active_scan_bspline_ = msg;
}

void ExplorationController::on_forwarded_request(const scan_planner::msg::GoalRequest &request) {
  if (!active_ || active_->turn || active_->reverse ||
      request.goal_id != active_->goal_id || request.attempt_id != active_->attempt_id ||
      request.waypoint_index != active_->stage_index || request.target.header.frame_id != options_.frame_id_)
    return;
  const auto &point=request.target.pose.position;
  const auto stamp=stamp_ns(request.target.header.stamp);
  if (stamp <= 0 || !(xy_distance({point.x,point.y},active_->xy)<.05) ||
      (active_->identified_request && active_->stamp && *active_->stamp!=stamp))
    return;
  // A forwarded stage has a complete identity. The coordinator's /clock
  // callback can lag ours, so its stamp need not order after input_stamp.
  active_->identified_request = true;
  bind_scan_goal(stamp);
}

void ExplorationController::on_raw_occupancy(const sensor_msgs::msg::PointCloud2 &msg) {
  if (read_occupancy_points(msg, raw_obstacles_))
    raw_occupancy_wall_ = wall_now();
}

void ExplorationController::on_foot_forces(const control_msgs::msg::DynamicJointState &msg) {
  foot_forces_.fill(0.);
  foot_force_stamp_=stamp_ns(msg.header.stamp);
  foot_force_wall_=wall_now();
  for(std::size_t i=0;i<std::min(msg.joint_names.size(),msg.interface_values.size());++i) {
    if(msg.joint_names[i]!="foot_force") continue;
    const auto &interfaces=msg.interface_values[i];
    for(std::size_t j=0;j<std::min(interfaces.interface_names.size(),interfaces.values.size());++j)
      for(std::size_t f=0;f<foot_frames_.size();++f)
        if(interfaces.interface_names[j]==std::string(foot_frames_[f])+"_force")
          foot_forces_[f]=interfaces.values[j];
  }
}

void ExplorationController::tick_observation() {
  if(pending_scan_failure_ && active_ && active_->stamp && pending_scan_failure_->first==*active_->stamp) {
    const auto pending=*pending_scan_failure_;pending_scan_failure_.reset();
    handle_scan_failure(pending.first,pending.second);
  }
  if(observation_task_ && observation_task_->pending_end) {
    const auto [success,reason,settled]=*observation_task_->pending_end;
    end_observation(success,reason,settled);return;
  }
  if(paused_ || options_.observation_only_ || !ready_)return;
  const double current=wall_now();
  if(!observation_task_)return;
  const auto &task=*observation_task_;
  if((task.initialized || task.floor_attempt) && task.frontier.id &&
     shared_layer_->pct->committed_floor_task_resolved(task.frontier.id)) {
    end_observation(true,"frontier_resolved_in_observation");return;
  }
  if(task.arm_pending) {handle_observation(current);return;}
  if(active_ || path_processor_.active()) {
    // Watch pose health and task deadlines without querying the map or
    // re-running selection while a turn is executing.
    if(!base_ || !base_yaw_ || current-base_wall_>options_.odom_timeout_ ||
       current-yaw_wall_>options_.odom_timeout_ || task.pose_wait_since ||
       current-task.began>options_.observation_timeout_+options_.observation_pose_wait_timeout_)
      handle_observation(current);
    return;
  }
  const bool frame_gated=task.targeted_blocker_scan;
  if(task.directional() && !frame_gated)return;
  if(!task.waiting_since ||
     (frame_gated?
     shared_layer_->last_integration_wall_seconds.load()>task.waiting_since:
     current-task.waiting_since>=options_.observation_flat_wait_))
    handle_observation(current);
}

void ExplorationController::report(const std::string &state, const std::string &detail) {
  // Only unfinished selection is resumed quickly. Executing, observing and
  // unavailable-data states keep their existing event/normal-timer handling.
  const bool resume=state=="waiting_for_map_update" || (options_.floor_exploration_ && (state=="pct_local_search_pending" || state=="pct_floor_frontier_settled" ||
      state=="pct_floor_local_empty" || state=="pct_floor_complete" || state=="pct_floor_handoff_with_deferred_frontiers" ||
      (state=="cross_layer_waiting_for_forward_route" && policy_.pct_frontiers_.query_pending)));
  if(resume)selection_resume_at_=wall_now()+options_.pct_query_resume_delay_;
  else selection_resume_at_.reset();
  // This is task lifetime, not a gait command. Mapping/planning/recovery
  // waits remain active; only completion, pause or observation mode releases
  // the execution coordinator's walking hold. SCAN still settles each goal.
  const bool task_active = !paused_ && !options_.observation_only_ &&
      state != "pct_region_complete" && state != "exploration_complete" &&
      state != "current_region_complete";
  if (!last_task_active_ || *last_task_active_ != task_active) {
    std_msgs::msg::Bool activity;
    activity.data = task_active;
    output_.task_active(activity);
    last_task_active_ = task_active;
  }
  std::ostringstream stream;
  stream << "{\"state\":\"" << state << "\",\"implementation\":\"cpp\",";
  stream << "\"map_version\":" << map_.version() << ",\"voxel_count\":" << map_.size();
  stream << ",\"paused\":" << (paused_ ? "true" : "false");
  stream << ",\"task_active\":" << (task_active ? "true" : "false");
  stream << ",\"observation_only\":" << (options_.observation_only_ ? "true" : "false");
  stream << ",\"global_search\":" << (cfg_.global_search ? "true" : "false");
  stream << ",\"global_frontier_valid\":" << (global_frontiers_.valid ? "true" : "false");
  stream << ",\"global_frontier_cells\":" << global_frontiers_.frontier_cells;
  stream << ",\"indexed_frontier_cells\":" << frontier_index_.cells().size();
  stream << ",\"frontier_index_repairs\":" << frontier_index_repairs_;
  stream << ",\"global_reachable_free_cells\":" << global_frontiers_.reachable_free_cells;
  stream << ",\"frontier_cells\":" << last_frontier_cells_;
  stream << ",\"frontier_clusters\":" << last_frontier_clusters_;
  stream << ",\"reachable_frontier_cells\":" << last_reachable_frontier_cells_;
  stream << ",\"candidate_goals\":" << last_candidate_goals_;
  stream << ",\"suppressed_goals\":" << last_suppressed_goals_;
  stream << ",\"open_start_neighbors\":" << last_open_start_neighbors_;
  stream << ",\"total_cells\":" << coverage_.total_cells;
  stream << ",\"known_free_cells\":" << coverage_.known_free_cells;
  stream << ",\"known_occupied_cells\":" << coverage_.known_occupied_cells;
  stream << ",\"unknown_cells\":" << coverage_.unknown_cells;
  stream << ",\"coverage_ratio\":" << coverage_.ratio();
  stream << ",\"coverage_valid\":" << (coverage_.valid ? "true" : "false");
  stream << ",\"reachability_valid\":" << (coverage_.reachability_valid ? "true" : "false");
  stream << ",\"reachable_cells\":" << coverage_.reachable_cells;
  stream << ",\"reachable_unknown_cells\":" << coverage_.reachable_unknown_cells;
  stream << ",\"excluded_unknown_cells\":" << coverage_.excluded_unknown_cells;
  stream << ",\"reachable_coverage_ratio\":" << coverage_.reachable_ratio();
  stream << ",\"unresolved_unknown_cells\":" << unresolved_unknown_cells_;
  stream << ",\"route_source\":\"" << path_processor_.source() << "\"";
  stream << ",\"control_mode\":\"" << mode_name(control_mode_) << "\"";
  stream << ",\"mission\":{\"return_enabled\":" << (options_.mission_return_home_?"true":"false")
         << ",\"exploration_finished\":" << (mission_exploration_finished_?"true":"false")
         << ",\"returning\":" << (mission_returning_?"true":"false")
         << ",\"returned_home\":" << (mission_finished_?"true":"false") << '}';
  stream << ",\"pct_selection_timing_ms\":{";
  bool first_timing=true;
  for(const auto &[phase,elapsed]:selection_timings_) {
    if(!first_timing)stream<<',';
    first_timing=false;
    stream<<json_string(phase)<<':'<<elapsed;
  }
  stream<<'}';
  stream<<",\"pct_selection_resumes\":"<<selection_resume_count_
        <<",\"pct_selection_resume_gap_ms\":"<<selection_resume_gap_ms_;
  stream << ",\"cross_layer_task\":{\"phase\":"<<json_string(policy_.cross_task_.phase())
         <<",\"progress_m\":"<<policy_.cross_task_.progress()<<",\"rise_m\":"<<policy_.cross_task_.rise()
         <<",\"ascent_yaw\":"<<policy_.cross_task_.ascent_yaw()
         <<",\"platform_height\":"<<(policy_.cross_task_.platform_height()?std::to_string(*policy_.cross_task_.platform_height()):"null")
         <<",\"reason\":"<<json_string(policy_.cross_task_.reason())
         <<",\"filtered_observers\":"<<policy_.cross_filtered_observers_<<'}';
  if(options_.frontier_source_=="pct")stream<<",\"height_review\":"<<height_review_status_;
  if(options_.floor_exploration_ && options_.frontier_source_=="pct") {
    stream<<",\"pct_floor\":{\"reference_kind\":\"measured_height_band\",\"number\":"<<policy_.floor_number_
          <<",\"height\":"<<(policy_.floor_height_?std::to_string(*policy_.floor_height_):"null")
          <<",\"phase\":"<<json_string(!policy_.floor_height_?"acquiring":
               policy_.cross_task_.deferred()?"ascent_deferred":
               policy_.cross_task_.phase()=="exploring_platform"?"exploring":
               (policy_.cross_task_.engaged() || recovery_.stair_reposition_)?"ascending":
               (policy_.floor_complete_ || skipping_start_floor())?"waiting_for_ascent":"exploring")
          <<",\"ordinary_started_sim\":"<<(policy_.floor_exploration_started_sim_?std::to_string(*policy_.floor_exploration_started_sim_):"null")
          <<",\"ordinary_started_wall\":"<<(policy_.floor_exploration_started_wall_?std::to_string(*policy_.floor_exploration_started_wall_):"null")
          <<",\"ordinary_ended_sim\":"<<(policy_.floor_exploration_ended_sim_?std::to_string(*policy_.floor_exploration_ended_sim_):"null")
          <<",\"ordinary_ended_wall\":"<<(policy_.floor_exploration_ended_wall_?std::to_string(*policy_.floor_exploration_ended_wall_):"null")
          <<",\"start_floor_skipped\":"<<(skipping_start_floor()?"true":"false")
          <<",\"contacting_feet\":"<<contacting_feet_
          <<",\"foot_sample\":"<<json_string(foot_sample_state_)
          <<",\"support_stable\":"<<(foot_support_stable_?"true":"false")
          <<",\"foot_height_span\":"<<foot_height_span_
          <<",\"confirmation_reason\":"<<json_string(policy_.floor_confirmation_reason_)
          <<",\"stable_foot_height\":"<<(stable_foot_height_?std::to_string(*stable_foot_height_):"null")
          <<",\"ascent_unknown_blockers\":"<<policy_.pct_frontiers_.unknown_blockers.size()
          <<",\"upward_support_targets\":"<<policy_.pct_frontiers_.upward_support_targets.size()
          <<",\"upward_result\":"<<json_string(policy_.upward_result_)
          <<",\"unknown_blockers\":[";
    const auto &blockers=policy_.pct_frontiers_.unknown_blockers;
    for(std::size_t i=0;i<std::min<std::size_t>(12,blockers.size());++i) {
      const auto &b=blockers[i];
      if(i)stream<<',';
      stream<<"{\"point\":["<<b.point[0]<<','<<b.point[1]<<','<<b.point[2]<<']'
            <<",\"slice\":"<<b.slice<<",\"map_version\":"<<b.version<<'}';
    }
    stream<<"]}";
  }
  stream << ",\"goal_id\":" << goal_id_;
  stream << ",\"planning_attempt_id\":" << planning_attempt_id_;
  stream << ",\"layer_map_version\":" << layer_version_;
  stream << ",\"observation_task_active\":" << (observation_task_?"true":"false")
         << ",\"observation_evidence\":" << observation_metadata_;
  if(observation_task_)stream << ",\"observation_desired_yaw\":" << observation_task_->center;
  if(observation_task_) {
    const auto &task=*observation_task_;const double current=wall_now();
    stream<<",\"observation_timing\":{\"phase\":"<<json_string(task.phase)
          <<",\"elapsed_s\":"<<std::max(0.,current-task.began)
          <<",\"pose_wait_s\":"<<task.pose_wait_seconds+
              (task.pose_wait_since?std::max(0.,current-*task.pose_wait_since):0.)
          <<",\"pose_age_s\":"<<(base_?std::to_string(std::max(0.,current-base_wall_)):"null")
          <<",\"yaw_age_s\":"<<(base_yaw_?std::to_string(std::max(0.,current-yaw_wall_)):"null")<<'}';
  }
  stream << ",\"route_stages\":" << path_processor_.waypoints().size();
  stream << ",\"route_stage_index\":" << path_processor_.index();
  stream << ",\"route_skipped_nearby\":" << route_skipped_nearby_;
  stream << ",\"scan_failure_count\":" << scan_failure_count_;
  if (!scan_last_failure_.empty())
    stream << ",\"scan_last_failure\":\"" << scan_last_failure_ << "\"";
  if (route_selection_target_)
    stream << ",\"selected_goal_x\":" << (*route_selection_target_)[0]
           << ",\"selected_goal_y\":" << (*route_selection_target_)[1];
  if (active_ && base_)
    stream << ",\"active_waypoint_x\":" << active_->xy[0]
           << ",\"active_waypoint_y\":" << active_->xy[1]
           << ",\"active_waypoint_distance\":"
           << xy_distance(active_->xy, {(*base_)[0], (*base_)[1]});
  stream<<",\"recovery_phase\":"<<json_string(recovery_.blocker_recovery_?"pct_blocker":
      recovery_.stair_reposition_?"stair_reposition":"idle")
        <<",\"recovery_goal_xy_tolerance\":"<<options_.recovery_goal_xy_tolerance_;
  stream << ",\"planner_mode\":\"" << options_.planner_mode_ << "\"";
  stream << ",\"observation_min_distance\":" << options_.observation_min_distance_
         << ",\"observation_goal_xy_tolerance\":" << options_.observation_goal_xy_tolerance_;
  stream << ",\"frontier_source\":\"" << options_.frontier_source_ << "\"";
  if (options_.frontier_source_=="pct") {
    stream << ",\"pct_frontier_valid\":" << (policy_.pct_frontiers_.valid?"true":"false")
           << ",\"pct_map_version\":" << policy_.pct_frontiers_.version
           << ",\"pct_frontier_ms\":" << policy_.pct_frontiers_.extraction_ms
           << ",\"pct_frontier_cells\":" << policy_.pct_frontiers_.frontier_cells
           << ",\"pct_frontier_clusters\":" << policy_.pct_frontiers_.clusters.size()
           << ",\"pct_known_surface_cells\":" << policy_.pct_frontiers_.known_surface_cells
           << ",\"pct_safe_cells\":" << policy_.pct_frontiers_.safe_cells
           << ",\"pct_no_observation_clusters\":" << policy_.pct_frontiers_.no_observation_clusters
           << ",\"pct_unreachable_observers\":" << policy_.pct_frontiers_.unreachable_observers
           << ",\"pct_query_ms\":" << policy_.pct_frontiers_.query_ms
           << ",\"pct_query_pending\":" << (policy_.pct_frontiers_.query_pending?"true":"false")
           << ",\"pct_support_query_pending\":" << (policy_.pct_frontiers_.support_query_pending?"true":"false")
           << ",\"pct_query_checked_clusters\":" << policy_.pct_frontiers_.query_checked_clusters
           << ",\"pct_observer_prepare_ms\":" << policy_.pct_frontiers_.observer_prepare_ms
           << ",\"pct_observer_geometry_ms\":" << policy_.pct_frontiers_.observer_geometry_ms
           << ",\"pct_observer_visibility_ms\":" << policy_.pct_frontiers_.observer_visibility_ms
           << ",\"pct_candidate_round_budget\":" << options_.pct_round_budget_
           << ",\"pct_selection_scope\":\"" << policy_.pct_frontiers_.scope << "\""
           << ",\"pct_local_fallback_count\":" << policy_.pct_local_fallback_count_
           << ",\"pct_scoped_frontier_cells\":" << policy_.pct_frontiers_.scoped_frontier_cells
           << ",\"pct_disconnected_floor_frontier_cells\":" << policy_.pct_frontiers_.disconnected_floor_frontier_cells
           << ",\"pct_pending_review_clusters\":" << policy_.pct_frontiers_.pending_review_clusters
           << ",\"pct_deferred_review_clusters\":" << policy_.pct_frontiers_.deferred_review_clusters
           << ",\"pct_missing_connection_clusters\":" << policy_.pct_frontiers_.missing_connection_clusters
           << ",\"pct_floor_known_barriers\":" << policy_.floor_known_barriers_
           << ",\"pct_floor_review_wait_ms\":" << 1000.*(policy_.floor_review_elapsed_+
                (policy_.floor_review_started_?wall_now()-*policy_.floor_review_started_:0.))
           << ",\"pct_floor_review_resolved_cells\":" << policy_.floor_review_resolved_
           << ",\"pct_reviewing\":" << (policy_.pct_frontiers_.reviewing?"true":"false")
           << ",\"pct_fast_floor\":" << (policy_.pct_frontiers_.fast_floor?"true":"false")
           << ",\"pct_fast_skipped_clusters\":" << policy_.pct_frontiers_.fast_skipped_clusters
           << ",\"pct_fast_skipped_cells\":" << policy_.pct_frontiers_.fast_skipped_cells
           << ",\"pct_failed_small_clusters\":" << policy_.pct_frontiers_.small_skipped_clusters
           << ",\"pct_failed_small_cells\":" << policy_.pct_frontiers_.small_skipped_cells
           << ",\"pct_low_yield_deferred_clusters\":" << policy_.pct_frontiers_.low_yield_deferred_clusters
           << ",\"pct_low_yield_deferred_cells\":" << policy_.pct_frontiers_.low_yield_deferred_cells
           << ",\"pct_global_first_after_local_failure_clusters\":" << policy_.pct_frontiers_.global_first_after_local_failure_clusters
           << ",\"pct_global_first_after_local_failure_cells\":" << policy_.pct_frontiers_.global_first_after_local_failure_cells
           << ",\"pct_floor_longest_selection_wait_ms\":" << policy_.floor_longest_selection_wait_ms_
           << ",\"pct_deferred_floor_frontier_cells\":" << policy_.pct_frontiers_.deferred_floor_frontier_cells
           << ",\"pct_local_window\":{\"horizontal_half_width\":" << cfg_.local_search_radius
           << ",\"height_above\":" << options_.pct_local_above_ << ",\"height_below\":" << options_.pct_local_below_
           << ",\"center\":[" << policy_.pct_frontiers_.window_center[0] << ',' << policy_.pct_frontiers_.window_center[1]
           << ',' << policy_.pct_frontiers_.window_center[2] << "]}"
           << ",\"pct_error\":" << json_string(policy_.pct_frontiers_.error)
           << ",\"pct_map\":" << policy_.pct_frontiers_.metadata
           << ",\"completion_source\":\"pct\"";
    if (selected_pct_)
      stream << ",\"selected_goal_z\":" << selected_pct_->frontier[2];
    if (path_processor_.current_z())
      stream << ",\"active_waypoint_z\":" << *path_processor_.current_z();
  }
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (!detail.empty()) stream << ",\"detail\":\"" << detail << "\"";
  stream << '}';
  if (stream.str() == last_status_) return; // 去重：若未变化则不发送
  std_msgs::msg::String msg;
  msg.data = stream.str();
  output_.status(msg);
  last_status_ = msg.data;
}

void ExplorationController::publish_coverage_grid(const std::unordered_map<Cell, int, CellHash> &grid) {
  if (!coverage_.valid) return;
  const double r = cfg_.resolution;
  const int x0 = static_cast<int>(std::ceil(cfg_.exploration_min_x / r - 0.5));
  const int x1 = static_cast<int>(std::floor(cfg_.exploration_max_x / r - 0.5));
  const int y0 = static_cast<int>(std::ceil(cfg_.exploration_min_y / r - 0.5));
  const int y1 = static_cast<int>(std::floor(cfg_.exploration_max_y / r - 0.5));
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = options_.frame_id_;
  msg.header.stamp = clock_->now();
  msg.info.map_load_time = msg.header.stamp;
  msg.info.resolution = r;
  msg.info.width = static_cast<uint32_t>(x1 - x0 + 1);
  msg.info.height = static_cast<uint32_t>(y1 - y0 + 1);
  msg.info.origin.position.x = x0 * r;
  msg.info.origin.position.y = y0 * r;
  msg.info.origin.orientation.w = 1.0;
  msg.data.reserve(coverage_.total_cells);
  for (int y = y0; y <= y1; ++y) for (int x = x0; x <= x1; ++x) {
    auto found = grid.find({x, y});
    msg.data.push_back(found == grid.end() ? -1 : (found->second == 0 ? 0 : 100));
  }
  output_.coverage(msg);
}

void ExplorationController::on_sensor(const nav_msgs::msg::Odometry &msg) {
  if (msg.header.frame_id != options_.frame_id_) return; // 检查坐标系
  const auto &p = msg.pose.pose.position;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return; // 数据有效性检查
  // 把本次结果放入容器，供下一轮处理。
  sensor_poses_.push_back({stamp_ns(msg.header.stamp), {p.x, p.y, p.z}});
  if (sensor_poses_.size() > 100) sensor_poses_.pop_front(); // 只保留最近 100 条
  process_clouds();
}

void ExplorationController::sample_support_height(const nav_msgs::msg::Odometry &msg,double q_norm) {
  if(!foot_tf_) return;
  const double current=wall_now();
  if(current-foot_sample_wall_<.08) return;
  if(current-foot_sample_wall_>.25) {
    foot_stable_since_.reset();
    four_feet_stable_since_.reset();
  }
  foot_sample_wall_=current;
  stable_foot_height_.reset();
  foot_support_stable_=false;
  contacting_foot_mask_=0;
  foot_height_span_=0.;
  contacting_feet_=0;
  std::array<double,4> heights{};
  const auto reject=[this](const char *why) {
    foot_sample_state_=why;
    foot_stable_since_.reset();
    four_feet_stable_since_.reset();
    foot_height_samples_.clear();
  };
  const auto stamp=stamp_ns(msg.header.stamp);
  if(current-foot_force_wall_>.25 || std::abs(foot_force_stamp_-stamp)>180000000)
    return reject("contact_data_stale");
  if(msg.child_frame_id.empty() || !std::isfinite(q_norm) || q_norm<.5 || q_norm>1.5)
    return reject("base_pose_invalid");
  const auto &q=msg.pose.pose.orientation;
  const tf2::Quaternion orientation(q.x/q_norm,q.y/q_norm,q.z/q_norm,q.w/q_norm);
  for(std::size_t f=0;f<foot_frames_.size();++f) {
    if(!std::isfinite(foot_forces_[f]) || foot_forces_[f]<options_.foot_force_min_) continue;
    try {
      const auto foot=foot_tf_->lookupTransform(msg.child_frame_id,foot_frames_[f],tf2::TimePointZero);
      if(std::abs(stamp_ns(foot.header.stamp)-stamp)>180000000) return reject("foot_tf_stale");
      const auto &p=foot.transform.translation;
      // GO2 foot frame is the center of its 2 cm collision sphere.
      const auto offset=tf2::quatRotate(orientation,tf2::Vector3(p.x,p.y,p.z));
      const Point contact{(*base_)[0]+offset.x(),(*base_)[1]+offset.y(),(*base_)[2]+offset.z()-.02};
      if(!std::all_of(contact.begin(),contact.end(),[](double value){return std::isfinite(value);}))
        return reject("foot_tf_invalid");
      foot_contact_points_[f]=contact;
      contacting_foot_mask_|=1u<<f;
      heights[contacting_feet_++]=contact[2];
    } catch(const tf2::TransformException &) {return reject("foot_tf_unavailable");}
  }
  if(contacting_feet_<3) return reject("insufficient_contact");
  std::sort(heights.begin(),heights.begin()+contacting_feet_);
  foot_height_span_=heights[contacting_feet_-1]-heights[0];
  const double step_limit=std::min(.16,shared_layer_?shared_layer_->max_step_height:.16);
  if(foot_height_span_>step_limit+1e-7)return reject("foot_support_exceeds_step_limit");
  const bool same_surface=foot_height_span_<=.04;
  const double median=(heights[contacting_feet_/2]+heights[(contacting_feet_-1)/2])*.5;
  const double yaw=*base_yaw_;
  bool contacts_changed=contacting_foot_mask_!=foot_reference_mask_;
  for(std::size_t f=0;f<foot_frames_.size();++f)
    if((contacting_foot_mask_&(1u<<f)) &&
       std::abs(foot_contact_points_[f][2]-foot_reference_contacts_[f][2])>.02)contacts_changed=true;
  if(!foot_stable_since_ || contacts_changed ||
     std::abs(std::remainder(yaw-foot_reference_yaw_,2*M_PI))>.12 ||
     std::hypot((*base_)[0]-foot_reference_xy_[0],(*base_)[1]-foot_reference_xy_[1])>.035) {
    foot_stable_since_=current;
    four_feet_stable_since_.reset();
    foot_reference_mask_=contacting_foot_mask_;
    foot_reference_contacts_=foot_contact_points_;
    foot_reference_xy_={(*base_)[0],(*base_)[1]};
    foot_reference_yaw_=yaw;
    foot_height_samples_.clear();
  }
  if(contacting_feet_<4) four_feet_stable_since_.reset();
  else if(!four_feet_stable_since_) four_feet_stable_since_=current;
  foot_height_samples_.push_back(median);
  if(foot_height_samples_.size()>6) foot_height_samples_.pop_front();
  foot_sample_state_="settling";
  if(current-*foot_stable_since_>=.40 && foot_height_samples_.size()>=3) {
    auto samples=foot_height_samples_;
    std::sort(samples.begin(),samples.end());
    foot_support_stable_=true;
    foot_sample_state_=same_surface?"stable":"stable_multilevel";
    // A stable stance can span two treads; its average is not a ground surface.
    if(same_surface)stable_foot_height_=(samples[samples.size()/2]+samples[(samples.size()-1)/2])*.5;
  }
}

void ExplorationController::on_base(const nav_msgs::msg::Odometry &msg) {
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (msg.header.frame_id != options_.frame_id_) return;
  const auto &p = msg.pose.pose.position;
  // 过滤包含无穷大或 NaN 的无效坐标。
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
  const auto &q = msg.pose.pose.orientation;
  const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (std::isfinite(q_norm) && q_norm >= 0.5 && q_norm <= 1.5) {
    const double x=q.x/q_norm,y=q.y/q_norm,z=q.z/q_norm,w=q.w/q_norm;
    const double yaw = std::atan2(2.0*(w*z+x*y),1.0-2.0*(y*y+z*z));
    base_attitude_={std::atan2(2.*(w*x+y*z),1.-2.*(x*x+y*y)),
                    std::asin(std::clamp(2.*(w*y-z*x),-1.,1.)),yaw};
    base_yaw_ = yaw;
    yaw_wall_ = wall_now();
  }
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (cfg_.global_search && options_.bounds_relative_to_start_ && !bounds_initialized_) {
    cfg_.exploration_min_x += p.x;
    cfg_.exploration_max_x += p.x;
    cfg_.exploration_min_y += p.y;
    cfg_.exploration_max_y += p.y;
    map_.set_exploration_bounds(cfg_.exploration_min_x, cfg_.exploration_max_x,
                                cfg_.exploration_min_y, cfg_.exploration_max_y);
    bounds_initialized_ = true;
  }
  base_ = Point{p.x, p.y, p.z};
  const auto &v=msg.twist.twist.linear;
  const auto &w=msg.twist.twist.angular;
  if(std::hypot(v.x,v.y,v.z)<=.03 && std::hypot(w.x,w.y,w.z)<=.10) {
    if(!base_quiet_since_ || wall_now()-base_wall_>options_.odom_timeout_)base_quiet_since_=wall_now();
  } else base_quiet_since_.reset();
  sample_support_height(msg,q_norm);
  // 把本次结果放入容器，供下一轮处理。
  base_poses_.push_back({stamp_ns(msg.header.stamp), *base_});
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (base_poses_.size() > 100) base_poses_.pop_front();
  base_wall_ = wall_now();
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (trajectory_.poses.empty() ||
      xy_distance({p.x, p.y}, {trajectory_.poses.back().pose.position.x,
                               trajectory_.poses.back().pose.position.y}) >= 0.1) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = msg.pose.pose;
    // 把本次结果放入容器，供下一轮处理。
    trajectory_.poses.push_back(pose);
    // 只有表达式为真，才执行下面缩进或花括号内的代码。
    if (trajectory_.poses.size() > 20000) trajectory_.poses.erase(trajectory_.poses.begin());
  }
  process_clouds();
}

bool ExplorationController::read_occupancy_points(const sensor_msgs::msg::PointCloud2 &msg,
    std::vector<XY> &output, std::vector<Point> *nearby) {
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (msg.header.frame_id != options_.frame_id_ || !base_) return false;
  bool x_found = false, y_found = false, z_found = false;
  // 依次处理收到或待发送的 ROS 消息。
  for (const auto &field : msg.fields) {
    // 只有表达式为真，才执行下面缩进或花括号内的代码。
    if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) x_found = true;
    // 只有表达式为真，才执行下面缩进或花括号内的代码。
    if (field.name == "y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) y_found = true;
    // 只有表达式为真，才执行下面缩进或花括号内的代码。
    if (field.name == "z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) z_found = true;
  }
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (!x_found || !y_found || !z_found) return false;
  // 清空临时状态，准备下一轮处理。
  output.clear();
  if(nearby)nearby->clear();
  sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
  const std::size_t total = static_cast<std::size_t>(msg.width) * msg.height;
  // 依次处理集合中的每一项。
  for (std::size_t i = 0; i < total; ++i, ++x, ++y, ++z) {
    // 过滤包含无穷大或 NaN 的无效坐标。
    if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z) &&
        std::abs(*z - (*base_)[2]) <= options_.occupancy_height_tolerance_) {
      // 把本次结果放入容器，供下一轮处理。
      output.push_back({*x, *y});
      if(nearby && std::hypot(*x-(*base_)[0],*y-(*base_)[1])<1.)
        nearby->push_back({*x,*y,*z});
    }
  }
  return true;
}

void ExplorationController::on_occupancy(const sensor_msgs::msg::PointCloud2 &msg) {
  if (read_occupancy_points(msg, external_obstacles_, &nearby_scan_inflation_))
    occupancy_wall_ = wall_now();
}

bool ExplorationController::scan_start_inflated(const Point &base,double heading) const {
  const double resolution=cfg_.planner_occupancy_resolution;
  // Matches the two longitudinal probes in the demo SCAN configuration.
  for(const double offset:{.07,-.07}) {
    const Point probe{base[0]+offset*std::cos(heading),
                          base[1]+offset*std::sin(heading),base[2]};
    for(const auto &p:nearby_scan_inflation_) {
      if(std::floor(p[0]/resolution)==std::floor(probe[0]/resolution) &&
         std::floor(p[1]/resolution)==std::floor(probe[1]/resolution) &&
         std::floor(p[2]/resolution)==std::floor(probe[2]/resolution))return true;
    }
  }
  return false;
}

bool ExplorationController::planner_occupancy_fresh(double current) const {
  // 把此函数算出的值交还给调用者。
  return occupancy_wall_ > 0.0 && current - occupancy_wall_ <= options_.occupancy_timeout_;
}

bool ExplorationController::planner_blocks(const XY &xy) const {
  // 把此函数算出的值交还给调用者。
  return std::any_of(external_obstacles_.begin(), external_obstacles_.end(),
      [this, &xy](const XY &obstacle) {
        // 把此函数算出的值交还给调用者。
        return xy_distance(xy, obstacle) <=
            std::max(0.0, cfg_.robot_radius - cfg_.planner_occupancy_radius);
      });
}

void ExplorationController::on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (msg->header.frame_id != options_.frame_id_) return;
  // 把本次结果放入容器，供下一轮处理。
  pending_clouds_.push_back(msg);
  if (pending_clouds_.size() > 5) pending_clouds_.pop_front(); // 最多缓存 5 帧点云
  process_clouds();
}

void ExplorationController::process_clouds() {
  // 必须同时拥有基座、里程计且基座未超时才能继续
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (!base_ || sensor_poses_.empty() || base_poses_.empty() ||
      wall_now() - base_wall_ > options_.odom_timeout_) return;
  // 依次处理集合中的每一项。
  for (auto it = pending_clouds_.begin(); it != pending_clouds_.end();) {
    int64_t stamp = stamp_ns((*it)->header.stamp);
    // 只有表达式为真，才执行下面缩进或花括号内的代码。
    if (integrate_cloud(**it))
      // 从容器中取走已处理或失效的数据。
      it = pending_clouds_.erase(it);
    // 上一条件不满足时，再检查这个分支。
    else if (sensor_poses_.back().first - stamp > options_.sync_tolerance_ * 1e9 &&
             base_poses_.back().first - stamp > options_.sync_tolerance_ * 1e9)
      it = pending_clouds_.erase(it); // 超出同步容差，直接丢弃
    // 前面的条件不成立时，执行这个备选分支。
    else
      ++it; // 仍等待更合适的里程计匹配
  }
}

bool ExplorationController::integrate_cloud(const sensor_msgs::msg::PointCloud2 &msg) {
  int64_t stamp = stamp_ns(msg.header.stamp);
  // 找到时间戳最接近的里程计姿态
  auto nearest = std::min_element(sensor_poses_.begin(), sensor_poses_.end(),
                              [&stamp](const auto &a, const auto &b) {
                                return std::llabs(a.first - stamp) < std::llabs(b.first - stamp); // 绝对值比较，找到最接近的时间戳
                              });
  if (std::llabs(nearest->first - stamp) > options_.sync_tolerance_ * 1e9) return false; // 同步容差外直接返回 false
  auto nearest_base = std::min_element(base_poses_.begin(), base_poses_.end(),
      [&stamp](const auto &a, const auto &b) {
        // 把此函数算出的值交还给调用者。
        return std::llabs(a.first - stamp) < std::llabs(b.first - stamp);
      });
  // 只有时间戳与本轮消息匹配时才采用这份数据。
  if (std::llabs(nearest_base->first - stamp) > options_.sync_tolerance_ * 1e9) return false;

  // 检查点云是否包含 x/y/z 三个字段（必须为 FLOAT32）
  bool x_found = false, y_found = false, z_found = false;
  // 依次处理收到或待发送的 ROS 消息。
  for (const auto &field : msg.fields) {
    // 只有表达式为真，才执行下面缩进或花括号内的代码。
    if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) x_found = true;
    // 只有表达式为真，才执行下面缩进或花括号内的代码。
    if (field.name == "y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) y_found = true;
    // 只有表达式为真，才执行下面缩进或花括号内的代码。
    if (field.name == "z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) z_found = true;
  }
  if (!x_found || !y_found || !z_found) return true; // 若缺失字段直接跳过（算作成功）

  pct_cloud_wall_=wall_now();  // Independent of 2-D map growth/capacity.
  // 抽稀采样点云，最多 options_.max_points_ 个点
  std::vector<Point> points;
  points.reserve(options_.max_points_);
  sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
  const std::size_t total = static_cast<std::size_t>(msg.width) * msg.height;
  // 依次处理一帧点云点集。
  for (std::size_t i = 0; i < total && static_cast<int>(points.size()) < options_.max_points_; ++i, ++x, ++y, ++z) {
    // 只有表达式为真，才执行下面缩进或花括号内的代码。
    if (i % static_cast<std::size_t>(options_.point_stride_) == 0)
      // 把本次结果放入容器，供下一轮处理。
      points.push_back({*x, *y, *z});
  }
  // 将抽稀后的点云融合进体素地图（使用最近的里程计姿态）
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (map_.integrate(nearest->second, points, nearest_base->second[2]))
    cloud_wall_ = wall_now();
  // 把此函数算出的值交还给调用者。
  return true;
}

void ExplorationController::on_input(const geometry_msgs::msg::PoseStamped &msg) {
  XY xy{msg.pose.position.x, msg.pose.position.y};
  // 若已有活动目标且与本次输入距离 < 0.05 m 且时间戳相同，则认为是重复输入，直接忽略
  // 只有时间戳与本轮消息匹配时才采用这份数据。
  if (active_ && xy_distance(xy, active_->xy) < 0.05 && stamp_ns(msg.header.stamp) == active_->input_stamp) return;
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if(observation_task_)end_observation(false,"manual_override");
  else cancel_recovery_action();
  if(policy_.cross_task_.engaged() || policy_.cross_task_.deferred() || recovery_.stair_reposition_)
    defer_cross_layer("manual_override");
  recovery_.reset();
  control_mode_ = ExplorationMode::flat;
  // An external manual goal supersedes the exploration route.
  active_.reset();
  reset_planned_route();
  paused_ = true;
  report("paused_for_manual_goal");
}

void ExplorationController::on_goal(const geometry_msgs::msg::PoseStamped &msg) {
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (msg.header.frame_id != options_.frame_id_) return;
  if (active_ && (active_->turn || active_->reverse)) return;
  XY xy{msg.pose.position.x, msg.pose.position.y};
  // 尚无对应数据或目标时，进入此分支。
  if (!active_) {
    paused_ = true;
    report("paused_for_manual_goal");
    // 结束当前函数，不再执行后面的语句。
    return;
  }
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (xy_distance(xy, active_->xy) < 0.05) {
    // The legacy PoseStamped path has no stage identity.
    const auto stamp=stamp_ns(msg.header.stamp);
    if (active_->identified_request || stamp < active_->input_stamp) return;
    bind_scan_goal(stamp);
  } else {
    // 完全不同的目标：先清除当前目标并暂停
    // 清空临时状态，准备下一轮处理。
    active_.reset();
    reset_planned_route();
  if(recovery_.stair_reposition_)block_stair_reposition("stair_reposition_manual_goal");
    recovery_.blocker_recovery_.reset();
    paused_ = true;
    report("paused_for_manual_goal");
  }
}

void ExplorationController::bind_scan_goal(std::int64_t stamp) {
  active_->stamp = stamp;
  path_processor_.bind_scan_goal(stamp);
  if (pending_scan_failure_ && pending_scan_failure_->first == stamp) {
    const auto pending = *pending_scan_failure_;
    pending_scan_failure_.reset();
    handle_scan_failure(pending.first, pending.second);
    return;
  }
  if (pending_scan_ack_ && stamp_ns(pending_scan_ack_->header.stamp) == stamp) {
    const auto pending = *pending_scan_ack_;
    pending_scan_ack_.reset();
    on_ack(pending);
  }
}

void ExplorationController::on_ack(const geometry_msgs::msg::PoseStamped &msg) {
  if (!active_ || active_->turn || active_->reverse || msg.header.frame_id != options_.frame_id_) return;
  const auto stamp = stamp_ns(msg.header.stamp);
  if (stamp <= 0) return;
  const auto &point = msg.pose.position;
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) return;
  const auto command = base_ ? path_processor_.dispatch(stamp,*base_) : std::nullopt;
  const double arrival_limit = command && command->xy_tolerance ? *command->xy_tolerance : options_.goal_reach_xy_;
  if (xy_distance({point.x,point.y},active_->xy)>arrival_limit+1e-6) return;
  if(!active_->stamp) {
    // Delivery can race the forwarded request. Bind only by its exact stamp.
    pending_scan_ack_=msg;
    return;
  }
  if (stamp != *active_->stamp) return;
  active_->stop_position=Point{point.x,point.y,point.z};
  active_->ack = true;
  // A matching identified request plus the executor's stamped zero/arrival
  // receipt is authoritative. A depth-one Bool can coalesce false->true
  // for an immediately reached intermediate point; it must not stall it.
  if(active_->identified_request)active_->complete=true;
  finish_if_complete();
}

void ExplorationController::on_complete(const std_msgs::msg::Bool &msg) {
  // 尚无对应数据或目标时，进入此分支。
  if (!active_ || active_->turn || active_->reverse) return;
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (!msg.data)
    active_->armed = true; // 接收到 false，先进入 armed 状态，等待下一个 true 才算真正完成
  // 上一条件不满足时，再检查这个分支。
  else if (active_->armed) {
    active_->complete = true;
    finish_if_complete();
  }
}

void ExplorationController::finish_if_complete() {
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (active_ && active_->ack && active_->complete && active_->stamp) {
    auto result = path_processor_.resolve_execution(*active_->stamp, true, "goal_reached",active_->stop_position);
    if (!result) return;
    publish_execution_result(*result);
    if (recovery_.complete_motion(active_->stop_position, wall_now(), true)) {
      active_.reset();
      reset_planned_route();
      report("pct_blocker_recovery_waiting","fresh_map_after_delegation");
      return;
    }
    if(recovery_.stair_reposition_ && path_processor_.is_last() &&
       recovery_.stair_reposition_->stage==StairReposition::Stage::forward)recovery_.stair_reposition_.reset();
    RCLCPP_INFO(logger_, "Frontier goal reached: %.2f, %.2f", active_->xy[0], active_->xy[1]);
    finish_active_goal();
    route_stage_reached_wall_=wall_now();
    // 清空临时状态，准备下一轮处理。
    active_.reset();
    advance_route();
    // 清空临时状态，准备下一轮处理。
    empty_since_.reset();
    report("goal_reached");
    try_dispatch_route();
  }
}

void ExplorationController::publish_execution_result(const ExecutionResult &result) {
  std_msgs::msg::String message;
  std::ostringstream out;
  out << "{\"goal_id\":" << result.goal_id
      << ",\"attempt_id\":" << result.attempt_id
      << ",\"stage_index\":" << result.stage_index
      << ",\"scan_goal_stamp\":" << result.scan_goal_stamp
      << ",\"source\":\"" << result.source
      << "\",\"status\":\"" << (result.success ? "succeeded" : "failed")
      << "\",\"reason\":\"" << result.reason << "\""
      << ",\"requested_xy_tolerance\":" << (result.requested_xy_tolerance?std::to_string(*result.requested_xy_tolerance):"null");
  if(result.actual_position)out<<",\"actual_position\":["<<(*result.actual_position)[0]<<','<<(*result.actual_position)[1]<<','<<(*result.actual_position)[2]<<']';
  if(result.goal_distance)out<<",\"goal_distance\":"<<*result.goal_distance;
  out<<'}';
  message.data = out.str();
  output_.execution_result(message);
}

std::optional<double> ExplorationController::nearest_obstacle_distance(
    const XY &point, const std::vector<XY> &obstacles,
    double last_update) const {
  if (last_update <= 0.0 ||
      wall_now() - last_update > options_.occupancy_timeout_ || obstacles.empty())
    return std::nullopt;
  double nearest = std::numeric_limits<double>::infinity();
  for (const auto &obstacle : obstacles)
    nearest = std::min(nearest, xy_distance(point, obstacle));
  return nearest;
}

void ExplorationController::publish_goal_diagnostic(const Active &goal, const char *event,
    const std::string &reason) {
  const auto append_distance = [](std::ostringstream &out, const char *key,
                                  std::optional<double> value) {
    out << ",\"" << key << "\":";
    if (value && std::isfinite(*value)) out << *value;
    else out << "null";
  };
  const XY current = base_ ?
      XY{(*base_)[0], (*base_)[1]} :
      XY{goal.pose_at_send[0], goal.pose_at_send[1]};
  std::ostringstream out;
  out << std::setprecision(9) << "{\"event\":\"" << event
      << "\",\"reason\":\"" << reason
      << "\",\"goal_id\":" << goal.goal_id
      << ",\"attempt_id\":" << goal.attempt_id
      << ",\"stage_index\":" << goal.stage_index
      << ",\"input_stamp\":" << goal.input_stamp
      << ",\"scan_goal_stamp\":" << goal.stamp.value_or(0)
      << ",\"source\":\"" << goal.source_at_send
      << "\",\"map_version_at_send\":" << goal.map_version_at_send
      << ",\"map_version_now\":" << map_.version()
      << ",\"layer_version_at_send\":" << goal.layer_version_at_send
      << ",\"layer_version_now\":" << layer_version_
      << ",\"start_x\":" << goal.pose_at_send[0]
      << ",\"start_y\":" << goal.pose_at_send[1]
      << ",\"start_z\":" << goal.pose_at_send[2]
      << ",\"current_x\":" << current[0]
      << ",\"current_y\":" << current[1]
      << ",\"target_x\":" << goal.xy[0]
      << ",\"target_y\":" << goal.xy[1]
      << ",\"target_distance_now\":" << xy_distance(current, goal.xy)
      << ",\"reference_points\":" << goal.reference_point_count
      << ",\"scan_bspline_traj_id\":"
      << (active_scan_bspline_ ? active_scan_bspline_->traj_id : -1);
  append_distance(out, "raw_distance_at_send", goal.raw_distance_at_send);
  append_distance(out, "inflated_distance_at_send", goal.inflated_distance_at_send);
  append_distance(out, "raw_distance_now",
                  nearest_obstacle_distance(current, raw_obstacles_, raw_occupancy_wall_));
  append_distance(out, "inflated_distance_now",
                  nearest_obstacle_distance(current, external_obstacles_, occupancy_wall_));
  out << '}';
  std_msgs::msg::String message;
  message.data = out.str();
  output_.goal_diagnostic(message);
  RCLCPP_INFO(logger_, "Goal diagnostic: %s", message.data.c_str());
  if (std::string(event) != "failed") return;
  nav_msgs::msg::Path reference, actual;
  reference.header.frame_id = actual.header.frame_id = options_.frame_id_;
  reference.header.stamp = actual.header.stamp = clock_->now();
  const auto add_reference = [&](const XY &p) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = reference.header;
    pose.pose.position.x = p[0];
    pose.pose.position.y = p[1];
    pose.pose.position.z = goal.pose_at_send[2];
    pose.pose.orientation.w = 1.0;
    reference.poses.push_back(pose);
  };
  if(goal.source_at_send!="recovery_scan_direct")
    add_reference({goal.pose_at_send[0], goal.pose_at_send[1]});
  for (const auto &point : goal.reference_route) add_reference(point);
  output_.diagnostic_reference(reference);
  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.header = actual.header;
  start_pose.pose.position.x = goal.pose_at_send[0];
  start_pose.pose.position.y = goal.pose_at_send[1];
  start_pose.pose.position.z = goal.pose_at_send[2];
  start_pose.pose.orientation.w = 1.0;
  actual.poses.push_back(start_pose);
  for (std::size_t i = goal.trajectory_start_index;
       i < trajectory_.poses.size(); ++i)
    actual.poses.push_back(trajectory_.poses[i]);
  output_.diagnostic_actual(actual);
  if (active_scan_bspline_)
    output_.diagnostic_bspline(*active_scan_bspline_);
}

void ExplorationController::handle_scan_failure(std::int64_t stamp, const std::string &reason) {
  if (!active_ || !active_->stamp || *active_->stamp != stamp) return;
  std::optional<LayerMapReadPhase> read;
  if(shared_layer_ && shared_layer_->pct) {
    read.emplace(*shared_layer_,true);
    if(!read->ready()) {pending_scan_failure_=std::make_pair(stamp,reason);return;}
  }
  auto result = path_processor_.resolve_execution(stamp, false, reason);
  if (!result) return;
  publish_goal_diagnostic(*active_, "failed", reason);
  publish_execution_result(*result);
  ++scan_failure_count_;
  scan_last_failure_ = reason;
  if(recovery_.blocker_recovery_ && recovery_.blocker_recovery_->action &&
     recovery_.blocker_recovery_->action->kind!=PctRecoveryAction::Kind::scan) {
    const bool delegated=recovery_.blocker_recovery_->action->kind==PctRecoveryAction::Kind::delegate;
    active_.reset();
    reset_planned_route();
    recovery_.blocker_recovery_->action.reset();
    if(delegated) {
      recovery_.blocker_recovery_->delegation_started=false;
      recovery_.blocker_recovery_->stopped_pose.reset();
      recovery_.blocker_recovery_->result_at=wall_now();
      report("pct_blocker_scan_delegate_failed",reason);
    }
    else if(recovery_.blocker_recovery_->platform_shift)finish_blocker_recovery("pct_landing_shift_failed");
    else report("pct_blocker_reposition_failed",reason);
    return;
  }
  if(recovery_.stair_reposition_ && active_->reverse) {
    if(recovery_.stair_reposition_->stage==StairReposition::Stage::reverse && reason=="reverse_no_progress") {
      recovery_.stair_reposition_->partial_reverse=true;
      recovery_.stair_reposition_->failure="stair_reverse_failed:"+reason;
      await_stair_stop();
    } else if(recovery_.stair_reposition_->stage==StairReposition::Stage::shift) {
      recovery_.stair_reposition_->stage=StairReposition::Stage::review;
      recovery_.stair_reposition_->failure="stair_shift_failed:"+reason;
      await_stair_stop();
    } else block_stair_reposition("stair_reverse_failed:"+reason);
    return;
  }
  if(recovery_.stair_reposition_ && recovery_.stair_reposition_->stage==StairReposition::Stage::forward) {
    if(base_ && (*base_)[2]>recovery_.stair_reposition_->action_start[2]+policy_.cross_task_.height_slack) {
      recovery_.stair_reposition_.reset();
    } else {
      recovery_.stair_reposition_->stage=StairReposition::Stage::review;
      recovery_.stair_reposition_->failure="stair_replan_failed:"+reason;
      await_stair_stop();
      return;
    }
  }
  const bool exploring_platform=options_.floor_exploration_ && policy_.cross_task_.phase()=="exploring_platform";
  if(reason=="planning_wait_timeout" || reason=="initialization_budget_exhausted") {
    // A missing SCAN trajectory is a computational failure, not evidence
    // that this frontier is a wall or that its floor attempts are exhausted.
    if(selected_pct_)policy_.pct_cooldowns_.push_back(
        {selected_pct_->frontier,wall_now()+options_.cooldown_});
    if((policy_.cross_task_.engaged() || policy_.cross_task_.deferred()) && selected_pct_observation_) {
      auto ground=*selected_pct_observation_;ground[2]-=options_.layer_body_height_;
      policy_.failed_stair_observers_[ground]=wall_now()+options_.cooldown_;
    }
    if(base_ && selected_pct_) {
      LayerMapReadPhase lock(*shared_layer_);
      shared_layer_->pct->observation_task(shared_layer_->current_version.load(),
          "cancel",selected_pct_->id,*base_);
    }
    active_.reset();reset_planned_route();
    report(policy_.cross_task_.engaged()?"cross_layer_recovery":"scan_plan_failed",reason);
    return;
  }
  if((policy_.cross_task_.engaged() || policy_.cross_task_.deferred()) && selected_pct_observation_ && !active_->turn && !active_->recovery) {
    auto ground=*selected_pct_observation_;
    ground[2]-=options_.layer_body_height_;
    policy_.failed_stair_observers_[ground]=wall_now()+options_.cooldown_;
  }
  if(!options_.floor_exploration_ && !exploring_platform && try_queue_stair_reposition(reason))return;
  const bool turning = active_->turn;
  const bool recovering = active_->recovery;
  const XY failed_target = route_selection_target_.value_or(active_->xy);
  const auto failed_pct=selected_pct_;
  const auto failed_observation=selected_pct_observation_;
  bool floor_attempt_settled=false;
  if(selected_floor_attempt_ && failed_pct && base_) {
    LayerMapReadPhase lock(*shared_layer_);
    const auto version=shared_layer_->current_version.load();
    floor_attempt_settled=shared_layer_->pct->observation_task(
        version,"floor_execution_failed",failed_pct->id,failed_observation.value_or(*base_)).ready;
    shared_layer_->pct->observation_task(version,"cancel",failed_pct->id,*base_);
  }
  active_.reset();
  reset_planned_route();
  if(observation_task_ && turning && !recovering) {
    end_observation(false,reason);return;
  }
  if(policy_.cross_task_.engaged()) {
    if(floor_attempt_settled) {
      report("pct_floor_frontier_settled",reason);
      return;
    }
    if(exploring_platform) {
      report("cross_layer_platform_waiting",reason);
    } else if(options_.floor_exploration_) {
      recover_blocked_ascent();
    } else report("cross_layer_recovery",reason);
    return;
  }
  if (failed_pct) {
    if(!floor_attempt_settled)policy_.pct_cooldowns_.push_back({failed_pct->frontier,wall_now()+options_.cooldown_});
  }
  else suppressed_.push_back({failed_target, wall_now() + options_.cooldown_});
  report("scan_plan_failed", reason);
}

void ExplorationController::on_scan_failure(const std_msgs::msg::String &msg) {
  const auto separator = msg.data.find('|');
  if (separator == std::string::npos || separator == 0 || !active_) return;
  std::int64_t stamp = 0;
  const auto *begin = msg.data.data();
  const auto *end = begin + separator;
  const auto parsed = std::from_chars(begin, end, stamp);
  if (parsed.ec != std::errc{} || parsed.ptr != end || stamp <= 0) return;
  const auto reason = msg.data.substr(separator + 1);
  if (reason.empty() || reason.size() > 80 ||
      !std::all_of(reason.begin(), reason.end(), [](unsigned char ch) {
        return (ch >= 'a' && ch <= 'z') || ch == '_';
      })) return;
  if (!active_->stamp) {
    pending_scan_failure_ = std::make_pair(stamp, reason);
    return;
  }
  handle_scan_failure(stamp, reason);
}

void ExplorationController::on_recovery_action_result(const std_msgs::msg::String &msg, bool turn) {
  if(observation_task_ && turn)RCLCPP_INFO(logger_,"Observation action result %s (active=%lld turn=%d)",msg.data.c_str(),static_cast<long long>(active_ && active_->stamp?*active_->stamp:0),active_?active_->turn:0);
  if (!active_ || !active_->stamp ||
      (turn ? !active_->turn : !active_->reverse)) return;
  const auto separator = msg.data.find('|');
  if (separator == std::string::npos ||
      msg.data.substr(separator + 1) != "succeeded") return;
  std::int64_t stamp = 0;
  const auto *begin = msg.data.data();
  const auto parsed = std::from_chars(begin, begin + separator, stamp);
  if (parsed.ec != std::errc{} || parsed.ptr != begin + separator ||
      stamp != *active_->stamp) return;
  const bool shifting=active_->source_at_send=="recovery_shift";
  auto result = path_processor_.resolve_execution(
      stamp, true, turn ? "turn_completed" : shifting ? "shift_completed" : "reverse_completed",
      shifting || recovery_.stair_reposition_?base_:std::nullopt);
  if (!result) {RCLCPP_WARN(logger_,"Action result has no matching unresolved route stage");return;}
  publish_execution_result(*result);
  if (!turn && recovery_.complete_motion(base_, wall_now(), false)) {
    active_.reset();
    reset_planned_route();
    report("pct_blocker_recovery_waiting","fresh_map_after_reposition");
    return;
  }
  if(recovery_.stair_reposition_ && !turn) {
    await_stair_stop();
    return;
  }
  if(turn && observation_task_ && !active_->recovery) {
    active_.reset();
    if(path_processor_.advance()) {
      try_dispatch_route();report("observation_turning");return;
    }
    reset_planned_route();
    observation_task_->turning=false;
    if(base_ && base_yaw_)arm_observation_view();
    if(observation_task_->waiting_since>0)report("observation_waiting_for_local_frames");
    return;
  }
  active_.reset();
  reset_planned_route();
}

Point ExplorationController::cross_ground_pose() const {
  auto point=*base_;
  point[2]-=options_.layer_body_height_;
  const bool feet_current=stable_foot_height_ && wall_now()-foot_sample_wall_<=.25;
  if(feet_current)point[2]=*stable_foot_height_;
  if(shared_layer_ && shared_layer_->pct && layer_map_fresh()) {
    LayerMapReadPhase lock(*shared_layer_,true);
    if(!lock.ready())return point;
    auto support_pose=*base_;
    support_pose[2]=point[2]+options_.layer_body_height_;
    const auto support=shared_layer_->pct->point_check(shared_layer_->current_version.load(),support_pose,
        false,std::nullopt,0.,false,feet_current?std::optional<XY>{{point[2],.04}}:std::nullopt,true);
    if(support.ready && support.ground_z)point[2]=*support.ground_z;
  }
  return point;
}

bool ExplorationController::stable_four_foot_support(double current) const {
  return foot_support_stable_ && four_feet_stable_since_ && current-foot_sample_wall_<=.25 &&
      foot_sample_wall_-*four_feet_stable_since_>=.40;
}

double ExplorationController::refresh_stair_heading(double fallback) {
  if(!base_ || !shared_layer_ || !shared_layer_->pct || !layer_map_fresh())return fallback;
  LayerMapReadPhase lock(*shared_layer_,true);
  if(!lock.ready())return fallback;
  const auto view=shared_layer_->pct->observation_task(shared_layer_->current_version.load(),
      "stair_heading",0,*base_,fallback);
  if(!view.ready)return fallback;
  if(std::abs(std::remainder(view.yaw-fallback,2*M_PI))>.10)
    RCLCPP_INFO(logger_,"Measured stair heading refreshed: %.3f -> %.3f rad",fallback,view.yaw);
  if(policy_.cross_task_.engaged() || policy_.cross_task_.deferred())
    policy_.cross_task_.set_ascent_yaw(view.yaw);
  return view.yaw;
}

void ExplorationController::finish_active_goal() {
  // 尚无对应数据或目标时，进入此分支。
  if (!active_ || !base_) return;
  if(mission_returning_)return;
  if (selected_pct_) {
    if (path_processor_.is_last() && selected_pct_observation_ && shared_layer_ && shared_layer_->pct) {
      if(policy_.cross_task_.engaged() && selected_pct_->id==0) {
        // A verified measured-support target is movement, not another view
        // task. Continue immediately; only a real blocker can request a scan.
        policy_.cross_task_.update(cross_ground_pose());
        policy_.cross_task_.observed();
        return;
      }
      // Arrival only starts an observation task. Actual pose, yaw and
      // subsequently integrated local evidence determine completion.
      ObservationTask task;task.frontier=*selected_pct_;task.began=wall_now();
      task.planned_pose=selected_pct_observation_;
      task.cross_layer=false;
      task.platform_frontier=selected_platform_frontier_;
      task.fast_floor=selected_fast_floor_;
      task.floor_attempt=selected_floor_attempt_;
      if((task.frontier.id==0 || task.cross_layer) && !task.frontier.observation_yaws.empty())
        task.stair_heading=task.frontier.observation_yaws.front();
      if(task.cross_layer){policy_.cross_task_.update(cross_ground_pose());policy_.cross_task_.observing();}
      observation_task_=std::move(task);
    }
    return;  // PCT success never uses 2-D new-cell or low-gain gates.
  }
  auto observed = map_.project({(*base_)[0], (*base_)[1]}).size();
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (path_processor_.waypoints().size() > path_processor_.index() + 1) return;
  if (!active_->recovery) {
    goal_progress_gate_.finish(active_->xy, active_->observed_cells_at_send, observed);
    // 低收益目标可以在没有高收益目标时重试，但短时间内不重复同一点。
    const auto gained = observed - std::min(observed, active_->observed_cells_at_send);
    if (gained < options_.min_new_cells_ && options_.low_yield_retry_cooldown_ > 0.0)
      suppressed_.push_back({route_selection_target_.value_or(active_->xy),
                             wall_now() + options_.low_yield_retry_cooldown_});
  }
}

void ExplorationController::publish_marker(const std::optional<XY> &goal) {
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = options_.frame_id_;
  marker.header.stamp = clock_->now();
  marker.ns = "selected_frontier";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = goal ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
  marker.pose.orientation.w = 1.0;
  // 根据当前目标是否存在或是否完成来决定下一步。
  if (goal) {
    marker.pose.position.x = (*goal)[0];
    marker.pose.position.y = (*goal)[1];
    marker.pose.position.z = path_processor_.current_z().value_or((*base_)[2]);
    marker.scale.x = marker.scale.y = marker.scale.z = 0.35;
    marker.color.r = 0.0; marker.color.g = 0.9; marker.color.b = 1.0; marker.color.a = 0.9;
  }
  // 把本次结果放入容器，供下一轮处理。
  array.markers.push_back(marker);
  output_.marker(array);
}

void ExplorationController::publish_pct_frontiers() {
  if(shared_layer_ && layer_map_fresh()) {
    LayerMapReadPhase lock(*shared_layer_,true);
    if(!lock.ready()) {pct_markers_dirty_=true;return;}
    pct_marker_version_=shared_layer_->current_version.load();
    if(!shared_layer_->pct->refresh_frontier_displays(pct_marker_version_,policy_.pct_frontiers_.clusters))
      RCLCPP_WARN(logger_,"Frontier display refresh failed: %s",shared_layer_->pct->last_error().c_str());
    pct_marker_wall_=wall_now();
  }
  visualization_msgs::msg::MarkerArray array;
  std::set<std::pair<std::string,int>> current;
  int marker_id = 0;
  for (const auto &cluster:policy_.pct_frontiers_.clusters) {
    if(cluster.display.empty())continue;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id=options_.frame_id_;marker.header.stamp=clock_->now();
    marker.ns="pct_surface_frontiers";marker.id=marker_id++;
    marker.type=visualization_msgs::msg::Marker::POINTS;
    marker.action=visualization_msgs::msg::Marker::ADD;marker.pose.orientation.w=1;
    marker.scale.x=marker.scale.y=0.12;
    marker.color.r=0.3+0.7*((cluster.id*37)%100)/100.0;
    marker.color.g=0.3+0.7*((cluster.id*61)%100)/100.0;
    marker.color.b=0.3+0.7*((cluster.id*83)%100)/100.0;marker.color.a=1;
    for (const auto &p:cluster.display) {
      geometry_msgs::msg::Point point;point.x=p[0];point.y=p[1];point.z=p[2]+0.05;
      marker.points.push_back(point);
    }
    if(!marker.points.empty()) {
      current.emplace(marker.ns,marker.id);
      array.markers.push_back(marker);
    }
    marker.ns="pct_observations";marker.type=visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.scale.x=marker.scale.y=marker.scale.z=0.20;marker.points.clear();
    for (const auto &p:cluster.observations) {
      geometry_msgs::msg::Point point;point.x=p[0];point.y=p[1];point.z=p[2]+options_.layer_body_height_;
      marker.points.push_back(point);
    }
    if(!marker.points.empty()) {
      current.emplace(marker.ns,marker.id);
      array.markers.push_back(std::move(marker));
    }
  }
  if (selected_pct_observation_) {
    visualization_msgs::msg::Marker selected;
    selected.header.frame_id=options_.frame_id_;selected.header.stamp=clock_->now();
    selected.ns="pct_selected_observation";selected.id=0;
    selected.type=visualization_msgs::msg::Marker::SPHERE;
    selected.action=visualization_msgs::msg::Marker::ADD;selected.pose.orientation.w=1;
    selected.pose.position.x=(*selected_pct_observation_)[0];
    selected.pose.position.y=(*selected_pct_observation_)[1];
    selected.pose.position.z=(*selected_pct_observation_)[2];
    selected.scale.x=selected.scale.y=selected.scale.z=0.35;
    selected.color.r=1;selected.color.g=1;selected.color.a=1;
    current.emplace(selected.ns,selected.id);
    array.markers.push_back(selected);
  }
  for(const auto &key:pct_marker_keys_) {
    if(current.count(key))continue;
    visualization_msgs::msg::Marker marker;
    marker.ns=key.first;marker.id=key.second;
    marker.action=visualization_msgs::msg::Marker::DELETE;
    array.markers.push_back(std::move(marker));
  }
  pct_marker_keys_=std::move(current);
  output_.pct_frontier(array);
}

void ExplorationController::arm_observation_view() {
  auto &task=*observation_task_;
  LayerMapReadPhase lock(*shared_layer_,true);
  if(!lock.ready()) {task.arm_pending=true;return;}
  task.arm_pending=false;
  shared_layer_->pct->observation_task(shared_layer_->current_version.load(),
      task.targeted_blocker_scan?"arm_blocker_review":"arm",
      task.frontier.id,*base_,*base_yaw_);
  task.waiting_since=wall_now();
}

void ExplorationController::observation_phase(const char *phase,double current) {
  if(!observation_task_)return;
  auto &task=*observation_task_;
  if(task.phase==phase)return;
  if(!task.phase.empty())task.phase_seconds[task.phase]+=std::max(0.,current-task.phase_since);
  task.phase=phase;task.phase_since=current;
}

void ExplorationController::end_observation(bool success,const std::string &reason,bool floor_attempt_settled) {
  if(!observation_task_)return;
  LayerMapReadPhase read(*shared_layer_,true);
  if(!read.ready()) {
    auto &task=*observation_task_;
    if(active_ && task.action_owner && active_->turn &&
       active_->goal_id==task.action_owner->first && active_->attempt_id==task.action_owner->second) {
      cancel_recovery_action();
      if(active_->stamp)if(auto result=path_processor_.resolve_execution(*active_->stamp,false,reason,base_))
        publish_execution_result(*result);
      active_.reset();pending_scan_ack_.reset();
    }
    if(!task.pending_end)task.pending_end=std::make_tuple(success,reason,floor_attempt_settled);
    observation_phase("map_wait",wall_now());report("observation_waiting_for_map");return;
  }
  observation_task_->pending_end.reset();
  const double ended=wall_now();
  auto &ending=*observation_task_;
  if(!ending.phase.empty())ending.phase_seconds[ending.phase]+=std::max(0.,ended-ending.phase_since);
  if(ending.pose_wait_since) {
    ending.pose_wait_seconds+=std::max(0.,ended-*ending.pose_wait_since);
    ending.pose_wait_since.reset();
  }
  // Retire the owned action BEFORE clearing its execution stage. A late
  // result is then an inert receipt, never a permanently active orphan.
  const bool owns_action=active_ && ending.action_owner && active_->turn &&
      active_->goal_id==ending.action_owner->first && active_->attempt_id==ending.action_owner->second;
  if(owns_action) {
    cancel_recovery_action();
    if(active_->stamp) {
      if(auto result=path_processor_.resolve_execution(*active_->stamp,false,reason,base_))
        publish_execution_result(*result);
    }
    active_.reset();pending_scan_ack_.reset();
  }
  const auto frontier=observation_task_->frontier;
  RCLCPP_INFO(logger_,"Observation task %s: %s",success?"completed":"unresolved",reason.c_str());
  const bool support_scan=observation_task_->support_scan;
  if(observation_task_->targeted_blocker_scan && shared_layer_ && shared_layer_->pct && base_) {
    LayerMapReadPhase lock(*shared_layer_);
    observation_metadata_=shared_layer_->pct->observation_task(shared_layer_->current_version.load(),
        "finish_blocker_review",0,*base_).metadata;
  }
  const bool platform_frontier=observation_task_->platform_frontier;
  auto continuation=std::move(observation_task_->continuation);
  const int continuation_attempt=observation_task_->floor_attempt;
  std::string settlement;
  if(success && !support_scan && base_ && base_yaw_) {
    LayerMapReadPhase lock(*shared_layer_);
    const auto result=shared_layer_->pct->observation_task(shared_layer_->current_version.load(),"finish",
        frontier.id,*base_,*base_yaw_,observation_task_->coverage);
    if(observation_task_->floor_attempt==2 && !platform_frontier && !policy_.floor_complete_)
      policy_.floor_review_resolved_+=result.resolved_frontier_cells;
    settlement=result.metadata;
    if(observation_task_->floor_attempt && result.floor_attempt=="inconclusive")
      policy_.pct_floor_wait_for_frame_after_=shared_layer_->last_integration_wall_seconds.load();
  } else if(!success && !support_scan && !floor_attempt_settled && reason!="paused" && reason!="manual_override") {
    if(observation_task_->floor_attempt && base_ && reason!="pose_timeout" && reason!="frontier_resolved") {
      LayerMapReadPhase lock(*shared_layer_);
      const auto result=shared_layer_->pct->observation_task(shared_layer_->current_version.load(),
          "floor_execution_failed",frontier.id,*base_);
      settlement=result.metadata;
    } else if(!observation_task_->floor_attempt) {
      policy_.pct_cooldowns_.push_back({frontier.frontier,wall_now()+options_.cooldown_});
    }
  }
  std::ostringstream event;
  event<<"{\"success\":"<<(success?"true":"false")<<",\"reason\":"<<std::quoted(reason)
       <<",\"cluster_id\":"<<frontier.id<<",\"evidence\":"<<observation_metadata_
       <<",\"duration_s\":"<<wall_now()-observation_task_->began
       <<",\"views_completed\":"<<observation_task_->views_completed;
  event<<",\"pose_wait_s\":"<<ending.pose_wait_seconds
       <<",\"pose_age_s\":"<<(base_?std::to_string(std::max(0.,ended-base_wall_)):"null")
       <<",\"yaw_age_s\":"<<(base_yaw_?std::to_string(std::max(0.,ended-yaw_wall_)):"null")
       <<",\"phase_seconds\":{";
  bool first_phase=true;
  for(const auto &[phase,seconds]:ending.phase_seconds) {
    if(!first_phase)event<<',';
    first_phase=false;event<<std::quoted(phase)<<':'<<seconds;
  }
  event<<'}';
  if(ending.action_owner)event<<",\"action_goal_id\":"<<ending.action_owner->first
      <<",\"action_attempt_id\":"<<ending.action_owner->second;
  if(observation_task_->floor_attempt)event<<",\"floor_frontier_attempt\":"<<observation_task_->floor_attempt;
  event<<",\"selection_map_version\":"<<frontier.revision
       <<",\"observation_map_version\":"<<shared_layer_->current_version.load();
  if(observation_task_->planned_pose) {
    const auto &planned=*observation_task_->planned_pose;
    event<<",\"planned_position\":["<<planned[0]<<','<<planned[1]<<','<<planned[2]<<']';
  }
  if(!settlement.empty())event<<",\"settlement\":"<<settlement;
  if(base_)event<<",\"actual_position\":["<<(*base_)[0]<<','<<(*base_)[1]<<','<<(*base_)[2]<<']';
  if(base_yaw_)event<<",\"actual_yaw\":"<<*base_yaw_;
  event<<",\"sweep_half_angle\":"<<observation_task_->coverage<<'}';
  std_msgs::msg::String feedback;feedback.data=event.str();output_.observation_feedback(feedback);
  if((!success || support_scan) && shared_layer_ && shared_layer_->pct && base_) {
    LayerMapReadPhase lock(*shared_layer_);
    const char *operation=support_scan && (continuation_attempt ||
        (recovery_.blocker_recovery_ && recovery_.blocker_recovery_->floor_attempt))?
        "cancel_recovery_view":"cancel";
    shared_layer_->pct->observation_task(shared_layer_->current_version.load(),operation,frontier.id,*base_);
  }
  observation_task_.reset();
  // No observer is allowed to erase a route already owned by a newer task.
  if(!active_)reset_planned_route();
  if (support_scan) recovery_.complete_scan(success);
  if(policy_.cross_task_.engaged()) {
    if(!platform_frontier)policy_.cross_task_.observed();
  }
  if(continuation && success && base_) {
    const auto next_z=continuation->current_z();
    const auto &next=continuation->current();
    const Point next_pose{next[0],next[1],next_z.value_or((*base_)[2])};
    LayerMapReadPhase lock(*shared_layer_);
    const auto checked=shared_layer_->pct->arrival_area(shared_layer_->current_version.load(),
        next_pose,options_.observation_goal_xy_tolerance_,policy_.cross_task_.ascent_yaw());
    if(!checked.ready) {
      begin_blocker_recovery(frontier,checked.unknown_blockers,true,0,"pct_stair_frontier",
                             std::nullopt,std::nullopt,true);
      report("cross_layer_waiting_for_forward_route","continuation_requires_observation");
      return;
    }
    path_processor_=std::move(*continuation);
    selected_pct_=frontier;
    selected_pct_observation_=path_processor_.xyz().empty()?std::nullopt:
        std::optional<Point>{path_processor_.xyz().back()};
    selected_platform_frontier_=platform_frontier;
    selected_floor_attempt_=continuation_attempt;
    route_selection_target_=XY{frontier.frontier[0],frontier.frontier[1]};
  }
  // Cross-layer selection checks the current footprint and platform before
  // choosing a route or recovery after a failed segment scan.
  report(success?"observation_completed":"observation_unresolved",reason);
}

bool ExplorationController::dispatch_observation_turn(double angle) {
  static const std::unordered_map<Cell,int,CellHash> empty_grid;
  static const std::vector<XY> empty_obstacles;
  LayerMapReadPhase lock(*shared_layer_);
  if(observation_task_ && observation_task_->support_scan && observation_task_->cross_layer && std::abs(angle)>.71)return false;
  PlanningGoal goal;goal.goal_id=++goal_id_;goal.attempt_id=++planning_attempt_id_;
  goal.mode=policy_.cross_task_.engaged()?ExplorationMode::cross_layer:ExplorationMode::flat;
  goal.layer_version=shared_layer_->current_version.load();
  goal.occupancy_version=map_.version();goal.request_layer=true;goal.request_occupancy=false;
  goal.recovery_request=RecoveryRequest::turn;goal.recovery_turn_angle=angle;
  goal.recovery_yaw=*base_yaw_;goal.observation={(*base_)[0],(*base_)[1]};goal.target=goal.observation;
  const PlanningSnapshot snapshot{empty_grid,empty_obstacles,cfg_,*base_,options_.layer_body_height_,true,nullptr,shared_layer_.get(),map_.version()};
  const auto plans=route_planner_.plan(goal,snapshot);
  const auto *chosen=plans.selected();
  if(!chosen || !path_processor_.load(goal,*chosen,false,&snapshot)) {
    for(const auto &result:plans.candidates)RCLCPP_WARN(logger_,"Observation turn rejected: %s, %s",result.reason.c_str(),result.backend_metadata.c_str());
    return false;
  }
  observation_task_->action_owner=std::make_pair(goal.goal_id,goal.attempt_id);
  observation_task_->turning=true;
  // Map work may temporarily age the last odometry message. Keep a valid
  // checked action queued until fresh odometry arrives, like a position route.
  try_dispatch_route();return true;
}

PctObservation ExplorationController::stair_observation_progress() {
  LayerMapReadPhase lock(*shared_layer_);
  const auto &task=*observation_task_;
  return shared_layer_->pct->observation_task(shared_layer_->current_version.load(),
      "stair_step_progress",0,*base_,policy_.cross_task_.ascent_yaw());
}

bool ExplorationController::handle_observation(double current) {
  if(!observation_task_)return false;
  if(paused_ || options_.observation_only_ || !ready_) {report("observation_paused");return true;}
  // Repeated short pose outages must not extend one task indefinitely.
  if(current-observation_task_->began>options_.observation_timeout_+options_.observation_pose_wait_timeout_) {
    end_observation(false,"observation_timeout");return true;
  }
  if(!base_ || !base_yaw_ || current-base_wall_>options_.odom_timeout_ || current-yaw_wall_>options_.odom_timeout_) {
    auto &task=*observation_task_;
    if(!task.pose_wait_since)task.pose_wait_since=current;
    observation_phase("pose_wait",current);
    if(current-*task.pose_wait_since>=options_.observation_pose_wait_timeout_)end_observation(false,"pose_timeout");
    else report("observation_waiting_for_pose");return true;
  }
  if(observation_task_->pose_wait_since) {
    observation_task_->pose_wait_seconds+=std::max(0.,current-*observation_task_->pose_wait_since);
    observation_task_->pose_wait_since.reset();
  }
  if(current-observation_task_->began-observation_task_->pose_wait_seconds>options_.observation_timeout_) {
    end_observation(false,"observation_timeout");return true;
  }
  if(active_) {observation_phase("turn",current);report("observation_turning");return true;}
  if(path_processor_.active()) {observation_phase("motion_queue",current);try_dispatch_route();return true;}
  LayerMapReadPhase read(*shared_layer_,true);
  if(!read.ready()) {observation_phase("map_wait",current);report("observation_waiting_for_map");return true;}
  if(observation_task_->arm_pending) {arm_observation_view();return true;}
  observation_phase(observation_task_->waiting_since>0?"evidence_wait":"view_check",current);
  auto &task=*observation_task_;
  const bool stair_frontier=task.cross_layer && !task.platform_frontier && !task.support_scan;
  // Already observed and reachable is sufficient: avoid a redundant turn
  // or dwell when the next stance is available before starting this sweep.
  if(task.next==0 && !task.waiting_since && !task.targeted_blocker_scan && stair_frontier) {
    const auto progress=stair_observation_progress();
    observation_metadata_=progress.metadata;
    if(progress.ready) {
      end_observation(true,"stair_frontier_continuation_ready");return true;
    }
  }
  if(!task.initialized) {
    PctObservation view;
    const bool directional=task.directional();
    {LayerMapReadPhase lock(*shared_layer_);
     view=shared_layer_->pct->observation_task(shared_layer_->current_version.load(),
         directional?"begin_forward":options_.floor_exploration_?"begin_floor":"begin_flat",task.frontier.id,*base_,
         directional?task.stair_heading.value_or(policy_.cross_task_.forward_yaw()):*base_yaw_,
         directional?std::min(options_.observation_sweep_,.35):options_.observation_sweep_);}
    observation_metadata_=view.metadata;
    RCLCPP_INFO(logger_,"Observation view: %s",view.metadata.c_str());
    if(!view.ready) {
      end_observation(false,view.reason,!view.floor_attempt.empty() && view.floor_attempt!="untracked");
      return true;
    }
    task.center=view.ready?view.yaw:policy_.cross_task_.ascent_yaw();
    task.coverage=0.;task.initialized=true;
    task.headings={task.center};
    // Climbing observes the selected target only. No fixed left/right sweep.
    if(!directional && !policy_.cross_task_.engaged() && view.side_yaw)
      task.headings.push_back(*view.side_yaw);
  }
  if(task.waiting_since>0) {
    const double dwell=task.directional()?options_.observation_wait_:options_.observation_flat_wait_;
    if(stair_frontier && !task.targeted_blocker_scan) {
      if(shared_layer_->last_integration_wall_seconds.load()>task.waiting_since) {
        const auto progress=stair_observation_progress();
        observation_metadata_=progress.metadata;
        if(progress.ready) {
          ++task.views_completed;
          end_observation(true,"stair_frontier_continuation_ready");return true;
        }
      }
      if(current-task.waiting_since<dwell) {report("observation_waiting_for_local_frames");return true;}
      task.waiting_since=0.;
      ++task.views_completed;
      if(task.next>=task.headings.size()) {
        end_observation(true,"sweep_executed");return true;
      }
    } else {
      if(task.targeted_blocker_scan) {
        PctObservation progress;
        {LayerMapReadPhase lock(*shared_layer_);
         progress=shared_layer_->pct->observation_task(shared_layer_->current_version.load(),
             "blocker_review_progress",0,*base_);}
        observation_metadata_=progress.metadata;
        if(progress.ready) {
          ++task.views_completed;
          end_observation(true,"stair_support_sweep_executed");return true;
        }
        if(current-task.waiting_since>=dwell) {
          end_observation(false,progress.fresh?"blocker_navigation_evidence_pending":"blocker_no_target_return");return true;
        }
        report(progress.fresh?"observation_waiting_for_navigation_evidence":
                             "observation_waiting_for_target_returns",progress.reason);return true;
      }
      if(current-task.waiting_since<dwell) {
        report("observation_waiting_for_local_frames");return true;
      }
      PctObservation progress;
      {LayerMapReadPhase lock(*shared_layer_);
       progress=shared_layer_->pct->observation_task(shared_layer_->current_version.load(),
           "progress",task.frontier.id,*base_);}
      observation_metadata_=progress.metadata;
      task.waiting_since=0.;
      ++task.views_completed;
    }
  }
  if(task.next>=task.headings.size()) {
    if(!task.directional() && !task.fast_floor && !policy_.cross_task_.engaged() &&
       options_.observation_full_sweep_ && task.next==task.headings.size()) {
      ++task.next;task.coverage=M_PI;
      if(!dispatch_observation_turn(2*M_PI))end_observation(false,"full_sweep_space_rejected");
    } else end_observation(true,task.support_scan?
        (task.cross_layer?"stair_support_sweep_executed":"blocker_sweep_executed"):"sweep_executed");
    return true;
  }
  const double heading=task.headings[task.next++];
  const double angle=std::atan2(std::sin(heading-*base_yaw_),std::cos(heading-*base_yaw_));
  const double offset=std::abs(std::remainder(heading-task.center,2*M_PI));
  if(std::abs(angle)<=.06) {
    arm_observation_view();
    if(task.waiting_since>0)report("observation_waiting_for_local_frames");
  } else if(!dispatch_observation_turn(angle)) {
    const bool primary_done=!task.directional() && task.views_completed>0;
    end_observation(primary_done,primary_done?"side_turn_space_rejected":"turn_space_rejected");
    return true;
  }
  task.coverage=std::max(task.coverage,offset);
  return true;
}

void ExplorationController::tick_pct(double current) {
  const double previous_cycle=selection_cycle_started_;
  run_pct_cycle(current);
  if(selection_cycle_started_!=previous_cycle) {
    selection_timings_["total_ms"]=1000.*(wall_now()-selection_cycle_started_);
    std::ostringstream timing;
    for(const auto &[phase,elapsed]:selection_timings_)timing<<' '<<phase<<'='<<elapsed;
    RCLCPP_INFO(logger_,"PCT selection timing:%s",timing.str().c_str());
  }
  if(selection_resume_at_) {
    selection_yielded_at_=wall_now();
    selection_resume_at_=selection_yielded_at_+options_.pct_query_resume_delay_;
  }
  if(shared_layer_ && wall_now()-pct_marker_wall_>=1. &&
     shared_layer_->current_version.load()!=pct_marker_version_)pct_markers_dirty_=true;
  if(pct_markers_dirty_) {
    pct_markers_dirty_=false;
    publish_pct_frontiers();
  }
}

void ExplorationController::tick_pending_selection() {
  if(!selection_resume_at_ || wall_now()<*selection_resume_at_)return;
  if(map_read_retry_ && !paused_ && ready_ && !options_.observation_only_) {
    map_read_retry_=false;selection_resume_at_.reset();++selection_resume_count_;tick_pct(wall_now());return;
  }
  if(active_ || path_processor_.active() || observation_task_ || recovery_.blocker_recovery_ ||
     recovery_.stair_reposition_ || paused_ || !ready_ || options_.observation_only_) {
    selection_resume_at_.reset();return;
  }
  // The previous callback has returned and released all map locks. This timer
  // never spins/waits while holding a read phase and uses the same selector.
  selection_resume_at_.reset();
  ++selection_resume_count_;
  selection_resume_gap_ms_=1000.*(wall_now()-selection_yielded_at_);
  tick_pct(wall_now());
}

void ExplorationController::tick() {
  double current = wall_now();
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (!trajectory_.poses.empty()) {
    trajectory_.header.stamp = clock_->now();
    output_.trajectory(trajectory_);
  }
  if (options_.frontier_source_=="pct") {
    // A regular 1 Hz tick must respect the same lock-free yield interval as
    // the continuation timer, rather than starting another batch immediately.
    if(selection_resume_at_ && current<*selection_resume_at_)return;
    tick_pct(current);return;
  }
  if(active_) {
    empty_since_.reset();
    report(active_->stamp?"waiting_for_scan_completion":"waiting_for_scan_forward");return;
  }
  if(path_processor_.active()) {
    if(!try_dispatch_route())report(paused_?"paused":"waiting_for_waypoint_coordinator");
    return;
  }
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  // 数据失效检查：基座、里程计、点云必须在对应超时范围内
  // 感知数据过期时暂停新的选点；已下发路点仍由 SCAN 执行。
  if (!base_ || current - base_wall_ > options_.odom_timeout_ ||
      cloud_wall_ == 0.0 || current - cloud_wall_ > options_.cloud_timeout_) {
    // 清空临时状态，准备下一轮处理。
    empty_since_.reset();
    report("waiting_for_fresh_data");
    // 结束当前函数，不再执行后面的语句。
    return;
  }
  // 清理已过期的 suppressed 前沿（冷却结束）
  // 从容器中取走已处理或失效的数据。
  suppressed_.erase(std::remove_if(suppressed_.begin(), suppressed_.end(),
      [current](const Suppressed &s) { return s.expiry <= current; }), suppressed_.end());
  auto projected = map_.project({(*base_)[0], (*base_)[1]});
  auto route_grid = cfg_.global_search ? map_.project_all() : projected;
  coverage_ = assess_reachable_coverage(projected, cfg_,
                                            {(*base_)[0], (*base_)[1]});
  publish_coverage_grid(projected);
  if (cfg_.global_search) {
    auto dirty = map_.take_dirty_cells();
    if (!frontier_index_initialized_) {
      frontier_index_.rebuild(route_grid);
      frontier_index_initialized_ = true;
    } else if (!dirty.empty()) {
      frontier_index_.update(route_grid, dirty);
    }
    global_frontiers_ = assess_global_frontiers(
        route_grid, cfg_, {(*base_)[0], (*base_)[1]},
        &frontier_index_.cells());
  }

  // A sent waypoint stays active until SCAN acknowledges completion or an
  // external manual goal takes over. Map changes and elapsed time do not
  // invalidate it here.

  // ------------------- 没有活动目标：尝试寻找新前沿 -------------------
  // 地图已经达到体素数量上限，不能继续插入。
  if (map_.size() >= cfg_.max_voxels) {
    empty_since_.reset();
    report("map_capacity_reached");
    // 结束当前函数，不再执行后面的语句。
    return;
  }
  if (control_mode_ == ExplorationMode::waiting_initial_data)
    control_mode_ = ExplorationMode::flat;
  // 投影到水平面，检查是否还有 free（未占据）空间
  // 矩形覆盖率只供显示；全局完成不依赖预设矩形是否覆盖整片场地。
  unresolved_unknown_cells_ = 0;
  bool has_free = cfg_.global_search ? global_frontiers_.valid :
      std::any_of(projected.begin(), projected.end(),
                  [](const auto &item) { return item.second == 0; });
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (!has_free) {
    empty_since_.reset();
    report("waiting_for_free_space");
    // 结束当前函数，不再执行后面的语句。
    return;
  }
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (options_.planner_mode_!="layer" && options_.require_planner_occupancy_ && !planner_occupancy_fresh(current)) {
    empty_since_.reset();
    report("waiting_for_planner_occupancy");
    // 结束当前函数，不再执行后面的语句。
    return;
  }
  static const std::vector<XY> no_route_obstacles;
  const auto &route_obstacles = planner_occupancy_fresh(current) ?
      external_obstacles_ : no_route_obstacles;
  if (cfg_.global_search && !global_frontiers_.valid) {
    empty_since_.reset();
    report("waiting_for_reachability_seed");
    return;
  }
  if (cfg_.global_search && global_frontiers_.frontier_cells == 0) {
    if (paused_) {
      empty_since_.reset();
      report("paused");
      return;
    }
    if (!empty_since_) empty_since_ = current;
    if (current - *empty_since_ >= options_.no_frontier_hold_) {
      // 只有完成前才独立全图复核；缓存遗漏会触发修复而非误报完成。
      auto audit = assess_global_frontiers(
          route_grid, cfg_, {(*base_)[0], (*base_)[1]});
      if (!audit.valid) {
        empty_since_.reset();
        report("waiting_for_reachability_seed");
        return;
      }
      if (audit.frontier_cells > 0) {
        frontier_index_.rebuild(route_grid);
        ++frontier_index_repairs_;
        global_frontiers_ = std::move(audit);
        empty_since_.reset();
        report("frontier_index_repaired");
        return;
      }
    }
    unresolved_unknown_cells_ = 0;
    last_frontier_cells_ = last_frontier_clusters_ = 0;
    last_reachable_frontier_cells_ = last_candidate_goals_ = 0;
    last_suppressed_goals_ = last_open_start_neighbors_ = 0;
    publish_marker(std::nullopt);
    report(current - *empty_since_ >= options_.no_frontier_hold_ ?
           "exploration_complete" : "waiting_for_coverage_confirmation");
    return;
  }
  // 构造被抑制的前沿列表（用于前沿选择的过滤）
  std::vector<XY> failed;
  // 依次处理水平坐标。
  for (const auto &s : suppressed_) failed.push_back(s.xy);
  const auto temporary_failures = failed;
  auto unproductive = goal_progress_gate_.blocked(projected.size());
  failed.insert(failed.end(), unproductive.begin(), unproductive.end());
  static const std::vector<XY> no_external_obstacles;
  const auto &obstacles = planner_occupancy_fresh(current) ?
      external_obstacles_ : no_external_obstacles;
  // 将右侧计算值存入 selection（前沿搜索结果）。
  auto selection = select_frontier(
      map_, *base_, cfg_, failed, obstacles, false,
      cfg_.global_search ? &global_frontiers_.cells : nullptr);
  bool search_all_observed = false;
  if (cfg_.global_search && !selection.goal && global_frontiers_.frontier_cells > 0) {
    auto beyond_bounds = select_frontier(map_, *base_, cfg_, failed,
                                             obstacles, true, &global_frontiers_.cells);
    if (beyond_bounds.goal || beyond_bounds.frontier_cells > selection.frontier_cells) {
      selection = std::move(beyond_bounds);
      search_all_observed = true;
    }
  }
  // 高收益目标用尽时，允许重新考虑低收益目标；仍避开正在冷却的失败目标。
  bool low_yield_goal = false;
  if (!selection.goal && !unproductive.empty()) {
    auto lower_yield = select_frontier(
        map_, *base_, cfg_, temporary_failures, obstacles, false,
        cfg_.global_search ? &global_frontiers_.cells : nullptr);
    bool lower_search_all = false;
    if (cfg_.global_search && !lower_yield.goal && global_frontiers_.frontier_cells > 0) {
      auto beyond_bounds = select_frontier(
          map_, *base_, cfg_, temporary_failures, obstacles, true,
          &global_frontiers_.cells);
      if (beyond_bounds.goal || beyond_bounds.frontier_cells > lower_yield.frontier_cells) {
        lower_yield = std::move(beyond_bounds);
        lower_search_all = true;
      }
    }
    if (lower_yield.goal) {
      selection = std::move(lower_yield);
      search_all_observed = lower_search_all;
      low_yield_goal = true;
    }
  }
  bool residual_goal = false;
  // 检查未知边界是否仍存在或是否满足簇规模。
  if (cfg_.global_search &&
      (selection.frontier_cells > 0 || global_frontiers_.frontier_cells > 0) &&
      selection.clusters == 0) {
    // 只有表达式为真，才执行下面缩进或花括号内的代码。
    if (!residual_since_) residual_since_ = current;
    // 检查未知边界是否仍存在或是否满足簇规模。
    if (current - *residual_since_ >= options_.no_frontier_hold_) {
      auto residual_cfg = cfg_;
      residual_cfg.min_frontier_cells = 1;
      auto residual = select_frontier(
          map_, *base_, residual_cfg, failed, obstacles, search_all_observed,
          &global_frontiers_.cells);
      if (!residual.goal && !search_all_observed &&
          global_frontiers_.frontier_cells > 0)
        residual = select_frontier(map_, *base_, residual_cfg, failed,
                                       obstacles, true, &global_frontiers_.cells);
      // 根据当前目标是否存在或是否完成来决定下一步。
      if (residual.goal) {
        selection.goal = residual.goal;
        selection.score = residual.score;
        residual_goal = true;
      }
    }
  } else {
    // 清空临时状态，准备下一轮处理。
    residual_since_.reset();
  }
  last_frontier_cells_ = selection.frontier_cells;
  last_frontier_clusters_ = selection.clusters;
  last_reachable_frontier_cells_ = selection.reachable_frontier_cells;
  last_candidate_goals_ = selection.candidate_goals;
  last_suppressed_goals_ = selection.suppressed_goals;
  last_open_start_neighbors_ = selection.open_start_neighbors;
  // 在 RViz 中展示选中的前沿（若有）
  publish_marker(selection.goal);
  // 根据当前目标是否存在或是否完成来决定下一步。
  if (!selection.goal) {
    if (cfg_.global_search) unresolved_unknown_cells_ = coverage_.reachable_unknown_cells;
    // 检查未知边界是否仍存在或是否满足簇规模。
    if (cfg_.global_search ? global_frontiers_.frontier_cells > 0 :
        selection.frontier_cells > 0) {
      // 清空临时状态，准备下一轮处理。
      empty_since_.reset();
      report(selection.frontier_cells == 0 ? "global_frontiers_unresolved" :
             selection.clusters > 0 ? "frontiers_unreachable_or_suppressed" :
             "frontiers_below_cluster_threshold");
    } else {
      // 全图模式已在上方通过独立复核退出；这里只处理局部模式。
      if (!empty_since_) empty_since_ = current;
      report(current - *empty_since_ >= options_.no_frontier_hold_ ?
             "current_region_complete" : "waiting_for_frontier");
    }
    // 结束当前函数，不再执行后面的语句。
    return;
  }
  // 清空临时状态，准备下一轮处理。
  empty_since_.reset();
  // 只有表达式为真，才执行下面缩进或花括号内的代码。
  if (options_.observation_only_) report("observation_only");
  // 上一条件不满足时，再检查这个分支。
  else if (paused_) report("paused");
  // 上一条件不满足时，再检查这个分支。
  else if (!ready_ || output_.waypoint_subscribers() == 0)
    report("waiting_for_waypoint_coordinator");
  // 前面的条件不成立时，执行这个备选分支。
  else
  prepare_route(*selection.goal, route_grid, obstacles,
                residual_goal ? "residual_frontier" :
                low_yield_goal ? "low_yield_frontier" : "", selection.score);
}

}  // namespace legbot_exploration
