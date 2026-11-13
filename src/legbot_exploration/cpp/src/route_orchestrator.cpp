#include "legbot_exploration/route_orchestrator.hpp"

#include <cmath>
#include <chrono>
#include <mutex>
#include <shared_mutex>
#include <utility>
#include <sstream>
#include <iomanip>

namespace legbot_exploration {

namespace {
PlannerFailureClass classify_failure(const std::string &reason) {
  if (reason == "ok")
    return PlannerFailureClass::none;
  if (reason == "unsafe_start" || reason == "unsafe_start_cell" ||
      reason == "layer_start_corridor_blocked" || reason == "layer_unsafe_start" || reason == "pct_unsafe_start")
    return PlannerFailureClass::unsafe_start;
  if (reason == "unsafe_viewpoint" || reason == "unsafe_endpoint_cell" ||
      reason == "unsafe_endpoint_approach" ||
      reason == "layer_goal_corridor_blocked" || reason == "layer_unsafe_goal" ||
      reason == "layer_unsafe_goal_connection" || reason == "pct_unsafe_goal" ||
      reason.rfind("arrival_",0)==0)
    return PlannerFailureClass::unsafe_goal;
  if (reason == "layer_planning_timeout" || reason == "pct_planning_timeout") return PlannerFailureClass::timeout;
  if (reason == "no_inflated_safe_layer_surface" || reason == "pct_backend_unavailable" || reason == "layer_map_version_changed")
    return PlannerFailureClass::unavailable;
  if (reason == "occupancy_route_disconnected" ||
      reason == "surface_route_disconnected" || reason == "pct_route_disconnected")
    return PlannerFailureClass::disconnected;
  return PlannerFailureClass::other;
}
}

const char *mode_name(ExplorationMode mode) {
  switch (mode) {
    case ExplorationMode::waiting_initial_data: return "waiting_initial_data";
    case ExplorationMode::flat: return "flat";
    case ExplorationMode::cross_layer: return "cross_layer";
    case ExplorationMode::recovery: return "recovery";
  }
  return "invalid";
}

const char *planner_name(PlannerKind kind) {
  switch (kind) {
    case PlannerKind::occupancy: return "occupancy";
    case PlannerKind::layer: return "layer";
    case PlannerKind::recovery_layer: return "recovery_layer";
    case PlannerKind::scan_direct: return "scan_direct";
    case PlannerKind::observation_turn: return "observation_turn";
  }
  return "invalid";
}

const char *planner_failure_name(PlannerFailureClass failure) {
  switch (failure) {
    case PlannerFailureClass::none: return "none";
    case PlannerFailureClass::unsafe_start: return "unsafe_start";
    case PlannerFailureClass::unsafe_goal: return "unsafe_goal";
    case PlannerFailureClass::disconnected: return "disconnected";
    case PlannerFailureClass::timeout: return "timeout";
    case PlannerFailureClass::unavailable: return "unavailable";
    case PlannerFailureClass::other: return "other";
  }
  return "other";
}

std::vector<PlannerResult> plan_candidates(
    const PlanningGoal &goal, const PlanningSnapshot &snapshot) {
  if(goal.recovery_request!=RecoveryRequest::position) return plan_recovery_request(goal,snapshot);
  if(goal.mode==ExplorationMode::recovery) return {};
  return plan_map_candidates(goal,snapshot);
}

std::vector<PlannerResult> plan_map_candidates(
    const PlanningGoal &goal, const PlanningSnapshot &snapshot) {
  std::vector<PlannerResult> results;
  // Both independent planners receive the same observation position.
  const XY start{snapshot.base[0], snapshot.base[1]};
  if (goal.request_occupancy) {
    PlannerResult occupancy_result;
    occupancy_result.goal = goal;
    occupancy_result.kind = PlannerKind::occupancy;
    occupancy_result.source = "occupancy";
    occupancy_result.route = plan_occupancy_route(
        snapshot.grid, start, goal.observation, snapshot.settings,
        snapshot.scan_obstacles, snapshot.free_observations);
    occupancy_result.reason = occupancy_result.route.reason;
    if (occupancy_result.reason == "ok") {
      occupancy_result.waypoints.assign(
          occupancy_result.route.points.begin() + 1,
          occupancy_result.route.points.end());
    }
    occupancy_result.failure_class = classify_failure(occupancy_result.reason);
    results.push_back(std::move(occupancy_result));
  }

  if (goal.request_layer) {
    PlannerResult layer_result;
    layer_result.goal = goal;
    layer_result.kind = PlannerKind::layer;
    layer_result.source = "layer";
    if (!snapshot.layer_fresh || !snapshot.shared_layer || !snapshot.shared_layer->pct) {
      layer_result.reason="layer_map_unavailable";
      layer_result.failure_class=PlannerFailureClass::unavailable;
      results.push_back(std::move(layer_result));
      return results;
    }
    const auto started = std::chrono::steady_clock::now();
    struct { PlannedRoute reference; std::vector<XY> waypoints; } layer_plan;
    auto &shared = *snapshot.shared_layer;
    // Pause map integration while the planner holds the read lock.
    {
      if (shared.pct) {
        LayerMapReadPhase read_lock(shared);
        if(goal.layer_version!=shared.current_version.load()) {
          layer_plan.reference.reason="layer_map_version_changed";
          layer_plan.reference.failure_stage="map_version";
        } else {
        const Point target=goal.observation_xyz.value_or(Point{goal.observation[0],goal.observation[1],
                           goal.candidate.frontier[2]+snapshot.body_height});
        PctResult pct;
        const bool explicit_surface=goal.candidate.source==CandidateSource::layer_frontier;
        const auto version=shared.current_version.load();
        const double arrival_heading=goal.observation_yaw.value_or(
            std::atan2(target[1]-snapshot.base[1],target[0]-snapshot.base[0]));
        const auto arrival=explicit_surface?
            shared.pct->arrival_area(version,target,goal.xy_tolerance.value_or(0.),arrival_heading):
            PctObservation{};
        if(explicit_surface && !arrival.ready) {
          pct.reason=arrival.reason;
          pct.metadata="{\"arrival_area\":"+arrival.metadata+"}";
          pct.goal_safe=false;
          pct.unknown_blockers=arrival.unknown_blockers;
        } else {
          pct=shared.pct->plan(version,snapshot.base,target,explicit_surface,goal.pct_floor,
                               goal.collect_unknown_blockers || goal.mode==ExplorationMode::cross_layer,goal.stair_segment);
          if(explicit_surface) {
            std::string arrival_metadata="\"arrival_area\":"+arrival.metadata;
            if(pct.reason=="ok" && !pct.points.empty()) {
              const auto &end=pct.points.back();
              Point from=snapshot.base;
              for(auto i=pct.points.rbegin()+1;i!=pct.points.rend();++i) {
                if(std::hypot(end[0]-(*i)[0],end[1]-(*i)[1])>1e-6) {from=*i;break;}
              }
              const double approach=std::atan2(end[1]-from[1],end[0]-from[0]);
              if(std::abs(std::remainder(approach-arrival_heading,2*std::acos(-1.)))>.35) {
                const auto stop=shared.pct->arrival_area(version,end,goal.xy_tolerance.value_or(0.),approach);
                arrival_metadata+=",\"arrival_approach_area\":"+stop.metadata;
                if(!stop.ready) {
                  pct.reason=stop.reason;
                  pct.goal_safe=false;
                  pct.unknown_blockers=stop.unknown_blockers;
                  pct.points.clear();
                }
              }
            }
            pct.metadata="{"+arrival_metadata+(pct.metadata.size()>2?","+pct.metadata.substr(1):"}");
          }
        }
        layer_result.xyz=std::move(pct.points);
        layer_result.start_ground_z=pct.start_ground_z;
        layer_result.unknown_blockers=std::move(pct.unknown_blockers);
        layer_result.backend_metadata=std::move(pct.metadata);
        if(pct.start_safe) {
          auto &check=layer_plan.reference.start_check;
          check.ready=*pct.start_safe;
          check.stage=check.ready?"ready":"actual_position";
          check.actual.evaluated=true;check.actual.safe=check.ready;
          check.actual.position=snapshot.base;check.actual.checked_cells=1;
          check.actual.map_version=shared.current_version.load();
        }
        if(pct.goal_safe) {
          auto &check=layer_plan.reference.goal_check;
          check.evaluated=true;check.safe=*pct.goal_safe;
          check.position=target;check.checked_cells=1;
          check.map_version=shared.current_version.load();
        }
        layer_plan.reference.reason=pct.reason;
        layer_plan.reference.failure_stage=pct.reason=="ok"?"ready":"project_pct";
        for(const auto &point:layer_result.xyz) {
          layer_plan.reference.points.push_back({point[0],point[1]});
          layer_plan.waypoints.push_back({point[0],point[1]});
        }
        }
      } else {
        layer_plan.reference.reason="pct_backend_unavailable";
        layer_plan.reference.failure_stage="backend";
      }
    }
    layer_result.planning_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    shared.last_planning_ms.store(layer_result.planning_ms);
    layer_result.goal=goal;
    layer_result.route = std::move(layer_plan.reference);
    layer_result.waypoints = std::move(layer_plan.waypoints);
    layer_result.reason = layer_result.route.reason;
    layer_result.failure_class = classify_failure(layer_result.reason);
    results.push_back(std::move(layer_result));
  }
  for (auto &r:results) {
    r.goal.layer_version=goal.layer_version;
    if (r.kind==PlannerKind::occupancy) {
      r.route.start_check.actual.map_version=goal.occupancy_version;
      r.route.start_check.node.map_version=goal.occupancy_version;
      r.route.goal_check.map_version=goal.occupancy_version;
    }
  }
  return results;
}

StartChecks check_normal_starts(const PlanningSnapshot &snapshot,bool occupancy,bool layer,std::optional<XY> floor) {
  StartChecks out;
  const XY start{snapshot.base[0],snapshot.base[1]};
  if (occupancy) out.occupancy=check_start(start,OccupancyMapView{
      snapshot.grid,snapshot.settings,snapshot.scan_obstacles,snapshot.free_observations,snapshot.occupancy_version});
  if (layer && snapshot.layer_fresh && snapshot.shared_layer && snapshot.shared_layer->pct) {
    auto &shared=*snapshot.shared_layer;
    {
      if(shared.pct) {
        LayerMapReadPhase lock(shared);
        StartAdmission admission;
        const auto checked=shared.pct->point_check(shared.current_version.load(),snapshot.base,
                                                  false,std::nullopt,0.,false,floor);
        admission.ready=checked.ready;admission.reason=checked.reason;admission.physical_conflict=checked.physical_conflict;
        admission.stage=admission.ready?"ready":"project_pct";
        admission.actual.evaluated=true;
        admission.actual.safe=admission.ready;
        admission.actual.position=snapshot.base;
        admission.actual.map_version=shared.current_version.load();
        out.layer=admission;
      }
    }
  }
  return out;
}

bool StartChecks::blind() const {
  if(ready()) return false;
  const auto blind=[](const StartAdmission &c) {
    if(c.physical_conflict) return false;
    if(c.reason=="no_ground_sample" || c.reason=="unknown_footprint" || c.reason=="outside_map") return true;
    const auto &r=c.actual.rejected;
    return (r[static_cast<std::size_t>(SafetyIssue::unknown)] || r[static_cast<std::size_t>(SafetyIssue::zero_free_evidence)]) &&
        !r[static_cast<std::size_t>(SafetyIssue::occupied)] && !r[static_cast<std::size_t>(SafetyIssue::scan_obstacle)];
  };
  // Every available backend must report missing evidence, not a physical conflict.
  return (occupancy || layer) && (!occupancy || blind(*occupancy)) && (!layer || blind(*layer));
}

const PlannerResult *select_plan(
    const PlanningGoal &goal, const std::vector<PlannerResult> &results) {
  const PlannerResult *best = nullptr;
  for (const auto &result : results) {
    if (result.goal.goal_id != goal.goal_id ||
        result.goal.attempt_id != goal.attempt_id ||
        result.goal.occupancy_version != goal.occupancy_version ||
        result.goal.layer_version != goal.layer_version || result.goal.recovery_request!=goal.recovery_request || !result.usable())
      continue;
    if (goal.mode == ExplorationMode::cross_layer &&
        result.kind!=PlannerKind::layer && result.kind!=PlannerKind::observation_turn) continue;
    if (goal.mode == ExplorationMode::recovery) {
      if (result.kind==PlannerKind::observation_turn && goal.recovery_request==RecoveryRequest::turn) {best=&result;continue;}
      if (result.kind==PlannerKind::scan_direct && goal.recovery_request==RecoveryRequest::delegate) {best=&result;continue;}
      if (result.kind==PlannerKind::recovery_layer && goal.request_layer) best=&result;
      continue;
    }
    if(result.kind==PlannerKind::observation_turn && goal.recovery_request==RecoveryRequest::turn)best=&result;
    else if (result.kind == PlannerKind::layer && goal.request_layer)
      best = &result;
    else if (result.kind == PlannerKind::occupancy &&
             goal.request_occupancy && best == nullptr)
      best = &result;
  }
  return best;
}

bool PathProcessor::load(const PlanningGoal &goal, const PlannerResult &result,
                         bool recovery, const PlanningSnapshot *snapshot) {
  if (!result.usable() || result.goal.goal_id != goal.goal_id ||
      result.goal.attempt_id != goal.attempt_id ||
      result.goal.occupancy_version != goal.occupancy_version ||
      result.goal.layer_version != goal.layer_version || result.goal.recovery_request!=goal.recovery_request)
    return false;
  auto execution_waypoints = result.waypoints;
  if (result.kind == PlannerKind::occupancy && !recovery) {
    if (!snapshot) return false;
    execution_waypoints = stage_occupancy_route(
        result.route, snapshot->grid, snapshot->settings,
        snapshot->scan_obstacles,
        snapshot->free_observations);
    if (execution_waypoints.empty()) return false;
  }
  if (!result.xyz.empty() && result.xyz.size()!=execution_waypoints.size()) return false;
  xyz_ = result.xyz;
  waypoints_ = std::move(execution_waypoints);
  references_.clear();
  shift_start_.reset();
  if(goal.recovery_request==RecoveryRequest::shift) shift_start_=result.route.points.front();
  if(goal.stair_segment && result.kind==PlannerKind::layer && xyz_.size()>1) {
    const auto stop=xyz_.back();
    // SCAN accepts a target alone or a reference with at least two points.
    if(xyz_.size()>2) references_.emplace_back(xyz_.begin()+1,xyz_.end());
    xyz_={stop};
    waypoints_={{stop[0],stop[1]}};
  } else if(result.kind==PlannerKind::layer && !recovery && snapshot && xyz_.size()>1) {
    // Admission has already checked the starting cell. Avoid a stopped mission
    // to its own grid anchor, then preserve the complete geometry in each run.
    // Native A* returns its admitted start cell as the first vertex. SCAN
    // starts the reference at the measured pose instead of stopping at that
    // cell center. A single-point route still owns its final arrival below.
    const std::size_t first=1;
    std::vector<Point> endpoints,run;
    for(std::size_t i=first;i<xyz_.size();++i) {
      run.push_back(xyz_[i]);
      bool stop=i+1==xyz_.size();
      if(!stop) {
        const auto &previous=i?xyz_[i-1]:snapshot->base;
        const double ax=xyz_[i][0]-previous[0],ay=xyz_[i][1]-previous[1];
        const double bx=xyz_[i+1][0]-xyz_[i][0],by=xyz_[i+1][1]-xyz_[i][1];
        const double length=std::hypot(ax,ay)*std::hypot(bx,by);
        stop=(length>1e-8 && (ax*bx+ay*by)<std::cos(std::acos(-1.)/4.)*length) ||
             std::abs(xyz_[i][2]-previous[2])>.04 || std::abs(xyz_[i+1][2]-xyz_[i][2])>.04;
      }
      if(stop) {
        endpoints.push_back(xyz_[i]);
        references_.push_back(run.size()>1?std::move(run):std::vector<Point>{});
        run.clear();
      }
    }
    xyz_=std::move(endpoints);waypoints_.clear();
    for(const auto &p:xyz_)waypoints_.push_back({p[0],p[1]});
  }
  index_ = 0;
  source_ = result.source;
  request_=goal.recovery_request;yaw_=goal.recovery_yaw;turn_angle_=goal.recovery_turn_angle;
  if(result.kind==PlannerKind::observation_turn) {
    // The execution adapter has a bounded per-action watchdog. Serialize
    // large requested rotations as <=1-radian actions, without changing the
    // controller's final heading/sweep or duplicating velocity control here.
    const auto stages=static_cast<std::size_t>(std::max(1.,std::ceil(std::abs(turn_angle_))));
    waypoints_.assign(stages,goal.observation);
    turn_angle_/=stages;
  }
  recovery_ = recovery;
  final_xy_tolerance_=goal.xy_tolerance;
  preserve_geometry_=goal.mode==ExplorationMode::recovery || result.kind==PlannerKind::layer || result.kind==PlannerKind::recovery_layer || result.kind==PlannerKind::scan_direct || result.kind==PlannerKind::observation_turn;
  goal_id_ = goal.goal_id;
  attempt_id_ = goal.attempt_id;
  begin_execution();
  return true;
}

void PathProcessor::clear() {
  waypoints_.clear();
  xyz_.clear();
  references_.clear();
  shift_start_.reset();
  index_ = 0;
  source_ = "none";
  final_xy_tolerance_.reset();
  request_=RecoveryRequest::position;yaw_=turn_angle_=0.0;
  recovery_ = false;
  preserve_geometry_=false;
  goal_id_ = 0;
  attempt_id_ = 0;
  begin_execution();
}

bool PathProcessor::advance() {
  if (!active()) return false;
  ++index_;
  begin_execution();
  return active();
}

void PathProcessor::begin_execution() {
  scan_goal_stamp_.reset();
  execution_resolved_ = false;
}

void PathProcessor::bind_scan_goal(std::int64_t stamp) {
  if (active() && stamp > 0 && !execution_resolved_)
    scan_goal_stamp_ = stamp;
}

std::optional<ExecutionResult> PathProcessor::resolve_execution(
    std::int64_t stamp, bool success, const std::string &reason, std::optional<Point> actual_position) {
  if (!active() || !scan_goal_stamp_ || *scan_goal_stamp_ != stamp ||
      execution_resolved_)
    return std::nullopt;
  execution_resolved_ = true;
  ExecutionResult result{goal_id_,attempt_id_,index_,stamp,source_,reason,success};
  if(is_last() && (request_==RecoveryRequest::position || request_==RecoveryRequest::delegate))
    result.requested_xy_tolerance=final_xy_tolerance_;
  result.actual_position=actual_position;
  if(actual_position)result.goal_distance=std::hypot((*actual_position)[0]-current()[0],(*actual_position)[1]-current()[1]);
  return result;
}

std::optional<ExecutionDispatch> PathProcessor::dispatch(std::int64_t stamp,const Point &base) const {
  if(!active() || execution_resolved_ || stamp<=0) return std::nullopt;
  ExecutionDispatch out;out.request=request_;out.target=current();out.z=current_z();
  if(index_<references_.size())out.reference_points=references_[index_];
  if(is_last() && (request_==RecoveryRequest::position || request_==RecoveryRequest::delegate))
    out.xy_tolerance=final_xy_tolerance_;
  std::ostringstream payload;payload<<std::setprecision(17)<<stamp;
  if(request_==RecoveryRequest::turn) payload<<'|'<<turn_angle_;
  else if(request_==RecoveryRequest::reverse) payload<<'|'<<base[0]<<'|'<<base[1]<<'|'<<current()[0]<<'|'<<current()[1]<<'|'<<yaw_;
  else if(request_==RecoveryRequest::shift && shift_start_)
    payload<<'|'<<(*shift_start_)[0]<<'|'<<(*shift_start_)[1]<<'|'<<current()[0]<<'|'<<current()[1]<<'|'<<yaw_<<"|shift";
  out.action_payload=payload.str();return out;
}

std::size_t PathProcessor::skip_within(const XY &position, double radius) {
  if (preserve_geometry_ || !std::isfinite(radius) || radius < 0.0) return 0;
  std::size_t skipped = 0;
  while (active() &&
         std::hypot(current()[0] - position[0], current()[1] - position[1]) <= radius) {
    ++skipped;
    advance();
  }
  return skipped;
}

}  // namespace legbot_exploration
