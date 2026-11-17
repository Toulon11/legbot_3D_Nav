#include "legbot_exploration/route_orchestrator.hpp"
#include "legbot_exploration/recovery_planner.hpp"
#include <algorithm>
#include <cmath>
#include <mutex>
#include <shared_mutex>

namespace legbot_exploration {
const char *recovery_request_name(RecoveryRequest request) {
  switch(request) {
    case RecoveryRequest::position:return "position";
    case RecoveryRequest::reverse:return "reverse";
    case RecoveryRequest::shift:return "shift";
    case RecoveryRequest::turn:return "turn";
    case RecoveryRequest::delegate:return "delegate";
  }
  return "invalid";
}
namespace {
// The map owner drops observations while a planning reader holds this guard.
using LayerRead=LayerMapReadPhase;
bool layer_available(const PlanningGoal &g,const PlanningSnapshot &s) {
  return s.layer_fresh && s.shared_layer && s.shared_layer->pct &&
      g.layer_version==s.shared_layer->current_version.load();
}

bool short_action_clear_of_scan_obstacles(const PlanningSnapshot &s,const XY &target,double stop_lead) {
  const double radius=s.settings.robot_radius+s.settings.waypoint_safety_margin;
  const double half_cell=s.settings.planner_occupancy_resolution*.5;
  const double dx=target[0]-s.base[0],dy=target[1]-s.base[1];
  const double length=std::hypot(dx,dy);
  const int samples=static_cast<int>(std::ceil(length/.04));
  for(const auto &obstacle:*s.scan_raw_obstacles) {
    const auto clearance=[&](double t) {
      return std::hypot(std::max(0.,std::abs(s.base[0]+t*dx-obstacle[0])-half_cell),
                        std::max(0.,std::abs(s.base[1]+t*dy-obstacle[1])-half_cell));
    };
    const double initial=clearance(0.);
    if(initial>radius+length) continue;
    double previous=initial;
    for(int i=1;i<=samples;++i) {
      const double d=clearance(static_cast<double>(i)/samples);
      if(d+1e-6<std::min(radius,previous)) return false;
      previous=d;
    }
    if(clearance(1.-stop_lead/length)+1e-6<radius) return false;
  }
  return true;
}

std::array<long,5> blocker_stance_key(const Point &gap,const Point &stance) {
  return {std::lround(gap[0]*100.),std::lround(gap[1]*100.),std::lround(gap[2]*20.),
          std::lround(stance[0]*10.),std::lround(stance[1]*10.)};
}

std::array<long,6> blocker_scan_key(const PctObservationPose &view) {
  const auto key=blocker_stance_key(view.gap,view.stance);
  return {key[0],key[1],key[2],key[3],key[4],std::lround(std::remainder(view.yaw,2*M_PI)*10.)};
}

bool still_unknown(PctBackend &map,std::uint64_t version,const Point &p,double body_height) {
  return map.observation_task(version,"blocker_pending",0,
      {p[0],p[1],p[2]+body_height}).ready;
}

bool verified_pct_short_move(const PlanningSnapshot &s,std::uint64_t version,const Point &target,bool reverse) {
  if(!s.scan_raw_obstacles ||
     !short_action_clear_of_scan_obstacles(s,{target[0],target[1]},reverse?.04:.07))return false;
  auto &map=*s.shared_layer->pct;
  return (reverse?map.plan_stair_reverse(version,s.base,target):map.plan_shift(version,s.base,target)).reason=="ok";
}
}

std::optional<double> verified_short_recovery_stop(
    const Point &start,const XY &target,const Point &stop,double yaw_error,bool partial_reverse) {
  const double vx=target[0]-start[0],vy=target[1]-start[1];
  const double length=std::hypot(vx,vy);
  if(length<1e-6)return std::nullopt;
  const double dx=stop[0]-start[0],dy=stop[1]-start[1];
  const double along=(dx*vx+dy*vy)/length,across=(dy*vx-dx*vy)/length;
  if(along>length+.05 || along<-.03 || std::abs(across)>.06 ||
     std::abs(std::remainder(yaw_error,2*M_PI))>.30 ||
     (!partial_reverse && std::hypot(along-length,across)>.08))return std::nullopt;
  return along;
}

std::vector<XY> stair_recovery_repositions(
    PctBackend &map,std::uint64_t version,const Point &base,const Point &forward,double yaw,double ascent,
    double tread_height,double max_step,bool reverse,
    const std::function<bool(const Point &,double)> &scan_inflated) {
  const double c=std::cos(yaw),s=std::sin(yaw);
  const auto inflated=[&](const Point &p) {
    return scan_inflated(p,std::atan2(forward[1]-p[1],forward[0]-p[0]));
  };
  const XY height_band{tread_height,std::min(.16,max_step)};
  std::vector<XY> out;
  if(reverse) {
    for(double length:{.25,.30,.35,.40}) {
      const XY target{base[0]-length*c,base[1]-length*s};
      Point stop{target[0],target[1],base[2]};
      const auto landing=map.point_check(version,stop,false,std::nullopt,0.,false,height_band);
      if(!landing.ready || !landing.ground_z)continue;
      stop[2]=base[2]+*landing.ground_z-tread_height;
      if(!inflated({target[0]+.06*c,target[1]+.06*s,stop[2]}) &&
         !map.observation_poses(version,base,Point{0.,0.,yaw},
             {Point{target[0],target[1],*landing.ground_z}},std::nullopt,std::nullopt,
             height_band,stop,.15,0).empty())out.push_back(target);
    }
  } else {
    for(double offset:{.22,-.22}) {
      Point target{base[0]-offset*std::sin(ascent),base[1]+offset*std::cos(ascent),base[2]};
      const auto landing=map.point_check(version,target,false,std::nullopt,0.,false,height_band);
      if(!landing.ready || !landing.ground_z)continue;
      target[2]+=*landing.ground_z-tread_height;
      if(inflated(target))continue;
      const auto support=[&](double distance) {
        const Point p{target[0]+distance*c,target[1]+distance*s,target[2]};
        return map.point_check(version,p,false,std::nullopt,0.,false,height_band).ready;
      };
      if(support(-.20) && support(.20) &&
         !map.observation_poses(version,base,Point{0.,0.,yaw},
             {Point{target[0],target[1],*landing.ground_z}},std::nullopt,std::nullopt,
             height_band,target,.15,0).empty())out.push_back({target[0],target[1]});
    }
  }
  return out;
}

std::optional<PctRecoveryAction> plan_pct_landing_shift(
    const PlanningSnapshot &s,std::uint64_t version,const XY &departure_floor,double tread_height,double ascent) {
  if(!s.layer_fresh || !s.shared_layer || !s.shared_layer->pct || !s.scan_raw_obstacles)return std::nullopt;
  LayerRead read(*s.shared_layer);
  auto &map=*s.shared_layer->pct;
  auto base=s.base;
  base[2]=tread_height+s.body_height;
  const auto platform=map.point_check(version,base,false,departure_floor,ascent,true);
  if(!platform.ready)return std::nullopt;
  const double step_limit=std::min(.16,s.shared_layer->max_step_height);
  for(double distance:{.35,.50,.60,.22}) {
    for(int i=0;i<8;++i) {
      const double angle=ascent+i*M_PI/4.;
      const Point stop{base[0]+distance*std::cos(angle),base[1]+distance*std::sin(angle),base[2]};
      const auto landing=map.point_check(version,stop,false,departure_floor,ascent);
      if(landing.ready && landing.ground_z && std::abs(*landing.ground_z-tread_height)<=step_limit+1e-7 &&
         !map.observation_poses(version,s.base,Point{0.,0.,angle},
             {Point{stop[0],stop[1],tread_height}},std::nullopt,std::nullopt,
             XY{tread_height,step_limit},stop,.15,0).empty() &&
         verified_pct_short_move(s,version,{stop[0],stop[1],s.base[2]},false))
        return PctRecoveryAction{PctRecoveryAction::Kind::shift,stop};
    }
  }
  return std::nullopt;
}

std::optional<double> PctBlockerRecovery::scan_heading(
    PctBackend &map,std::uint64_t version,const Point &position,const Point &attitude,
    const Point &gap,const std::vector<PctUnknownBlocker> &blockers,double body_height,
    const PctRecoveryConstraints &constraints,bool review) {
  if(constraints.permits_scan && !constraints.permits_scan(gap))return std::nullopt;
  if(!map.observation_task(version,"blocker_available",0,
      {gap[0],gap[1],gap[2]+body_height}).ready)return std::nullopt;
  if(!review && !still_unknown(map,version,gap,body_height))return std::nullopt;
  std::vector<Point> gaps{gap};
  for(const auto &blocker:blockers) {
    if(gaps.size()>=12)break;
    if(std::find(gaps.begin(),gaps.end(),blocker.point)==gaps.end())gaps.push_back(blocker.point);
  }
  const auto views=map.observation_poses(
      version,position,attitude,gaps,false,constraints.ascent,constraints.height_band,std::nullopt,.15,1,true);
  const auto selected=std::find_if(views.begin(),views.end(),[&](const auto &view){return view.gap==gap;});
  if(selected==views.end() || scans_.count(blocker_scan_key(*selected)))return std::nullopt;
  for(const auto &view:views)
    if(std::abs(std::remainder(selected->yaw-view.yaw,2*M_PI))<=.06)
      scans_.insert(blocker_scan_key(view));
  return selected->yaw;
}

std::optional<PctRecoveryAction> PctBlockerRecovery::next(
    const PlanningSnapshot &s,std::uint64_t version,const Point &attitude,
    const std::vector<PctUnknownBlocker> &blockers,const PctRecoveryConstraints &constraints,
    bool can_move,
    const std::function<bool(const Point &)> &stance_clear,const std::vector<Point> &review,double arrival_tolerance) {
  rejections_.clear();
  pending_=false;
  if(!s.layer_fresh || !s.shared_layer || !s.shared_layer->pct ||
     version!=s.shared_layer->current_version.load()) {
    ++rejections_["map_unavailable_or_changed"];return std::nullopt;
  }
  LayerRead read(*s.shared_layer);
  auto &map=*s.shared_layer->pct;
  std::vector<Point> gaps;
  const double radius=s.settings.robot_radius+s.settings.waypoint_safety_margin;
  for(const auto &blocker:blockers) {
    const auto &p=blocker.point;
    if(!still_unknown(map,version,p,s.body_height))continue;
    if(constraints.ascent) {
      const double dx=p[0]-s.base[0],dy=p[1]-s.base[1];
      const double c=std::cos(*constraints.ascent),d=std::sin(*constraints.ascent);
      const double along=dx*c+dy*d,across=-dx*d+dy*c;
      if(along<-radius || along>4.*radius || std::abs(across)>2.*radius)continue;
    }
    if(std::find(gaps.begin(),gaps.end(),p)==gaps.end())gaps.push_back(p);
  }
  std::sort(gaps.begin(),gaps.end(),[&](const auto &a,const auto &b) {
    return std::hypot(a[0]-s.base[0],a[1]-s.base[1])<std::hypot(b[0]-s.base[0],b[1]-s.base[1]);
  });
  if(gaps.size()>12)gaps.resize(12);
  for(const auto &point:review)
    if(std::find(gaps.begin(),gaps.end(),point)==gaps.end() &&
       map.observation_task(version,"blocker_available",0,
           {point[0],point[1],point[2]+s.body_height}).ready)gaps.push_back(point);
  const auto can_scan=[&](const Point &point) {
    return !constraints.permits_scan || constraints.permits_scan(point);
  };
  gaps.erase(std::remove_if(gaps.begin(),gaps.end(),[&](const auto &point) {
    return !can_scan(point);
  }),gaps.end());
  if(!gaps.empty() && scans_.size()<24) {
    const auto standing=map.observation_poses(
        version,s.base,attitude,gaps,false,constraints.ascent,constraints.height_band,std::nullopt,.15,1,true);
    if(standing.empty())++rejections_["no_safe_stationary_view"];
    for(const auto &view:standing) {
      if(scans_.count(blocker_scan_key(view))) {++rejections_["stationary_view_already_attempted"];continue;}
      for(const auto &other:standing)
        if(std::abs(std::remainder(view.yaw-other.yaw,2*M_PI))<=.06)
          scans_.insert(blocker_scan_key(other));
      return PctRecoveryAction{PctRecoveryAction::Kind::scan,s.base,view.gap,view.yaw};
    }
    if(can_move && moves_.size()<6) {
      const auto views=map.observation_poses(
          version,s.base,attitude,gaps,true,constraints.ascent,constraints.height_band);
      if(views.empty())++rejections_["no_safe_short_move_view"];
      for(const auto &view:views) {
        if(view.motion==PctObservationPose::Motion::stationary || scans_.count(blocker_scan_key(view)))continue;
        if(!stance_clear(view.stance)) {++rejections_["short_move_scan_inflated"];continue;}
        const auto key=blocker_stance_key(view.gap,view.stance);
        if(moves_.count(key))continue;
        const XY stop{view.stance[0],view.stance[1]};
        const bool reverse=view.motion==PctObservationPose::Motion::reverse;
        if(constraints.ascent && !reverse) {
          const double dx=stop[0]-s.base[0],dy=stop[1]-s.base[1];
          const int samples=static_cast<int>(std::ceil(std::hypot(dx,dy)/.04));
          bool supported=true;
          for(int i=0;i<=samples && supported;++i) {
            const double t=static_cast<double>(i)/samples;
            for(const auto &contact:constraints.contacts) {
              const Point foot{contact[0]+t*dx,contact[1]+t*dy,
                               contact[2]+s.body_height+t*(view.stance[2]-s.base[2])};
              if(!map.point_check(version,foot,false,std::nullopt,0.,false,
                                  XY{contact[2],std::min(.16,s.shared_layer->max_step_height)},true).ready) {
                supported=false;
                break;
              }
            }
          }
          if(!supported) {++rejections_["shift_contact_support_unverified"];continue;}
        }
        const Point target=view.stance;
        if(!verified_pct_short_move(s,version,target,reverse)) {
          ++rejections_[reverse?"short_reverse_rejected":"short_shift_rejected"];continue;
        }
        moves_.insert(key);
        return PctRecoveryAction{reverse?PctRecoveryAction::Kind::reverse:PctRecoveryAction::Kind::shift,
                                 target,view.gap,attitude[2]};
      }
    }
  }
  // A near-filtered gap can remain invisible from every short-action stance.
  // Reuse the public pose search around that gap, then verify its native route.
  // This recovery may return one step down; ascent progress filters apply to
  // exploration goals, not to a verified observation reposition.
  if(can_move && !gaps.empty()) {
    // Native routes cannot start inside an unobserved footprint. Preserve
    // the existing SCAN recovery handoff instead of exhausting its endpoints
    // while trying the same invalid PCT start against different destinations.
    if(!map.start_ready(version,s.base,constraints.height_band)) {
      ++rejections_["targeted_route_start_unverified"];return std::nullopt;
    }
    const std::vector<Point> targets(gaps.begin(),gaps.begin()+std::min<std::size_t>(3,gaps.size()));
    const auto views=map.observation_poses(version,s.base,attitude,targets,std::nullopt,
                                         constraints.ascent,constraints.height_band,std::nullopt,arrival_tolerance);
    if(views.empty())++rejections_["no_safe_targeted_observer"];
    for(const auto &view:views) {
      if(std::hypot(view.stance[0]-s.base[0],view.stance[1]-s.base[1])<=arrival_tolerance ||
         scans_.count(blocker_scan_key(view)))continue;
      if(!stance_clear(view.stance)) {++rejections_["targeted_observer_scan_inflated"];continue;}
      if(std::any_of(observer_routes_.begin(),observer_routes_.end(),[&](const Point &old) {
        return std::hypot(old[0]-view.stance[0],old[1]-view.stance[1])<=.20 &&
               std::abs(old[2]-view.stance[2])<=.05;
      }))continue;
      observer_routes_.push_back(view.stance);
      const auto route=map.plan(version,s.base,view.stance,false,constraints.height_band);
      if(route.reason!="ok") {
        ++rejections_["observer_route:"+route.reason];
        pending_=true;return std::nullopt; // Try another stance next cycle, release the map lock now.
      }
      delegated_.push_back(view.stance);
      return PctRecoveryAction{PctRecoveryAction::Kind::delegate,view.stance,view.gap,view.yaw,true};
    }
  } else if(!can_move)++rejections_["motion_not_ready"];
  if(gaps.empty())++rejections_["no_pending_observation_target"];
  return std::nullopt;
}

std::optional<PctRecoveryAction> PctBlockerRecovery::delegate(
    const PlanningSnapshot &s,std::uint64_t version,const PctRecoveryConstraints &constraints,
    const std::vector<Point> &goals,double arrival_tolerance) {
  if(!s.layer_fresh || !s.shared_layer || !s.shared_layer->pct)return std::nullopt;
  LayerRead read(*s.shared_layer);
  if(version!=s.shared_layer->current_version.load())return std::nullopt;
  auto &map=*s.shared_layer->pct;
  const double separation=s.settings.robot_radius+s.settings.waypoint_safety_margin;
  const auto bearing=[&](const Point &p) { return std::atan2(p[1]-s.base[1],p[0]-s.base[0]); };
  const auto different_direction=[&](const Point &p) {
    return delegated_.empty() ||
        std::abs(std::remainder(bearing(p)-bearing(delegated_.back()),2*M_PI))>=M_PI/4.;
  };
  for(const bool prefer_different_direction:{true,false}) {
    for(const auto &goal:goals) {
      if(different_direction(goal)!=prefer_different_direction ||
         std::hypot(goal[0]-s.base[0],goal[1]-s.base[1])<=arrival_tolerance ||
         (constraints.permits_goal && !constraints.permits_goal(goal)))continue;
      if(std::any_of(delegated_.begin(),delegated_.end(),[&](const Point &previous) {
        return std::hypot(goal[0]-previous[0],goal[1]-previous[1])<=separation &&
               std::abs(goal[2]-previous[2])<=.08;
      }))continue;
      const auto checked=map.point_check(version,goal,false,std::nullopt,0.,false,constraints.height_band);
      if(checked.ready && checked.ground_z && std::abs(*checked.ground_z+s.body_height-goal[2])<=.02 &&
         !map.observation_poses(version,s.base,Point{0.,0.,bearing(goal)},
             {Point{goal[0],goal[1],*checked.ground_z}},std::nullopt,constraints.ascent,
             constraints.height_band,goal,arrival_tolerance,0).empty()) {
        delegated_.push_back(goal);
        return PctRecoveryAction{PctRecoveryAction::Kind::delegate,goal};
      }
    }
  }
  return std::nullopt;
}

std::vector<PlannerResult> plan_recovery_request(const PlanningGoal &goal,const PlanningSnapshot &s) {
  if(goal.mode!=ExplorationMode::recovery && goal.recovery_request!=RecoveryRequest::turn) return {};
  const bool finite=std::isfinite(goal.recovery_yaw)&&std::isfinite(goal.recovery_turn_angle)&&
      std::isfinite(goal.observation[0])&&std::isfinite(goal.observation[1]);
  std::vector<PlannerResult> out;
  auto base=[&](bool layer) {
    PlannerResult r;r.goal=goal;r.kind=layer?PlannerKind::recovery_layer:PlannerKind::observation_turn;
    r.source=goal.recovery_request==RecoveryRequest::shift?"recovery_shift":
        goal.recovery_request==RecoveryRequest::reverse?"recovery_reverse":"recovery_turn";
    r.reason="invalid_recovery_request";r.failure_class=PlannerFailureClass::other;return r;
  };
  if(goal.recovery_request==RecoveryRequest::shift) {
    auto r=base(true);
    const double length=std::hypot(goal.observation[0]-s.base[0],goal.observation[1]-s.base[1]);
    if(!finite || !std::isfinite(length) || !goal.request_layer || length<.20-1e-6 || length>.60+1e-6) return {r};
    if(!layer_available(goal,s) || !s.scan_raw_obstacles) {
      r.reason="shift_map_unavailable";r.failure_class=PlannerFailureClass::unavailable;return {r};
    }
    LayerRead read(*s.shared_layer);
    if(!layer_available(goal,s)) {
      r.reason="layer_map_version_changed";r.failure_class=PlannerFailureClass::unavailable;return {r};
    }
    if(!short_action_clear_of_scan_obstacles(s,goal.observation,.07)) {
      r.reason="shift_scan_occupied_corridor";return {r};
    }
    const Point target=goal.observation_xyz.value_or(
        Point{goal.observation[0],goal.observation[1],s.base[2]});
    auto checked=s.shared_layer->pct->plan_shift(goal.layer_version,s.base,target);
    r.reason=checked.reason;r.backend_metadata=std::move(checked.metadata);
    if(r.reason=="ok") {
      for(const auto &p:checked.points)r.route.points.push_back({p[0],p[1]});
      r.waypoints={goal.observation};r.failure_class=PlannerFailureClass::none;
    }
    return {r};
  }
  if(goal.recovery_request==RecoveryRequest::delegate) {
    auto r=base(false);r.kind=PlannerKind::scan_direct;r.source="recovery_scan_direct";
    if(!finite || !goal.request_layer || !goal.observation_xyz ||
       !std::isfinite((*goal.observation_xyz)[2]) ||
       std::hypot((*goal.observation_xyz)[0]-goal.observation[0],
                  (*goal.observation_xyz)[1]-goal.observation[1])>1e-6) return {r};
    if(!layer_available(goal,s)) {
      r.reason="layer_map_unavailable_or_changed";
      r.failure_class=PlannerFailureClass::unavailable;
      return {r};
    }
    LayerRead read(*s.shared_layer);
    if(goal.layer_version!=s.shared_layer->current_version.load()) {
      r.reason="layer_map_version_changed";
      r.failure_class=PlannerFailureClass::unavailable;
      return {r};
    }
    const auto checked=s.shared_layer->pct->point_check(goal.layer_version,*goal.observation_xyz,
        false,std::nullopt,0.,false,goal.pct_floor);
    const auto arrival=s.shared_layer->pct->arrival_area(goal.layer_version,*goal.observation_xyz,
        goal.xy_tolerance.value_or(0.),std::atan2(goal.observation[1]-s.base[1],goal.observation[0]-s.base[0]));
    r.backend_metadata="{\"path_validated\":false,\"execution_owner\":\"scan\",\"goal_check\":"+
        checked.metadata+",\"arrival_area\":"+arrival.metadata+"}";
    if(checked.ready && arrival.ready) {
      r.waypoints={goal.observation};r.xyz={*goal.observation_xyz};r.reason="delegated_to_scan";
      r.failure_class=PlannerFailureClass::none;
    } else {r.reason=checked.ready?arrival.reason:checked.reason;r.failure_class=PlannerFailureClass::unsafe_goal;}
    out.push_back(std::move(r));return out;
  }
  for(bool layer:{false,true}) {
    if(layer?!goal.request_layer:(!goal.request_occupancy || goal.recovery_request!=RecoveryRequest::turn)) continue;
    auto r=base(layer);
    if(!finite || (goal.recovery_request==RecoveryRequest::turn &&
          (std::abs(goal.recovery_turn_angle)<1e-6 || std::abs(goal.recovery_turn_angle)>2*std::acos(-1.0)))) {out.push_back(r);continue;}
    if(goal.recovery_request==RecoveryRequest::reverse) {
      const XY start{s.base[0],s.base[1]};
      const double length=std::hypot(goal.observation[0]-start[0],goal.observation[1]-start[1]);
      const double along=(goal.observation[0]-start[0])*std::cos(goal.recovery_yaw)+(goal.observation[1]-start[1])*std::sin(goal.recovery_yaw);
      const double lateral=-(goal.observation[0]-start[0])*std::sin(goal.recovery_yaw)+(goal.observation[1]-start[1])*std::cos(goal.recovery_yaw);
      if(length<.20-1e-6 || length>.60+1e-6 || along>=0 || std::abs(lateral)>.01) {out.push_back(r);continue;}
    }
    bool safe=false;
    if(layer) {
      if(!layer_available(goal,s)) {r.reason="layer_map_unavailable_or_changed";r.failure_class=PlannerFailureClass::unavailable;out.push_back(r);continue;}
      LayerRead read(*s.shared_layer);
      if(goal.layer_version!=s.shared_layer->current_version.load()) {r.reason="layer_map_version_changed";out.push_back(r);continue;}
      if(goal.recovery_request==RecoveryRequest::reverse) {
        const Point target=goal.observation_xyz.value_or(
        Point{goal.observation[0],goal.observation[1],s.base[2]});
        if(!s.scan_raw_obstacles || !short_action_clear_of_scan_obstacles(s,goal.observation,.04)) {
          r.reason="stair_reverse_scan_corridor_unverified";out.push_back(r);continue;
        }
        auto native=s.shared_layer->pct->plan_stair_reverse(goal.layer_version,s.base,target);
        r.reason=native.reason;r.backend_metadata=native.metadata;
        for(const auto &p:native.points) r.route.points.push_back({p[0],p[1]});
        safe=r.reason=="ok";
      } else {
        // Circular footprint bounds all intermediate body headings.
        const auto check=s.shared_layer->pct->point_check(goal.layer_version,s.base);
        safe=check.ready;r.reason=check.reason;r.backend_metadata=check.metadata;
      }
    } else {
      if(goal.occupancy_version!=s.occupancy_version) {r.reason="occupancy_map_version_changed";r.failure_class=PlannerFailureClass::unavailable;out.push_back(r);continue;}
      r.route.goal_check=point_safe_check({s.base[0],s.base[1]},OccupancyMapView{s.grid,s.settings,s.scan_obstacles,s.free_observations,s.occupancy_version});
      safe=r.route.goal_check.safe;r.reason=r.route.goal_check.reason();
    }
    if(safe) {
      r.waypoints={goal.observation};r.reason="ok";r.failure_class=PlannerFailureClass::none;
      if(goal.recovery_request==RecoveryRequest::turn) {
        r.kind=PlannerKind::observation_turn;
        r.source="observation_turn";
      }
    }
    out.push_back(std::move(r));
  }
  return out;
}
} // namespace legbot_exploration
