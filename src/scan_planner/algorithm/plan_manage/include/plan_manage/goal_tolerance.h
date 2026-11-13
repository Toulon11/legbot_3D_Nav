#pragma once

#include <scan_planner/msg/goal_request.hpp>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace scan_planner {
inline bool validGoalTolerance(const msg::GoalRequest &request) {
  return !request.has_xy_tolerance ||
      (std::isfinite(request.xy_tolerance) &&
       request.xy_tolerance >= msg::GoalRequest::MIN_XY_TOLERANCE &&
       request.xy_tolerance <= msg::GoalRequest::MAX_XY_TOLERANCE);
}

inline bool validGoalReference(const msg::GoalRequest &request) {
  const auto &points=request.reference_points;
  if(points.empty())return true;
  if(points.size()<2 || points.size()>10000)return false;
  for(const auto &p:points)
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return false;
  const auto &last=points.back(), &target=request.target.pose.position;
  return std::hypot(last.x-target.x,last.y-target.y)<1e-6 &&
      std::abs(last.z-target.z)<1e-6;
}

// Diagnostics carry the command's own identity; no separate options topic is
// consulted to execute a goal. Invalid numeric values are never emitted as JSON.
inline std::string goalToleranceStatus(const msg::GoalRequest &request,
    const char *source, const char *event, double effective,
    double x=0, double y=0, double z=0, double distance=-1) {
  const auto &stamp=request.target.header.stamp;
  const auto ns=static_cast<std::int64_t>(stamp.sec)*1000000000LL+stamp.nanosec;
  std::ostringstream out;out<<std::setprecision(17);
  out<<"{\"source\":\""<<source<<"\",\"event\":\""<<event
     <<"\",\"stamp_ns\":"<<ns<<",\"goal_id\":"<<request.goal_id
     <<",\"attempt_id\":"<<request.attempt_id
     <<",\"waypoint_index\":"<<request.waypoint_index
     <<",\"has_xy_tolerance\":"<<(request.has_xy_tolerance?"true":"false")
     <<",\"requested_xy_tolerance\":";
  if(request.has_xy_tolerance && std::isfinite(request.xy_tolerance))out<<request.xy_tolerance;
  else out<<"null";
  out<<",\"effective_xy_tolerance\":";
  if(std::isfinite(effective)&&effective>0)out<<effective;else out<<"null";
  if(distance>=0 && std::isfinite(distance))
    out<<",\"actual_position\":["<<x<<','<<y<<','<<z<<"],\"goal_distance\":"<<distance;
  out<<'}';return out.str();
}
} // namespace scan_planner
