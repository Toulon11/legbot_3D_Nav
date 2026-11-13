#include "legbot_exploration/point_safety.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace legbot_exploration {
namespace {
Cell cell_of(const XY &p, double r) {
  return {static_cast<int>(std::floor(p[0] / r)),
          static_cast<int>(std::floor(p[1] / r))};
}
XY center(Cell c, double r) { return {(c.x + 0.5) * r, (c.y + 0.5) * r}; }
double distance(const XY &a, const XY &b) {
  return std::hypot(a[0] - b[0], a[1] - b[1]);
}
double cell_distance(const XY &p, Cell c, double r) {
  return std::hypot(std::max({c.x*r-p[0], 0.0, p[0]-(c.x+1)*r}),
                    std::max({c.y*r-p[1], 0.0, p[1]-(c.y+1)*r}));
}
template<class Visit> void disk_cells(const XY &p, double radius, double r, Visit visit) {
  const int x0 = static_cast<int>(std::floor((p[0]-radius)/r));
  const int x1 = static_cast<int>(std::floor((p[0]+radius)/r));
  const int y0 = static_cast<int>(std::floor((p[1]-radius)/r));
  const int y1 = static_cast<int>(std::floor((p[1]+radius)/r));
  for (int x=x0; x<=x1; ++x) for (int y=y0; y<=y1; ++y)
    if (cell_distance(p, {x,y}, r) <= radius+1e-9) visit(Cell{x,y});
}
template<class View> StartAdmission admit(const XY &p, const View &map, double resolution) {
  StartAdmission out;
  out.actual=point_safe_check(p,map);
  out.stage="actual_position";
  if (!out.actual.safe) return out;
  out.anchor=center(out.actual.cell,resolution);
  out.node=point_safe_check(out.anchor,map);
  out.stage="start_node";
  if (!out.node.safe) return out;
  out.connection=segment_safe_check(p,out.anchor,map);
  out.stage="start_connection";
  if (!out.connection.safe) return out;
  out.ready=true; out.stage="ready";
  return out;
}
}

const char *safety_issue_name(SafetyIssue issue) {
  static constexpr const char *names[]={"invalid_input","unknown","occupied",
    "scan_obstacle","zero_free_evidence","no_surface","height_mismatch",
    "insufficient_support","insufficient_hits","non_flat_surface",
    "clearance_unobserved","clearance_blocked","disconnected_support"};
  const auto i=static_cast<std::size_t>(issue);
  return i<static_cast<std::size_t>(SafetyIssue::count)?names[i]:"invalid_input";
}
std::string PointSafetyResult::reason() const {
  for (std::size_t i=0;i<rejected.size();++i)
    if (rejected[i]) return safety_issue_name(static_cast<SafetyIssue>(i));
  return "ok";
}
std::string safety_json(const PointSafetyResult &r) {
  std::ostringstream out;
  out<<"{\"evaluated\":"<<(r.evaluated?"true":"false")<<",\"safe\":"<<(r.evaluated?(r.safe?"true":"false"):"null")<<",\"map_version\":"<<r.map_version
     <<",\"position\":["<<(std::isfinite(r.position[0])?r.position[0]:0.0)<<','
     <<(std::isfinite(r.position[1])?r.position[1]:0.0)<<','
     <<(std::isfinite(r.position[2])?r.position[2]:0.0)<<']'
     <<",\"cell\":["<<r.cell.x<<','<<r.cell.y<<']'
     <<",\"checked_cells\":"<<r.checked_cells<<",\"rejected\":{";
  bool first=true;
  for (std::size_t i=0;i<r.rejected.size();++i) if (r.rejected[i]) {
    if (!first) out<<',';
    first=false; out<<'"'<<safety_issue_name(static_cast<SafetyIssue>(i))<<"\":"<<r.rejected[i];
  }
  out<<"}}"; return out.str();
}
std::string admission_json(const StartAdmission &r) {
  std::ostringstream out;
  out<<"{\"ready\":"<<(r.ready?"true":"false")<<",\"stage\":\""<<r.stage
     <<"\",\"reason\":\""<<r.reason<<"\",\"physical_conflict\":"<<(r.physical_conflict?"true":"false")
     <<",\"actual\":"<<safety_json(r.actual)<<",\"node\":"<<safety_json(r.node)
     <<",\"connection\":"<<safety_json(r.connection.failure)<<'}';
  return out.str();
}
StartAdmission check_start(const XY &p,const OccupancyMapView &map) { return admit(p,map,map.settings.resolution); }
}  // namespace legbot_exploration
