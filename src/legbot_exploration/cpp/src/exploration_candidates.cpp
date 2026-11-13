#include "legbot_exploration/exploration_candidate.hpp"

#include <algorithm>
#include <cmath>

namespace legbot_exploration {
namespace {
double distance(const XY &a,const XY &b) {return std::hypot(a[0]-b[0],a[1]-b[1]);}
XY center(Cell cell,double r) {return {(cell.x+0.5)*r,(cell.y+0.5)*r};}
template<class Blocked> bool visible(const XY &a,const XY &b,double resolution,Blocked blocked) {
  const int steps=std::max(1,static_cast<int>(std::ceil(distance(a,b)/(resolution*0.4))));
  for(int i=0;i<=steps;++i) {
    const double t=static_cast<double>(i)/steps;
    const Cell cell{static_cast<int>(std::floor((a[0]+t*(b[0]-a[0]))/resolution)),
                    static_cast<int>(std::floor((a[1]+t*(b[1]-a[1]))/resolution))};
    if(blocked(cell)) return false;
  }
  return true;
}
void sort_nearest(std::vector<XY> &points,const XY &origin) {
  std::sort(points.begin(),points.end(),[&](const auto &a,const auto &b) {
    const double da=distance(a,origin),db=distance(b,origin);
    return da==db?a<b:da<db;
  });
}
}
std::vector<XY> nearby_safe_viewpoints(
    const std::unordered_map<Cell,int,CellHash> &grid,const XY &start,
    const Settings &cfg,const std::vector<XY> &obstacles,double min_distance,double max_distance) {
  std::vector<XY> out;
  if(cfg.resolution<=0 || min_distance<0 || max_distance<min_distance ||
     !std::isfinite(distance(start,start))) return out;
  const OccupancyMapView map{grid,cfg,obstacles};
  for(const auto &[cell,state]:grid) {
    if(state!=0) continue;
    const auto p=center(cell,cfg.resolution);
    const double d=distance(start,p);
    if(d>=min_distance && d<=max_distance && point_safe_check(p,map).safe) out.push_back(p);
  }
  sort_nearest(out,start);return out;
}
std::vector<XY> observation_viewpoints(
    const std::unordered_map<Cell,int,CellHash> &grid,const XY &frontier,
    const Settings &cfg,const std::vector<XY> &obstacles,double radius) {
  std::vector<XY> out;
  if(cfg.resolution<=0 || radius<=0 || !std::isfinite(distance(frontier,frontier))) return out;
  Settings endpoint_cfg=cfg;
  endpoint_cfg.robot_radius+=0.15;
  const OccupancyMapView map{grid,endpoint_cfg,obstacles};
  for(const auto &[cell,state]:grid) {
    if(state!=0) continue;
    const auto p=center(cell,cfg.resolution);
    if(distance(p,frontier)>std::max(radius,cfg.resolution*1.1) || !point_safe_check(p,map).safe) continue;
    if(visible(p,frontier,cfg.resolution,[&](Cell c) {
      const auto found=grid.find(c);return found!=grid.end() && found->second==1;
    })) out.push_back(p);
  }
  sort_nearest(out,frontier);return out;
}
}  // namespace legbot_exploration
