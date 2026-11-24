#pragma once
#include "legbot_exploration/single_floor_exploration.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace legbot_exploration {

class StairScanBudget {
 public:
  bool allows(const Point &region) const {
    return std::count_if(regions_.begin(),regions_.end(),[&](const auto &previous) {
      return std::hypot(previous[0]-region[0],previous[1]-region[1])<=.65 &&
          std::abs(previous[2]-region[2])<=.18;
    })<2;
  }

  bool reserve(const Point &region,const Point &ground) {
    if(!allows(region))return false;
    regions_.push_back(region);
    if(!origin_)origin_=ground;
    return true;
  }

  void advance(const Point &ground,double yaw,double minimum_travel,double minimum_rise,double height_slack) {
    if(origin_) {
      const auto &from=*origin_;
      const double forward=(ground[0]-from[0])*std::cos(yaw)+(ground[1]-from[1])*std::sin(yaw);
      if(ground[2]-from[2]<minimum_rise &&
         (forward<minimum_travel || ground[2]<from[2]-height_slack))return;
    }
    regions_.clear();
    origin_=ground;
  }

  void reset() {
    regions_.clear();
    origin_.reset();
  }

 private:
  std::vector<Point> regions_;
  std::optional<Point> origin_;
};

// Controller policy only: no map access, geometry acceptance, or motion output.
class CrossLayerTask {
 public:
  double minimum_rise=.08+1e-7, height_slack=.05, progress_slack=.15;
  bool engaged() const {return phase_=="approaching" || phase_=="ascending" ||
      phase_=="observing" || phase_=="observing_support" || phase_=="exploring_platform" || phase_=="waiting";}
  bool deferred() const {return phase_=="deferred";}
  const std::string &phase() const {return phase_;}
  double progress() const {return std::max(historical_progress_,completed_distance_+progress_);}
  double highest_height() const {return highest_;}
  double rise() const {return highest_-entry_height_;}
  double ascent_yaw() const {return ascent_yaw_.value_or(forward_yaw());}
  std::optional<double> platform_height() const {return platform_height_;}
  void set_ascent_yaw(double yaw) {ascent_yaw_=yaw;}
  const std::string &reason() const {return reason_;}

  void clear() {
    phase_="idle";
    route_.clear();arcs_.clear();
    progress_=completed_distance_=historical_progress_=entry_height_=highest_=0.;
    reposition_height_.reset();
    ascent_yaw_.reset();
    platform_height_.reset();
    reason_.clear();
  }
  // Adopt an actually reached, publicly checked support even if the move
  // began in ordinary exploration or recovery. No synthetic route is created.
  void start_from_support(const Point &support,double departure_height) {
    clear();entry_height_=departure_height;highest_=support[2];
    route_={support};arcs_={0.};
    phase_=rise()>=minimum_rise?"ascending":"approaching";
    reason_="measured_height_changed";
  }
  bool at_progress(const Point &base) const {
    return std::abs(base[2]-highest_)<=.16+1e-7;
  }
  bool can_resume(const Point &base) const {return deferred() && at_progress(base);}
  void resume() {phase_="waiting";reason_="resumed";}
  void defer(const std::string &why) {phase_="deferred";reason_=why;}
  bool begin_reposition(const Point &base) {
    if(!engaged() || rise()<minimum_rise*.5 || !at_progress(base) ||
       (reposition_height_ && base[2]<=*reposition_height_+height_slack))return false;
    reposition_height_=base[2];
    phase_="repositioning";reason_="scan_start_inflated";
    return true;
  }
  void finish_reposition(const Point &base) {
    historical_progress_=progress();
    progress_=project(base).s;
    phase_="waiting";reason_="repositioned";
  }
  void observing() {if(engaged()){phase_="observing";}}
  void exploring_platform(double height) {
    if(!platform_height_)platform_height_=height;
    phase_="exploring_platform";reason_="measuring_landing";
  }
  void leave_platform_candidate() {
    platform_height_.reset();
    if(phase_=="exploring_platform")phase_="ascending";
  }
  bool begin_support_scan() {
    if(!engaged() || phase_=="observing_support")return false;
    phase_="observing_support";reason_="forward_support_missing";
    return true;
  }
  void observed() {
    if(!engaged())return;
    // A normal same-height view must not reactivate ascent before that
    // height band's local/global first pass is settled.
    if(phase_=="exploring_platform")return;
    if(phase_=="observing_support")reason_="awaiting_forward_route_after_support_scan";
    phase_="waiting";
  }
  void complete() {phase_="completed";reason_="safe_platform";}
  double forward_yaw() const {
    if(route_.size()<2)return 0.;
    const auto at=std::min<std::size_t>(route_.size()-2,
        std::upper_bound(arcs_.begin(),arcs_.end(),progress_)-arcs_.begin()-1);
    return std::atan2(route_[at+1][1]-route_[at][1],route_[at+1][0]-route_[at][0]);
  }
  void update(const Point &base) {
    if(!engaged())return;
    const auto p=project(base);
    progress_=std::max(progress_,p.s);
    highest_=std::max(highest_,base[2]);
    if(phase_=="approaching" && rise()>=minimum_rise)phase_="ascending";
  }
  bool allows_recovery(const Point &base,const Point &point,double step_limit) const {
    return std::abs(point[2]-base[2])<=std::min(.16,step_limit)+1e-7;
  }
  bool allows(const Point &point) const {
    return (!engaged() && !deferred()) || point[2]>=highest_-height_slack;
  }
  bool route_allowed(const Point &,const std::vector<Point> &route,
                     std::optional<double> =std::nullopt) const {
    // Called after common native safety admission. Direction and a complete
    // stair shape impose no additional corridor or monotonic-route gates.
    return !route.empty();
  }
  bool starts_above_departure(const Point &base,const std::vector<Point> &route,
                              std::optional<double> departure_height) const {
    const double threshold=minimum_rise;
    return !route.empty() && route.back()[2]-departure_height.value_or(base[2])>=threshold;
  }

  // Called only AFTER native planning and ordinary safety admission succeed.
  bool accept(const Point &base,const std::vector<Point> &route,
              std::optional<double> departure_height=std::nullopt,bool scheduled_ascent=false) {
    if(route.empty() || std::none_of(route.begin(),route.end(),[&](const auto &p){return distance(base,p)>1e-6;}))
      return false;
    if(!engaged() && !scheduled_ascent && !starts_above_departure(base,route,departure_height))return false;
    if(!route_allowed(base,route,departure_height))return false;
    // A small intermediate landing can have no remaining observation tasks.
    // Only an accepted upward route may start a new platform search height.
    if(platform_height_ && route.back()[2]>*platform_height_+height_slack)
      platform_height_.reset();
    if(!engaged()) {
      clear();
      entry_height_=departure_height.value_or(base[2]);
      highest_=base[2];
      phase_="approaching";
    }
    // Re-anchor the remaining verified route at the actual pose. Height and
    // completed-task progress remain persistent across planning decisions.
    completed_distance_+=progress_;
    route_.clear();route_.push_back(base);
    for(const auto &p:route)if(distance(route_.back(),p)>1e-6)route_.push_back(p);
    if(route_.size()<2)return false;
    arcs_={0.};for(std::size_t i=1;i<route_.size();++i)arcs_.push_back(arcs_.back()+distance(route_[i-1],route_[i]));
    progress_=0.;reason_="verified_upward_route";
    if(rise()>=minimum_rise)phase_="ascending";
    return true;
  }

 private:
  std::optional<double> reposition_height_,ascent_yaw_,platform_height_;
  double historical_progress_=0.;
  struct Projection {double s=0.,distance=std::numeric_limits<double>::infinity();};
  static double distance(const Point &a,const Point &b) {return std::hypot(std::hypot(a[0]-b[0],a[1]-b[1]),a[2]-b[2]);}
  Projection project(const Point &point) const {
    Projection best;
    for(std::size_t i=1;i<route_.size();++i) {
      const auto &a=route_[i-1];const auto &b=route_[i];
      const double length=arcs_[i]-arcs_[i-1];
      if(length<1e-6)continue;
      double dot=0.;for(int j=0;j<3;++j)dot+=(point[j]-a[j])*(b[j]-a[j]);
      const double t=std::clamp(dot/(length*length),0.,1.);
      Point on{};for(int j=0;j<3;++j)on[j]=a[j]+t*(b[j]-a[j]);
      const double d=distance(on,point);
      if(d<best.distance)best={arcs_[i-1]+t*length,d};
    }
    // At a stair entrance the measured ascent can turn back from the approach
    // route. Extend along that ascent when available; native planning still
    // verifies the actual route to each candidate.
    if(route_.size()>1) {
      const auto &a=route_[route_.size()-2];const auto &b=route_.back();
      const Point direction=ascent_yaw_?Point{std::cos(*ascent_yaw_),std::sin(*ascent_yaw_),0.}:
          Point{b[0]-a[0],b[1]-a[1],b[2]-a[2]};
      const double length=distance({0.,0.,0.},direction);double dot=0.;
      for(int j=0;j<3;++j)dot+=(point[j]-b[j])*direction[j];
      const double beyond=dot/(length*length);
      if(beyond>0.) {
        Point on{};for(int j=0;j<3;++j)on[j]=b[j]+beyond*direction[j];
        const double d=distance(on,point);
        if(d<best.distance)best={arcs_.back()+beyond*length,d};
      }
    }
    return best;
  }
  std::string phase_="idle",reason_;
  std::vector<Point> route_;
  std::vector<double> arcs_;
  double progress_=0.,completed_distance_=0.,entry_height_=0.,highest_=0.;
};
}  // namespace legbot_exploration
