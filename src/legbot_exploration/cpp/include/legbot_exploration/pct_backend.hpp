#pragma once
#include "legbot_exploration/single_floor_exploration.hpp"
#include <memory>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace legbot_exploration {
struct LayerPoint { double x=0, y=0, z=0; };
struct PctUnknownBlocker {
  Point point;  // Estimated ground XYZ for observation only.
  int slice;
  std::uint64_t version;
};
struct FloorViewLimits {
  double xy_tolerance, first_budget, review_budget, review_slice;
  int max_attempts=2;
  double max_step=0., min_rise=0.;
};
struct PctInspection {
  // Wire payloads only, generated from the current locked PCT map.
  std::vector<std::uint8_t> surfaces,display,returns,ceilings;
  std::vector<Point> preview_nodes,preview_candidates,path;
  std::string preview_state="disabled",metadata="{}";
  bool valid=false;
};
struct PctFrontier {
  std::uint64_t id=0,revision=0;
  Point frontier{};  // Unknown-boundary reference height; observers use measured ground XYZ.
  Cell cell{};
  double gain=0.0,observer_ms=0.0;
  std::size_t cells=0;
  int attempt=0;
  std::vector<Point> observations, display;
  std::vector<double> observation_yaws;
  std::string floor_failure;
  std::optional<Point> first_rejected;
  std::vector<PctUnknownBlocker> unknown_blockers;
};
struct PctFrontiers {
  struct StairView {
    Point ground;
    double yaw;
    std::uint64_t evidence=0;
  };
  bool valid=false;
  std::uint64_t version=0, geometry_version=0;
  std::size_t frontier_cells=0,known_surface_cells=0,safe_cells=0,no_observation_clusters=0;
  std::size_t unreachable_observers=0;
  std::size_t scoped_frontier_cells=0;
  std::size_t deferred_floor_frontier_cells=0;
  std::size_t disconnected_floor_frontier_cells=0;
  std::size_t pending_review_clusters=0;
  bool reviewing=false;
  bool fast_floor=false;
  std::size_t fast_skipped_clusters=0,fast_skipped_cells=0;
  std::size_t stair_flights=0;
  std::size_t stair_step_unsafe=0;
  bool stair_platform_visible=false;
  double stair_index_ms=0.;
  std::vector<PctUnknownBlocker> unknown_blockers;
  std::string scope="global";
  Point window_center{};
  double query_ms=0.;
  bool query_pending=false;
  std::size_t query_checked_clusters=0;
  double observer_prepare_ms=0.,observer_geometry_ms=0.,observer_visibility_ms=0.;
  std::map<std::string,double> observer_phase_ms;
  std::size_t pending_stair_candidates=0;
  double extraction_ms=0.0;
  std::vector<PctFrontier> clusters;
  std::vector<std::uint64_t> no_view_clusters, observer_timeout_clusters;
  std::vector<StairView> stair_targets, stair_step_targets, stair_landing_targets, saved_stair_entrances;
  std::string error, metadata="{}";
};
struct PctResult {
  std::string reason, metadata = "{}";
  std::vector<Point> points;
  std::vector<PctUnknownBlocker> unknown_blockers;
  std::optional<bool> start_safe, goal_safe;
  std::optional<double> start_ground_z;
};
struct PctObservation {
  bool ready=false, fresh=false;
  double yaw=0.;
  std::optional<double> side_yaw;
  std::optional<Point> region;
  std::size_t gain=0;
  std::string reason, floor_attempt, metadata="{}";
  std::vector<PctUnknownBlocker> unknown_blockers;
};
struct PctPointCheck {
  bool ready=false, physical_conflict=false;
  std::string reason, metadata="{}";
  std::optional<double> ground_z, platform_height;
};
struct PctObservationPose {
  enum class Motion { stationary, shift, reverse };
  Point gap, stance;
  double yaw;
  Motion motion;
};
struct PctStairProbe {
  PctFrontier frontier;
  bool active=false,finished=false,returning=false,approaching=false,climbing=false;
  std::string reason,metadata="{}";
};
// The map owner serializes append with plan/read operations using SharedLayerMap.
// Only the current frame is exposed to Python; cumulative storage is sparse height evidence.
class PctBackend {
 public:
  PctBackend(std::string root, double reference_height,
             std::size_t max_surface_records, std::size_t max_cells, std::string settings_json="{}");
  ~PctBackend();
  void append(const std::vector<LayerPoint> &points,std::optional<Point> sensor_origin=std::nullopt);
  bool refresh(std::uint64_t version);
  PctInspection inspect(std::uint64_t version,const Point &base,bool display,
                        std::size_t max_display,bool preview,double radius,
                        double min_gain,std::size_t max_candidates,std::size_t max_nodes);
  PctResult plan(std::uint64_t version, const Point &start, const Point &target,
      bool explicit_surface=false, std::optional<XY> floor=std::nullopt,
      bool collect_blockers=false,bool stair_segment=false);
  PctResult plan_shift(std::uint64_t version,const Point &start,const Point &target);
  PctResult plan_stair_reverse(std::uint64_t version,const Point &start,const Point &target);
  // Body XYZ and roll/pitch/yaw in; surface XYZ targets in. A fixed stance
  // and zero minimum gain admit navigation only, never bypass arrival safety.
  std::vector<PctObservationPose> observation_poses(std::uint64_t version,const Point &base,
      const Point &attitude,const std::vector<Point> &targets,std::optional<bool> include_moves=std::nullopt,
      std::optional<double> ascent=std::nullopt,std::optional<XY> floor=std::nullopt,
      std::optional<Point> stance=std::nullopt,double tolerance=.15,int minimum_gain=1);
  PctFrontiers frontiers(std::uint64_t version,const Point &base=Point{},double minimum_distance=0.0,
                        std::optional<Point> window=std::nullopt,double query_budget=0.,
                        std::optional<Point> continuation=std::nullopt,
                        std::optional<XY> floor=std::nullopt,bool geometry_only=false,
                        std::optional<XY> platform_floor=std::nullopt,
                        std::optional<FloorViewLimits> floor_selection=std::nullopt,
                        std::optional<double> stair_heading=std::nullopt);
  PctStairProbe floor_stair_probe(std::uint64_t version,const Point &base,const XY &floor,double yaw,
                                bool depart,bool review,double tolerance);
  bool complete_observation(std::uint64_t version,const PctFrontier &frontier,const Point &point);
  std::vector<Point> recovery_targets(std::uint64_t version, const Point &start);
  std::vector<Point> stair_review_points(std::uint64_t version,const Point &base,double heading);
  PctPointCheck point_check(std::uint64_t version, const Point &point,bool platform=false,
                            std::optional<XY> departure_floor=std::nullopt,double landing_heading=0.,
                            bool landing_recovery=false,std::optional<XY> floor=std::nullopt,
                            bool tread_support=false,std::optional<double> platform_reference=std::nullopt);
  PctObservation observation_task(std::uint64_t version,const std::string &operation,
      std::uint64_t cluster,const Point &point,double yaw=0.,double half_angle=0.,
      const std::vector<PctUnknownBlocker> *blockers=nullptr);
  PctObservation arrival_area(std::uint64_t version,const Point &target,double xy_tolerance,double heading);
  bool start_ready(std::uint64_t version, const Point &start,std::optional<XY> floor=std::nullopt);
  std::optional<std::uint64_t> recovery_evidence(
      std::uint64_t version,const Point &base,std::optional<Point> forward=std::nullopt);
  std::size_t point_count() const;
  const std::string &last_error() const;
  const std::string &map_metadata() const;
 private:
  PctResult plan_with_method(const char *method,std::uint64_t version,const Point &start,
                             const Point &target,std::optional<XY> floor=std::nullopt);
  std::vector<Point> read_points(const char *method,std::uint64_t version,const Point &base,
      std::optional<double> heading=std::nullopt);
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace legbot_exploration
