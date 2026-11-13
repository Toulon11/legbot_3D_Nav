#pragma once

#include "legbot_exploration/point_safety.hpp"
#include <optional>

namespace legbot_exploration {

enum class CandidateSource { occupancy_frontier, layer_frontier, recovery };
struct SurfaceLocation {
  Cell column;
  double height = 0.0;  // Absolute height distinguishes overlapping floors.
};
// Both future frontier providers return this value to the controller. Planners
// consume the chosen task; they never pick frontiers or declare a floor complete.
struct ExplorationCandidate {
  std::uint64_t id = 0;
  CandidateSource source = CandidateSource::occupancy_frontier;
  Point frontier{};
  std::optional<SurfaceLocation> surface;
  double expected_gain = 0.0;
  double selection_score = 0.0;
  std::uint64_t map_version = 0;
  bool allow_occupancy = true;
  bool allow_layer = true;
};
// Target providers used by the controller. They do not claim path connectivity.
std::vector<XY> nearby_safe_viewpoints(
    const std::unordered_map<Cell,int,CellHash> &grid,const XY &start,
    const Settings &cfg,const std::vector<XY> &scan_obstacles,
    double min_distance,double max_distance);
std::vector<XY> observation_viewpoints(
    const std::unordered_map<Cell,int,CellHash> &grid,const XY &frontier,
    const Settings &cfg,const std::vector<XY> &scan_obstacles,
    double viewpoint_radius=2.0);
inline const char *candidate_source_name(CandidateSource source) {
  switch(source) {
    case CandidateSource::occupancy_frontier:return "occupancy_frontier";
    case CandidateSource::layer_frontier:return "layer_frontier";
    case CandidateSource::recovery:return "recovery";
  }
  return "invalid";
}
}  // namespace legbot_exploration
