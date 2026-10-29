#pragma once

#include "legbot_exploration/multi_layer_map.hpp"

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace legbot_exploration {

// This graph is a planning preview. An edge means that the observed surface
// heights are locally compatible; it is not a footstep or collision guarantee.
struct SurfaceGraphConfig {
  double xy_resolution = 0.10;
  double search_radius = 7.0;
  double max_step_height = 0.16;
  double max_slope_radians = 0.40;
  double min_height_gain = 0.07;
  double nominal_body_height = 0.32;
  double start_xy_tolerance = 0.45;
  double start_z_tolerance = 0.30;
  int min_support_columns = 5;
  int max_nodes = 25000;
  int max_preview_candidates = 128;
  int max_preview_nodes = 4000;
  int max_preview_edges = 6000;
};

struct SurfaceGraphPreview {
  std::string state = "no_map";
  std::size_t graph_nodes = 0;
  std::size_t graph_edges = 0;
  std::size_t rising_candidates = 0;
  std::size_t expanded_nodes = 0;
  bool nodes_truncated = false;
  bool contains_unobserved_clearance = false;
  std::vector<LayerPoint> preview_nodes;
  std::vector<std::pair<LayerPoint, LayerPoint>> preview_edges;
  std::vector<LayerPoint> preview_candidates;
  std::vector<LayerPoint> path;
};

// Search from the robot's current supporting surface toward a higher,
// reachable step candidate. No ROS messages or motor commands are produced.
SurfaceGraphPreview search_rising_surface(
    const std::vector<LayerSurface> &surfaces, const LayerPoint &base_pose,
    const SurfaceGraphConfig &config);

}  // namespace legbot_exploration
