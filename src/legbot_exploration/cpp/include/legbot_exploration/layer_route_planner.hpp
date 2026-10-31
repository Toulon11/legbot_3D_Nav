#pragma once

#include "legbot_exploration/same_floor_route_planner.hpp"
#include "legbot_exploration/multi_layer_map.hpp"

namespace legbot_exploration {

// A layer planner owns both its dense A* reference and the execution waypoints.
// The route processor forwards waypoints without changing their geometry.
struct LayerPlan {
  PlannedRoute reference;
  std::vector<XY> waypoints;
};

LayerPlan plan_layer_route(const std::vector<LayerSurface> &surfaces,
                           const std::vector<Point> &occupied_voxels,
                           const Point &base, const XY &target,
                           double resolution, double nominal_body_height,
                           const Settings &cfg, double max_segment_length,
                           double search_radius = 7.0);

// The caller holds SharedLayerMap::mutex for reading throughout this call.
// No full surface or occupied-voxel array is materialized.
LayerPlan plan_layer_route(const OnlineLayerMap &map,
                           const Point &base, const XY &target,
                           double resolution, double nominal_body_height,
                           const Settings &cfg, double max_segment_length,
                           double search_radius, double timeout_ms);

}  // namespace legbot_exploration
