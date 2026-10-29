#include "legbot_exploration/surface_graph.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace legbot_exploration {
namespace {

struct Cell {
  int x, y;
  bool operator==(const Cell &other) const { return x == other.x && y == other.y; }
};
struct CellHash {
  std::size_t operator()(const Cell &cell) const {
    std::size_t seed = std::hash<int>{}(cell.x);
    return seed ^ (std::hash<int>{}(cell.y) + 0x9e3779b9u + (seed << 6u) + (seed >> 2u));
  }
};
struct Node {
  const LayerSurface *surface;
  LayerPoint point;
  std::vector<std::pair<int, double>> neighbors;
};
struct QueueItem {
  double estimate, cost;
  int index;
  bool operator<(const QueueItem &other) const { return estimate > other.estimate; }
};

double distance(const LayerPoint &a, const LayerPoint &b) {
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}
LayerPoint point_of(const LayerSurface &s, double resolution) {
  return {(s.x_cell + 0.5) * resolution, (s.y_cell + 0.5) * resolution, s.height};
}

}  // namespace

SurfaceGraphPreview search_rising_surface(
    const std::vector<LayerSurface> &surfaces, const LayerPoint &base_pose,
    const SurfaceGraphConfig &cfg) {
  if (!(cfg.xy_resolution > 0 && cfg.search_radius > 0 && cfg.max_step_height > 0 &&
        cfg.max_slope_radians > 0 && cfg.min_height_gain > 0 &&
        cfg.start_xy_tolerance > 0 && cfg.start_z_tolerance > 0 &&
        cfg.min_support_columns > 0 && cfg.max_nodes > 0 &&
        cfg.max_preview_candidates > 0 && cfg.max_preview_nodes > 0 &&
        cfg.max_preview_edges > 0))
    throw std::invalid_argument("invalid surface graph configuration");

  SurfaceGraphPreview result;
  if (surfaces.empty()) return result;
  if (!std::isfinite(base_pose.x) || !std::isfinite(base_pose.y) ||
      !std::isfinite(base_pose.z)) {
    result.state = "invalid_base_pose";
    return result;
  }

  // Retain nearby, supported surfaces. A column may contribute several nodes
  // at different measured heights, so slice_index is never used as a node ID.
  std::vector<const LayerSurface *> nearby;
  nearby.reserve(std::min(surfaces.size(), static_cast<std::size_t>(cfg.max_nodes)));
  const double radius_sq = cfg.search_radius * cfg.search_radius;
  for (const auto &s : surfaces) {
    if (!std::isfinite(s.height) || s.hit_observations < 1 ||
        s.support_columns < cfg.min_support_columns ||
        s.clearance == ClearanceKind::blocked ||
        (s.kind != SurfaceKind::flat && s.kind != SurfaceKind::step_candidate)) continue;
    const LayerPoint p = point_of(s, cfg.xy_resolution);
    const double dx = p.x - base_pose.x, dy = p.y - base_pose.y;
    if (dx * dx + dy * dy <= radius_sq) nearby.push_back(&s);
  }
  if (nearby.empty()) {
    result.state = "no_supported_surface";
    return result;
  }
  if (nearby.size() > static_cast<std::size_t>(cfg.max_nodes)) {
    result.nodes_truncated = true;
    const auto proximity = [&](const LayerSurface *a, const LayerSurface *b) {
      const LayerPoint pa = point_of(*a, cfg.xy_resolution);
      const LayerPoint pb = point_of(*b, cfg.xy_resolution);
      const double da = std::hypot(pa.x - base_pose.x, pa.y - base_pose.y);
      const double db = std::hypot(pb.x - base_pose.x, pb.y - base_pose.y);
      return da == db ? pa.z < pb.z : da < db;
    };
    std::nth_element(nearby.begin(), nearby.begin() + cfg.max_nodes,
                     nearby.end(), proximity);
    nearby.resize(cfg.max_nodes);
  }

  std::vector<Node> nodes;
  nodes.reserve(nearby.size());
  std::unordered_map<Cell, std::vector<int>, CellHash> columns;
  columns.reserve(nearby.size());
  for (const LayerSurface *surface : nearby) {
    const int index = static_cast<int>(nodes.size());
    nodes.push_back({surface, point_of(*surface, cfg.xy_resolution), {}});
    columns[{surface->x_cell, surface->y_cell}].push_back(index);
  }
  result.graph_nodes = nodes.size();

  const double desired_ground_z = base_pose.z - cfg.nominal_body_height;
  int start = -1;
  double best_start = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
    const auto &p = nodes[i].point;
    const double xy = std::hypot(p.x - base_pose.x, p.y - base_pose.y);
    const double dz = std::abs(p.z - desired_ground_z);
    if (xy > cfg.start_xy_tolerance || dz > cfg.start_z_tolerance) continue;
    const double score = xy + 2.0 * dz;
    if (score < best_start) { best_start = score; start = i; }
  }
  if (start < 0) {
    result.state = "no_surface_under_robot";
    return result;
  }

  // Only 4-connected horizontal neighbors are joined. For a flat-flat edge,
  // enforce the configured slope; an edge touching a step may use the larger
  // single-step height. Unknown overhead is kept as a provisional preview.
  const double ramp_height = std::tan(cfg.max_slope_radians) * cfg.xy_resolution;
  constexpr std::array<Cell, 2> positive_neighbors{{{1, 0}, {0, 1}}};
  std::vector<std::pair<int, int>> edges;
  for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
    const auto *surface = nodes[i].surface;
    for (const Cell &offset : positive_neighbors) {
      const auto it = columns.find({surface->x_cell + offset.x,
                                    surface->y_cell + offset.y});
      if (it == columns.end()) continue;
      for (int j : it->second) {
        const double dz = std::abs(nodes[i].point.z - nodes[j].point.z);
        const bool step = surface->kind == SurfaceKind::step_candidate ||
                          nodes[j].surface->kind == SurfaceKind::step_candidate;
        if (dz > (step ? cfg.max_step_height : ramp_height) + 1e-6) continue;
        const double penalty = (step ? 0.02 : 0.0) +
            ((surface->clearance == ClearanceKind::unobserved ||
              nodes[j].surface->clearance == ClearanceKind::unobserved) ? 0.05 : 0.0);
        const double cost = distance(nodes[i].point, nodes[j].point) + penalty;
        nodes[i].neighbors.push_back({j, cost});
        nodes[j].neighbors.push_back({i, cost});
        edges.push_back({i, j});
      }
    }
  }
  result.graph_edges = edges.size();
  const std::size_t node_stride = std::max<std::size_t>(
      1, (nodes.size() + cfg.max_preview_nodes - 1) / cfg.max_preview_nodes);
  for (std::size_t i = 0; i < nodes.size(); i += node_stride)
    result.preview_nodes.push_back(nodes[i].point);
  const std::size_t edge_stride = std::max<std::size_t>(
      1, (edges.size() + cfg.max_preview_edges - 1) / cfg.max_preview_edges);
  for (std::size_t i = 0; i < edges.size(); i += edge_stride)
    result.preview_edges.push_back({nodes[edges[i].first].point,
                                    nodes[edges[i].second].point});

  std::vector<int> candidates;
  for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
    if (nodes[i].surface->kind == SurfaceKind::step_candidate &&
        nodes[i].point.z >= nodes[start].point.z + cfg.min_height_gain)
      candidates.push_back(i);
  }
  result.rising_candidates = candidates.size();
  if (candidates.empty()) {
    result.state = "no_rising_step_candidate";
    return result;
  }
  // Display only nearby candidates, but search every candidate in the graph.
  // Otherwise a reachable stair could be missed behind many disconnected
  // yellow cells near the robot.
  std::sort(candidates.begin(), candidates.end(), [&](int a, int b) {
    const double da = distance(nodes[start].point, nodes[a].point);
    const double db = distance(nodes[start].point, nodes[b].point);
    return da == db ? a < b : da < db;
  });
  for (std::size_t i = 0;
       i < std::min(candidates.size(), static_cast<std::size_t>(cfg.max_preview_candidates)); ++i)
    result.preview_candidates.push_back(nodes[candidates[i]].point);

  std::vector<uint8_t> goal(nodes.size(), 0);
  LayerPoint low{std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity()};
  LayerPoint high{-low.x, -low.y, -low.z};
  for (int candidate : candidates) {
    goal[candidate] = 1;
    const auto &point = nodes[candidate].point;
    low.x = std::min(low.x, point.x); low.y = std::min(low.y, point.y);
    low.z = std::min(low.z, point.z);
    high.x = std::max(high.x, point.x); high.y = std::max(high.y, point.y);
    high.z = std::max(high.z, point.z);
  }
  // Distance to the candidates' bounding box never exceeds distance to any
  // candidate, so it remains an admissible A* heuristic for multiple goals.
  const auto heuristic = [&](int index) {
    const auto &p = nodes[index].point;
    const auto axis_gap = [](double v, double min_v, double max_v) {
      return std::max({min_v - v, 0.0, v - max_v});
    };
    return std::hypot(std::hypot(axis_gap(p.x, low.x, high.x),
                                 axis_gap(p.y, low.y, high.y)),
                      axis_gap(p.z, low.z, high.z));
  };
  std::vector<double> best(nodes.size(), std::numeric_limits<double>::infinity());
  std::vector<int> parent(nodes.size(), -1);
  std::priority_queue<QueueItem> open;
  best[start] = 0.0;
  open.push({heuristic(start), 0.0, start});
  int reached = -1;
  while (!open.empty()) {
    const QueueItem current = open.top();
    open.pop();
    if (current.cost > best[current.index] + 1e-9) continue;
    ++result.expanded_nodes;
    if (goal[current.index]) { reached = current.index; break; }
    for (const auto &[neighbor, cost] : nodes[current.index].neighbors) {
      const double next = current.cost + cost;
      if (next + 1e-9 >= best[neighbor]) continue;
      best[neighbor] = next;
      parent[neighbor] = current.index;
      open.push({next + heuristic(neighbor), next, neighbor});
    }
  }
  if (reached < 0) {
    result.state = "rising_candidates_unreachable_in_window";
    return result;
  }
  std::vector<int> reverse_path;
  for (int index = reached; index >= 0; index = parent[index])
    reverse_path.push_back(index);
  for (auto it = reverse_path.rbegin(); it != reverse_path.rend(); ++it) {
    result.path.push_back(nodes[*it].point);
    result.contains_unobserved_clearance |=
        nodes[*it].surface->clearance == ClearanceKind::unobserved;
  }
  result.state = result.contains_unobserved_clearance ?
      "preview_path_clearance_unobserved" : "preview_path_observed";
  return result;
}

}  // namespace legbot_exploration
