#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace legbot_exploration {

struct Settings {
  double resolution = 0.25;
  double max_ray_range = 9.0;
  std::size_t max_voxels = 900000;
  double obstacle_min_relative_z = -0.15;
  double obstacle_max_relative_z = 0.85;
  double free_min_relative_z = -0.45;
  double robot_radius = 0.35;
  double local_search_radius = 10.0;
  double min_goal_distance = 1.0;
  double max_goal_distance = 8.0;
  int min_frontier_cells = 3;
  double frontier_gain_weight = 1.0;
  double path_cost_weight = 1.0;
  double unknown_path_multiplier = 1.5;
};

struct Cell {
  int x = 0, y = 0;
  bool operator==(const Cell &rhs) const { return x == rhs.x && y == rhs.y; }
};
struct Voxel {
  int x = 0, y = 0, z = 0;
  bool operator==(const Voxel &rhs) const {
    return x == rhs.x && y == rhs.y && z == rhs.z;
  }
};
struct CellHash {
  std::size_t operator()(const Cell &c) const;
};
struct VoxelHash {
  std::size_t operator()(const Voxel &v) const;
};
using Point = std::array<double, 3>;
using XY = std::array<double, 2>;

class VoxelMap {
 public:
  explicit VoxelMap(Settings settings) : cfg_(settings) {}
  Voxel key(const Point &point) const;
  Point center(const Voxel &voxel) const;
  XY center(const Cell &cell) const;
  std::size_t integrate(const Point &origin, const std::vector<Point> &points, double base_z);
  // Z evidence is stored relative to the base height at observation time.
  std::unordered_map<Cell, int, CellHash> project(const XY &center_xy) const;
  std::size_t size() const { return voxels_.size(); }
  std::size_t version() const { return version_; }

 private:
  void mark(const Voxel &voxel, bool hit);
  Settings cfg_;
  std::unordered_map<Voxel, int, VoxelHash> voxels_;
  std::size_t version_ = 0;
};

struct Selection {
  std::optional<XY> goal;
  double score = 0.0;
  std::size_t clusters = 0;
};
Selection select_frontier(const VoxelMap &map, const Point &robot, const Settings &cfg,
                          const std::vector<XY> &suppressed,
                          const std::vector<XY> &external_obstacles = {});

}  // namespace legbot_exploration
