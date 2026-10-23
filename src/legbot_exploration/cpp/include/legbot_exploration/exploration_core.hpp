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
  bool global_search = false;
  double exploration_min_x = -31.5;
  double exploration_max_x = -0.5;
  double exploration_min_y = -15.5;
  double exploration_max_y = 15.5;
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
  void set_exploration_bounds(double min_x, double max_x, double min_y, double max_y);
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
  std::size_t frontier_cells = 0;
  std::size_t reachable_frontier_cells = 0;
  std::size_t candidate_goals = 0;
  std::size_t suppressed_goals = 0;
  std::size_t open_start_neighbors = 0;
};

// Permit a trajectory backtrack only for a real, currently unreachable
// frontier cluster. A backtrack that observes too little new XY area is not
// repeated until the map gains enough genuinely new cells.
class RecoveryGate {
 public:
  explicit RecoveryGate(std::size_t min_new_cells) : min_new_cells_(min_new_cells) {}
  bool eligible(const Selection &selection, std::size_t observed_cells) const;
  bool waiting_for_new_cells(std::size_t observed_cells) const;
  void start(std::size_t observed_cells);
  void finish(std::size_t observed_cells);

 private:
  std::size_t min_new_cells_;
  std::optional<std::size_t> started_at_;
  std::optional<std::size_t> blocked_at_;
};

// A reached goal that revealed no new XY cells stays suppressed until fresh
// observations justify trying that area again.
class GoalProgressGate {
 public:
  explicit GoalProgressGate(std::size_t min_new_cells) : min_new_cells_(min_new_cells) {}
  void finish(const XY &goal, std::size_t before, std::size_t after);
  std::vector<XY> blocked(std::size_t observed_cells);

 private:
  struct Entry { XY goal; std::size_t unlock_at; };
  std::size_t min_new_cells_;
  std::vector<Entry> entries_;
};

Selection select_frontier(const VoxelMap &map, const Point &robot, const Settings &cfg,
                          const std::vector<XY> &suppressed,
                          const std::vector<XY> &external_obstacles = {});

}  // namespace legbot_exploration
