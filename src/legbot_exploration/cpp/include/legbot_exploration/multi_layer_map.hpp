#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace legbot_exploration {

// All heights in this map are absolute heights in the configured map frame.
struct LayerMapConfig {
  double xy_resolution = 0.10;
  double z_resolution = 0.10;
  double slice_height = 0.50;
  double max_ray_range = 12.0;
  double surface_merge_gap = 0.15;
  double support_height_tolerance = 0.20;
  double min_clearance = 0.55;
  double max_slope_radians = 0.40;
  double max_step_height = 0.16;
  int support_radius_cells = 2;
  int min_support_columns = 5;
  int min_hit_observations = 2;
  int max_surfaces_per_column = 8;
  int max_voxels = 8000000;
};

struct LayerPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

enum class SurfaceKind : uint8_t {
  insufficient_support,
  flat,
  step_candidate,
  steep_or_disconnected,
};

enum class ClearanceKind : uint8_t { blocked, observed_clear, unobserved };

struct LayerSurface {
  int x_cell = 0;
  int y_cell = 0;
  int slice_index = 0;  // A height bin for inspection, not a semantic floor ID.
  double height = 0.0;
  double ceiling = 0.0;  // NaN means no overhead return has been observed.
  int support_columns = 0;
  int hit_observations = 0;
  double last_observed = 0.0;
  SurfaceKind kind = SurfaceKind::insufficient_support;
  ClearanceKind clearance = ClearanceKind::unobserved;
};

struct LayerMapStats {
  std::size_t voxels = 0;
  std::size_t columns = 0;
  std::size_t surfaces = 0;
  std::size_t flat = 0;
  std::size_t step_candidates = 0;
  std::size_t insufficient_support = 0;
  std::size_t blocked_clearance = 0;
  std::size_t unobserved_clearance = 0;
  bool map_full = false;
  uint64_t version = 0;
};

class OnlineLayerMap {
 public:
  explicit OnlineLayerMap(LayerMapConfig config);
  std::size_t integrate(const LayerPoint &origin, const std::vector<LayerPoint> &points,
                        double stamp_seconds = 0.0, std::size_t ray_stride = 1);
  std::size_t rebuild_dirty();
  std::vector<LayerSurface> surfaces() const;
  std::vector<LayerSurface> surfaces_at(double x, double y) const;
  LayerMapStats stats() const;
  bool full() const { return map_full_; }

 private:
  struct Voxel {
    int x = 0, y = 0, z = 0;
    bool operator==(const Voxel &rhs) const {
      return x == rhs.x && y == rhs.y && z == rhs.z;
    }
  };
  struct Cell {
    int x = 0, y = 0;
    bool operator==(const Cell &rhs) const { return x == rhs.x && y == rhs.y; }
  };
  struct VoxelHash {
    std::size_t operator()(const Voxel &v) const;
  };
  struct CellHash {
    std::size_t operator()(const Cell &c) const;
  };
  struct Evidence {
    int score = 0;
    int hits = 0;
    int downward_hits = 0;
    int rays = 0;
    double z_sum = 0.0;
    double downward_z_sum = 0.0;
    double last_observed = 0.0;
  };
  struct Candidate {
    double height = 0.0;
    int hits = 0;
    double last_observed = 0.0;
  };

  Voxel key(const LayerPoint &point) const;
  void update(const Voxel &voxel, bool hit, bool downward_hit = false,
              double point_z = 0.0, double stamp_seconds = 0.0);
  std::vector<Candidate> candidates(const Cell &cell) const;
  std::vector<LayerSurface> derive(const Cell &cell) const;
  ClearanceKind clearance(const Cell &cell, double ground, double *ceiling) const;
  void mark_dirty(const Cell &cell);

  LayerMapConfig cfg_;
  std::unordered_map<Voxel, Evidence, VoxelHash> evidence_;
  std::unordered_map<Cell, std::vector<int>, CellHash> column_z_;
  std::unordered_map<Cell, std::vector<LayerSurface>, CellHash> surfaces_;
  std::unordered_set<Cell, CellHash> dirty_;
  bool map_full_ = false;
  uint64_t version_ = 0;
};

}  // namespace legbot_exploration
